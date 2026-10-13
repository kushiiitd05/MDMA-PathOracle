"""BVI planner node.

Generates the 30x30 map, runs synchronous BVI and queue BVI (ABVI) from
bvi_core, cross-checks their cost-to-go maps and publishes everything for
RViz2 with transient-local (latched) QoS.

Topics:
  /map            nav_msgs/OccupancyGrid     free = 0, obstacle = 100
  /bvi_values     visualization_msgs/MarkerArray  J heat-map + labels + start/goal
  /bvi_path       nav_msgs/Path              optimal path from synchronous BVI
  /abvi_path      nav_msgs/Path              optimal path from queue BVI
  /bvi_policy     visualization_msgs/MarkerArray  pi(s) arrows (sync)
  /abvi_policy    visualization_msgs/MarkerArray  pi(s) arrows (queue)
  /bvi_stats      visualization_msgs/Marker  text panel with the comparison
"""
from __future__ import annotations

from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from bvi_core.bvi import run_and_crosscheck
from bvi_core.difficulty import QUANTILES, select_difficulty_seeds
from bvi_core.grid import GRID_SIZE, generate_reachable_map
from bvi_core.metrics import count_turns
from bvi_core.path import NoPathError, extract_path, validate_path
from bvi_core.verify import certify

from . import markers
from .runner import run_node

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
BVI_COLOR, ABVI_COLOR = "#1c7ed6", "#e8590c"


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("bvi_planner")
        p = {
            "difficulty": self.declare_parameter("difficulty", "medium").value,
            "seed": self.declare_parameter("seed", -1).value,
            "start": self.declare_parameter("start", [1, 1]).value,
            "goal": self.declare_parameter("goal", [GRID_SIZE - 2, GRID_SIZE - 2]).value,
            "resolution": self.declare_parameter("resolution", 1.0).value,
            "show_text": self.declare_parameter("show_text", True).value,
            "seconds_per_move": self.declare_parameter("seconds_per_move", 0.4).value,
        }
        start, goal = self._cell(p["start"], "start"), self._cell(p["goal"], "goal")
        res = float(p["resolution"])
        if res <= 0:
            raise ValueError("resolution must be > 0")

        seed, label = self._pick_seed(p["difficulty"], int(p["seed"]), start, goal)
        grid, rejected = generate_reachable_map(seed, start, goal)
        if rejected:
            self.get_logger().warn(f"seeds {rejected} disconnected -> using seed {grid.seed}")

        sync, queue = run_and_crosscheck(grid)   # raises if J maps differ
        self.get_logger().info("cross-check OK: J_sync == J_queue on all cells")

        paths, certificates = {}, {}
        for name, result in (("BVI", sync), ("ABVI", queue)):
            try:
                path = extract_path(grid, result.values, result.policy)
                validate_path(grid, result.values, path)
            except NoPathError as exc:
                self.get_logger().error(f"{name}: {exc}")
                path = None
            certificates[name] = certify(grid, result.values, path)
            for line in certificates[name].lines():
                log = self.get_logger().info if certificates[name].passed else self.get_logger().error
                log(f"{name} {line}")
            paths[name] = path or []

        stamp = self.get_clock().now().to_msg()
        pubs = {
            "map": self.create_publisher(OccupancyGrid, "map", LATCHED),
            "values": self.create_publisher(MarkerArray, "bvi_values", LATCHED),
            "bvi_path": self.create_publisher(Path, "bvi_path", LATCHED),
            "abvi_path": self.create_publisher(Path, "abvi_path", LATCHED),
            "bvi_policy": self.create_publisher(MarkerArray, "bvi_policy", LATCHED),
            "abvi_policy": self.create_publisher(MarkerArray, "abvi_policy", LATCHED),
            "stats": self.create_publisher(Marker, "bvi_stats", LATCHED),
        }
        pubs["map"].publish(markers.occupancy_grid(grid, res, stamp))
        pubs["values"].publish(markers.value_markers(grid, queue, res, stamp, p["show_text"]))
        pubs["bvi_path"].publish(markers.path_msg(paths["BVI"], res, stamp, 0.15 * res))
        pubs["abvi_path"].publish(markers.path_msg(paths["ABVI"], res, stamp, 0.2 * res))
        pubs["bvi_policy"].publish(
            markers.policy_markers(sync.policy, res, stamp, BVI_COLOR, "bvi_policy"))
        pubs["abvi_policy"].publish(
            markers.policy_markers(queue.policy, res, stamp, ABVI_COLOR, "abvi_policy"))

        lines = self._summary(label, grid, sync, queue, paths, float(p["seconds_per_move"]))
        lines += [f"{name}: {cert.summary()}" for name, cert in certificates.items()]
        pubs["stats"].publish(markers.text_panel(lines, res, grid.width, grid.height, stamp))
        for line in lines:
            self.get_logger().info(line)
        self._pubs = pubs  # keep publishers alive

    def _cell(self, value, name: str) -> tuple[int, int]:
        if len(value) != 2 or not all(0 <= int(v) < GRID_SIZE for v in value):
            raise ValueError(f"{name} must be [x, y] inside the {GRID_SIZE}x{GRID_SIZE} grid")
        return int(value[0]), int(value[1])

    def _pick_seed(self, difficulty: str, seed: int, start, goal) -> tuple[int, str]:
        if seed >= 0:
            return seed, "custom"
        if difficulty not in QUANTILES:
            raise ValueError(f"difficulty must be one of {list(QUANTILES)}")
        self.get_logger().info("selecting difficulty seeds (500-map pool)...")
        report = select_difficulty_seeds(start, goal)
        chosen = report["selected"][difficulty]
        self.get_logger().info(f"{difficulty}: seed {chosen['seed']} "
                               f"(stretch {chosen['stretch']:.3f}, turns {chosen['turns']})")
        return chosen["seed"], difficulty

    @staticmethod
    def _summary(label, grid, sync, queue, paths, sec_per_move) -> list[str]:
        j0 = sync.J(grid.start)
        lines = [
            f"difficulty: {label}   seed: {grid.seed}",
            f"30x30, obstacles {grid.num_obstacles} (20%)",
            f"start {grid.start}  goal {grid.goal}",
            f"J(start) = {j0:.0f}   cross-check J_BVI == J_ABVI: OK",
            "",
        ]
        for name, res, extra in (("BVI  (sync, blue)", sync, f"sweeps {sync.stats['sweeps']}"),
                                 ("ABVI (queue, orange)", queue,
                                  f"queue pops {queue.stats['queue_pops']}")):
            path = paths[name.split()[0]]
            moves = len(path) - 1
            lines += [
                name,
                f"  runtime {res.stats['runtime_s'] * 1e3:.2f} ms   {extra}",
                f"  state updates {res.stats['state_updates']}"
                f"   value changes {res.stats['value_changes']}",
                f"  path cost {moves}   turns {count_turns(path)}"
                f"   travel time {moves * sec_per_move:.1f} s",
                "",
            ]
        return lines


def main(args=None) -> None:
    run_node(PlannerNode, args)


if __name__ == "__main__":
    main()

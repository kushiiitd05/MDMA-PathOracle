"""Independent live verifier.

Uses ONLY what is published on ROS topics (/map and a path topic): rebuilds
the grid from the OccupancyGrid, recomputes J with reference synchronous BVI,
and runs the optimality certificate (bvi_core.verify) on the published path.
Result -> /optimality_report (std_msgs/String, latched) and the log.
"""
from __future__ import annotations

import math

import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from bvi_core.bvi import synchronous_bvi
from bvi_core.grid import GridMap
from bvi_core.verify import certify

from .runner import run_node

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
PATH_TOPICS = ("bvi_path", "abvi_path")


class OptimalityChecker(Node):
    def __init__(self) -> None:
        super().__init__("optimality_checker")
        self.map_msg: OccupancyGrid | None = None
        self.paths: dict[str, Path] = {}
        self.reports: dict[str, str] = {}
        self.pub = self.create_publisher(String, "optimality_report", LATCHED)
        self.create_subscription(OccupancyGrid, "map", self._on_map, LATCHED)
        for topic in PATH_TOPICS:
            self.create_subscription(Path, topic, lambda m, t=topic: self._on_path(t, m), LATCHED)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self._check_all()

    def _on_path(self, topic: str, msg: Path) -> None:
        self.paths[topic] = msg
        self._check_all()

    def _check_all(self) -> None:
        if self.map_msg is None:
            return
        for topic, path_msg in self.paths.items():
            if topic not in self.reports:
                self.reports[topic] = self._check(topic, path_msg)
        if self.reports:
            self.pub.publish(String(data="\n\n".join(self.reports[t] for t in sorted(self.reports))))

    def _check(self, topic: str, path_msg: Path) -> str:
        info = self.map_msg.info
        res = info.resolution
        occupied = np.array(self.map_msg.data, dtype=np.int16).reshape(info.height, info.width) >= 50
        cells = [(math.floor(p.pose.position.x / res), math.floor(p.pose.position.y / res))
                 for p in path_msg.poses]
        if not cells:
            text = f"/{topic}: empty path (no path published)"
            self.get_logger().warn(text)
            return text
        try:
            grid = GridMap(occupied, start=cells[0], goal=cells[-1])
        except ValueError as exc:
            text = f"/{topic}: INVALID - {exc}"
            self.get_logger().error(text)
            return text
        reference = synchronous_bvi(grid)   # recomputed from the map alone
        cert = certify(grid, reference.values, cells)
        lines = [f"/{topic}  (J recomputed independently from /map, "
                 f"{int(reference.stats['sweeps'])} sweeps)"] + cert.lines()
        log = self.get_logger().info if cert.passed else self.get_logger().error
        for line in lines:
            log(line)
        return "\n".join(lines)


def main(args=None) -> None:
    run_node(OptimalityChecker, args)


if __name__ == "__main__":
    main()

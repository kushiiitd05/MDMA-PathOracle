"""Grid workspace, N/S/E/W motion model and seeded random map generation.

Coordinate convention (identical to ROS nav_msgs/OccupancyGrid):
    cell = (x, y), x = column, y = row, (0, 0) is the bottom-left cell.
    Arrays are indexed as array[y, x].
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

Cell = tuple[int, int]

GRID_SIZE = 30
OBSTACLE_FRACTION = 0.20
NUM_OBSTACLES = round(GRID_SIZE * GRID_SIZE * OBSTACLE_FRACTION)  # 180
# One-cell margin from opposite corners.
DEFAULT_START: Cell = (1, 1)
DEFAULT_GOAL: Cell = (GRID_SIZE - 2, GRID_SIZE - 2)
MAX_MAP_ATTEMPTS = 1000

# Motion primitives (dx, dy). Dict order is the deterministic tie-break order.
ACTIONS: dict[str, Cell] = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
STEP_COST = 1


@dataclass(frozen=True, eq=False)
class GridMap:
    """Immutable occupancy grid with start/goal and cached neighbour lists."""

    occupied: np.ndarray  # bool, shape (height, width)
    start: Cell
    goal: Cell
    seed: int | None = None
    _succ: dict[Cell, tuple[Cell, ...]] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        occ = np.array(self.occupied, dtype=bool, copy=True)
        occ.setflags(write=False)
        object.__setattr__(self, "occupied", occ)
        for name, cell in (("start", self.start), ("goal", self.goal)):
            if not self.in_bounds(cell):
                raise ValueError(f"{name} {cell} is outside the {self.width}x{self.height} grid")
            if occ[cell[1], cell[0]]:
                raise ValueError(f"{name} {cell} is an obstacle")
        succ = {c: tuple(self._valid_moves(c)) for c in self.free_cells()}
        object.__setattr__(self, "_succ", succ)

    @property
    def width(self) -> int:
        return self.occupied.shape[1]

    @property
    def height(self) -> int:
        return self.occupied.shape[0]

    @property
    def num_obstacles(self) -> int:
        return int(self.occupied.sum())

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and not self.occupied[cell[1], cell[0]]

    def free_cells(self) -> list[Cell]:
        ys, xs = np.nonzero(~self.occupied)
        return [(int(x), int(y)) for y, x in zip(ys, xs)]

    def _valid_moves(self, cell: Cell) -> list[Cell]:
        x, y = cell
        moves = ((x + dx, y + dy) for dx, dy in ACTIONS.values())
        return [c for c in moves if self.is_free(c)]

    def successors(self, cell: Cell) -> tuple[Cell, ...]:
        """States reachable from `cell` with one valid N/S/E/W action."""
        return self._succ.get(cell, ())

    def predecessors(self, cell: Cell) -> tuple[Cell, ...]:
        """States that can reach `cell` in one action.

        N/S/E/W moves are reversible with equal cost, so Pred(s) == Succ(s).
        """
        return self.successors(cell)


def empty_grid(width: int, height: int, start: Cell, goal: Cell,
               obstacles: tuple[Cell, ...] = ()) -> GridMap:
    """Hand-built grid for tests and tiny examples."""
    occ = np.zeros((height, width), dtype=bool)
    for x, y in obstacles:
        occ[y, x] = True
    return GridMap(occ, start, goal)


def generate_map(seed: int, start: Cell = DEFAULT_START, goal: Cell = DEFAULT_GOAL,
                 size: int = GRID_SIZE, num_obstacles: int = NUM_OBSTACLES) -> GridMap:
    """Place exactly `num_obstacles` distinct obstacles uniformly at random.

    Start and goal are excluded from the candidate set, so they can never be
    overwritten. The map is NOT guaranteed to be connected.
    """
    occ = np.zeros((size, size), dtype=bool)
    blocked = {start[1] * size + start[0], goal[1] * size + goal[0]}
    candidates = np.array([i for i in range(size * size) if i not in blocked])
    if num_obstacles > len(candidates):
        raise ValueError("too many obstacles for grid size")
    rng = np.random.default_rng(seed)
    chosen = rng.choice(candidates, size=num_obstacles, replace=False)
    occ.flat[chosen] = True
    return GridMap(occ, start, goal, seed=seed)


def connected_to_goal(grid: GridMap) -> set[Cell]:
    """Plain flood fill used only as a map-acceptance check (no costs)."""
    seen = {grid.goal}
    frontier = deque([grid.goal])
    while frontier:
        for nxt in grid.predecessors(frontier.popleft()):
            if nxt not in seen:
                seen.add(nxt)
                frontier.append(nxt)
    return seen


def generate_reachable_map(seed: int, start: Cell = DEFAULT_START,
                           goal: Cell = DEFAULT_GOAL,
                           max_attempts: int = MAX_MAP_ATTEMPTS) -> tuple[GridMap, list[int]]:
    """Documented rejection rule: try seed, seed+1, ... until start and goal
    are connected. Returns the accepted map (its `.seed` is the seed actually
    used) and the list of rejected seeds.
    """
    rejected: list[int] = []
    for s in range(seed, seed + max_attempts):
        grid = generate_map(s, start, goal)
        if start in connected_to_goal(grid):
            return grid, rejected
        rejected.append(s)
    raise RuntimeError(f"no connected map in seeds {seed}..{seed + max_attempts - 1}")

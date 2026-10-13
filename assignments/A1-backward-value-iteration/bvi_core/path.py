"""Optimal path recovery from a converged cost-to-go map, plus validation."""
from __future__ import annotations

import math

import numpy as np

from .bvi import bellman_backup
from .grid import Cell, GridMap


class NoPathError(RuntimeError):
    """Start is not connected to goal (J(start) = inf)."""


def extract_path(grid: GridMap, values: np.ndarray,
                 policy: dict[Cell, Cell] | None = None) -> list[Cell]:
    """Follow pi(s) (or argmin_{s'} [1 + J(s')] when no policy) from start."""
    start, goal = grid.start, grid.goal
    j_start = values[start[1], start[0]]
    if not math.isfinite(j_start):
        raise NoPathError(f"goal {goal} unreachable from start {start}")

    path = [start]
    current = start
    while current != goal:
        if policy is not None:
            nxt = policy.get(current)
        else:
            _, nxt, _ = bellman_backup(grid, values, current)
        if nxt is None:
            raise NoPathError(f"no successor from {current}")
        path.append(nxt)
        current = nxt
        if len(path) > j_start + 1:
            raise RuntimeError("path longer than J(start): value map inconsistent")
    return path


def validate_path(grid: GridMap, values: np.ndarray, path: list[Cell]) -> None:
    """Raise ValueError if any optimality/legality condition fails."""
    def J(c: Cell) -> float:
        return float(values[c[1], c[0]])

    checks = [
        (len(path) > 0, "path is empty"),
        (path and path[0] == grid.start, "path does not begin at start"),
        (path and path[-1] == grid.goal, "path does not end at goal"),
        (all(grid.is_free(c) for c in path), "path enters an obstacle or leaves the grid"),
        (all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1 for a, b in zip(path, path[1:])),
         "consecutive cells are not a single N/S/E/W move"),
        (all(J(a) - J(b) == 1 for a, b in zip(path, path[1:])),
         "J does not decrease by exactly 1 per step"),
        (path and len(path) - 1 == J(grid.start), "len(path) - 1 != J(start)"),
    ]
    failed = [msg for ok, msg in checks if not ok]
    if failed:
        raise ValueError("invalid path: " + "; ".join(failed))

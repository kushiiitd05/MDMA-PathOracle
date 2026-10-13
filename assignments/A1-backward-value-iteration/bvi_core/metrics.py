"""Path/map difficulty metrics."""
from __future__ import annotations

from .grid import Cell


def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def count_turns(path: list[Cell]) -> int:
    """Number of heading changes along the path."""
    moves = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])]
    return sum(1 for m1, m2 in zip(moves, moves[1:]) if m1 != m2)


def path_stretch(d_opt: float, start: Cell, goal: Cell) -> float:
    """d_opt / d_manhattan  (1.0 = no detour needed)."""
    d_man = manhattan(start, goal)
    return 1.0 if d_man == 0 else d_opt / d_man

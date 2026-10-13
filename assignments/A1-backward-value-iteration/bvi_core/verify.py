"""Algorithm-independent optimality certificate for a cost-to-go map + path.

Given only the grid, a value map J and a path, it proves optimality:

  (C1) J(goal) = 0 and every obstacle has J = inf.
  (C2) Bellman residual is zero: J(s) = min_{s'} [1 + J(s')] for every free
       non-goal state (J is a fixed point of the Bellman operator).
  (C3) Reachability: J(s) < inf  <=>  s is connected to the goal
       (checked against a cost-free flood fill).
  (C4) 1-Lipschitz / dual feasibility: |J(u) - J(v)| <= 1 for every pair of
       adjacent reachable cells, and J(s) >= Manhattan(s, goal).
       => along ANY legal path J drops by at most 1 per move, so every path
          from s to goal has length >= J(s)   (lower bound, weak duality).
  (C5) The path is legal and has exactly J(start) moves  (bound achieved).

  C4 + C5  =>  the path is optimal.
  C1 + C2 + C3 => J is THE optimal cost-to-go: on a finite graph with
  positive edge costs the Bellman equation has a unique solution that is
  finite on the reachable set and 0 at the goal.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .grid import STEP_COST, Cell, GridMap, connected_to_goal
from .metrics import manhattan


@dataclass(frozen=True)
class Check:
    name: str
    passed: bool
    detail: str


@dataclass(frozen=True)
class Certificate:
    checks: tuple[Check, ...]

    @property
    def passed(self) -> bool:
        return all(c.passed for c in self.checks)

    def summary(self) -> str:
        ok = sum(c.passed for c in self.checks)
        return f"OPTIMALITY CERTIFICATE: {'PASS' if self.passed else 'FAIL'} ({ok}/{len(self.checks)})"

    def lines(self) -> list[str]:
        return [self.summary()] + [
            f"  [{'ok' if c.passed else 'XX'}] {c.name}: {c.detail}" for c in self.checks
        ]


def _J(values: np.ndarray, c: Cell) -> float:
    return float(values[c[1], c[0]])


def certify(grid: GridMap, values: np.ndarray, path: list[Cell] | None) -> Certificate:
    free = grid.free_cells()
    checks = []

    # C1 boundary conditions
    obstacles_inf = bool(np.all(np.isinf(values[grid.occupied])))
    checks.append(Check("C1 boundary", _J(values, grid.goal) == 0 and obstacles_inf,
                        f"J(goal)={_J(values, grid.goal):g}, obstacles all inf={obstacles_inf}"))

    # C2 Bellman residual
    worst, worst_cell = 0.0, None
    for s in free:
        if s == grid.goal:
            continue
        rhs = min((STEP_COST + _J(values, sp) for sp in grid.successors(s)), default=math.inf)
        lhs = _J(values, s)
        residual = 0.0 if lhs == rhs else math.inf if math.isinf(lhs) or math.isinf(rhs) \
            else abs(lhs - rhs)
        if residual > worst:
            worst, worst_cell = residual, s
    checks.append(Check("C2 Bellman fixed point", worst == 0,
                        f"max |J - min(1+J')| = {worst:g} over {len(free) - 1} states"
                        + (f" (worst {worst_cell})" if worst_cell else "")))

    # C3 reachability
    connected = connected_to_goal(grid)
    bad = [s for s in free if math.isfinite(_J(values, s)) != (s in connected)]
    checks.append(Check("C3 finite <=> reachable", not bad,
                        f"{len(connected)} reachable, {len(free) - len(connected)} unreachable free"
                        + (f", {len(bad)} mismatches e.g. {bad[:3]}" if bad else "")))

    # C4 Lipschitz + Manhattan lower bound
    lip_bad = [(u, v) for u in connected for v in grid.successors(u)
               if abs(_J(values, u) - _J(values, v)) > STEP_COST]
    man_bad = [s for s in connected if _J(values, s) < manhattan(s, grid.goal)]
    checks.append(Check("C4 lower bound", not lip_bad and not man_bad,
                        f"|J(u)-J(v)|<=1 on all edges: {not lip_bad}; "
                        f"J>=Manhattan: {not man_bad}"))

    # C5 path achieves the bound
    if path is None:
        ok = math.isinf(_J(values, grid.start))
        checks.append(Check("C5 path", ok, "no path and J(start)=inf" if ok
                            else "path missing although J(start) finite"))
    else:
        legal = (bool(path) and path[0] == grid.start and path[-1] == grid.goal
                 and all(grid.is_free(c) for c in path)
                 and all(manhattan(a, b) == 1 for a, b in zip(path, path[1:])))
        moves = len(path) - 1
        tight = legal and moves == _J(values, grid.start)
        checks.append(Check("C5 path optimal", tight,
                            f"legal={legal}, moves={moves}, J(start)={_J(values, grid.start):g}"))

    return Certificate(tuple(checks))

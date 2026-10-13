"""Backward Value Iteration for the unit-cost N/S/E/W grid.

Bellman cost-to-go equation (lecture notation, c(s, s') = 1):

    J(goal) = 0
    J(s)    = min_{s' in Succ(s)} [ 1 + J(s') ]     for free s != goal
    J(s)    = inf                                   obstacle / unreachable

Two update schedules of the SAME equation are provided:

A. synchronous_bvi  -- ordinary BVI: full Jacobi sweeps, J_{i+1} from J_i,
                       until one full sweep changes nothing.
B. queue_bvi        -- asynchronous BVI (ABVI): only states whose successor
                       just changed are backed up; new values are used
                       immediately; FIFO queue; stops when queue is empty.

No heuristic, no priority heap, no library shortest-path call.
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from .grid import ACTIONS, STEP_COST, Cell, GridMap

INF = math.inf


@dataclass(frozen=True)
class BVIResult:
    algorithm: str
    values: np.ndarray            # float, [y, x]; inf = obstacle/unreachable
    policy: dict[Cell, Cell]      # state -> best successor
    stats: dict[str, float]
    # Convergence trace: (cumulative state updates, #states with finite J),
    # sampled after every sweep (sync) or every queue pop (queue).
    trace: tuple[tuple[int, int], ...] = ()

    def J(self, cell: Cell) -> float:
        return float(self.values[cell[1], cell[0]])


def _initial_values(grid: GridMap) -> np.ndarray:
    values = np.full((grid.height, grid.width), INF)
    values[grid.goal[1], grid.goal[0]] = 0.0
    return values


def bellman_backup(grid: GridMap, values: np.ndarray, s: Cell) -> tuple[float, Cell | None, int]:
    """min_{s' in Succ(s)} [1 + J(s')].

    Returns (value, argmin successor, number of successors evaluated).
    Ties are broken by ACTIONS order (N, S, E, W) because only a strictly
    smaller candidate replaces the current best.
    """
    best, best_succ = INF, None
    succ = grid.successors(s)
    for sp in succ:
        candidate = STEP_COST + values[sp[1], sp[0]]
        if candidate < best:
            best, best_succ = candidate, sp
    return best, best_succ, len(succ)


def greedy_policy(grid: GridMap, values: np.ndarray) -> dict[Cell, Cell]:
    """pi(s) = argmin_{s'} [1 + J(s')] for every reachable non-goal state."""
    policy = {}
    for s in grid.free_cells():
        if s != grid.goal and math.isfinite(values[s[1], s[0]]):
            _, best_succ, _ = bellman_backup(grid, values, s)
            policy[s] = best_succ
    return policy


def synchronous_bvi(grid: GridMap) -> BVIResult:
    """A. Reference (ordinary, synchronous) BVI with full sweeps."""
    values = _initial_values(grid)
    states = [s for s in grid.free_cells() if s != grid.goal]
    # A shortest path visits each state at most once, so values settle after
    # at most |states| sweeps; +1 for the final "no change" sweep.
    max_sweeps = len(states) + 1
    sweeps = attempts = evaluations = changes = 0
    trace = []

    t0 = time.perf_counter()
    while True:
        sweeps += 1
        previous = values.copy()          # J_i, read-only during this sweep
        changed = False
        for s in states:
            attempts += 1
            new_value, _, n = bellman_backup(grid, previous, s)
            evaluations += n
            if new_value != previous[s[1], s[0]]:
                values[s[1], s[0]] = new_value   # written into J_{i+1}
                changes += 1
                changed = True
        trace.append((attempts, changes + 1))   # +1: the goal
        if not changed:
            break
        if sweeps >= max_sweeps:
            raise RuntimeError("synchronous BVI did not converge")
    runtime = time.perf_counter() - t0

    return BVIResult(
        algorithm="synchronous",
        values=values,
        policy=greedy_policy(grid, values),
        stats={
            "runtime_s": runtime,
            "sweeps": sweeps,
            "state_updates": attempts,
            "successor_evaluations": evaluations,
            "value_changes": changes,
        },
        trace=tuple(trace),
    )


def queue_bvi(grid: GridMap, full_backup: bool = True) -> BVIResult:
    """B. Optimized asynchronous (queue / FIFO wavefront) BVI.

    Pred(y) says WHO may need updating after J(y) changed;
    Succ(p) says HOW to compute p's value (Bellman backup).

    full_backup=True : J(p) <- min_{s' in Succ(p)} [1 + J(s')]   (lecture ABVI)
    full_backup=False: J(p) <- min(J(p), 1 + J(y))               (edge relaxation)

    Both give identical J on this graph. FIFO is used because every edge
    costs 1: states leave the queue in non-decreasing J order.
    """
    values = _initial_values(grid)
    policy: dict[Cell, Cell] = {}
    queue = deque([grid.goal])
    pops = backups = evaluations = changes = peak_queue = 0
    trace = []

    t0 = time.perf_counter()
    while queue:
        peak_queue = max(peak_queue, len(queue))
        y = queue.popleft()
        pops += 1
        for p in grid.predecessors(y):
            if p == grid.goal:
                continue
            backups += 1
            if full_backup:
                candidate, best_succ, n = bellman_backup(grid, values, p)
                evaluations += n
            else:
                candidate, best_succ = STEP_COST + values[y[1], y[0]], y
                evaluations += 1
            if candidate < values[p[1], p[0]]:
                values[p[1], p[0]] = candidate
                policy[p] = best_succ
                changes += 1
                queue.append(p)
        trace.append((backups, changes + 1))
    runtime = time.perf_counter() - t0

    return BVIResult(
        algorithm="queue" if full_backup else "queue-relax",
        values=values,
        policy=policy,
        stats={
            "runtime_s": runtime,
            "queue_pops": pops,
            "peak_queue": peak_queue,
            "state_updates": backups,
            "successor_evaluations": evaluations,
            "value_changes": changes,
        },
        trace=tuple(trace),
    )


ALGORITHMS = {
    "sync": synchronous_bvi,
    "queue": queue_bvi,
    "queue-relax": lambda grid: queue_bvi(grid, full_backup=False),
}


def run_bvi(grid: GridMap, algorithm: str = "queue") -> BVIResult:
    if algorithm not in ALGORITHMS:
        raise ValueError(f"unknown algorithm {algorithm!r}; choose from {sorted(ALGORITHMS)}")
    return ALGORITHMS[algorithm](grid)


def value_mismatches(a: np.ndarray, b: np.ndarray) -> list[Cell]:
    """Cells whose cost-to-go differs (inf == inf counts as equal)."""
    ys, xs = np.nonzero(~((a == b) | (np.isinf(a) & np.isinf(b))))
    return [(int(x), int(y)) for y, x in zip(ys, xs)]


def run_and_crosscheck(grid: GridMap) -> tuple[BVIResult, BVIResult]:
    """Run both schedules and raise if their J maps differ anywhere."""
    sync, queue = synchronous_bvi(grid), queue_bvi(grid)
    diff = value_mismatches(sync.values, queue.values)
    if diff:
        raise AssertionError(f"BVI mismatch at {len(diff)} cells, e.g. {diff[:5]}")
    return sync, queue

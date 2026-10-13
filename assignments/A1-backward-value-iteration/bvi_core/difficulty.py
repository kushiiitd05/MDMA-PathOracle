"""Easy / medium / hard map selection by path stretch quantiles.

Protocol (fully reproducible, no hand-picking):
  1. Candidate seeds POOL_BASE_SEED .. POOL_BASE_SEED + POOL_SIZE - 1.
  2. Each seed -> one map with exactly 180 obstacles (20%). Maps where start
     and goal are disconnected are rejected and counted (never re-drawn).
  3. For each accepted map: d_opt = J(start) from queue BVI,
     stretch = d_opt / manhattan(start, goal), turns of the selected path.
  4. Sort by (stretch, turns, seed); pick rank round(q * (n - 1)) for
     q = 0.20 (easy), 0.50 (medium), 0.90 (hard).
Difficulty = navigation difficulty of the start->goal path, not BVI cost:
BVI always computes J for the whole reachable state space.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

from .bvi import queue_bvi
from .grid import DEFAULT_GOAL, DEFAULT_START, Cell, GridMap, generate_map
from .metrics import count_turns, manhattan, path_stretch
from .path import NoPathError, extract_path

POOL_BASE_SEED = 0
POOL_SIZE = 500
QUANTILES = {"easy": 0.20, "medium": 0.50, "hard": 0.90}


@dataclass(frozen=True)
class MapStats:
    seed: int
    d_opt: int
    d_manhattan: int
    stretch: float
    turns: int


def evaluate_map(grid: GridMap) -> MapStats | None:
    """Stats for one map, or None if start and goal are disconnected."""
    res = queue_bvi(grid)
    try:
        path = extract_path(grid, res.values, res.policy)
    except NoPathError:
        return None
    d_opt = int(res.J(grid.start))
    return MapStats(
        seed=grid.seed,
        d_opt=d_opt,
        d_manhattan=manhattan(grid.start, grid.goal),
        stretch=path_stretch(d_opt, grid.start, grid.goal),
        turns=count_turns(path),
    )


def build_pool(size: int = POOL_SIZE, base_seed: int = POOL_BASE_SEED,
               start: Cell = DEFAULT_START, goal: Cell = DEFAULT_GOAL
               ) -> tuple[list[MapStats], list[int]]:
    accepted, rejected = [], []
    for seed in range(base_seed, base_seed + size):
        stats = evaluate_map(generate_map(seed, start, goal))
        if stats is None:
            rejected.append(seed)
        else:
            accepted.append(stats)
    return accepted, rejected


def select_by_quantile(pool: list[MapStats],
                       quantiles: dict[str, float] = QUANTILES) -> dict[str, MapStats]:
    if not pool:
        raise ValueError("empty map pool")
    ranked = sorted(pool, key=lambda m: (m.stretch, m.turns, m.seed))
    last = len(ranked) - 1
    return {name: ranked[round(q * last)] for name, q in quantiles.items()}


def select_difficulty_seeds(start: Cell = DEFAULT_START, goal: Cell = DEFAULT_GOAL,
                            size: int = POOL_SIZE, base_seed: int = POOL_BASE_SEED) -> dict:
    """Run the full protocol and return a JSON-serialisable report."""
    pool, rejected = build_pool(size, base_seed, start, goal)
    chosen = select_by_quantile(pool)
    stretches = sorted(m.stretch for m in pool)
    return {
        "protocol": {
            "pool_seeds": [base_seed, base_seed + size - 1],
            "start": list(start),
            "goal": list(goal),
            "obstacles": 180,
            "grid": [30, 30],
            "quantiles": QUANTILES,
            "rank_key": "(stretch, turns, seed)",
        },
        "accepted": len(pool),
        "rejected_disconnected": len(rejected),
        "stretch_min_median_max": [stretches[0], stretches[len(stretches) // 2], stretches[-1]],
        "selected": {name: asdict(m) for name, m in chosen.items()},
    }


def load_or_select(cache: Path, start: Cell = DEFAULT_START,
                   goal: Cell = DEFAULT_GOAL) -> dict:
    """Use cached report if it matches start/goal, else recompute and save."""
    if cache.exists():
        report = json.loads(cache.read_text())
        proto = report.get("protocol", {})
        if proto.get("start") == list(start) and proto.get("goal") == list(goal):
            return report
    report = select_difficulty_seeds(start, goal)
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(json.dumps(report, indent=2))
    return report

#!/usr/bin/env python3
"""Collect every number used in report/build_report.py -> results/report_data.json.

    python standalone/report_data.py [--repeats 5] [--scale-seeds 5]

Sections written:
  maps     easy/medium/hard (seeds from results/difficulty_seeds.json): both algorithms,
           cost (updates, evaluations, sweeps/pops, runtime) and space (tracemalloc peak,
           peak queue length, stored J cells), plus the convergence trace and the J array.
  scaling  square grids of side 10..80 (20 % obstacles, corner-to-corner), mean over seeds.
No result is typed by hand: the report reads this file.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import tracemalloc
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bvi_core.bvi import queue_bvi, synchronous_bvi, value_mismatches  # noqa: E402
from bvi_core.difficulty import QUANTILES, load_or_select  # noqa: E402
from bvi_core.grid import connected_to_goal, generate_map, generate_reachable_map  # noqa: E402
from bvi_core.metrics import count_turns, manhattan  # noqa: E402
from bvi_core.path import extract_path  # noqa: E402
from bvi_core.verify import certify  # noqa: E402

RESULTS = ROOT / "results"
SIDES = (10, 20, 30, 40, 60, 80)
ALGOS = {"sync": synchronous_bvi, "queue": queue_bvi}


def measure(fn, grid, repeats: int) -> dict:
    """Best-of-`repeats` runtime, plus the peak Python heap allocated by one run."""
    runs = [fn(grid) for _ in range(repeats)]
    tracemalloc.start()
    fn(grid)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    res = runs[0]
    stats = dict(res.stats)
    stats["runtime_s"] = min(r.stats["runtime_s"] for r in runs)
    stats["peak_heap_kib"] = peak / 1024
    return {"result": res, "stats": stats}


def collect_maps(repeats: int) -> dict:
    report = load_or_select(RESULTS / "difficulty_seeds.json")
    out = {}
    for level in QUANTILES:
        sel = report["selected"][level]
        grid, _ = generate_reachable_map(sel["seed"])
        runs = {name: measure(fn, grid, repeats) for name, fn in ALGOS.items()}
        if value_mismatches(runs["sync"]["result"].values, runs["queue"]["result"].values):
            raise AssertionError(f"{level}: J maps differ")
        res_q = runs["queue"]["result"]
        path = extract_path(grid, res_q.values, res_q.policy)
        reach = int((res_q.values < float("inf")).sum())
        entry = {
            "seed": grid.seed,
            "obstacles": grid.num_obstacles,
            "free_states": len(grid.free_cells()),
            "reachable": reach,
            "unreachable_free": len(grid.free_cells()) - reach,
            "J_start": int(res_q.J(grid.start)),
            "manhattan": manhattan(grid.start, grid.goal),
            "stretch": res_q.J(grid.start) / manhattan(grid.start, grid.goal),
            "turns": count_turns(path),
            "path": [list(c) for c in path],
            "occupied": grid.occupied.astype(int).tolist(),
            "values": [[None if v == float("inf") else int(v) for v in row] for row in res_q.values.tolist()],
            "start": list(grid.start),
            "goal": list(grid.goal),
        }
        for name, m in runs.items():
            res = m["result"]
            cert = certify(grid, res.values, extract_path(grid, res.values, res.policy))
            entry[name] = {**m["stats"], "certificate": cert.passed, "trace": [list(t) for t in res.trace]}
        out[level] = entry
    return out


def collect_scaling(seeds: int) -> list[dict]:
    rows = []
    for side in SIDES:
        start, goal = (1, 1), (side - 2, side - 2)
        obstacles = round(0.2 * side * side)
        acc = {name: {"updates": [], "evals": [], "ms": [], "heap": [], "iters": [], "peak_queue": []} for name in ALGOS}
        used, seed = 0, 1000 * side
        while used < seeds:
            grid = generate_map(seed, start, goal, size=side, num_obstacles=obstacles)
            seed += 1
            if start not in connected_to_goal(grid):
                continue
            used += 1
            for name, fn in ALGOS.items():
                m = measure(fn, grid, 1)["stats"]
                a = acc[name]
                a["updates"].append(m["state_updates"])
                a["evals"].append(m["successor_evaluations"])
                a["ms"].append(m["runtime_s"] * 1e3)
                a["heap"].append(m["peak_heap_kib"])
                a["iters"].append(m.get("sweeps", m.get("queue_pops")))
                if "peak_queue" in m:
                    a["peak_queue"].append(m["peak_queue"])
        row = {"side": side, "states": side * side - obstacles, "maps": used}
        for name, a in acc.items():
            row[name] = {k: statistics.fmean(v) for k, v in a.items() if v}
        rows.append(row)
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--scale-seeds", type=int, default=5)
    args = ap.parse_args()
    data = {"maps": collect_maps(args.repeats), "scaling": collect_scaling(args.scale_seeds)}
    (RESULTS / "report_data.json").write_text(json.dumps(data))
    for level, e in data["maps"].items():
        print(level, e["seed"], "J(start)", e["J_start"], "peak queue", e["queue"]["peak_queue"],
              "heap KiB sync/queue", round(e["sync"]["peak_heap_kib"], 1), round(e["queue"]["peak_heap_kib"], 1))
    for r in data["scaling"]:
        print(r["side"], r["states"], "updates", round(r["sync"]["updates"]), round(r["queue"]["updates"]),
              "ms", round(r["sync"]["ms"], 1), round(r["queue"]["ms"], 1))


if __name__ == "__main__":
    main()

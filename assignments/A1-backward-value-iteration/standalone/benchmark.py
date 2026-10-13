#!/usr/bin/env python3
"""Synchronous vs queue BVI over many reachable random maps.

    python3 benchmark.py --maps 200 --repeats 3
Writes results/benchmark.csv, results/benchmark.md, results/benchmark.png.
"""
from __future__ import annotations

import argparse
import csv
import statistics as st
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.ticker  # noqa: E402

from bvi_core.bvi import queue_bvi, synchronous_bvi, value_mismatches  # noqa: E402
from bvi_core.grid import connected_to_goal, generate_map  # noqa: E402
from bvi_core.path import extract_path, validate_path  # noqa: E402
from bvi_core.verify import certify  # noqa: E402

RESULTS = ROOT / "results"
ALGOS = {
    "sync": synchronous_bvi,
    "queue": queue_bvi,
    "queue-relax": lambda g: queue_bvi(g, full_backup=False),
}
SYNC_COLOR, QUEUE_COLOR, RELAX_COLOR = "#1c7ed6", "#e8590c", "#868e96"


def best_of(fn, grid, repeats):
    """Run `repeats` times; keep counts from the first, min runtime overall."""
    runs = [fn(grid) for _ in range(repeats)]
    best = min(r.stats["runtime_s"] for r in runs)
    return runs[0], best


def run(maps: int, base_seed: int, repeats: int) -> tuple[list[dict], int]:
    rows, rejected, seed = [], 0, base_seed
    while len(rows) < maps:
        grid = generate_map(seed)
        seed += 1
        if grid.start not in connected_to_goal(grid):
            rejected += 1
            continue
        row = {"seed": grid.seed}
        results = {}
        for name, fn in ALGOS.items():
            res, runtime = best_of(fn, grid, repeats)
            results[name] = res
            row[f"{name}_runtime_ms"] = runtime * 1e3
            row[f"{name}_state_updates"] = int(res.stats["state_updates"])
            row[f"{name}_succ_evals"] = int(res.stats["successor_evaluations"])
            row[f"{name}_value_changes"] = int(res.stats["value_changes"])
        row["sync_sweeps"] = int(results["sync"].stats["sweeps"])
        row["queue_pops"] = int(results["queue"].stats["queue_pops"])
        for name in ("queue", "queue-relax"):
            if value_mismatches(results["sync"].values, results[name].values):
                raise AssertionError(f"seed {grid.seed}: sync and {name} J differ")
        path = extract_path(grid, results["queue"].values, results["queue"].policy)
        validate_path(grid, results["queue"].values, path)
        if not certify(grid, results["queue"].values, path).passed:
            raise AssertionError(f"seed {grid.seed}: optimality certificate failed")
        row["path_length"] = len(path) - 1
        rows.append(row)
    return rows, rejected


def summarize(rows: list[dict], rejected: int, repeats: int) -> str:
    def col(k):
        return [r[k] for r in rows]

    def mm(k, fmt="{:.0f}"):
        v = col(k)
        return f"{fmt.format(st.mean(v))} / {fmt.format(st.median(v))}"

    n = len(rows)
    su, qu = st.mean(col("sync_state_updates")), st.mean(col("queue_state_updates"))
    se, qe = st.mean(col("sync_succ_evals")), st.mean(col("queue_succ_evals"))
    lines = [
        f"# BVI benchmark ({n} reachable 30x30 maps, 180 obstacles each)",
        "",
        f"Seeds {rows[0]['seed']}..{rows[-1]['seed']}; {rejected} disconnected maps skipped. "
        f"Runtime = best of {repeats} runs (CPython, single thread). "
        "All maps: J_sync == J_queue == J_queue-relax on every cell; path validated; "
        "optimality certificate (bvi_core/verify.py) passed.",
        "",
        "| metric (mean / median) | synchronous BVI | queue BVI (Bellman backup) | queue BVI (edge relax) |",
        "|---|---|---|---|",
        f"| runtime [ms] | {mm('sync_runtime_ms', '{:.2f}')} | {mm('queue_runtime_ms', '{:.2f}')} "
        f"| {mm('queue-relax_runtime_ms', '{:.2f}')} |",
        f"| state updates attempted | {mm('sync_state_updates')} | {mm('queue_state_updates')} "
        f"| {mm('queue-relax_state_updates')} |",
        f"| successor evaluations | {mm('sync_succ_evals')} | {mm('queue_succ_evals')} "
        f"| {mm('queue-relax_succ_evals')} |",
        f"| successful value changes | {mm('sync_value_changes')} | {mm('queue_value_changes')} "
        f"| {mm('queue-relax_value_changes')} |",
        f"| sweeps / queue pops | {mm('sync_sweeps', '{:.1f}')} sweeps | {mm('queue_pops')} pops | same pops |",
        f"| optimal path length | {mm('path_length', '{:.1f}')} | same | same |",
        "",
        f"Update-count ratio sync/queue: **{su / qu:.1f}x** state updates, "
        f"**{se / qe:.1f}x** successor evaluations.",
        "",
        "Timings are sub-40 ms on a 900-cell grid and include Python overhead; the",
        "update counts are the hardware-independent comparison.",
    ]
    return "\n".join(lines) + "\n"


def plot(rows: list[dict], out: Path) -> None:
    """Median bar + min-max whisker per algorithm (spread is tiny on these maps)."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), facecolor="white")
    specs = [
        ("state_updates", "Bellman state updates attempted", "{:,.0f}"),
        ("succ_evals", "successor evaluations 1 + J(s')", "{:,.0f}"),
        ("runtime_ms", "runtime [ms], best of repeats", "{:.2f}"),
    ]
    names = [("sync", "synchronous", SYNC_COLOR), ("queue", "queue (backup)", QUEUE_COLOR),
             ("queue-relax", "queue (relax)", RELAX_COLOR)]
    for ax, (key, title, fmt) in zip(axes, specs):
        for i, (name, _, color) in enumerate(names):
            d = [r[f"{name}_{key}"] for r in rows]
            med = st.median(d)
            ax.bar(i, med, 0.6, color=color)
            ax.vlines(i, min(d), max(d), color="#1f2328", linewidth=1.2)
            ax.annotate(fmt.format(med), (i, max(d)), xytext=(0, 4),
                        textcoords="offset points", ha="center", fontsize=9, color="#1f2328")
        ax.set_yscale("log")
        ax.set_ylim(top=max(r[f"sync_{key}"] for r in rows) * 3)
        ax.yaxis.set_major_locator(matplotlib.ticker.LogLocator(subs=(1, 2, 5)))
        ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
        ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        ax.set_xticks(range(len(names)), [n for _, n, _ in names], fontsize=9)
        ax.set_title(title, fontsize=11, loc="left", color="#1f2328")
        ax.grid(axis="y", color="#e9ecef", linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.tick_params(colors="#57606a", labelsize=8)
    fig.suptitle(f"Synchronous vs queue BVI on {len(rows)} random 30x30 maps "
                 "(bar = median, whisker = min-max, log scale)",
                 x=0.01, ha="left", fontsize=13, fontweight="bold", color="#1f2328")
    fig.tight_layout()
    fig.savefig(out, dpi=150)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--maps", type=int, default=200)
    p.add_argument("--base-seed", type=int, default=10_000)
    p.add_argument("--repeats", type=int, default=3)
    args = p.parse_args(argv)
    if args.maps < 1 or args.repeats < 1:
        p.error("--maps and --repeats must be >= 1")

    rows, rejected = run(args.maps, args.base_seed, args.repeats)
    RESULTS.mkdir(exist_ok=True)
    with open(RESULTS / "benchmark.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = summarize(rows, rejected, args.repeats)
    (RESULTS / "benchmark.md").write_text(report)
    plot(rows, RESULTS / "benchmark.png")
    print(report)
    print("wrote results/benchmark.csv, results/benchmark.md, results/benchmark.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())

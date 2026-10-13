#!/usr/bin/env python3
"""6-run experiment: {easy, medium, hard} x {synchronous BVI, queue ABVI}.

    python3 tradeoff.py [--seconds-per-move 0.4] [--repeats 5]
Writes results/tradeoff.md and results/tradeoff.png.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.ticker  # noqa: E402

from bvi_core.bvi import queue_bvi, synchronous_bvi, value_mismatches  # noqa: E402
from bvi_core.difficulty import QUANTILES, load_or_select  # noqa: E402
from bvi_core.grid import generate_reachable_map  # noqa: E402
from bvi_core.metrics import count_turns  # noqa: E402
from bvi_core.path import extract_path  # noqa: E402
from bvi_core.verify import certify  # noqa: E402

RESULTS = ROOT / "results"
ALGOS = {"BVI (sync)": (synchronous_bvi, "#1c7ed6"), "ABVI (queue)": (queue_bvi, "#e8590c")}
STYLES = {"easy": "-", "medium": "--", "hard": ":"}
INK, MUTED = "#1f2328", "#57606a"


def run(repeats: int, sec_per_move: float) -> list[dict]:
    report = load_or_select(RESULTS / "difficulty_seeds.json")
    rows = []
    for level in QUANTILES:
        grid, _ = generate_reachable_map(report["selected"][level]["seed"])
        results = {}
        for name, (fn, _) in ALGOS.items():
            runs = [fn(grid) for _ in range(repeats)]
            res = runs[0]
            results[name] = res
            path = extract_path(grid, res.values, res.policy)
            cert = certify(grid, res.values, path)
            s = res.stats
            rows.append({
                "difficulty": level, "seed": grid.seed, "algorithm": name,
                "iterations": f"{s['sweeps']} sweeps" if "sweeps" in s else f"{s['queue_pops']} pops",
                "state_updates": int(s["state_updates"]),
                "succ_evals": int(s["successor_evaluations"]),
                "value_changes": int(s["value_changes"]),
                "runtime_ms": min(r.stats["runtime_s"] for r in runs) * 1e3,
                "J_start": int(res.J(grid.start)),
                "turns": count_turns(path),
                "travel_s": (len(path) - 1) * sec_per_move,
                "certificate": "PASS" if cert.passed else "FAIL",
                "trace": res.trace,
            })
        a, b = results.values()
        if value_mismatches(a.values, b.values):
            raise AssertionError(f"{level}: J maps differ")
    return rows


def write_table(rows: list[dict], sec_per_move: float, out: Path) -> str:
    head = ("| map | seed | algorithm | sweeps / pops | state updates | successor evals "
            "| value changes | runtime [ms] | J(start) | turns | travel time [s] | certificate |")
    lines = ["# Easy / medium / hard x BVI / ABVI", "",
             f"Travel time = path cost x {sec_per_move} s per move (robot animation rate).",
             "Both algorithms give identical J on every cell of every map.", "",
             head, "|" + "---|" * 12]
    for r in rows:
        lines.append(
            f"| {r['difficulty']} | {r['seed']} | {r['algorithm']} | {r['iterations']} "
            f"| {r['state_updates']:,} | {r['succ_evals']:,} | {r['value_changes']} "
            f"| {r['runtime_ms']:.2f} | {r['J_start']} | {r['turns']} | {r['travel_s']:.1f} "
            f"| {r['certificate']} |")
    text = "\n".join(lines) + "\n"
    out.write_text(text)
    return text


def plot(rows: list[dict], out: Path) -> None:
    fig = plt.figure(figsize=(15, 9.5), facecolor="white")
    gs = fig.add_gridspec(2, 2, height_ratios=[1.15, 1], hspace=0.38, wspace=0.22)

    # (a) convergence: how much work until all reachable states have their value
    ax = fig.add_subplot(gs[0, :])
    for r in rows:
        color = ALGOS[r["algorithm"]][1]
        xs = [0] + [u for u, _ in r["trace"]]
        ys = [1] + [n for _, n in r["trace"]]
        ax.step(xs, ys, where="post", color=color, linestyle=STYLES[r["difficulty"]],
                linewidth=2, label=f"{r['algorithm']} - {r['difficulty']}")
    ax.set_xscale("symlog", linthresh=100)
    ax.set_xlabel("cumulative Bellman state updates (symlog)", color=MUTED)
    ax.set_ylabel("states with final finite J", color=MUTED)
    ax.set_title("(a) Convergence: value information spreading from the goal",
                 loc="left", color=INK, fontsize=12)
    ax.legend(ncol=2, frameon=False, fontsize=9, loc="upper left")
    _style(ax)

    # (b) work and (c) time, grouped by difficulty
    levels = list(QUANTILES)
    for pos, key, title, fmt in ((gs[1, 0], "state_updates", "(b) Bellman state updates (log)", "{:,.0f}"),
                                 (gs[1, 1], "runtime_ms", "(c) runtime [ms], best of repeats (log)", "{:.2f}")):
        ax = fig.add_subplot(pos)
        width = 0.36
        for k, (name, (_, color)) in enumerate(ALGOS.items()):
            vals = [next(r[key] for r in rows if r["difficulty"] == lv and r["algorithm"] == name)
                    for lv in levels]
            xs = [i + (k - 0.5) * (width + 0.04) for i in range(len(levels))]
            ax.bar(xs, vals, width, color=color, label=name)
            for x, v in zip(xs, vals):
                ax.annotate(fmt.format(v), (x, v), xytext=(0, 3), textcoords="offset points",
                            ha="center", fontsize=8, color=INK)
        ax.set_yscale("log")
        ax.set_xticks(range(len(levels)),
                      [f"{lv}\nseed {next(r['seed'] for r in rows if r['difficulty'] == lv)}"
                       for lv in levels])
        ax.set_title(title, loc="left", color=INK, fontsize=12)
        ax.legend(frameon=False, fontsize=9, ncol=2, loc="upper left")
        ax.set_ylim(top=max(r[key] for r in rows) * 3.5)
        ax.yaxis.set_major_locator(matplotlib.ticker.LogLocator(subs=(1, 2, 5)))
        ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
        ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        _style(ax)

    fig.suptitle("BVI vs ABVI trade-off on the easy / medium / hard maps",
                 x=0.06, ha="left", fontsize=15, fontweight="bold", color=INK)
    fig.savefig(out, dpi=150, bbox_inches="tight")


def _style(ax) -> None:
    ax.grid(axis="y", color="#e9ecef", linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.tick_params(colors=MUTED, labelsize=9)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seconds-per-move", type=float, default=0.4)
    p.add_argument("--repeats", type=int, default=5)
    args = p.parse_args(argv)
    if args.seconds_per_move <= 0 or args.repeats < 1:
        p.error("--seconds-per-move must be > 0 and --repeats >= 1")
    rows = run(args.repeats, args.seconds_per_move)
    RESULTS.mkdir(exist_ok=True)
    print(write_table(rows, args.seconds_per_move, RESULTS / "tradeoff.md"))
    plot(rows, RESULTS / "tradeoff.png")
    print("wrote results/tradeoff.md, results/tradeoff.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Standalone (no ROS) BVI demo.

    python3 run_demo.py --difficulty easy
    python3 run_demo.py --seed 42 --compare-bvi --show-policy --save
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bvi_core.bvi import run_and_crosscheck, run_bvi  # noqa: E402
from bvi_core.difficulty import QUANTILES, load_or_select  # noqa: E402
from bvi_core.grid import DEFAULT_GOAL, DEFAULT_START, generate_reachable_map  # noqa: E402
from bvi_core.metrics import count_turns, path_stretch  # noqa: E402
from bvi_core.path import NoPathError, extract_path, validate_path  # noqa: E402
from bvi_core.verify import certify  # noqa: E402

RESULTS = ROOT / "results"


def parse_cell(text: str) -> tuple[int, int]:
    try:
        x, y = (int(v) for v in text.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected X,Y got {text!r}") from exc
    if not (0 <= x < 30 and 0 <= y < 30):
        raise argparse.ArgumentTypeError(f"{text} outside 30x30 grid")
    return x, y


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    src = p.add_mutually_exclusive_group()
    src.add_argument("--difficulty", choices=list(QUANTILES), default="medium")
    src.add_argument("--seed", type=int, help="explicit map seed (overrides --difficulty)")
    p.add_argument("--start", type=parse_cell, default=DEFAULT_START, help="X,Y")
    p.add_argument("--goal", type=parse_cell, default=DEFAULT_GOAL, help="X,Y")
    p.add_argument("--algorithm", choices=["queue", "sync", "queue-relax"], default="queue")
    p.add_argument("--show-values", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--show-policy", action="store_true")
    p.add_argument("--compare-bvi", action="store_true",
                   help="run synchronous AND queue BVI, verify identical J, show both stats")
    p.add_argument("--save", action="store_true", help="write PNG to results/")
    p.add_argument("--no-show", action="store_true", help="do not open a window")
    return p.parse_args(argv)


def print_stats(result) -> None:
    body = ", ".join(f"{k}={v:.6f}" if k == "runtime_s" else f"{k}={int(v)}"
                     for k, v in result.stats.items())
    print(f"  [{result.algorithm:>11}] {body}")


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.start == args.goal:
        print("note: start == goal, path length will be 0")

    if args.seed is not None:
        label, requested = "custom", args.seed
    else:
        report = load_or_select(RESULTS / "difficulty_seeds.json", args.start, args.goal)
        label, requested = args.difficulty, report["selected"][args.difficulty]["seed"]

    grid, rejected = generate_reachable_map(requested, args.start, args.goal)
    if rejected:
        print(f"seeds {rejected} disconnected start/goal -> rejected; using seed {grid.seed}")
    print(f"map: difficulty={label} seed={grid.seed} obstacles={grid.num_obstacles} "
          f"start={grid.start} goal={grid.goal}")

    extra = ()
    if args.compare_bvi:
        sync, queue = run_and_crosscheck(grid)
        print("cross-check: synchronous J == queue J on all 900 cells  -> OK")
        by_name = {"sync": sync, "queue": queue}
        result = by_name.get(args.algorithm) or run_bvi(grid, args.algorithm)
        extra = tuple(r for r in (sync, queue) if r is not result)
        print_stats(sync)
        print_stats(queue)
    else:
        result = run_bvi(grid, args.algorithm)
        print_stats(result)

    try:
        path = extract_path(grid, result.values, result.policy)
        validate_path(grid, result.values, path)
    except NoPathError as exc:
        print(f"NO PATH: {exc}")
        path = None
    else:
        stretch = path_stretch(len(path) - 1, grid.start, grid.goal)
        print(f"path: J(start)={int(result.J(grid.start))} moves={len(path) - 1} "
              f"turns={count_turns(path)} stretch={stretch:.3f}  validation -> OK")

    certificate = certify(grid, result.values, path)
    print("\n".join(certificate.lines()))
    if not certificate.passed:
        print("ERROR: optimality certificate failed")
        return 1

    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    from plotting import render_figure
    import matplotlib.pyplot as plt

    notes = (("cross-check: J_sync == J_queue  ✓",) if args.compare_bvi else ()) + (
        certificate.summary(),)
    fig = render_figure(grid, result, path, difficulty=label, extra_results=extra,
                        show_values=args.show_values, show_policy=args.show_policy,
                        notes=notes)
    if args.save:
        RESULTS.mkdir(exist_ok=True)
        suffix = "_policy" if args.show_policy else ""
        out = RESULTS / f"bvi_{label}_seed{grid.seed}_{result.algorithm}{suffix}.png"
        fig.savefig(out, dpi=150)
        print(f"saved {out.relative_to(ROOT)}")
    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Matplotlib rendering of a BVI cost-to-go map and optimal path."""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.patches import Patch, Rectangle  # noqa: E402

from bvi_core.bvi import BVIResult  # noqa: E402
from bvi_core.grid import Cell, GridMap  # noqa: E402

COST_CMAP = plt.get_cmap("Blues")
OBSTACLE_COLOR = "#2b2d33"
UNREACHABLE_COLOR = "#d6d6d6"
PATH_COLOR = "#e8590c"
START_COLOR = "#2f9e44"
GOAL_COLOR = "#c92a2a"
INK = "#1f2328"
INK_MUTED = "#57606a"


def _text_color(norm_value: float) -> str:
    return "white" if norm_value > 0.55 else INK


def draw_cost_map(ax, grid: GridMap, result: BVIResult, path: list[Cell] | None,
                  show_values: bool = True, show_policy: bool = False) -> None:
    J = result.values
    finite = np.isfinite(J)
    j_max = float(J[finite].max()) if finite.any() else 1.0
    extent = (-0.5, grid.width - 0.5, -0.5, grid.height - 0.5)

    # Sequential layer: finite J only (clipped away from the white end).
    norm_J = np.where(finite, J / max(j_max, 1.0), np.nan)
    shown = 0.12 + 0.88 * norm_J
    ax.imshow(shown, origin="lower", cmap=COST_CMAP, vmin=0, vmax=1,
              extent=extent, interpolation="nearest")
    # Categorical layers: obstacles and unreachable free cells.
    unreachable = ~finite & ~grid.occupied
    ax.imshow(np.where(grid.occupied, 1.0, np.nan), origin="lower",
              cmap=ListedColormap([OBSTACLE_COLOR]), extent=extent, interpolation="nearest")
    for y, x in zip(*np.nonzero(unreachable)):
        ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor=UNREACHABLE_COLOR,
                               hatch="////", edgecolor="#9a9a9a", linewidth=0))

    # Cell grid lines.
    ax.set_xticks(np.arange(-0.5, grid.width, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.height, 1), minor=True)
    ax.grid(which="minor", color="white", linewidth=0.6)
    ax.tick_params(which="minor", length=0)
    ax.set_xticks(range(0, grid.width, 5))
    ax.set_yticks(range(0, grid.height, 5))
    ax.tick_params(labelsize=8, colors=INK_MUTED)
    for side in ax.spines.values():
        side.set_visible(False)

    if show_values:
        on_path = set(path or ())
        for y in range(grid.height):
            for x in range(grid.width):
                if grid.occupied[y, x]:
                    continue
                if (x, y) in on_path and (x, y) not in (grid.start, grid.goal):
                    ax.text(x, y, f"{int(J[y, x])}", ha="center", va="center", zorder=6,
                            fontsize=5.8, color=INK, fontweight="bold",
                            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec=PATH_COLOR, lw=0.8))
                elif finite[y, x]:
                    ax.text(x, y, f"{int(J[y, x])}", ha="center", va="center",
                            fontsize=5.8, color=_text_color(shown[y, x]))
                else:
                    ax.text(x, y, "∞", ha="center", va="center", fontsize=7, color=INK)

    if show_policy:
        pts = [(s, t) for s, t in result.policy.items()]
        if pts:
            xs = np.array([s[0] for s, _ in pts], float)
            ys = np.array([s[1] for s, _ in pts], float)
            dx = np.array([t[0] - s[0] for s, t in pts], float)
            dy = np.array([t[1] - s[1] for s, t in pts], float)
            offset = 0.28 if show_values else 0.0   # keep arrows clear of numbers
            ax.quiver(xs - 0.18 * dx, ys - offset - 0.18 * dy, dx, dy,
                      angles="xy", scale_units="xy", scale=2.8, width=0.0025,
                      color=INK_MUTED, alpha=0.8, zorder=3)

    if path:
        px, py = zip(*path)
        ax.plot(px, py, color="white", linewidth=5.0, solid_capstyle="round", zorder=4)
        ax.plot(px, py, color=PATH_COLOR, linewidth=2.6, solid_capstyle="round", zorder=5)

    for cell, color, label in ((grid.start, START_COLOR, "S"), (grid.goal, GOAL_COLOR, "G")):
        ax.scatter(*cell, s=260, color=color, edgecolor="white", linewidth=2, zorder=6)
        ax.text(*cell, label, ha="center", va="center", color="white",
                fontsize=9, fontweight="bold", zorder=7)

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x (cell)", color=INK_MUTED, fontsize=9)
    ax.set_ylabel("y (cell)", color=INK_MUTED, fontsize=9)
    return j_max


def _format_stats(result: BVIResult) -> list[str]:
    s = result.stats
    lines = [f"{result.algorithm} BVI",
             f"  runtime        {s['runtime_s'] * 1e3:8.2f} ms"]
    if "sweeps" in s:
        lines.append(f"  sweeps         {int(s['sweeps']):8d}")
    if "queue_pops" in s:
        lines.append(f"  queue pops     {int(s['queue_pops']):8d}")
    lines += [f"  state updates  {int(s['state_updates']):8d}",
              f"  succ. evals    {int(s['successor_evaluations']):8d}",
              f"  value changes  {int(s['value_changes']):8d}"]
    return lines


def render_figure(grid: GridMap, result: BVIResult, path: list[Cell] | None, *,
                  difficulty: str, extra_results: tuple[BVIResult, ...] = (),
                  show_values: bool = True, show_policy: bool = False,
                  notes: tuple[str, ...] = ()):
    fig = plt.figure(figsize=(15.5, 11.5), facecolor="white")
    ax = fig.add_axes([0.04, 0.05, 0.66, 0.88])
    j_max = draw_cost_map(ax, grid, result, path, show_values, show_policy)
    j_start = result.J(grid.start)
    j_start_txt = "∞ (no path)" if math.isinf(j_start) else f"{int(j_start)}"

    fig.suptitle("Backward Value Iteration — cost-to-go J(s) and optimal path",
                 x=0.04, y=0.975, ha="left", fontsize=15, color=INK, fontweight="bold")

    info = [
        f"difficulty     {difficulty}",
        f"seed           {grid.seed}",
        f"grid           {grid.width} x {grid.height}",
        f"obstacles      {grid.num_obstacles} ({grid.num_obstacles / grid.occupied.size:.0%})",
        f"start / goal   {grid.start} / {grid.goal}",
        f"J(start)       {j_start_txt}",
        f"path length    {len(path) - 1 if path else '—'} moves",
        f"reachable      {int(np.isfinite(result.values).sum())} states",
        "",
        *_format_stats(result),
    ]
    for other in extra_results:
        info += ["", *_format_stats(other)]
    if notes:
        info += ["", *notes]
    fig.text(0.72, 0.90, "\n".join(info), va="top", ha="left", family="monospace",
             fontsize=10, color=INK, linespacing=1.45)

    # Colour bar for J.
    cax = fig.add_axes([0.72, 0.12, 0.22, 0.018])
    sm = plt.cm.ScalarMappable(cmap=COST_CMAP, norm=plt.Normalize(-0.12 * j_max / 0.88, j_max))
    cb = fig.colorbar(sm, cax=cax, orientation="horizontal")
    cb.ax.set_xlim(0, j_max)
    cb.set_label("cost-to-go J(s)  (moves to goal)", fontsize=9, color=INK_MUTED)
    cb.ax.tick_params(labelsize=8, colors=INK_MUTED)
    cb.outline.set_visible(False)

    handles = [
        Patch(facecolor=OBSTACLE_COLOR, label="obstacle (J = ∞)"),
        Patch(facecolor=UNREACHABLE_COLOR, hatch="////", edgecolor="#9a9a9a",
              label="free but unreachable (J = ∞)"),
        plt.Line2D([], [], color=PATH_COLOR, linewidth=2.6, label="optimal path"),
        plt.Line2D([], [], marker="o", linestyle="", color=START_COLOR, markersize=10, label="start"),
        plt.Line2D([], [], marker="o", linestyle="", color=GOAL_COLOR, markersize=10, label="goal"),
    ]
    fig.legend(handles=handles, loc="lower left", bbox_to_anchor=(0.715, 0.17),
               frameon=False, fontsize=9)
    return fig

"""Report figures, drawn from results/report_data.json (no value is typed by hand).

Colour grammar (shared with report_design.py): ultramarine = the verified / optimised result,
ink = the reference, amber = the GOAL marker and limitations only.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent / "figures"
INK, ULTRA, AMBER = "#0E1116", "#1B36C4", "#C77800"
GREY, SOFT, RULE = "#4A515E", "#878E9A", "#D8DCE2"
HEAT = LinearSegmentedColormap.from_list("heat", ["#F1F3FB", "#B9C3EE", "#6074D6", ULTRA, "#0F1E7A"])
plt.rcParams.update({"font.family": ["Menlo", "DejaVu Sans Mono"], "font.size": 7, "axes.edgecolor": SOFT,
                     "axes.labelcolor": GREY, "xtick.color": GREY, "ytick.color": GREY, "axes.linewidth": 0.6,
                     "savefig.dpi": 300})
LEVELS = ("easy", "medium", "hard")


def load() -> dict:
    return json.loads((ROOT / "results" / "report_data.json").read_text())


def draw_map(ax, e: dict, d_max: float | None = None, path: bool = True) -> None:
    """Heat-map of J (cells with J > d_max left blank), obstacles in ink, S and G points."""
    n = len(e["occupied"])
    occ = np.array(e["occupied"], bool)
    J = np.array([[np.nan if v is None else v for v in row] for row in e["values"]], float)
    shown = J.copy()
    if d_max is not None:
        shown[shown > d_max] = np.nan
    ax.imshow(np.where(occ, np.nan, shown), origin="lower", cmap=HEAT, vmin=0, vmax=np.nanmax(J), interpolation="nearest",
              extent=(-0.5, n - 0.5, -0.5, n - 0.5))
    for y, x in zip(*np.nonzero(occ)):
        ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, color=INK, lw=0))
    for y, x in zip(*np.nonzero(~occ & np.isnan(J))):       # free but unreachable
        ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, fill=False, hatch="////", edgecolor=SOFT, lw=0))
    if path:
        px, py = zip(*e["path"])
        ax.plot(px, py, color="white", lw=3.2, solid_capstyle="round", zorder=3)
        ax.plot(px, py, color=AMBER, lw=1.7, solid_capstyle="round", zorder=4)
    sx, sy = e["start"]
    gx, gy = e["goal"]
    ax.scatter([sx], [sy], s=46, color="white", edgecolor=INK, linewidth=1.1, zorder=6)
    ax.scatter([gx], [gy], s=62, color=AMBER, edgecolor=INK, linewidth=1.1, zorder=6, marker="*")
    ax.set_xlim(-0.5, n - 0.5)
    ax.set_ylim(-0.5, n - 0.5)
    ax.set_xticks([0, n // 2, n - 1])
    ax.set_yticks([0, n // 2, n - 1])
    ax.tick_params(length=2, labelsize=6)
    ax.set_aspect("equal")
    for s in ax.spines.values():
        s.set_color(RULE)


def fig_maps(data: dict) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(7.4, 2.62))
    for ax, level in zip(axes, LEVELS):
        e = data["maps"][level]
        draw_map(ax, e)
        ax.set_title(f"{level}  ·  seed {e['seed']}", fontsize=7.4, color=INK, pad=3, loc="left", fontweight="bold")
        ax.text(0.99, -0.16, f"J(start)={e['J_start']}  stretch {e['stretch']:.3f}  turns {e['turns']}", transform=ax.transAxes,
                ha="right", fontsize=6, color=GREY)
    fig.subplots_adjust(left=0.035, right=0.995, top=0.9, bottom=0.17, wspace=0.12)
    fig.savefig(OUT / "fig_maps.png")
    plt.close(fig)


def fig_wavefront(data: dict) -> None:
    e = data["maps"]["hard"]
    jmax = max(v for row in e["values"] for v in row if v is not None)
    cuts = [8, 20, 36, int(jmax)]
    fig, axes = plt.subplots(1, 4, figsize=(7.4, 1.95))
    for ax, d in zip(axes, cuts):
        draw_map(ax, e, d_max=d, path=False)
        n_done = sum(1 for row in e["values"] for v in row if v is not None and v <= d)
        ax.set_title(f"J ≤ {d}", fontsize=7.2, color=INK, pad=2, loc="left", fontweight="bold")
        ax.text(0.99, -0.18, f"{n_done} of {e['reachable']} final", transform=ax.transAxes, ha="right", fontsize=5.8, color=GREY)
        ax.set_xticks([])
        ax.set_yticks([])
    fig.subplots_adjust(left=0.01, right=0.995, top=0.88, bottom=0.14, wspace=0.07)
    fig.savefig(OUT / "fig_wavefront.png")
    plt.close(fig)


def fig_cost(data: dict) -> None:
    fig, (a, b) = plt.subplots(1, 2, figsize=(7.4, 2.05), gridspec_kw={"width_ratios": [1.25, 1]})
    e = data["maps"]["hard"]
    for name, color, label in (("sync", INK, "synchronous BVI"), ("queue", ULTRA, "queue ABVI")):
        t = np.array(e[name]["trace"])
        a.step(t[:, 0], t[:, 1], where="post", color=color, lw=1.4, label=label)
    a.set_xscale("log")
    a.set_xlabel("cumulative state updates (log)")
    a.set_ylabel("states with final J")
    a.axhline(e["reachable"], color=RULE, lw=0.6, zorder=0)
    a.legend(frameon=False, fontsize=6.4, loc="upper left")
    a.set_title("hard map: when does each algorithm finish?", fontsize=7, color=INK, loc="left", pad=3, fontweight="bold")
    x = np.arange(3)
    w = 0.36
    sync = [data["maps"][l]["sync"]["state_updates"] for l in LEVELS]
    que = [data["maps"][l]["queue"]["state_updates"] for l in LEVELS]
    b.bar(x - w / 2, sync, w, color=INK, label="BVI")
    b.bar(x + w / 2, que, w, color=ULTRA, label="ABVI")
    for i, (s, q) in enumerate(zip(sync, que)):
        b.text(i - w / 2, s * 1.05, f"{s / 1000:.1f}k", ha="center", fontsize=5.8, color=INK)
        b.text(i + w / 2, q * 1.05, f"{q / 1000:.1f}k", ha="center", fontsize=5.8, color=ULTRA)
    b.set_yscale("log")
    b.set_ylim(500, 120000)
    b.set_xticks(x, LEVELS)
    b.set_ylabel("state updates (log)")
    b.set_title("work per difficulty level", fontsize=7, color=INK, loc="left", pad=3, fontweight="bold")
    for ax in (a, b):
        ax.spines[["top", "right"]].set_visible(False)
    fig.subplots_adjust(left=0.07, right=0.995, top=0.88, bottom=0.2, wspace=0.28)
    fig.savefig(OUT / "fig_cost.png")
    plt.close(fig)


def slope(xs, ys) -> float:
    return float(np.polyfit(np.log(xs), np.log(ys), 1)[0])


def fig_scaling(data: dict) -> dict:
    rows = data["scaling"]
    X = np.array([r["states"] for r in rows], float)
    fits = {}
    fig, axes = plt.subplots(1, 3, figsize=(7.4, 1.95))
    a, b, c = axes
    for name, color, label in (("sync", INK, "BVI"), ("queue", ULTRA, "ABVI")):
        up = np.array([r[name]["updates"] for r in rows])
        ms = np.array([r[name]["ms"] for r in rows])
        fits[name] = {"updates": slope(X, up), "ms": slope(X, ms)}
        a.loglog(X, up, "o-", color=color, ms=3, lw=1.2, label=f"{label}  ~ |X|$^{{{fits[name]['updates']:.2f}}}$")
        b.loglog(X, ms, "o-", color=color, ms=3, lw=1.2, label=f"{label}  ~ |X|$^{{{fits[name]['ms']:.2f}}}$")
    a.set_ylabel("state updates")
    b.set_ylabel("runtime [ms]")
    for ax, t in ((a, "cost: updates"), (b, "cost: runtime")):
        ax.set_xlabel("free states |X|")
        ax.legend(frameon=False, fontsize=6, loc="upper left")
        ax.set_title(t, fontsize=7, color=INK, loc="left", pad=3, fontweight="bold")
    pq = np.array([r["queue"]["peak_queue"] for r in rows])
    fits["peak_queue"] = slope(X, pq)
    c.loglog(X, 2 * X, "o-", color=INK, ms=3, lw=1.2, label="BVI: J$_i$ and J$_{i+1}$  = 2|X|")
    c.loglog(X, X + pq, "o-", color=ULTRA, ms=3, lw=1.2, label="ABVI: J + queue peak")
    c.loglog(X, pq, "s--", color=AMBER, ms=3, lw=1.0, label=f"queue peak ~ |X|$^{{{fits['peak_queue']:.2f}}}$")
    c.set_xlabel("free states |X|")
    c.set_ylabel("stored J-values (words)")
    c.set_ylim(5, 1.2e5)
    c.legend(frameon=False, fontsize=5.6, loc="upper left")
    c.set_title("space: working set", fontsize=7, color=INK, loc="left", pad=3, fontweight="bold")
    for ax in axes:
        ax.spines[["top", "right"]].set_visible(False)
        ax.tick_params(labelsize=6)
    fig.subplots_adjust(left=0.065, right=0.995, top=0.88, bottom=0.2, wspace=0.3)
    fig.savefig(OUT / "fig_scaling.png")
    plt.close(fig)
    return fits


def crop_rviz() -> None:
    from PIL import Image
    for src, dst in (("hard_mid", "rviz_mid"), ("hard_policy_mid", "rviz_policy")):
        Image.open(ROOT / "results" / "rviz" / f"{src}.png").crop((430, 165, 1520, 840)).save(OUT / f"{dst}.png")


def main() -> None:
    OUT.mkdir(exist_ok=True)
    data = load()
    fig_maps(data)
    fig_wavefront(data)
    fig_cost(data)
    fits = fig_scaling(data)
    crop_rviz()
    (OUT / "fits.json").write_text(json.dumps(fits))
    print("figures ->", OUT, fits)


if __name__ == "__main__":
    main()

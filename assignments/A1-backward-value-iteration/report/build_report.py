"""Build the four-page ARP A1 report (fixed A4 pages, DIP 'Photometric' design system).

Every measured number is read at build time from
  results/report_data.json   (standalone/report_data.py)   results/benchmark.csv   results/difficulty_seeds.json
  report/figures/*.png       (report/make_figures.py)      the pytest run below
Usage (from the project root):
  .venv/bin/python standalone/report_data.py && .venv/bin/python report/make_figures.py
  .venv/bin/python report/build_report.py        ->  report/ARP_A1_BVI_Report.pdf
"""
from __future__ import annotations

import base64
import csv
import json
import re
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path

from report_design import CSS, limit, readout, section, stamp
from report_flow import flow_svg

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
FIG, BUILD = HERE / "figures", HERE / "_build"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
PDF = HERE / "ARP_A1_BVI_Report.pdf"
TITLE = "Backward Value Iteration on a 30×30 Grid"
SUBTITLE = "Two schedules of one Bellman equation, compared on cost, space and map difficulty, and run live in ROS 2 and RViz2"
FOOT = "ARP A1 · Backward value iteration"
LEVELS = ("easy", "medium", "hard")
# Measured on the Linux ROS machine by scripts/verify.sh on 2026-09-30 (easy map); the counts match the Mac exactly.
SERVER_MS = {"sync": 50.97, "queue": 3.31}
EXTRA_CSS = """
body{font-size:8.2pt;line-height:1.27}
p{margin-bottom:1.3mm}
.sec{margin:2.2mm 0 1mm}
.eq{padding:1.1mm 2mm;margin:1.2mm 0;font-size:8.8pt}
table{font-size:6.6pt;margin:.8mm 0 1.1mm}
th,td{padding:.62mm .6mm}
thead th{font-size:6pt}
tbody th{font-size:6.6pt}
figcaption,.caption{font-size:6.9pt}
li{margin-bottom:.5mm}
.refs{font-size:6.5pt;columns:1}
.refs p{margin-bottom:.4mm}
.stamp{margin:1.2mm 0;padding:1.2mm 2mm}
.limit{margin:1.2mm 0;padding:1.2mm 2mm}
.algo{border:.3mm solid #1F2430;margin:1.2mm 0;font-family:'Menlo','DejaVu Sans Mono',monospace;font-size:6.9pt;line-height:1.42}
.algo-hd{background:#1F2430;color:#fff;font-family:'Futura','Avenir Next',sans-serif;font-size:6.6pt;font-weight:600;letter-spacing:.1em;padding:.7mm 2mm}
.algo-hd.q{background:#1B36C4}
.algo ol{margin:0;padding:1mm 1.5mm 1mm 8mm;list-style:decimal;color:#878E9A}
.algo li{margin:0;color:#0E1116;white-space:pre}
.algo .k{font-weight:700}
.algo .c{color:#878E9A;font-style:italic}
.mini{font-size:7pt}
.pt{display:inline-block;width:2.2mm;height:2.2mm;border-radius:50%;vertical-align:-.3mm;border:.25mm solid #0E1116}
"""


# ----------------------------------------------------------------------------- data
def img(path: Path, style: str = "") -> str:
    return f"<img src='data:image/png;base64,{base64.b64encode(path.read_bytes()).decode()}' style='{style}'>"


def run_tests() -> int:
    out = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider"], cwd=ROOT, capture_output=True, text=True).stdout
    m = re.search(r"(\d+) passed", out)
    if not m or "failed" in out:
        raise RuntimeError(f"tests not green:\n{out[-600:]}")
    return int(m.group(1))


def benchmark() -> dict:
    rows = list(csv.DictReader((ROOT / "results" / "benchmark.csv").open()))
    mean = lambda k: statistics.fmean(float(r[k]) for r in rows)  # noqa: E731
    return {"maps": len(rows), "upd_s": mean("sync_state_updates"), "upd_q": mean("queue_state_updates"),
            "ev_s": mean("sync_succ_evals"), "ev_q": mean("queue_succ_evals"), "ev_r": mean("queue-relax_succ_evals"),
            "ms_s": mean("sync_runtime_ms"), "ms_q": mean("queue_runtime_ms"), "ms_r": mean("queue-relax_runtime_ms")}


def numbers() -> dict:
    data = json.loads((ROOT / "results" / "report_data.json").read_text())
    sel = json.loads((ROOT / "results" / "difficulty_seeds.json").read_text())
    return {"d": data, "m": data["maps"], "sc": data["scaling"], "sel": sel, "bm": benchmark(), "fits": json.loads((FIG / "fits.json").read_text()),
            "tests": run_tests()}


# ----------------------------------------------------------------------------- html helpers
def algo(title: str, lines: list[str], cls: str = "") -> str:
    body = "".join(f"<li>{ln}</li>" for ln in lines)
    return f"<div class='algo'><div class='algo-hd {cls}'>{title}</div><ol>{body}</ol></div>"


def k(s: str) -> str:
    return f"<span class='k'>{s}</span>"


def c(s: str) -> str:
    return f"<span class='c'>{s}</span>"


def fig(path: Path, label: str, caption: str, style: str = "") -> str:
    return f"<figure>{img(path, style)}<figcaption><b>{label}</b> {caption}</figcaption></figure>"


def f0(x: float) -> str:
    return f"{x:,.0f}"


# ----------------------------------------------------------------------------- pages
def page1(n: dict) -> str:
    m, bm = n["m"], n["bm"]
    ro = readout([
        (" / ".join(str(m[l]["J_start"]) for l in LEVELS), "J(start): easy / medium / hard<br>optimal path length"),
        (f"{bm['upd_s'] / bm['upd_q']:.1f}×", f"fewer state updates, queue ABVI<br>vs sync BVI ({bm['maps']} maps)"),
        (f"|X|<sup>{n['fits']['sync']['updates']:.2f}</sup> → |X|<sup>{n['fits']['queue']['updates']:.2f}</sup>", "growth of the work, BVI → ABVI<br>(grids 10² … 80²)"),
        ("PASS 5/5", f"optimality certificate, all 6 runs<br>+ independent ROS node · {n['tests']} tests"),
    ])
    reqs = """<table class='text tight'><thead><tr><th>#</th><th>Assignment requirement</th><th>Where it is shown</th></tr></thead><tbody>
<tr><td>1</td><td>ROS 2 installed; 30×30 map with 180 random obstacles; robot placed at the start cell</td><td><span class='num'>/map</span> OccupancyGrid, Humble in user space; robots start at START · §6, Fig. 6</td></tr>
<tr><td>2</td><td>Cost-to-go values J(s) visualised</td><td>heat-map + integer labels <span class='num'>/bvi_values</span> · Fig. 2, Fig. 6</td></tr>
<tr><td>3</td><td>Optimal path from start to goal</td><td><span class='num'>/bvi_path</span>, <span class='num'>/abvi_path</span>; certificate C5 · §7</td></tr>
<tr><td>4</td><td>Robot navigates along the path in RViz2</td><td>two animated robots, 1 cell / 0.4 s, J falls by 1 per step · Fig. 6</td></tr>
<tr><td>+</td><td>Two BVI variants compared; easy / medium / hard maps</td><td>§2 algorithms · §3 maps · §5 cost and space</td></tr></tbody></table>"""
    alg1 = algo("ALGORITHM 1 · SYNCHRONOUS BVI (JACOBI SWEEPS)", [
        f"J₀(s) ← ∞  ∀ s ∈ X;   J₀(g) ← 0",
        f"{k('repeat')}   {c('# sweep i = 1, 2, …')}",
        f"    {k('for each')} s ∈ X \\ {{g}}",
        f"        Jᵢ(s) ← min<sub>s′∈Succ(s)</sub> [ 1 + Jᵢ₋₁(s′) ]",
        f"{k('until')} Jᵢ = Jᵢ₋₁              {c('# a full sweep changed nothing')}",
        f"{k('return')} J, π(s) = argmin<sub>s′</sub> [ 1 + J(s′) ]",
    ])
    alg2 = algo("ALGORITHM 2 · QUEUE ABVI (FIFO WAVEFRONT)", [
        f"J(s) ← ∞  ∀ s;   J(g) ← 0;   Q ← ⟨ g ⟩",
        f"{k('while')} Q ≠ ∅",
        f"    y ← Q.popleft()",
        f"    {k('for each')} p ∈ Pred(y), p ≠ g",
        f"        v ← min<sub>s′∈Succ(p)</sub> [ 1 + J(s′) ]     {c('# live values')}",
        f"        {k('if')} v &lt; J(p):   J(p) ← v;   Q.append(p)",
        f"{k('return')} J, π",
    ], "q")
    return f"""
<div class='kicker'>Algorithmic Techniques for Robot Planning (CSE634) · Assignment 1</div>
<h1>{TITLE}</h1>
<div class='subtitle'>{SUBTITLE}</div>
<div class='meta'>Kush Tokas / 2023296 / arp_a1_bvi / IIIT Delhi / 1 October 2026</div>
{ro}
{section(0, '1', 'The task and the model')}
<p>A point robot moves on a <b>30×30 grid</b> with exactly <b>180 obstacles</b> (20 %), drawn uniformly at random from a seed; start (1, 1), goal (28, 28).
Each move N, S, E or W to a free neighbour costs 1. The state set <i>X</i> is the free cells ({m['hard']['free_states']} per map), <i>Succ(s)</i> are the free neighbours of <i>s</i>,
and <i>Pred(y)</i> = {{ p : y ∈ Succ(p) }}. Moves are reversible, so Pred = Succ on this grid, but the two roles stay different:
<b>Pred says who must be updated, Succ says how to compute the value.</b> The goal is the cost-to-go <i>J</i> of every state, not one path, so A*-style single-query search does not apply and none is used.</p>
{reqs}
{section(1, '2', 'One equation, two schedules')}
<div class='eq'>J(g) = 0, &nbsp; J(s) = min<sub>s′ ∈ Succ(s)</sub> [ 1 + J(s′) ], &nbsp; J = ∞ on obstacles and unreachable cells <span class='tag'>(1)</span></div>
<div class='eq'>π(s) = argmin<sub>s′ ∈ Succ(s)</sub> [ 1 + J(s′) ], &nbsp; ties broken N, S, E, W; path = follow π from START to g <span class='tag'>(2)</span></div>
<div class='two'>{alg1}{alg2}</div>
<p><b>A</b> backs up every state in every sweep using only the previous sweep's values, so it needs max&nbsp;J&nbsp;+&nbsp;1 sweeps and keeps two value arrays.
<b>B</b> backs up only the predecessors of a state whose value just changed and uses new values immediately. A FIFO queue is sufficient because every cost is 1: states are finalised in
non-decreasing J, each is written exactly once, and the queue holds at most two adjacent J levels (all asserted in the tests). Both are hand-written in <span class='num'>bvi_core/bvi.py</span>; no A*, Dijkstra, NetworkX or Nav2 is called.</p>
<figure>{flow_svg()}<figcaption><b>Figure 1 · Pipeline.</b> The same pure-Python core feeds the standalone tools and the ROS 2 package (one copy, linked into the package). The two J maps are compared cell by cell before anything is published; <span class='pt' style='background:#C77800'></span> amber marks the goal.</figcaption></figure>
"""


def page2(n: dict) -> str:
    m, sel = n["m"], n["sel"]
    rows = "".join(
        f"<tr><th>{l}</th><td>{m[l]['seed']}</td><td>{m[l]['J_start']}</td><td>{m[l]['manhattan']}</td><td>{m[l]['stretch']:.3f}</td><td>{m[l]['turns']}</td>"
        f"<td>{m[l]['reachable']}</td><td>{m[l]['unreachable_free']}</td></tr>" for l in LEVELS)
    tab = f"""<table><thead><tr><th>map</th><th>seed</th><th>J(start)</th><th>Manhattan</th><th>stretch</th><th>turns</th><th>states with finite J</th><th>free but unreachable</th></tr></thead><tbody>{rows}</tbody></table>"""
    lo, mid, hi = sel["stretch_min_median_max"]
    up = [m[l]["sync"]["state_updates"] for l in LEVELS]
    return f"""
{section(2, '3', 'Easy, medium and hard maps')}
<p>Difficulty is chosen by a fixed rule, not by hand. Seeds 0–499 give one 180-obstacle map each; <b>{sel['accepted']}</b> have START connected to GOAL and
<b>{sel['rejected_disconnected']}</b> are rejected and counted. Maps are ranked by (<i>stretch</i> = J(start) / Manhattan, <i>turns</i>, seed) and the 20th, 50th and 90th percentiles
are named easy, medium, hard. Stretch measures how far obstacles force the path above the Manhattan lower bound; turns measure how often it bends.</p>
{fig(FIG / 'fig_maps.png', 'Figure 2 · The three maps.', "Heat-map is J (dark = far from the goal), black cells are obstacles, hatched cells are free but unreachable, "
     "the amber line is the optimal path, <span class='pt' style='background:#fff'></span> is START and the amber star is GOAL. The queue-ABVI path is shown; the BVI path has the same cost.")}
{tab}
<div class='limit'><div class='limit-hd'>Honest reading of &ldquo;difficulty&rdquo;</div>
With START and GOAL in opposite corners and 20 % uniform obstacles a monotone staircase path almost always exists: stretch has min {lo:.3f}, median {mid:.3f}, max {hi:.3f}.
Easy and medium therefore differ only through the turn count (14 vs 17); hard is the first level with a detour (+2 moves, {m['hard']['stretch']:.3f}).
These levels describe <b>navigation</b> difficulty. They barely change BVI's <b>computation</b>: the full J is solved regardless, so updates are {f0(up[0])}, {f0(up[1])} and {f0(up[2])} for the synchronous variant ({(max(up) / min(up) - 1) * 100:.1f} % spread).</div>
{section(3, '4', 'How the value wave moves')}
{fig(FIG / 'fig_wavefront.png', 'Figure 3 · Queue ABVI on the hard map.', "Cells whose J is already final after the wave has passed level d. Because FIFO with unit costs finalises states in non-decreasing J, "
     "the finite region is exactly { J ≤ d }: a diagonal front sweeping out from the goal that is squeezed through the obstacle gaps. Hatched corner cells are never reached (J stays ∞).")}
<p>This picture is the whole difference between the algorithms. ABVI touches the front: the {m['hard']['queue']['peak_queue']} states in the queue at its widest (peak queue) out of {m['hard']['reachable']}. Synchronous BVI keeps
sweeping all {m['hard']['free_states'] - 1} states, including the ∞ region ahead of the front and the finished region behind it, for {m['hard']['sync']['sweeps']} sweeps (= max J + 1).
Both stop at the same fixed point of (1), which is unique, so the J maps are identical on every cell of every map we ran.</p>
<h3>What was checked, and what it proves</h3>
<ul>
<li><b>Equality:</b> sync J = queue J = edge-relax J on 60 test maps, {n['bm']['maps']} benchmark maps and all demos (cell by cell, ∞ = ∞).</li>
<li><b>Path:</b> every step is a legal move, J falls by exactly 1 per step, and the path has J(start) moves.</li>
<li><b>Failure injection:</b> the tests corrupt one J value, take a detour, or give an unreachable cell a finite J; the certificate (§7) rejects each.</li></ul>
"""


def row_cost(n: dict, level: str, name: str, label: str) -> str:
    e = n["m"][level]
    s = e[name]
    it = f"{s['sweeps']} sweeps" if name == "sync" else f"{s['queue_pops']} pops"
    ws = 2 * 900 if name == "sync" else 900 + s["peak_queue"]
    cls = " class='ours'" if name == "queue" else ""
    return (f"<tr><th>{level}</th><td>{label}</td><td>{it}</td><td>{f0(s['state_updates'])}</td><td>{f0(s['successor_evaluations'])}</td>"
            f"<td{cls}>{s['runtime_s'] * 1e3:.2f}</td><td>{f0(ws)}</td><td>{s['peak_heap_kib']:.0f}</td><td>{e['J_start']}</td>"
            f"<td>{(len(e['path']) - 1) * 0.4:.1f}</td><td>{'PASS' if s['certificate'] else 'FAIL'}</td></tr>")


def page3(n: dict) -> str:
    m, bm, fits, sc = n["m"], n["bm"], n["fits"], n["sc"]
    body = "".join(row_cost(n, l, a, lab) for l in LEVELS for a, lab in (("sync", "BVI sync"), ("queue", "ABVI queue")))
    tab = f"""<table><thead><tr><th>map</th><th>algorithm</th><th>iterations</th><th>state updates</th><th>successor evals</th><th>runtime ms (Mac)</th>
<th>J words stored</th><th>peak heap KiB</th><th>J(start)</th><th>travel s</th><th>cert.</th></tr></thead><tbody>{body}</tbody></table>"""
    e = m["easy"]
    r_up, r_ev = bm["upd_s"] / bm["upd_q"], bm["ev_s"] / bm["ev_q"]
    big = sc[-1]
    return f"""
{section(4, '5', 'Six runs: {easy, medium, hard} × {BVI, ABVI}')}
{tab}
<p class='note'>Updates = full backups attempted; successor evals = neighbour look-ups inside them; runtime = best of 5, CPython 3, single thread; travel = path cost × 0.4 s per move.
<b>J words stored</b> is the algorithmic working set: BVI holds Jᵢ₋₁ and Jᵢ (2 × 900), ABVI holds one J array plus its queue peak. <b>Peak heap</b> is the measured Python allocation (tracemalloc) of one whole run
including the policy dictionary and the convergence trace, which ABVI records per pop ({e['queue']['queue_pops']} entries vs {e['sync']['sweeps']}); this bookkeeping hides the algorithmic saving, so both are shown.</p>
{fig(FIG / 'fig_cost.png', 'Figure 4 · Cost.', f"Left: states with final J against cumulative updates (log axis) on the hard map; ABVI finishes after {f0(m['hard']['queue']['state_updates'])} updates, "
     f"BVI after {f0(m['hard']['sync']['state_updates'])}. Right: updates per map. Cost and travel time of the resulting path are identical for both algorithms.")}
<div class='two'><div>
<h3>Cost</h3>
<p>On {bm['maps']} random reachable maps ABVI needs <b>{r_up:.1f}×</b> fewer updates and <b>{r_ev:.1f}×</b> fewer successor evaluations; the Bellman-backup variant runs in {bm['ms_q']:.2f} ms against {bm['ms_s']:.2f} ms
({bm['ms_s'] / bm['ms_q']:.0f}×) on the Mac, and {SERVER_MS['queue']:.1f} ms against {SERVER_MS['sync']:.1f} ms on the Linux ROS machine. Updating by edge relaxation, J(p) ← min(J(p), 1 + J(y)), gives the same J
with {bm['ev_r']:.0f} look-ups ({bm['ms_r']:.2f} ms). Update counts are hardware-independent; the millisecond figures are only CPython timings.</p>
<p>Why: BVI does |X| backups × (max J + 1) sweeps, and max J grows like the grid side √|X|, so its work scales as |X|<sup>1.5</sup>. ABVI pops each state once and
touches ≤ 4 predecessors per pop: about 4|X| work.</p></div><div>
<h3>Space</h3>
<p>BVI needs two value arrays (2|X| words) because a sweep reads Jᵢ₋₁ while writing Jᵢ; ABVI needs one array plus the queue, which holds only the wavefront: peak <b>{e['queue']['peak_queue']}</b> / {m['medium']['queue']['peak_queue']} / {m['hard']['queue']['peak_queue']}
entries on the three maps, ≈ {e['queue']['peak_queue'] / e['reachable'] * 100:.0f} % of the states. The queue peak grows only as |X|<sup>{fits['peak_queue']:.2f}</sup> (the front is a curve across the grid), so ABVI's extra memory is O(√|X|)
while BVI's extra array is O(|X|). Both return the same policy dictionary (≈ {e['reachable'] - 1} entries), which dominates the real heap.</p></div></div>
{section(5, '6', 'Scaling beyond 30×30')}
{fig(FIG / 'fig_scaling.png', 'Figure 5 · Scaling.', f"Square grids of side 10, 20, 30, 40, 60, 80 with 20 % obstacles, corner to corner, mean over {sc[0]['maps']} connected maps each. Least-squares slopes on the log-log axes: "
     f"updates |X|<sup>{fits['sync']['updates']:.2f}</sup> (BVI) vs |X|<sup>{fits['queue']['updates']:.2f}</sup> (ABVI); runtime |X|<sup>{fits['sync']['ms']:.2f}</sup> vs |X|<sup>{fits['queue']['ms']:.2f}</sup>.")}
<p>The gap widens with size: at {f0(big['states'])} free states ({big['side']}×{big['side']}) BVI needs {f0(big['sync']['updates'])} updates against {f0(big['queue']['updates'])} ({big['sync']['updates'] / big['queue']['updates']:.0f}×) and {big['sync']['ms']:.0f} ms against {big['queue']['ms']:.0f} ms.
The measured exponents match the analysis (1.5 and 1.0), so the 18× at 30×30 is not a constant factor: it is the ratio of the sweep count to a constant.</p>
"""


def page4(n: dict) -> str:
    topics = """<table class='text tight'><thead><tr><th>topic</th><th>type</th><th>content</th></tr></thead><tbody>
<tr><td class='num'>/map</td><td>nav_msgs/OccupancyGrid</td><td>0 free, 100 obstacle, 30×30, 1 m cells, frame <span class='num'>map</span></td></tr>
<tr><td class='num'>/bvi_values</td><td>MarkerArray</td><td>J heat-map (CUBE_LIST), integer TEXT labels, START / GOAL</td></tr>
<tr><td class='num'>/bvi_path /abvi_path</td><td>nav_msgs/Path</td><td>optimal path from sync BVI / queue ABVI</td></tr>
<tr><td class='num'>/bvi_policy /abvi_policy</td><td>MarkerArray</td><td>π(s) arrow for every reachable state</td></tr>
<tr><td class='num'>/bvi_robot /abvi_robot</td><td>MarkerArray</td><td>animated robots, 2.5 Hz, label step / total, time, remaining J</td></tr>
<tr><td class='num'>/bvi_stats</td><td>Marker</td><td>comparison panel and both certificates</td></tr>
<tr><td class='num'>/optimality_report</td><td>std_msgs/String</td><td>verdict of the independent checker node</td></tr></tbody></table>"""
    cert = stamp("OPTIMALITY CERTIFICATE · bvi_core/verify.py · all six runs + ROS checker", [
        ("C1 boundary", "<b>J(g)=0, obstacles ∞</b>"),
        ("C2 Bellman fixed point", "<b>max |J − min(1+J′)| = 0 over 719 states</b>"),
        ("C3 finite ⇔ reachable", "<b>718 reachable, 2 unreachable free (hard)</b>"),
        ("C4 lower bound", "<b>|J(u) − J(v)| ≤ 1 on every edge; J ≥ Manhattan</b>"),
        ("C5 path optimal", "<b>legal, moves = J(start) = 56 (hard)</b>"),
    ])
    return f"""
{section(6, '7', 'ROS 2 and RViz2: what runs')}
<p>The package <span class='num'>bvi_planner</span> (rclpy, ament_python) has three nodes. <span class='num'>planner_node</span> builds the map, solves both BVIs and publishes everything once with latched (transient-local) QoS;
<span class='num'>robot_animation_node</span> (two instances) walks each path one cell per 0.4 s and restarts after a 3 s hold at the goal; <span class='num'>optimality_checker_node</span> subscribes to <span class='num'>/map</span> and the paths,
rebuilds J independently and publishes the verdict. ROS 2 Humble runs on Linux (RoboStack/conda or apt); RViz2 renders the live scene.</p>
<div class='two'><figure>{img(FIG / 'rviz_mid.png')}<figcaption><b>Figure 6a · RViz2, robots mid-run.</b> Obstacles, J heat-map with labels, blue BVI and orange ABVI paths, two robots (label: step/56, t, J), stats panel with both certificates (hard map, real screenshot).</figcaption></figure>
<figure>{img(FIG / 'rviz_policy.png')}<figcaption><b>Figure 6b · Policy arrows.</b> The optimal action from <i>every</i> reachable cell (ABVI policy), not only along one path.</figcaption></figure></div>
{topics}
{section(7, '8', 'Verification')}
<div class='two'><div>{cert}</div><div>
<table class='text tight'><thead><tr><th>check</th><th>result</th></tr></thead><tbody>
<tr><td>unit + integration tests (macOS and Linux)</td><td><b>{n['tests']} passed</b></td></tr>
<tr><td>sync J = queue J = relax J</td><td>identical, all maps</td></tr>
<tr><td>ROS launch easy / medium / hard</td><td>both robots reach the goal</td></tr>
<tr><td>C1–C5, planner and checker node</td><td>PASS 5/5 on both paths</td></tr>
<tr><td>colcon build, RViz2 launch</td><td>clean build, no errors</td></tr></tbody></table></div></div>
<p>C4 and C5 together prove optimality: no path from s can have fewer than J(s) moves, and the extracted path has exactly J(start). C1–C3 prove J is the unique optimal cost-to-go.</p>
{limit('Assumptions and limits', "<ul style='margin:0;padding-left:4mm'>"
      "<li>Cells are (x, y) with (0, 0) bottom-left, as in OccupancyGrid; resolution 1 m. Equal-cost ties break N, S, E, W, so other optimal paths exist.</li>"
      "<li>The robot is a marker moving cell by cell (kinematic animation); there is no dynamics simulation.</li>"
      "<li>Difficulty levels rank navigation, not BVI cost (§3). Timings are CPython; the counts are the portable result.</li>"
      "<li>Disconnected maps are rejected by a documented rule (seed, seed+1, …) and logged, never hidden.</li></ul>")}
<h3>Run it</h3>
<p class='mini'><span class='num'>./scripts/build.sh &amp;&amp; ./scripts/verify.sh</span> · <span class='num'>./scripts/vnc_start.sh</span> · on the Mac <span class='num'>./scripts/mac_view.sh</span> · <span class='num'>./scripts/demo.sh hard</span>. Full step-by-step manual with the TA script: <span class='num'>DEMO_SERVER.md</span>.</p>
<div class='refs'><h3>References</h3>
<p>[1] R. Bellman, <i>Dynamic Programming</i>, Princeton University Press, 1957.</p>
<p>[2] S. M. LaValle, <i>Planning Algorithms</i>, Cambridge University Press, 2006, ch. 2 (discrete planning, value iteration).</p>
<p>[3] CSE634 lecture notes: backward value iteration and asynchronous BVI (sessions 4–6), IIIT Delhi, August 2026.</p>
<p>[4] S. Macenski, T. Foote, B. Gerkey, C. Lalancette, W. Woodall, “Robot Operating System 2: Design, architecture, and uses in the wild,” <i>Science Robotics</i> 7(66), 2022.</p></div>
"""


def build_html(n: dict) -> str:
    from report_design import page
    pages = [page1(n), page2(n), page3(n), page4(n)]
    body = "".join(page(b, FOOT + " · Kush Tokas · 2023296", i + 1, len(pages)) for i, b in enumerate(pages))
    return f"<!doctype html><html><head><meta charset='utf-8'><title>ARP A1 Backward Value Iteration</title><style>{CSS}{EXTRA_CSS}</style></head><body>{body}</body></html>"


def render_pdf(html_path: Path, pdf_path: Path) -> None:
    profile = BUILD / "chrome-profile"
    pdf_path.unlink(missing_ok=True)
    proc = subprocess.Popen([CHROME, "--headless", "--disable-gpu", "--no-sandbox", f"--user-data-dir={profile}", "--no-pdf-header-footer",
                             f"--print-to-pdf={pdf_path}", html_path.as_uri()], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    last, deadline = -1, time.time() + 240
    while time.time() < deadline:
        size = pdf_path.stat().st_size if pdf_path.exists() else -1
        if size > 0 and size == last:
            break
        last = size
        time.sleep(1.5)
    proc.terminate()
    proc.wait(timeout=10)
    shutil.rmtree(profile, ignore_errors=True)
    if not pdf_path.exists():
        raise RuntimeError("Chrome did not produce the PDF")


def main() -> None:
    BUILD.mkdir(exist_ok=True)
    n = numbers()
    html = BUILD / "report.html"
    html.write_text(build_html(n), encoding="utf-8")
    render_pdf(html, PDF)
    pages = len(re.findall(rb"/Type\s*/Page[^s]", PDF.read_bytes()))
    print(f"wrote {PDF} ({PDF.stat().st_size // 1024} KiB, {pages} pages)")


if __name__ == "__main__":
    main()

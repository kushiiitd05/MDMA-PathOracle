"""Pipeline diagram for the report: map -> two BVI schedules -> J -> path/policy -> ROS 2 topics -> RViz2.

The GOAL node is amber (same colour as the goal marker in the figures); verified artefacts are ultramarine.
"""
from __future__ import annotations

from report_design import AMBER, GREY_SOFT, GREY_TEXT, INK, MONO, SANS, ULTRA

W, H = 980, 150


def _box(x, y, w, h, title, sub="", color=INK, fill="#FFFFFF", dash=False) -> str:
    stroke = " stroke-dasharray='3 2'" if dash else ""
    t = (f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='3' fill='{fill}' stroke='{color}' stroke-width='0.9'{stroke}/>"
         f"<text x='{x + w / 2}' y='{y + (13 if sub else h / 2 + 3.2)}' text-anchor='middle' font-family={SANS!r} "
         f"font-size='8.3' font-weight='600' fill='{INK}'>{title}</text>")
    for i, line in enumerate(sub.split("|") if sub else []):
        t += (f"<text x='{x + w / 2}' y='{y + 24 + 8.6 * i}' text-anchor='middle' font-family={MONO!r} "
              f"font-size='7' fill='{GREY_TEXT}'>{line}</text>")
    return t


def _arrow(x1, y1, x2, y2, label="", color=GREY_SOFT) -> str:
    s = f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='{color}' stroke-width='0.9' marker-end='url(#ah)'/>"
    if label:
        s += (f"<text x='{(x1 + x2) / 2}' y='{min(y1, y2) - 3.5}' text-anchor='middle' font-family={MONO!r} "
              f"font-size='6.6' fill='{GREY_TEXT}'>{label}</text>")
    return s


def flow_svg() -> str:
    p = [f"<svg viewBox='0 0 {W} {H}' width='100%' xmlns='http://www.w3.org/2000/svg'>",
         f"<defs><marker id='ah' viewBox='0 0 8 8' refX='7' refY='4' markerWidth='5' markerHeight='5' orient='auto'>"
         f"<path d='M0,0 L8,4 L0,8 z' fill='{GREY_SOFT}'/></marker></defs>"]
    p.append(f"<text x='0' y='9' font-family={SANS!r} font-size='7.4' font-weight='600' letter-spacing='1.4' fill='{GREY_SOFT}'>PLANNING CORE  (bvi_core, numpy only)</text>")
    p.append(f"<text x='640' y='9' font-family={SANS!r} font-size='7.4' font-weight='600' letter-spacing='1.4' fill='{GREY_SOFT}'>ROS 2 HUMBLE  (bvi_planner)</text>")
    # core lane
    p.append(_box(0, 18, 110, 44, "Map", "30×30, seed|180 obstacles"))
    p.append(_arrow(110, 40, 150, 40))
    p.append(_box(150, 18, 140, 20, "A · synchronous BVI", dash=False))
    p.append(_box(150, 44, 140, 20, "B · queue ABVI", color=ULTRA))
    p.append(_arrow(290, 28, 330, 40))
    p.append(_arrow(290, 54, 330, 40))
    p.append(_box(330, 18, 116, 44, "J(s)  ==  J(s)", "cell-by-cell|cross-check", color=ULTRA))
    p.append(_arrow(446, 40, 486, 40))
    p.append(_box(486, 18, 120, 44, "π(s), path", "argmin 1+J(s')|from START", color=ULTRA))
    p.append(_box(486, 76, 120, 30, "GOAL  J = 0", color=AMBER))
    p.append(_arrow(546, 76, 546, 62, color=AMBER))
    # ROS lane
    p.append(_arrow(606, 40, 646, 40, "latched"))
    p.append(_box(646, 18, 140, 44, "planner_node", "/map /bvi_values|/bvi_path /abvi_path"))
    p.append(_arrow(786, 40, 826, 40))
    p.append(_box(826, 18, 154, 44, "RViz2", "heat-map, paths, arrows|stats panel"))
    p.append(_box(646, 76, 140, 44, "robot_animation ×2", "/bvi_robot /abvi_robot|2.5 Hz, 1 cell / 0.4 s"))
    p.append(_arrow(716, 62, 716, 76))
    p.append(_box(826, 76, 154, 44, "optimality_checker", "rebuilds J from /map|/optimality_report", color=ULTRA))
    p.append(_arrow(786, 58, 826, 86))
    p.append("</svg>")
    return "".join(p)

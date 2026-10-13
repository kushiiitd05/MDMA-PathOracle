"""Builders for RViz messages. Pure message construction, no planning logic."""
from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from bvi_core.bvi import BVIResult
from bvi_core.grid import Cell, GridMap

FRAME = "map"
# Sequential "Blues" ramp: light = near goal, dark = far from goal.
_RAMP = [(0.87, 0.92, 0.97), (0.62, 0.79, 0.88), (0.26, 0.57, 0.78), (0.03, 0.19, 0.42)]


def rgba(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def hex_rgba(code: str, a: float = 1.0) -> ColorRGBA:
    code = code.lstrip("#")
    r, g, b = (int(code[i:i + 2], 16) / 255 for i in (0, 2, 4))
    return rgba(r, g, b, a)


def ramp_color(t: float) -> tuple[float, float, float]:
    t = min(max(t, 0.0), 1.0) * (len(_RAMP) - 1)
    i = min(int(t), len(_RAMP) - 2)
    f = t - i
    return tuple(a + (b - a) * f for a, b in zip(_RAMP[i], _RAMP[i + 1]))


def cell_center(cell: Cell, res: float, z: float = 0.0) -> Point:
    return Point(x=(cell[0] + 0.5) * res, y=(cell[1] + 0.5) * res, z=z)


def _marker(ns: str, mid: int, mtype: int, stamp: Time) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME
    m.header.stamp = stamp
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def occupancy_grid(grid: GridMap, res: float, stamp: Time) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id = FRAME
    msg.header.stamp = stamp
    msg.info.resolution = float(res)
    msg.info.width, msg.info.height = grid.width, grid.height
    msg.info.origin.orientation.w = 1.0
    # Row-major, row 0 = bottom (y = 0): same layout as grid.occupied.
    msg.data = [100 if v else 0 for v in grid.occupied.flatten()]
    return msg


def path_msg(path: list[Cell], res: float, stamp: Time, z: float) -> Path:
    msg = Path()
    msg.header.frame_id = FRAME
    msg.header.stamp = stamp
    for cell in path:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = cell_center(cell, res, z)
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def value_markers(grid: GridMap, result: BVIResult, res: float, stamp: Time,
                  show_text: bool) -> MarkerArray:
    """Heat-map tiles (one CUBE_LIST) + optional integer J labels."""
    J = result.values
    finite = [c for c in grid.free_cells() if math.isfinite(J[c[1], c[0]])]
    j_max = max((J[c[1], c[0]] for c in finite), default=1.0) or 1.0
    out = MarkerArray()
    out.markers.append(_delete_all(stamp))

    tiles = _marker("cost_to_go", 0, Marker.CUBE_LIST, stamp)
    tiles.scale.x = tiles.scale.y = res * 0.96
    tiles.scale.z = 0.02 * res
    for c in grid.free_cells():
        j = J[c[1], c[0]]
        tiles.points.append(cell_center(c, res, 0.01 * res))
        if math.isfinite(j):
            tiles.colors.append(rgba(*ramp_color(0.1 + 0.9 * j / j_max), 0.95))
        else:
            tiles.colors.append(rgba(0.75, 0.75, 0.75, 0.95))
    out.markers.append(tiles)

    if show_text:
        for i, c in enumerate(grid.free_cells()):
            j = J[c[1], c[0]]
            label = _marker("cost_labels", i, Marker.TEXT_VIEW_FACING, stamp)
            label.pose.position = cell_center(c, res, 0.08 * res)
            label.scale.z = 0.38 * res
            if math.isfinite(j):
                label.text = str(int(j))
                dark = (0.1 + 0.9 * j / j_max) > 0.55
                label.color = rgba(1, 1, 1) if dark else rgba(0.1, 0.1, 0.12)
            else:
                label.text = "inf"
                label.color = rgba(0.2, 0.2, 0.2)
            out.markers.append(label)

    for mid, (cell, color, text) in enumerate(((grid.start, "#2f9e44", "START"),
                                               (grid.goal, "#c92a2a", "GOAL"))):
        disc = _marker("endpoints", mid, Marker.CYLINDER, stamp)
        disc.pose.position = cell_center(cell, res, 0.03 * res)
        disc.scale.x = disc.scale.y = 0.9 * res
        disc.scale.z = 0.04 * res
        disc.color = hex_rgba(color, 0.9)
        out.markers.append(disc)
        tag = _marker("endpoints", 10 + mid, Marker.TEXT_VIEW_FACING, stamp)
        tag.pose.position = cell_center(cell, res, 0.9 * res)
        tag.scale.z = 0.7 * res
        tag.text = text
        tag.color = hex_rgba(color)
        out.markers.append(tag)
    return out


def policy_markers(policy: dict[Cell, Cell], res: float, stamp: Time,
                   color: str, ns: str) -> MarkerArray:
    """Arrow pi(s) -> best successor for every reachable state."""
    out = MarkerArray()
    out.markers.append(_delete_all(stamp))
    for i, (s, nxt) in enumerate(sorted(policy.items())):
        arrow = _marker(ns, i, Marker.ARROW, stamp)
        a, b = cell_center(s, res, 0.12 * res), cell_center(nxt, res, 0.12 * res)
        # Shorten to 60% of the move so arrows stay inside their cell.
        b = Point(x=a.x + 0.6 * (b.x - a.x), y=a.y + 0.6 * (b.y - a.y), z=a.z)
        a = Point(x=a.x - 0.2 * (b.x - a.x), y=a.y - 0.2 * (b.y - a.y), z=a.z)
        arrow.points = [a, b]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05 * res, 0.15 * res, 0.18 * res
        arrow.color = hex_rgba(color, 0.9)
        out.markers.append(arrow)
    return out


def text_panel(lines: list[str], res: float, width: int, height: int,
               stamp: Time) -> Marker:
    panel = _marker("stats", 0, Marker.TEXT_VIEW_FACING, stamp)
    panel.pose.position = Point(x=(width + 10.5) * res, y=height * 0.5 * res, z=0.1)
    panel.scale.z = 0.85 * res
    panel.color = rgba(1, 1, 1)
    panel.text = "\n".join(lines)
    return panel


def _delete_all(stamp: Time) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME
    m.header.stamp = stamp
    m.action = Marker.DELETEALL
    return m

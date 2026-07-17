# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Canonical reference path battery for the controller benchmark.

Every path starts at the origin facing +x in the robot frame. Each
:class:`PoseStamped` waypoint carries the path-tangent yaw at that point.
"""

from __future__ import annotations

import math

from dimos.memory2.vis.space.elements import Point, Polyline, Text
from dimos.memory2.vis.space.space import Space
from dimos.msgs.geometry_msgs.Point import Point as GeoPoint
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path

# Plot styling constants for the trajectory renderers below.
_REF_COLOR = "#cccccc"  # reference path = light gray
_EXE_COLOR = "#1f77b4"  # single-cohort executed path = blue
_START_COLOR = "#2ecc71"  # green start marker
_END_COLOR = "#e74c3c"  # red end marker
_REF_WIDTH = 0.06
_EXE_WIDTH = 0.03
_MARKER_RADIUS = 0.06


def _xy_to_path(executed_xy: list[tuple[float, float]]) -> Path:
    """Wrap (x, y) tuples in a nav_msgs.Path so memory2 Polyline can render them."""
    poses = [
        PoseStamped(
            position=Vector3(x, y, 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
        )
        for x, y in executed_xy
    ]
    return Path(poses=poses)


def _pose(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _path_from_xy(xs: list[float], ys: list[float]) -> Path:
    """Build a Path with tangent yaw at each waypoint."""
    n = len(xs)
    poses: list[PoseStamped] = []
    for i in range(n):
        if i < n - 1:
            dx = xs[i + 1] - xs[i]
            dy = ys[i + 1] - ys[i]
        else:
            dx = xs[i] - xs[i - 1]
            dy = ys[i] - ys[i - 1]
        yaw = math.atan2(dy, dx)
        poses.append(_pose(xs[i], ys[i], yaw))
    return Path(poses=poses)


# ---------------------------------------------------------------------------
# Path generators
# ---------------------------------------------------------------------------


def straight_line(length: float = 5.0, step: float = 0.05) -> Path:
    n = round(length / step)
    xs = [i * step for i in range(n + 1)]
    ys = [0.0] * (n + 1)
    return _path_from_xy(xs, ys)


def single_corner(leg_length: float = 2.0, angle_deg: float = 90.0, step: float = 0.05) -> Path:
    """Two straight legs meeting at one corner.

    Robot starts at origin going +x, drives ``leg_length``, turns by
    ``angle_deg`` (left positive), drives another ``leg_length``.
    """
    angle = math.radians(angle_deg)
    n_leg = round(leg_length / step)

    xs: list[float] = []
    ys: list[float] = []
    for i in range(n_leg + 1):
        xs.append(i * step)
        ys.append(0.0)
    corner_x, corner_y = xs[-1], ys[-1]
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    for i in range(1, n_leg + 1):
        d = i * step
        xs.append(corner_x + d * cos_a)
        ys.append(corner_y + d * sin_a)
    return _path_from_xy(xs, ys)


def circle(radius: float = 1.0, n_points: int = 100) -> Path:
    """Closed circle, robot starts at origin going +x, curves left.

    Center at (0, ``radius``). Last waypoint coincides with the first.
    """
    xs: list[float] = []
    ys: list[float] = []
    for i in range(n_points + 1):
        theta = 2.0 * math.pi * i / n_points
        xs.append(radius * math.sin(theta))
        ys.append(radius * (1.0 - math.cos(theta)))
    return _path_from_xy(xs, ys)


def figure_eight(loop_radius: float = 1.0, n_points: int = 200) -> Path:
    """Lemniscate of Gerono.

    x(t) = R sin(2t), y(t) = R sin(t), t in [0, 2pi].
    Starts at origin going +x.
    """
    xs: list[float] = []
    ys: list[float] = []
    for i in range(n_points + 1):
        t = 2.0 * math.pi * i / n_points
        xs.append(loop_radius * math.sin(2.0 * t))
        ys.append(loop_radius * math.sin(t))
    return _path_from_xy(xs, ys)


def slalom(
    cone_spacing: float = 1.0,
    lateral_offset: float = 0.5,
    n_cones: int = 5,
    points_per_cone: int = 20,
) -> Path:
    """Smooth slalom past ``n_cones`` cones, alternating sides.

    Cones sit at (i * cone_spacing, +/-lateral_offset). The path is a
    sinusoid that crosses the centerline between cones.
    """
    total_length = (n_cones + 1) * cone_spacing
    n = n_cones * points_per_cone + points_per_cone
    xs: list[float] = []
    ys: list[float] = []
    for i in range(n + 1):
        x = total_length * i / n
        y = lateral_offset * math.sin(math.pi * x / cone_spacing)
        xs.append(x)
        ys.append(y)
    return _path_from_xy(xs, ys)


def square(side: float = 2.0, step: float = 0.05) -> Path:
    """Closed square. Origin → +x → +y → -x → -y back to origin."""
    n_side = round(side / step)

    xs: list[float] = []
    ys: list[float] = []
    # leg 1: +x
    for i in range(n_side + 1):
        xs.append(i * step)
        ys.append(0.0)
    # leg 2: +y
    for i in range(1, n_side + 1):
        xs.append(side)
        ys.append(i * step)
    # leg 3: -x
    for i in range(1, n_side + 1):
        xs.append(side - i * step)
        ys.append(side)
    # leg 4: -y
    for i in range(1, n_side + 1):
        xs.append(0.0)
        ys.append(side - i * step)
    return _path_from_xy(xs, ys)


# ---------------------------------------------------------------------------
# Battery registry
# ---------------------------------------------------------------------------


def smooth_corner(
    leg_length: float = 2.0,
    angle_deg: float = 90.0,
    arc_radius: float = 0.5,
    step: float = 0.05,
) -> Path:
    """Two straight legs joined by a finite-radius arc — geometrically tunable.

    Unlike :func:`single_corner` (sharp 90° point with effectively zero radius),
    this path replaces the corner with an arc of radius ``arc_radius``. That
    gives a well-defined minimum curvature, so geometric tuning methods
    (lookahead = 2·R, etc.) can compute an actual answer instead of "infeasible".
    """
    angle = math.radians(angle_deg)
    n_leg = round(leg_length / step)
    cos_a, sin_a = math.cos(angle), math.sin(angle)

    xs: list[float] = [i * step for i in range(n_leg + 1)]
    ys: list[float] = [0.0] * (n_leg + 1)

    # Center of the arc: perpendicular to leg 1, at distance arc_radius
    cx = xs[-1]
    cy = ys[-1] + arc_radius  # arc center is to the left for a +90° turn
    # Arc starts at (xs[-1], ys[-1]) heading +x (angle from center = -π/2)
    # and ends heading at angle `angle_deg` (angle from center = -π/2 + angle)
    n_arc = max(2, round(abs(angle) * arc_radius / step))
    for i in range(1, n_arc + 1):
        theta_offset = (angle * i / n_arc) - math.pi / 2
        xs.append(cx + arc_radius * math.cos(theta_offset))
        ys.append(cy + arc_radius * math.sin(theta_offset))

    # Second leg: starts at end of arc, heads in direction `angle`
    end_x, end_y = xs[-1], ys[-1]
    for i in range(1, n_leg + 1):
        d = i * step
        xs.append(end_x + d * cos_a)
        ys.append(end_y + d * sin_a)
    return _path_from_xy(xs, ys)


def rounded_square(side: float = 2.0, arc_radius: float = 0.5, step: float = 0.05) -> Path:
    """Closed square with filleted corners — the curved counterpart to
    :func:`square`.

    Each 90° vertex is replaced by a quarter-circle arc of radius
    ``arc_radius``, so the path has bounded curvature (kappa = 1/arc_radius)
    everywhere instead of the infinite curvature of a sharp corner. Lets a
    trajectory tracker hold the geometry at speed without slowing to a stop
    at the vertices. Straight legs are shortened by ``arc_radius`` at each
    end to make room for the arcs. Same traversal sense as ``square``
    (origin → +x → +y → -x → -y), starting at the midpoint of the bottom
    edge so the start lies on a straight segment.
    """
    if arc_radius > side / 2.0:
        raise ValueError("arc_radius must be <= side/2")
    leg = side - 2.0 * arc_radius  # straight portion of each edge

    # Corner centers (inset by arc_radius from each true vertex).
    corners = [
        (side - arc_radius, arc_radius),  # bottom-right
        (side - arc_radius, side - arc_radius),  # top-right
        (arc_radius, side - arc_radius),  # top-left
        (arc_radius, arc_radius),  # bottom-left
    ]
    # Outgoing direction of each leg (after the corner before it).
    leg_dirs = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]

    xs: list[float] = []
    ys: list[float] = []
    n_arc = max(2, round((math.pi / 2.0) * arc_radius / step))

    # Start at the midpoint of the bottom edge, heading +x.
    px, py = side / 2.0, 0.0
    xs.append(px)
    ys.append(py)
    # First half-leg up to the bottom-right corner arc.
    half_leg = (side / 2.0) - arc_radius
    for i in range(1, max(1, round(half_leg / step)) + 1):
        xs.append(px + i * step)
        ys.append(0.0)

    for k in range(4):
        cx, cy = corners[k]
        # Quarter-circle arc around this corner: start angle points from the
        # center back toward the incoming leg, sweeping +90° (left turn).
        start_angle = math.atan2(ys[-1] - cy, xs[-1] - cx)
        for i in range(1, n_arc + 1):
            a = start_angle + (math.pi / 2.0) * i / n_arc
            xs.append(cx + arc_radius * math.cos(a))
            ys.append(cy + arc_radius * math.sin(a))
        # Straight leg out to the next corner (or back to start on the last).
        dx, dy = leg_dirs[(k + 1) % 4]
        ex, ey = xs[-1], ys[-1]
        run = leg if k < 3 else half_leg
        for i in range(1, max(1, round(run / step)) + 1):
            xs.append(ex + i * step * dx)
            ys.append(ey + i * step * dy)
    return _path_from_xy(xs, ys)


def sidestep_1m(distance: float = 1.0, n_points: int = 20) -> Path:
    """End up ``distance`` m to the left of start, facing forward.

    Path waypoints sit on a straight line from (0, 0) to (0, distance), all
    with yaw=0. Path-followers will interpret this as a goal 90° to the
    left — they typically rotate to face it, drive there, then rotate back
    to yaw=0 for arrival. Tests off-axis-goal handling more than true
    lateral velocity (Go2 has minimal native vy authority over WebRTC).
    """
    poses: list[PoseStamped] = []
    for i in range(n_points + 1):
        a = i / n_points
        poses.append(_pose(0.0, a * distance, 0.0))
    return Path(poses=poses)


def short_battery() -> dict[str, Path]:
    """3-path battery for the hardware setpoint benchmark.

    Tighter than `default_battery()` — only enough to get a 6-controller
    comparison done in 15-20 min of robot time. Exercises:
      - ``straight_2m``: trivial forward driving (best-case test).
      - ``corner_90``: in-path heading change (steering authority test).
      - ``sidestep_1m``: off-axis goal (turn-then-drive test).
    """
    return {
        "straight_2m": straight_line(length=2.0),
        "corner_90": single_corner(leg_length=2.0, angle_deg=90.0),
        "sidestep_1m": sidestep_1m(),
    }


def default_battery() -> dict[str, Path]:
    """All canonical paths used for the standard benchmark report."""
    return {
        "straight_2m": straight_line(length=2.0),
        "straight_5m": straight_line(length=5.0),
        "corner_90": single_corner(leg_length=2.0, angle_deg=90.0),
        "smooth_corner_R0.5": smooth_corner(leg_length=2.0, angle_deg=90.0, arc_radius=0.5),
        "sidestep_1m": sidestep_1m(),
        "circle_R0.5": circle(radius=0.5),
        "circle_R1.0": circle(radius=1.0),
        "circle_R2.0": circle(radius=2.0),
        "figure_eight_R1.0": figure_eight(loop_radius=1.0),
        "slalom_5cones": slalom(),
        "square_2m": square(side=2.0),
    }


# ---------------------------------------------------------------------------
# SVG rendering (for visual fixtures)
# ---------------------------------------------------------------------------


# Cohort palette for multi_trajectory_to_svg overlays. 10 distinct colors so
# the current cohort matrix (10 entries) doesn't have any color collisions.
_COHORT_COLORS = (
    "#1f77b4",  # blue
    "#d62728",  # red
    "#2ca02c",  # green
    "#ff7f0e",  # orange
    "#9467bd",  # purple
    "#17becf",  # cyan
    "#e377c2",  # pink
    "#8c564b",  # brown
    "#bcbd22",  # olive
    "#000000",  # black
)


def path_to_svg(path: Path, size_px: int = 400, margin_px: int = 20) -> str:
    """Render a Path as an SVG polyline via memory2.vis.space.

    ``size_px`` / ``margin_px`` are kept for API compatibility but ignored;
    Space picks its own dimensions from world-space content bounds.
    """
    if not path.poses:
        return Space().to_svg()

    sp = Space()
    sp.add(Polyline(msg=path, color="#000000", width=_REF_WIDTH))
    sp.add(Point(msg=path.poses[0], color=_START_COLOR, radius=_MARKER_RADIUS))
    sp.add(Point(msg=path.poses[-1], color=_END_COLOR, radius=_MARKER_RADIUS))
    return sp.to_svg()


def trajectory_to_svg(
    reference: Path,
    executed_xy: list[tuple[float, float]],
    size_px: int = 500,
    margin_px: int = 20,
) -> str:
    """Reference path (gray) + executed trajectory (blue), via memory2.vis.space."""
    if not reference.poses or not executed_xy:
        return Space().to_svg()

    sp = Space()
    sp.add(Polyline(msg=reference, color=_REF_COLOR, width=_REF_WIDTH))
    sp.add(Polyline(msg=_xy_to_path(executed_xy), color=_EXE_COLOR, width=_EXE_WIDTH))
    sx, sy = executed_xy[0]
    ex, ey = executed_xy[-1]
    sp.add(Point(msg=GeoPoint(sx, sy, 0.0), color=_START_COLOR, radius=_MARKER_RADIUS))
    sp.add(Point(msg=GeoPoint(ex, ey, 0.0), color=_END_COLOR, radius=_MARKER_RADIUS))
    return sp.to_svg()


def multi_trajectory_to_svg(
    reference: Path,
    cohorts: dict[str, list[tuple[float, float]]],
    size_px: int = 600,
    margin_px: int = 30,
    title: str | None = None,
) -> str:
    """Reference + multiple executed trajectories overlaid, via memory2.vis.space.

    Each cohort gets a distinct color from ``_COHORT_COLORS`` (10 unique
    entries; no collisions for the current cohort matrix). A small dot at
    each cohort's start position helps disambiguate when overlapping lines
    converge. The legend is emitted as ``Polyline`` stubs + ``Text`` labels
    placed in world space below the plot bounds, so it sits inside the
    auto-fit viewBox alongside the trajectories. Axes / grid / tick labels
    are drawn by memory2's Space renderer (`show_axes=True`).
    """
    if not reference.poses:
        return Space().to_svg()

    sp = Space()
    sp.add(Polyline(msg=reference, color=_REF_COLOR, width=_REF_WIDTH * 1.4))

    # Establish bounds for legend placement (below the path) and title (above).
    all_ys = [p.position.y for p in reference.poses]
    all_xs = [p.position.x for p in reference.poses]
    for xy in cohorts.values():
        all_ys.extend(y for _, y in xy)
        all_xs.extend(x for x, _ in xy)
    y_min = min(all_ys) if all_ys else 0.0
    y_max = max(all_ys) if all_ys else 1.0
    x_min = min(all_xs) if all_xs else 0.0

    if title:
        sp.add(Text(position=(x_min, y_max + 0.3, 0.0), text=title, font_size=14.0))

    # Cohort polylines + start dots.
    for i, (name, xy) in enumerate(cohorts.items()):
        color = _COHORT_COLORS[i % len(_COHORT_COLORS)]
        if xy:
            sp.add(Polyline(msg=_xy_to_path(xy), color=color, width=_EXE_WIDTH))
            sx, sy = xy[0]
            sp.add(Point(msg=GeoPoint(sx, sy, 0.0), color=color, radius=_MARKER_RADIUS * 0.7))
        # Legend row (world coords below the plot).
        ly = y_min - 0.4 - i * 0.25
        sp.add(
            Polyline(
                msg=Path(
                    poses=[
                        PoseStamped(
                            position=Vector3(x_min, ly, 0.0),
                            orientation=Quaternion.from_euler(Vector3(0, 0, 0)),
                        ),
                        PoseStamped(
                            position=Vector3(x_min + 0.4, ly, 0.0),
                            orientation=Quaternion.from_euler(Vector3(0, 0, 0)),
                        ),
                    ]
                ),
                color=color,
                width=_EXE_WIDTH,
            )
        )
        sp.add(Text(position=(x_min + 0.5, ly, 0.0), text=name, font_size=12.0, color=color))

    return sp.to_svg()


__all__ = [
    "circle",
    "default_battery",
    "figure_eight",
    "multi_trajectory_to_svg",
    "path_to_svg",
    "rounded_square",
    "single_corner",
    "slalom",
    "smooth_corner",
    "square",
    "straight_line",
    "trajectory_to_svg",
]

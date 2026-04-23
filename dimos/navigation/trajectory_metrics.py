# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on the "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Planar trajectory tracking metrics (P0-2).

This module is the **normative** definition for telemetry, plots, and tests that
speak about along-track error, cross-track error, divergence, and commanded
planar speed. Human-readable notes for issue 921 live under ``__dev/feat/921/``
until the review PR; keep this file aligned with those notes.

**Frames**

- Path geometry and horizontal positions use the same **map / plan** horizontal
  frame as ``Path`` and ``PathDistancer`` (x forward, y left in that frame).
- ``commanded_planar_speed`` uses **base-link** linear ``Twist`` components
  (``linear.x`` forward, ``linear.y`` lateral) as emitted on ``cmd_vel`` for
  omnidirectional bases.

**Along-track and cross-track (reference pose)**

Given a reference pose ``(x_ref, y_ref, yaw_ref)`` and measured
``(x_meas, y_meas)``, express the position error in a frame aligned with the
reference heading (x = forward along reference, y = left):

- **Along-track error** ``e_at`` (m): signed component of ``(p_meas - p_ref)``
  on the reference forward axis. Positive means the measured point is **ahead**
  of the reference in the reference forward direction.
- **Cross-track error** ``e_ct`` (m): signed component on the left axis.
  Positive means the measured point is **to the left** of the reference when
  facing along ``yaw_ref``.
- **Heading error** ``e_psi`` (rad): ``angle_diff(yaw_meas, yaw_ref)`` so it
  matches ``dimos.utils.trigonometry.angle_diff`` conventions used in
  navigation.

**Divergence from target**

- **Planar position divergence** (m): ``hypot(e_at, e_ct)``. This is the
  default scalar for **speed vs planar position error** style plots when a
  full reference pose exists.
- **Planar pose divergence** (mixed units unless scaled): ``sqrt(e_at**2 +
  e_ct**2 + (yaw_weight_rad_to_m * e_psi)**2)``. ``yaw_weight_rad_to_m`` maps
  heading into an equivalent meter scale for a single RMS; set to ``0`` to
  recover position-only divergence.

**Polyline reference (spatial path only)**

For a piecewise-linear path, the closest point **on segments** (not only on
vertices) defines a foot point, tangent, and arc length ``s`` from the first
vertex to the foot. **Signed cross-track** uses the left-of-tangent rule
above. **Along-track error vs a time-parameterized reference** needs a scalar
``s_ref`` from the trajectory (arc length along the same polyline): ``e_at =
s_meas - s_ref``. Pure geometry without ``s_ref`` does not define longitudinal
error relative to a moving target; logging and controllers should supply
``s_ref`` once trajectories are timed.

**Commanded speed for plots**

At each control tick, sample ``commanded_planar_speed(cmd_twist)`` using the
``Twist`` actually published. This is the magnitude of the horizontal command
in the base frame, **not** a finite difference of odometry.

**Context-dependent tolerance**

Obstacle clearance and corridor width are **planner-side** inputs. Tracking
tolerances may be tightened in tight clearance and relaxed in open space. This
module exposes ``TrackingTolerance`` plus ``scale_tolerance_by_clearance`` as a
**deterministic example** of how clearance can modulate tolerances; production
planners may replace that mapping while keeping the same error definitions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.utils.trigonometry import angle_diff


def pose_errors_vs_reference(
    x_meas: float,
    y_meas: float,
    yaw_meas: float,
    x_ref: float,
    y_ref: float,
    yaw_ref: float,
) -> tuple[float, float, float]:
    """Along-track (m), cross-track (m), heading error (rad) vs a reference pose."""
    dx = x_meas - x_ref
    dy = y_meas - y_ref
    c = math.cos(yaw_ref)
    s = math.sin(yaw_ref)
    e_at = dx * c + dy * s
    e_ct = -dx * s + dy * c
    e_psi = angle_diff(yaw_meas, yaw_ref)
    return (e_at, e_ct, e_psi)


def planar_position_divergence(e_at: float, e_ct: float) -> float:
    """Euclidean planar position error (m) from along- and cross-track components."""
    return float(math.hypot(e_at, e_ct))


def planar_pose_divergence(
    e_at: float,
    e_ct: float,
    e_psi: float,
    *,
    yaw_weight_rad_to_m: float = 0.0,
) -> float:
    """RMS of planar position and optional yaw, with yaw scaled to meters."""
    w = yaw_weight_rad_to_m
    return float(math.sqrt(e_at * e_at + e_ct * e_ct + (w * e_psi) * (w * e_psi)))


def commanded_planar_speed(cmd: Twist) -> float:
    """Horizontal speed (m/s) from a commanded ``Twist`` in the base link frame."""
    return float(math.hypot(cmd.linear.x, cmd.linear.y))


@dataclass(frozen=True)
class PolylineProjection:
    """Closest point on a polyline and path coordinates at the foot."""

    foot_xy: tuple[float, float]
    segment_start_index: int
    tangent_yaw: float
    s_along_path_m: float
    signed_cross_track_m: float


def project_to_polyline(
    x: float, y: float, polyline_xy: NDArray[np.float64]
) -> PolylineProjection:
    """Project a point onto a ``(N, 2)`` polyline in the map frame."""
    if polyline_xy.ndim != 2 or polyline_xy.shape[1] != 2:
        raise ValueError("polyline_xy must have shape (N, 2)")
    n = polyline_xy.shape[0]
    if n == 0:
        raise ValueError("polyline_xy must be non-empty")
    if n == 1:
        fx = float(polyline_xy[0, 0])
        fy = float(polyline_xy[0, 1])
        lat = float(math.hypot(x - fx, y - fy))
        return PolylineProjection(
            (fx, fy),
            0,
            0.0,
            0.0,
            lat,
        )

    p = np.array([x, y], dtype=np.float64)
    prefix_s = np.zeros(n, dtype=np.float64)
    for i in range(1, n):
        prefix_s[i] = prefix_s[i - 1] + float(
            np.linalg.norm(polyline_xy[i] - polyline_xy[i - 1])
        )

    best_d2 = math.inf
    best_foot = polyline_xy[0].copy()
    best_seg = 0
    best_t = 0.0
    best_seg_len = 0.0

    for i in range(n - 1):
        a = polyline_xy[i]
        b = polyline_xy[i + 1]
        ab = b - a
        seg_len2 = float(ab @ ab)
        if seg_len2 <= 1e-18:
            foot = a
            t_param = 0.0
            seg_len = 0.0
        else:
            t_param = float(np.clip(((p - a) @ ab) / seg_len2, 0.0, 1.0))
            foot = a + t_param * ab
            seg_len = math.sqrt(seg_len2)
        d2 = float((p - foot) @ (p - foot))
        if d2 < best_d2:
            best_d2 = d2
            best_foot = foot
            best_seg = i
            best_t = t_param
            best_seg_len = seg_len

    s_along = float(prefix_s[best_seg] + best_t * best_seg_len)

    a = polyline_xy[best_seg]
    b = polyline_xy[best_seg + 1]
    ab = b - a
    seg_len = float(np.linalg.norm(ab))
    if seg_len < 1e-9:
        yaw = 0.0
        tx, ty = 1.0, 0.0
    else:
        tx, ty = float(ab[0] / seg_len), float(ab[1] / seg_len)
        yaw = math.atan2(ab[1], ab[0])
    nx, ny = -ty, tx
    vx = x - float(best_foot[0])
    vy = y - float(best_foot[1])
    signed_ct = vx * nx + vy * ny
    return PolylineProjection(
        (float(best_foot[0]), float(best_foot[1])),
        best_seg,
        yaw,
        s_along,
        float(signed_ct),
    )


def along_track_progress_error(s_meas_m: float, s_ref_m: float) -> float:
    """Signed arc-length error (m): measured minus reference along the same path."""
    return float(s_meas_m - s_ref_m)


@dataclass(frozen=True)
class TrackingTolerance:
    """Absolute envelopes for tracking errors (planner or policy may supply these)."""

    along_track_m: float
    cross_track_m: float
    heading_rad: float

    def satisfied(
        self,
        e_at: float,
        e_ct: float,
        e_psi: float,
    ) -> bool:
        return (
            abs(e_at) <= self.along_track_m
            and abs(e_ct) <= self.cross_track_m
            and abs(e_psi) <= self.heading_rad
        )


def scale_tolerance_by_clearance(
    base: TrackingTolerance,
    clearance_m: float,
    *,
    tight_clearance_m: float = 0.35,
    loose_clearance_m: float = 2.0,
    tight_scale: float = 0.55,
    loose_scale: float = 1.15,
) -> TrackingTolerance:
    """Piecewise-linear scale of tolerances vs a scalar clearance (example policy).

    When ``clearance_m`` is small, tolerances shrink (tighter tracking in narrow
    space). When clearance is large, tolerances grow modestly. Values are
    illustrative; planners may substitute their own mapping.
    """
    c = clearance_m
    if c <= tight_clearance_m:
        k = tight_scale
    elif c >= loose_clearance_m:
        k = loose_scale
    else:
        u = (c - tight_clearance_m) / (loose_clearance_m - tight_clearance_m)
        k = tight_scale + u * (loose_scale - tight_scale)
    return TrackingTolerance(
        along_track_m=base.along_track_m * k,
        cross_track_m=base.cross_track_m * k,
        heading_rad=base.heading_rad * k,
    )

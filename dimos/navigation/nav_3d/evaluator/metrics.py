# Copyright 2026 Dimensional Inc.
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

"""Scoring: SPL, the gates, and the walked reference, all on the pipeline's map."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.voxel_keys import (
    cylinder_offsets,
    key_centers,
    keys_contain,
    offset_deltas,
    voxel_keys,
)

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.config import EvalConfig
    from dimos.navigation.nav_3d.evaluator.recording import Trajectory


def path_length(waypoints: NDArray[np.float32]) -> float:
    if len(waypoints) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(waypoints, axis=0), axis=1).sum())


def arc_lengths(points: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.float64]:
    """Cumulative 3D arc length at each point, starting at zero."""
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(steps)]).astype(np.float64)


def goal_reached(
    waypoints: NDArray[np.float32], goal: tuple[float, float, float], tolerance: float
) -> bool:
    return bool(np.linalg.norm(waypoints[-1] - np.asarray(goal, dtype=np.float32)) <= tolerance)


# Clearance margins are only measured out to this horizontal distance from
# the body surface. Anything farther reports the cap.
MARGIN_CAP_M = 0.3

# Floor on any length used as a divisor or a direction.
MIN_LENGTH_M = 1e-6


def densify(points: NDArray[np.float32], step: float) -> NDArray[np.float32]:
    """Resample a polyline so consecutive samples are at most step apart."""
    if len(points) < 2:
        return points.astype(np.float32)
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    n = np.maximum(np.ceil(seg / step).astype(np.int64), 1)
    idx = np.repeat(np.arange(len(n)), n)
    starts = np.concatenate([[0], np.cumsum(n)[:-1]])
    t = ((np.arange(n.sum()) - starts[idx] + 1) / n[idx])[:, None]
    body = points[idx] * (1 - t) + points[idx + 1] * t
    dense: NDArray[np.float32] = np.concatenate([points[:1], body]).astype(np.float32)
    return dense


def chord_directions(samples: NDArray[np.float32], span: float) -> NDArray[np.float64]:
    """Heading from the foot-to-foot chord, steadier than the local tangent."""
    if len(samples) < 2:
        return np.tile(np.array([1.0, 0.0, 0.0]), (len(samples), 1))
    pts = samples.astype(np.float64)
    arc = arc_lengths(pts)
    half = span / 2.0
    back_arc = np.clip(arc - half, arc[0], arc[-1])
    front_arc = np.clip(arc + half, arc[0], arc[-1])
    back = np.column_stack([np.interp(back_arc, arc, pts[:, c]) for c in range(3)])
    front = np.column_stack([np.interp(front_arc, arc, pts[:, c]) for c in range(3)])
    fwd = front - back
    norm = np.linalg.norm(fwd, axis=1, keepdims=True)
    # A path of zero arc length has no heading, so fall back to a valid frame
    # rather than handing a singular basis to the caller.
    fwd = np.where(norm > MIN_LENGTH_M, fwd, np.array([1.0, 0.0, 0.0]))
    unit: NDArray[np.float64] = fwd / np.maximum(
        np.linalg.norm(fwd, axis=1, keepdims=True), MIN_LENGTH_M
    )
    return unit


def body_frames(
    samples: NDArray[np.float32], robot_length: float
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Per-sample body axes: forward along the chord, lateral flat, up with it."""
    fwd = chord_directions(samples, robot_length)
    lateral = np.cross(np.array([0.0, 0.0, 1.0]), fwd)
    ln = np.linalg.norm(lateral, axis=1, keepdims=True)
    lateral = np.where(
        ln > MIN_LENGTH_M, lateral / np.maximum(ln, MIN_LENGTH_M), np.array([0.0, 1.0, 0.0])
    )
    up = np.cross(fwd, lateral)
    return fwd, lateral, up


@dataclass
class BodyBox:
    """The body box at one path sample, in the pose the gate tested it."""

    center: tuple[float, float, float]
    # Body axes as a quaternion, xyzw.
    rotation: tuple[float, float, float, float]


def body_box_half_extents(cfg: EvalConfig) -> tuple[float, float, float]:
    """Half extents of the gate's body box, along forward, lateral and up."""
    return (
        cfg.robot_length / 2.0,
        cfg.robot_width / 2.0,
        (cfg.body_clearance - cfg.ground_margin) / 2.0,
    )


def _body_boxes(
    centers: NDArray[np.float64],
    fwd: NDArray[np.float64],
    lateral: NDArray[np.float64],
    up: NDArray[np.float64],
    idx: NDArray[np.int64],
) -> list[BodyBox]:
    """The boxes at the given samples, for drawing where a path hit something."""
    # Lazy: scipy is a heavy import for a module the runner loads per case.
    from scipy.spatial.transform import Rotation

    if len(idx) == 0:
        return []
    axes = np.stack([fwd[idx], lateral[idx], up[idx]], axis=-1)
    quats = Rotation.from_matrix(axes).as_quat()
    return [
        BodyBox(
            (float(c[0]), float(c[1]), float(c[2])),
            (float(q[0]), float(q[1]), float(q[2]), float(q[3])),
        )
        for c, q in zip(centers[idx], quats, strict=True)
    ]


@dataclass
class GateResult:
    """Collision check of a path against a voxel map key set."""

    valid: bool
    # Body poses that collided, so a viewer draws the robot where it hit.
    collision_boxes: list[BodyBox]
    # Distance from the body surface to the nearest occupied voxel in the band,
    # minimized along the path. Negative is penetration, capped at MARGIN_CAP_M.
    min_clearance_m: float


def _voxel_hits(
    samples: NDArray[np.float32],
    voxel_size: float,
    offsets: NDArray[np.int64],
    map_keys: NDArray[np.int64],
) -> tuple[NDArray[np.bool_], NDArray[np.int64], NDArray[np.int64]]:
    """Per-sample membership against map_keys, over each sample's offset cylinder.

    hit has shape (len(samples), len(offsets)); base_keys[s] + deltas[o]
    recovers the key behind a True at [s, o].
    """
    base = voxel_keys(samples, voxel_size)
    deltas = offset_deltas(offsets)
    unique_base, inverse = np.unique(base, return_inverse=True)
    hit = keys_contain(map_keys, (unique_base[:, None] + deltas[None, :]).ravel()).reshape(
        len(unique_base), len(deltas)
    )
    return np.asarray(hit[inverse]), base, deltas


def check_path(
    waypoints: NDArray[np.float32], map_keys: NDArray[np.int64], cfg: EvalConfig
) -> GateResult:
    """Sweep the body box along the path. Only the trunk band collides."""
    voxel_size = cfg.voxel_size
    samples = densify(waypoints, voxel_size / 2)
    fwd, lateral, up = body_frames(samples, cfg.robot_length)
    half_len, half_wid, half_band = body_box_half_extents(cfg)
    mid_h = (cfg.ground_margin + cfg.body_clearance) / 2.0
    centers = samples + mid_h * up
    pad = MARGIN_CAP_M + voxel_size
    # The candidate cylinder only has to reach the band, so bound it by the
    # path's own steepest pitch instead of assuming any orientation.
    sin_p = float(np.abs(fwd[:, 2]).max())
    cos_p = float(np.sqrt(max(1.0 - sin_p * sin_p, 0.0)))
    reach_z = (half_len + pad) * sin_p + half_band * cos_p + voxel_size
    # Union with the level window: a steep segment lowers the pitched window,
    # which would otherwise let a flat stretch of the same path hide a collision.
    offsets = cylinder_offsets(
        float(np.hypot(half_len, half_wid)) + pad + (mid_h + half_band) * sin_p,
        min(mid_h * cos_p - reach_z, mid_h - half_band - voxel_size),
        max(mid_h * cos_p + reach_z, mid_h + half_band + voxel_size),
        voxel_size,
    )
    hit, base, deltas = _voxel_hits(samples, voxel_size, offsets, map_keys)
    s_idx, o_idx = np.nonzero(hit)
    if len(s_idx) == 0:
        return GateResult(True, [], MARGIN_CAP_M)
    delta = key_centers(base[s_idx] + deltas[o_idx], voxel_size) - centers[s_idx]
    # The band decides membership on its own, so drop the candidates outside it
    # before paying for the footprint distance.
    vertical = (delta * up[s_idx]).sum(1)
    in_band = np.abs(vertical) <= half_band
    if not in_band.any():
        return GateResult(True, [], MARGIN_CAP_M)
    delta, s_idx = delta[in_band], s_idx[in_band]
    along = (delta * fwd[s_idx]).sum(1)
    across = (delta * lateral[s_idx]).sum(1)
    # Signed distance to the oriented footprint rectangle, negative inside.
    qx = np.abs(along) - half_len
    qy = np.abs(across) - half_wid
    sdf = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0)) + np.minimum(np.maximum(qx, qy), 0.0)
    colliding = np.unique(s_idx[sdf <= 0.0])
    return GateResult(
        valid=len(colliding) == 0,
        collision_boxes=_body_boxes(centers, fwd, lateral, up, colliding),
        min_clearance_m=min(float(sdf.min()), MARGIN_CAP_M),
    )


@dataclass
class SupportResult:
    """Ground check: every path sample must stand on mapped occupancy."""

    valid: bool
    unsupported_points: NDArray[np.float32]


def check_support(
    waypoints: NDArray[np.float32], support_keys: NDArray[np.int64], cfg: EvalConfig
) -> SupportResult:
    """Require mapped ground beneath every path sample."""
    voxel_size = cfg.voxel_size
    samples = densify(waypoints, voxel_size)
    offsets = cylinder_offsets(cfg.support_radius_m, -cfg.support_depth_m, voxel_size, voxel_size)
    hit, _, _ = _voxel_hits(samples, voxel_size, offsets, support_keys)
    supported = hit.any(axis=1)
    return SupportResult(bool(supported.all()), samples[~supported])


@dataclass
class KinematicsResult:
    """Steppability check of the path profile."""

    valid: bool
    violation_points: NDArray[np.float32]


def _resample(waypoints: NDArray[np.float32], spacing: float) -> NDArray[np.float32]:
    """Points every spacing meters of 3D arc length along the polyline."""
    arc = arc_lengths(waypoints)
    if arc[-1] <= spacing:
        return waypoints[[0, -1]]
    s = np.append(np.arange(0.0, arc[-1], spacing), arc[-1])
    return np.stack([np.interp(s, arc, waypoints[:, i]) for i in range(3)], axis=1).astype(
        np.float32
    )


def check_kinematics(waypoints: NDArray[np.float32], cfg: EvalConfig) -> KinematicsResult:
    """Reject paths that climb steeper than the robot can."""
    if len(waypoints) < 2:
        return KinematicsResult(True, waypoints[:0])
    profile = _resample(waypoints, cfg.kinematic_window_m)
    d = np.diff(profile, axis=0)
    rise = np.abs(d[:, 2])
    run = np.linalg.norm(d[:, :2], axis=1)
    bad = rise > np.maximum(run * cfg.max_slope, cfg.max_step_m)
    return KinematicsResult(not bad.any(), profile[1:][bad])


@dataclass
class Reference:
    """Demonstrated route between a case's endpoints."""

    length: float
    snapped: bool
    # When the robot stood at the start about to walk the route. Inf when
    # the endpoints are off the trajectory or no causal pair exists.
    start_ts: float
    # True when the goal was visited before the chosen start visit, so a
    # planner at start_ts targets a place the robot has already been.
    causal: bool


@dataclass
class _Visits:
    """Poses near each endpoint, with the walked length of every pairing."""

    near_s: NDArray[np.int64]
    near_g: NDArray[np.int64]
    totals: NDArray[np.float64]


def _visits(
    trajectory: Trajectory,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    cfg: EvalConfig,
) -> _Visits | None:
    """None when either endpoint is farther than visit_radius_m from the trajectory."""
    foot = trajectory.foot(cfg.robot_height)
    ds = np.linalg.norm(foot - np.asarray(start, dtype=np.float32), axis=1)
    dg = np.linalg.norm(foot - np.asarray(goal, dtype=np.float32), axis=1)
    if ds.min() > cfg.visit_radius_m or dg.min() > cfg.visit_radius_m:
        return None
    arcs = trajectory.arc_lengths()
    near_s = np.flatnonzero(ds <= cfg.visit_radius_m)
    near_g = np.flatnonzero(dg <= cfg.visit_radius_m)
    totals = (
        np.abs(arcs[near_s][:, None] - arcs[near_g][None, :])
        + ds[near_s][:, None]
        + dg[near_g][None, :]
    )
    return _Visits(near_s, near_g, totals)


def reference_length(
    trajectory: Trajectory,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    cfg: EvalConfig,
) -> Reference:
    """Shortest walked length between the endpoints, preferring causal pairings."""
    visits = _visits(trajectory, start, goal, cfg)
    if visits is None:
        straight = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
        return Reference(straight, False, float("inf"), False)
    totals = visits.totals
    backward = trajectory.ts[visits.near_g][None, :] <= trajectory.ts[visits.near_s][:, None]
    causal = bool(backward.any())
    if causal:
        totals = np.where(backward, totals, np.inf)
    best = np.unravel_index(totals.argmin(), totals.shape)
    i = int(visits.near_s[best[0]])
    start_ts = float(trajectory.ts[i]) if causal else float("inf")
    return Reference(max(float(totals[best]), MIN_LENGTH_M), True, start_ts, causal)


def spl(success: bool, l_ref: float, p_len: float) -> float:
    if not success:
        return 0.0
    return l_ref / max(p_len, l_ref, MIN_LENGTH_M)


def timing_stats(samples_ms: list[float]) -> dict[str, float]:
    if not samples_ms:
        return {"p50": 0.0, "p95": 0.0, "max": 0.0}
    arr = np.asarray(samples_ms)
    return {
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(arr.max()),
    }

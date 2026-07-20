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

"""Scoring for the nav-3d evaluator: SPL, the path validity gate, references."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.final_map import (
    cylinder_offsets,
    densify,
    key_centers,
    keys_contain,
    offset_keys,
)

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.recording import Trajectory


def path_length(waypoints: NDArray[np.float32]) -> float:
    if len(waypoints) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(waypoints, axis=0), axis=1).sum())


def goal_reached(
    waypoints: NDArray[np.float32], goal: tuple[float, float, float], tolerance: float
) -> bool:
    return bool(np.linalg.norm(waypoints[-1] - np.asarray(goal, dtype=np.float32)) <= tolerance)


# Clearance margins are only measured out to this horizontal distance from
# the body surface. Anything farther reports the cap.
MARGIN_CAP_M = 0.3


@dataclass
class GateResult:
    """Collision check of a path against an obstacle key set."""

    valid: bool
    collision_points: NDArray[np.float32]
    # Horizontal distance from the body surface to the nearest obstacle in
    # the gate's z band, minimized along the path. Negative is penetration
    # depth, capped at MARGIN_CAP_M when nothing is near. Gives a smooth
    # how-close-to-flipping signal next to the binary verdict.
    min_clearance_m: float


def chord_directions(samples: NDArray[np.float32], span: float) -> NDArray[np.float64]:
    """Unit direction from a point span/2 behind each sample to a point span/2
    ahead, measured along the path.

    This is the rigid body's heading: the chord from the rear feet to the front
    feet, not the local tangent between two points under the body center, which
    on stepped terrain flips between flat treads and vertical risers.
    """
    if len(samples) < 2:
        return np.tile(np.array([1.0, 0.0, 0.0]), (len(samples), 1))
    pts = samples.astype(np.float64)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    half = span / 2.0
    back_arc = np.clip(arc - half, arc[0], arc[-1])
    front_arc = np.clip(arc + half, arc[0], arc[-1])
    back = np.column_stack([np.interp(back_arc, arc, pts[:, c]) for c in range(3)])
    front = np.column_stack([np.interp(front_arc, arc, pts[:, c]) for c in range(3)])
    fwd = front - back
    return fwd / np.maximum(np.linalg.norm(fwd, axis=1, keepdims=True), 1e-9)


def body_frames(
    samples: NDArray[np.float32], robot_length: float
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Per-sample body axes: forward along the robot-length chord, lateral
    horizontal, up tilted with the slope, so the box yaws and pitches with the
    body rather than the terrain right under its center."""
    fwd = chord_directions(samples, robot_length)
    lateral = np.cross(np.array([0.0, 0.0, 1.0]), fwd)
    ln = np.linalg.norm(lateral, axis=1, keepdims=True)
    lateral = np.where(ln > 1e-6, lateral / np.maximum(ln, 1e-9), np.array([0.0, 1.0, 0.0]))
    up = np.cross(fwd, lateral)
    return fwd, lateral, up


def check_path(
    waypoints: NDArray[np.float32],
    obstacle_keys: NDArray[np.int64],
    voxel_size: float,
    robot_length: float,
    robot_width: float,
    ground_margin: float,
    body_clearance: float,
) -> GateResult:
    """Sweep the robot body box along foot-level waypoints against obstacles.

    At each sample the body is a box of the robot's length and width, centered
    over the path point and rotated in place: yawed and pitched along the
    robot-length chord, so it stays over the path rather than sliding onto the
    chord. Its vertical span is the ground_margin to body_clearance band up the
    tilted body axis, so the legs and the ground below never count, only the
    elevated body. Candidate voxels come from a padded voxelized cylinder that
    covers the box at any orientation and are tested against the exact box.
    """
    samples = densify(waypoints, voxel_size / 2)
    fwd, lateral, up = body_frames(samples, robot_length)
    half_len = robot_length / 2.0
    half_wid = robot_width / 2.0
    half_band = (body_clearance - ground_margin) / 2.0
    mid = np.array([0.0, 0.0, (ground_margin + body_clearance) / 2.0])
    circ = float(np.hypot(half_len, half_wid))
    offsets = cylinder_offsets(
        circ + MARGIN_CAP_M + voxel_size,
        -(half_len + MARGIN_CAP_M + voxel_size),
        body_clearance + half_len + MARGIN_CAP_M + voxel_size,
        voxel_size,
    )
    keys = offset_keys(samples, offsets, voxel_size)
    candidate = keys_contain(obstacle_keys, keys.ravel()).reshape(keys.shape)
    s_idx, o_idx = np.nonzero(candidate)
    if len(s_idx) == 0:
        return GateResult(valid=True, collision_points=samples[:0], min_clearance_m=MARGIN_CAP_M)
    # Offset from the box center, which sits mid-band directly over the sample.
    delta = key_centers(keys[s_idx, o_idx], voxel_size) - samples[s_idx] - mid
    along = (delta * fwd[s_idx]).sum(1)
    across = (delta * lateral[s_idx]).sum(1)
    vertical = (delta * up[s_idx]).sum(1)
    # Signed distance to the oriented footprint rectangle, negative inside.
    qx = np.abs(along) - half_len
    qy = np.abs(across) - half_wid
    sdf = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0)) + np.minimum(np.maximum(qx, qy), 0.0)
    in_band = np.abs(vertical) <= half_band
    exact = in_band & (sdf <= 0.0)
    clearance = float(sdf[in_band].min()) if in_band.any() else MARGIN_CAP_M
    colliding = np.unique(s_idx[exact])
    return GateResult(
        valid=len(colliding) == 0,
        collision_points=samples[colliding],
        min_clearance_m=min(clearance, MARGIN_CAP_M),
    )


@dataclass
class SupportResult:
    """Ground check: every path sample must stand on mapped occupancy."""

    valid: bool
    unsupported_points: NDArray[np.float32]


def check_support(
    waypoints: NDArray[np.float32],
    support_keys: NDArray[np.int64],
    voxel_size: float,
    radius: float,
    depth: float,
) -> SupportResult:
    """Require occupied voxels beneath every path sample.

    A path across a void collides with nothing, so the collision gate alone
    cannot catch fabricated bridges. Each densified sample must have at least
    one occupied voxel within radius horizontally and from depth below the
    foot up to one voxel above it.
    """
    samples = densify(waypoints, voxel_size)
    offsets = cylinder_offsets(radius, -depth, voxel_size, voxel_size)
    keys = offset_keys(samples, offsets, voxel_size)
    supported = keys_contain(support_keys, keys.ravel()).reshape(keys.shape).any(axis=1)
    return SupportResult(bool(supported.all()), samples[~supported])


@dataclass
class KinematicsResult:
    """Steppability check of the path profile."""

    valid: bool
    violation_points: NDArray[np.float32]


def _resample(waypoints: NDArray[np.float32], spacing: float) -> NDArray[np.float32]:
    """Points every spacing meters of 3D arc length along the polyline."""
    steps = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(steps)])
    if arc[-1] <= spacing:
        return waypoints[[0, -1]]
    s = np.append(np.arange(0.0, arc[-1], spacing), arc[-1])
    return np.stack([np.interp(s, arc, waypoints[:, i]) for i in range(3)], axis=1).astype(
        np.float32
    )


def check_kinematics(
    waypoints: NDArray[np.float32],
    max_slope: float,
    max_step_m: float,
    window_m: float,
) -> KinematicsResult:
    """Reject paths that climb steeper than the robot can.

    The profile is resampled at window_m of arc length so single-cell
    quantization in planner waypoints does not read as a cliff. Each
    resampled segment may rise at most max_slope times its horizontal run,
    with a max_step_m floor so stair risers between close samples pass.
    """
    if len(waypoints) < 2:
        return KinematicsResult(True, waypoints[:0])
    profile = _resample(waypoints, window_m)
    d = np.diff(profile, axis=0)
    rise = np.abs(d[:, 2])
    run = np.linalg.norm(d[:, :2], axis=1)
    bad = rise > np.maximum(run * max_slope, max_step_m)
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


def reference_length(
    trajectory: Trajectory,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    robot_height: float,
    max_snap_m: float = 1.0,
) -> Reference:
    """Shortest walked length the trajectory demonstrates between start and goal.

    Minimizes route length over every combination of start and goal visits,
    not the route between single nearest poses. Only causal pairs count when
    one exists: the goal visited before the start. Falls back to straight-line
    distance when either endpoint is farther than max_snap_m from the trajectory.
    """
    foot = trajectory.positions - np.array([0.0, 0.0, robot_height], dtype=np.float32)
    s = np.asarray(start, dtype=np.float32)
    g = np.asarray(goal, dtype=np.float32)
    ds = np.linalg.norm(foot - s, axis=1)
    dg = np.linalg.norm(foot - g, axis=1)
    if ds.min() > max_snap_m or dg.min() > max_snap_m:
        return Reference(float(np.linalg.norm(g - s)), False, float("inf"), False)
    arcs = trajectory.arc_lengths()
    near_s = np.flatnonzero(ds <= max_snap_m)
    near_g = np.flatnonzero(dg <= max_snap_m)
    totals = (
        np.abs(arcs[near_s][:, None] - arcs[near_g][None, :])
        + ds[near_s][:, None]
        + dg[near_g][None, :]
    )
    backward = trajectory.ts[near_g][None, :] <= trajectory.ts[near_s][:, None]
    causal = bool(backward.any())
    if causal:
        totals = np.where(backward, totals, np.inf)
    best = np.unravel_index(totals.argmin(), totals.shape)
    i = int(near_s[best[0]])
    start_ts = float(trajectory.ts[i]) if causal else float("inf")
    return Reference(max(float(totals[best]), 1e-6), True, start_ts, causal)


def ground_truth_route(
    trajectory: Trajectory,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    robot_height: float,
    max_snap_m: float = 1.0,
) -> NDArray[np.float32] | None:
    """Foot-level polyline of the shortest walk the robot took between start and
    goal, or None when either endpoint is off the trajectory.

    Unlike reference_length this ignores causality: it describes the terrain
    between two places, so the nearest-in-time pass is what we want, not the
    causal one. Picking the visit pair with the least trajectory between them
    keeps the route local instead of a building-spanning detour.
    """
    foot = trajectory.positions - np.array([0.0, 0.0, robot_height], dtype=np.float32)
    s = np.asarray(start, dtype=np.float32)
    g = np.asarray(goal, dtype=np.float32)
    ds = np.linalg.norm(foot - s, axis=1)
    dg = np.linalg.norm(foot - g, axis=1)
    if ds.min() > max_snap_m or dg.min() > max_snap_m:
        return None
    arcs = trajectory.arc_lengths()
    near_s = np.flatnonzero(ds <= max_snap_m)
    near_g = np.flatnonzero(dg <= max_snap_m)
    totals = (
        np.abs(arcs[near_s][:, None] - arcs[near_g][None, :])
        + ds[near_s][:, None]
        + dg[near_g][None, :]
    )
    best = np.unravel_index(totals.argmin(), totals.shape)
    i = int(near_s[best[0]])
    j = int(near_g[best[1]])
    # Orient the slice start-to-goal so the route reads in the case's direction.
    route = foot[i : j + 1] if i <= j else foot[j : i + 1][::-1]
    return route.astype(np.float32)


def spl(success: bool, l_ref: float, p_len: float) -> float:
    if not success:
        return 0.0
    return l_ref / max(p_len, l_ref)


def soft_progress(
    end: NDArray[np.float32] | None,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
) -> float:
    """Fraction of the start-goal distance covered by the path endpoint."""
    d0 = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
    if end is None or d0 < 1e-6:
        return 0.0
    d1 = float(np.linalg.norm(np.asarray(goal, dtype=np.float32) - end))
    return float(np.clip(1.0 - d1 / d0, 0.0, 1.0))


def timing_stats(samples_ms: list[float]) -> dict[str, float]:
    if not samples_ms:
        return {"p50": 0.0, "p95": 0.0, "max": 0.0}
    arr = np.asarray(samples_ms)
    return {
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(arr.max()),
    }

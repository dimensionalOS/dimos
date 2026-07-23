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

"""End-to-end integration: ambiguity gate + robust aggregation, on SIMULATED data.

Every pose here is SYNTHETIC and CONSTRUCTED (seeded ``np.random.default_rng``,
no hardware, no recording). A real recording has NO ground truth for a tag's
pose -- that unknown IS the relocalization problem -- so correctness can only be
proven on a scene we built and therefore know the answer to.

The scene: ONE marker at a KNOWN ``world_T_marker`` (its true world pose) and a
KNOWN ``map_T_marker`` (the surveyed tag pose in the map). A pinhole camera
``K``/``D`` views it from KNOWN ``world_T_optical`` poses; each glimpse's corner
pixels come from ``cv2.projectPoints`` of the true (or mirror-flipped) marker
pose, plus deterministic sub-pixel noise. Those corners drive the FULL published
path together -- ``ambiguity_gated_pose`` (the per-glimpse IPPE mirror gate) ->
``TagAggregator.observe`` / ``robust_estimate`` (the Huber fusion) -> compose the
fused ``world_T_marker`` with the known ``map_T_marker`` into the candidate
``map_T_world`` the judge would receive. Every assertion is against the
constructed truth ``map_T_world_truth = map_T_marker @ inv(world_T_marker)``.

The mirror flip is a 180 deg rotation about the tag's x-axis right-multiplied in
the MARKER frame (the classic planar-PnP two-fold ambiguity). Being a
right-multiply with zero translation it leaves the marker's world POSITION
unchanged and inverts only its ORIENTATION -- so the flip corrupts the candidate
through the ``inv(world_T_marker)`` composition, not through the fused point.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.perception.fiducial.apriltag_aggregation import (
    AggregationConfig,
    TagAggregator,
    TagObservation,
    matrix_from_pose7,
    pose7_from_matrix,
    tag_side_px,
    view_quality,
)
from dimos.perception.fiducial.marker_pose import ambiguity_gated_pose

_MARKER_LENGTH_M = 0.15
_MARKER_ID = 6
# Pinhole intrinsics (SIMULATED camera): fx=fy=600 px, principal point at a
# 640x480 image centre; zero distortion so projectPoints is a clean pinhole.
_K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
_D = np.zeros((5, 1))
_SIGMA_PX = 0.3  # per-corner Gaussian pixel noise: enough to make weak-perspective views genuinely ambiguous

# --- KNOWN ground truth (the whole point of a constructed scene) ---
# The marker's true pose in the world (LIO) frame the detector publishes in.
_WORLD_T_MARKER = np.eye(4)
_WORLD_T_MARKER[:3, :3] = Rotation.from_euler("xyz", (5.0, -8.0, 12.0), degrees=True).as_matrix()
_WORLD_T_MARKER[:3, 3] = (0.4, -0.2, 0.9)
# The surveyed tag pose in the global map frame (the fiducial prior's map_T_tag).
_MAP_T_MARKER = np.eye(4)
_MAP_T_MARKER[:3, :3] = Rotation.from_euler("xyz", (2.0, 3.0, 90.0), degrees=True).as_matrix()
_MAP_T_MARKER[:3, 3] = (5.0, 3.0, 1.2)
# 180 deg about the tag x-axis, right-multiplied in the marker frame == the PnP mirror flip.
_MARKER_FLIP = np.eye(4)
_MARKER_FLIP[:3, :3] = Rotation.from_rotvec(math.pi * np.array([1.0, 0.0, 0.0])).as_matrix()

# The candidate map_T_world the judge SHOULD receive, and the one a swallowed
# flip produces instead. Same composition the FiducialPrior uses:
# map_T_world = map_T_marker @ inv(world_T_marker_fused).
_MAP_T_WORLD_TRUTH = _MAP_T_MARKER @ np.linalg.inv(_WORLD_T_MARKER)
_MAP_T_WORLD_FLIP = _MAP_T_MARKER @ np.linalg.inv(_WORLD_T_MARKER @ _MARKER_FLIP)


def _object_points(marker_length_m: float) -> np.ndarray:
    """The four planar tag corners in the marker frame (OpenCV ArUco order, Z=0)."""
    h = marker_length_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)


def _look_at(cam_pos: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    """Frame_T_optical for a camera at ``cam_pos`` looking at ``target`` (optical
    convention: +Z forward toward the target, +X right, +Y down)."""
    z = target - cam_pos
    z = z / np.linalg.norm(z)
    if abs(float(np.dot(z, up))) > 0.98:  # up nearly parallel to view -> pick another
        up = np.array([1.0, 0.0, 0.0])
    x = np.cross(up, z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    frame_t_optical = np.eye(4)
    frame_t_optical[:3, :3] = np.column_stack([x, y, z])
    frame_t_optical[:3, 3] = cam_pos
    return frame_t_optical


def _marker_frame_camera(elev_deg: float, azim_deg: float, dist_m: float) -> np.ndarray:
    """world_T_optical for a camera on a sphere AROUND the marker, placed in the
    MARKER frame so obliquity is controlled directly: ``elev_deg`` is measured off
    the tag plane toward the tag normal (+Z), so 90 deg is fronto-parallel (weak
    perspective, mirror-ambiguous) and lower is oblique (strong perspective)."""
    e, a = math.radians(elev_deg), math.radians(azim_deg)
    pos_marker = dist_m * np.array(
        [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)]
    )
    marker_t_optical = _look_at(pos_marker, np.zeros(3), up=np.array([0.0, 1.0, 0.0]))
    return np.asarray(_WORLD_T_MARKER @ marker_t_optical)


def _glimpse_corners(
    world_t_optical: np.ndarray, flipped: bool, rng: np.random.Generator
) -> np.ndarray:
    """The tag's corner pixels for one camera pose: project the true (or
    mirror-flipped) marker, then add deterministic sub-pixel noise."""
    optical_t_marker = np.linalg.inv(world_t_optical) @ _WORLD_T_MARKER
    if flipped:
        optical_t_marker = optical_t_marker @ _MARKER_FLIP
    rvec = cv2.Rodrigues(optical_t_marker[:3, :3])[0]
    tvec = optical_t_marker[:3, 3]
    projected, _ = cv2.projectPoints(_object_points(_MARKER_LENGTH_M), rvec, tvec, _K, _D)
    noisy = projected.reshape(4, 2) + rng.normal(0.0, _SIGMA_PX, (4, 2))
    return noisy.astype(np.float32)


def _run_full_path(
    glimpses: list[tuple[float, float, float, bool]],
    ambiguity_ratio_min: float,
    seed: int,
) -> tuple[np.ndarray, int, int]:
    """Drive the whole published pipeline on the constructed glimpses and return
    ``(candidate map_T_world, n_kept, n_rejected)``.

    Per glimpse: corners -> ``ambiguity_gated_pose`` (the gate) -> lift the gated
    ``optical_T_marker`` into the world with the KNOWN ``world_T_optical`` ->
    ``TagAggregator.observe`` (Huber fusion buffer). After all glimpses, fuse via
    ``robust_estimate`` and compose the candidate exactly as ``FiducialPrior``
    does. ``ts`` steps 0.1 s so every glimpse lands in one 5 s fusion window.
    """
    rng = np.random.default_rng(seed)
    aggregator = TagAggregator(AggregationConfig())
    ts, n_kept, n_rejected = 0.0, 0, 0
    for elev_deg, azim_deg, dist_m, flipped in glimpses:
        world_t_optical = _marker_frame_camera(elev_deg, azim_deg, dist_m)
        corners_px = _glimpse_corners(world_t_optical, flipped, rng)
        gated = ambiguity_gated_pose(
            corners_px,
            _MARKER_LENGTH_M,
            _K,
            _D,
            distortion_model=None,
            ambiguity_ratio_min=ambiguity_ratio_min,
        )
        ts += 0.1
        if gated is None:  # gate rejected this glimpse as mirror-ambiguous
            n_rejected += 1
            continue
        optical_t_marker, reproj_px = gated
        world_t_marker = world_t_optical @ optical_t_marker
        distance_m, view_angle_deg = view_quality(pose7_from_matrix(optical_t_marker))
        reason = aggregator.observe(
            TagObservation(
                ts=ts,
                marker_id=_MARKER_ID,
                pose=pose7_from_matrix(world_t_marker),
                distance_m=distance_m,
                view_angle_deg=view_angle_deg,
                reproj_px=reproj_px,
                tag_px=tag_side_px(corners_px),
            )
        )
        if reason is None:
            n_kept += 1
        else:
            n_rejected += 1
    estimate = aggregator.robust_estimate(_MARKER_ID)
    assert estimate is not None, "fusion needs >= min_observations kept glimpses"
    world_t_marker_fused = matrix_from_pose7(estimate.pose)
    map_t_world = _MAP_T_MARKER @ np.linalg.inv(world_t_marker_fused)
    return map_t_world, n_kept, n_rejected


def _pose_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """(rotation_deg, translation_m) between two 4x4 poses."""
    rot_deg = (
        (Rotation.from_matrix(a[:3, :3]) * Rotation.from_matrix(b[:3, :3]).inv()).magnitude()
        * 180.0
        / math.pi
    )
    trans_m = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    return rot_deg, trans_m


def test_gate_plus_fusion_recovers_known_pose() -> None:
    """Gate ON + fusion recover the CONSTRUCTED truth. A clean MAJORITY of oblique
    (strong-perspective) glimpses feeding the true marker pose, plus a MINORITY of
    fronto-parallel glimpses whose corners are the MIRROR-FLIPPED pose. With the
    ambiguity gate at ratio 2 the flipped fronto-parallel views are rejected as
    mirror-ambiguous (and any that slip through are out-voted by the clean
    majority), so the fused candidate ``map_T_world`` recovers ``map_T_world_truth``
    to within a fraction of a degree and a few mm -- and is ~180 deg from the flip.
    Proves the gate and the Huber fusion recover the known answer together."""
    clean = [
        (58.0 + 3.0 * (i % 4), float(az), 0.60, False) for i, az in enumerate(range(0, 360, 40))
    ]
    flips = [(88.0, float(az), 0.60, True) for az in (10, 100, 190, 280)]
    candidate, n_kept, n_rejected = _run_full_path(clean + flips, ambiguity_ratio_min=2.0, seed=7)

    rot_truth_deg, trans_truth_m = _pose_error(candidate, _MAP_T_WORLD_TRUTH)
    rot_flip_deg, _ = _pose_error(candidate, _MAP_T_WORLD_FLIP)
    assert n_rejected >= 1  # the gate actually fired on the mirror-ambiguous flips
    assert rot_truth_deg < 3.0  # measured 0.19 deg
    assert trans_truth_m < 0.05  # measured 0.003 m
    assert rot_flip_deg > 150.0  # measured 179.8 deg: did NOT land on the flip


def test_flip_majority_without_gate_is_wrong() -> None:
    """Gate OFF lets the flip win -- the exact Go2 failure. Same scene shape,
    inverted: a MAJORITY of fronto-parallel MIRROR-FLIPPED glimpses plus a clean
    minority, with the ambiguity gate effectively off (ratio 1.0 can never reject).
    Nothing strips the flips, the medoid lands in the flipped majority, and the
    fused candidate tracks ``map_T_world_flip`` -- ~180 deg and >1 m from truth.
    Isolates the gate as the load-bearing difference from the test above."""
    flips = [(88.0, float(az), 0.60, True) for az in range(0, 360, 40)]
    clean = [(60.0, float(az), 0.60, False) for az in (30, 150, 270)]
    candidate, n_kept, n_rejected = _run_full_path(flips + clean, ambiguity_ratio_min=1.0, seed=7)

    rot_truth_deg, trans_truth_m = _pose_error(candidate, _MAP_T_WORLD_TRUTH)
    rot_flip_deg, trans_flip_m = _pose_error(candidate, _MAP_T_WORLD_FLIP)
    assert n_rejected == 0  # gate off: nothing rejected, every flip reaches fusion
    assert rot_truth_deg > 150.0  # measured 178.8 deg: far from truth in rotation
    assert trans_truth_m > 1.0  # measured 1.78 m: far from truth in position
    assert (
        rot_flip_deg < 10.0 and trans_flip_m < 0.05
    )  # measured 1.2 deg / 0.018 m: landed on the flip

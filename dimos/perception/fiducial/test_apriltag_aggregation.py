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

"""Tests for the robust multi-sighting AprilTag aggregation core.

All poses are SIMULATED (numpy-seeded, deterministic) -- no hardware, no
recording. Each test's docstring names the invariant it pins.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.perception.fiducial.apriltag_aggregation import (
    REF_DISTANCE_M,
    REF_REPROJ_PX,
    AggregationConfig,
    TagAggregator,
    TagObservation,
    aggregate_visits,
    cluster_by_time,
    cluster_medoid,
    gate_reason,
    matrix_from_pose7,
    pose7_from_matrix,
    robust_cluster_pose,
    tag_covariance,
    tag_noise_scale,
    tag_side_px,
    view_quality,
)
from dimos.perception.fiducial.marker_pose import ambiguity_gated_pose


def _pose(translation: tuple[float, float, float], rot: Rotation) -> tuple[float, ...]:
    q = rot.as_quat()  # x, y, z, w
    return (*translation, float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _obs(
    ts: float,
    marker_id: int,
    pose: tuple[float, ...],
    **quality: float | tuple[float, float] | None,
) -> TagObservation:
    return TagObservation(ts=ts, marker_id=marker_id, pose=pose, **quality)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "pose, expected_distance, expected_angle, tol",
    [
        # Head-on tag facing the camera: reads its true range and a ~0.003 deg floor.
        pytest.param(_pose((0.0, 0.0, 0.8), Rotation.identity()), 0.8, 0.0, 0.01, id="head_on"),
        # Rotating the tag 40 deg about Y reads that tilt back exactly.
        pytest.param(
            _pose((0.0, 0.0, 1.0), Rotation.from_euler("y", 40.0, degrees=True)),
            1.0,
            40.0,
            1e-4,
            id="oblique",
        ),
    ],
)
def test_view_quality_reads_distance_and_tilt(
    pose: tuple[float, ...], expected_distance: float, expected_angle: float, tol: float
) -> None:
    """view_quality reads back the tag's true distance and the angle of its normal off
    the line of sight -- ~0 deg head-on, growing to the exact tilt as the tag rotates."""
    distance, view_angle = view_quality(pose)
    assert abs(distance - expected_distance) < 1e-9
    assert abs(view_angle - expected_angle) < tol


def test_tag_side_px_of_known_square() -> None:
    """A 30x30 px axis-aligned corner quad has tag side length 30 px."""
    corners = np.array([[0.0, 0.0], [30.0, 0.0], [30.0, 30.0], [0.0, 30.0]])
    assert abs(tag_side_px(corners) - 30.0) < 1e-9


def test_cluster_by_time_splits_on_gap_and_by_marker() -> None:
    """Same-marker sightings within the gap form one cluster; a > gap jump starts
    a new one; different markers never share a cluster."""
    identity = _pose((0.0, 0.0, 1.0), Rotation.identity())
    obs = [
        _obs(0.0, 7, identity),
        _obs(1.0, 7, identity),
        _obs(2.0, 7, identity),
        _obs(20.0, 7, identity),  # 18 s gap -> new visit
        _obs(0.5, 9, identity),  # different marker
    ]
    clusters = cluster_by_time(obs, gap_s=5.0)
    sizes = sorted(len(c) for c in clusters)
    assert sizes == [1, 1, 3]  # marker 7: {3, 1}, marker 9: {1}
    for cluster in clusters:
        assert len({o.marker_id for o in cluster}) == 1


@pytest.mark.parametrize(
    "translations, has_outlier",
    [
        pytest.param(
            [
                (1.0, 2.0, 3.0),
                (1.02, 2.0, 3.0),
                (0.98, 2.0, 3.0),
                (1.0, 2.03, 3.0),
                (1.0, 1.97, 3.0),
            ],
            False,
            id="all_inliers_exact_mean",
        ),
        pytest.param([(1.0, 2.0, 3.0)] * 6 + [(6.0, 2.0, 3.0)], True, id="outlier_downweighted"),
    ],
)
def test_robust_cluster_pose_translation_holds_the_inlier_consensus(
    translations: list[tuple[float, float, float]], has_outlier: bool
) -> None:
    """Huber IRLS on translation: with every sample inside huber_delta_m no sighting is
    discounted, so the aggregate is the exact arithmetic mean; add a lone 5 m outlier and
    IRLS drives its weight down so the aggregate holds the ~1.0 m inlier consensus rather
    than the outlier-dragged naive mean."""
    cluster = [_obs(float(i), 1, _pose(t, Rotation.identity())) for i, t in enumerate(translations)]
    aggregated = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    if has_outlier:
        assert float(np.mean([t[0] for t in translations])) > 1.5  # outlier drags the plain mean
        assert abs(aggregated[0] - 1.0) < 0.05  # Huber holds the aggregate at consensus
    else:
        mean_xyz = np.mean(np.array(translations), axis=0)
        assert np.max(np.abs(np.array(aggregated[:3]) - mean_xyz)) < 1e-9


def test_robust_cluster_pose_markley_quaternion_mean() -> None:
    """The aggregated rotation is the Markley eigen-mean: symmetric small yaws about Z
    (-2..+2 deg) average to ~0 deg, and q/-q double-cover does not cancel it."""
    rots = [Rotation.from_euler("z", d, degrees=True) for d in (-2.0, -1.0, 0.0, 1.0, 2.0)]
    poses = [_pose((0.0, 0.0, 1.0), r) for r in rots]
    # Flip the sign of one quaternion (same rotation, q == -q) to exercise the
    # hemisphere alignment inside the aggregation.
    poses[1] = tuple(-c for c in poses[1])  # type: ignore[assignment]
    cluster = [_obs(float(i), 1, p) for i, p in enumerate(poses)]
    aggregated = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    yaw = Rotation.from_quat(aggregated[3:7]).as_euler("xyz", degrees=True)[2]
    assert abs(yaw) < 0.2  # symmetric spread averages to ~0 deg


def test_robust_aggregation_beats_median_single_sighting() -> None:
    """THE benefit mechanism, deterministically: 15 sightings of one fixed marker
    with ~2 deg per-sighting orientation noise. The Huber-aggregated orientation error
    is smaller than the MEDIAN single-sighting orientation error -- the variance
    reduction that shrinks lever-arm fix scatter."""
    rng = np.random.default_rng(7)
    truth = Rotation.identity()
    poses = []
    single_errs_deg = []
    for _ in range(15):
        noise = Rotation.from_rotvec(rng.normal(0, math.radians(2.0), 3))
        r = noise * truth
        poses.append(_pose((0.5, -0.3, 0.9), r))
        single_errs_deg.append(noise.magnitude() * 180.0 / math.pi)
    cluster = [_obs(float(i), 1, p) for i, p in enumerate(poses)]
    aggregated = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    aggregated_err_deg = (
        (Rotation.from_quat(aggregated[3:7]) * truth.inv()).magnitude() * 180.0 / math.pi
    )
    assert aggregated_err_deg < float(np.median(single_errs_deg))


def test_cluster_medoid_seed_uses_rotation_distance() -> None:
    """The medoid seed weighs ROTATION, not just translation: with all poses at
    one translation and yaws symmetric about 0 deg, the rotationally-central pose
    (yaw 0) is the medoid. With rotation_weight 0 the rotation term vanishes and
    the equal-translation tie falls to the first pose -- proving the weight is
    what makes rotation count in the seed robust_cluster_pose starts from."""
    yaws_deg = (-10.0, -5.0, 0.0, 5.0, 10.0)
    cluster = [
        _obs(float(i), 1, _pose((1.0, 1.0, 1.0), Rotation.from_euler("z", y, degrees=True)))
        for i, y in enumerate(yaws_deg)
    ]
    central = cluster_medoid(cluster, rotation_weight_m_per_rad=0.5)
    central_yaw = Rotation.from_quat(central.pose[3:7]).as_euler("xyz", degrees=True)[2]
    assert abs(central_yaw) < 1e-9  # the yaw-0 pose, most central in rotation
    tie = cluster_medoid(cluster, rotation_weight_m_per_rad=0.0)
    assert tie.ts == 0.0  # rotation ignored -> equal-translation tie takes the first


@pytest.mark.parametrize(
    "n_clean, n_flip, lands_on",
    [
        pytest.param(5, 2, "clean", id="clean_majority_rejects_flips"),
        pytest.param(2, 5, "flip", id="follows_flip_majority"),
    ],
)
def test_robust_cluster_pose_is_a_majority_vote_over_mirror_flips(
    n_clean: int, n_flip: int, lands_on: str
) -> None:
    """Known-truth mirror ambiguity: robust aggregation is a majority vote, not an oracle.
    Flips are 180 deg about X (the PnP mirror solution the gate can miss), a genuinely
    different rotation from the q==-q sign flip tested above. A clean majority (5 vs 2)
    out-votes the flips and the aggregate lands on identity; once the flips DOMINATE
    (5 vs 2) IRLS seeds and converges inside the flipped cluster instead."""
    flip = Rotation.from_rotvec((math.pi, 0.0, 0.0))
    poses = [_pose((0.0, 0.0, 1.0), Rotation.identity()) for _ in range(n_clean)]
    poses += [_pose((0.0, 0.0, 1.0), flip) for _ in range(n_flip)]
    cluster = [_obs(float(i), 1, p) for i, p in enumerate(poses)]
    aggregated = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    reference = Rotation.identity() if lands_on == "clean" else flip
    err_deg = (Rotation.from_quat(aggregated[3:7]) * reference.inv()).magnitude() * 180.0 / math.pi
    assert err_deg < 0.5


def test_gate_reason_each_gate_fires_and_none_skips() -> None:
    """Every gate rejects its own failure; a None field disables only its gate."""
    cfg = AggregationConfig()
    good = _pose((0.0, 0.0, 0.5), Rotation.identity())
    base = dict(distance_m=0.5, view_angle_deg=5.0, reproj_px=1.0, tag_px=40.0)
    assert gate_reason(_obs(0.0, 1, good, **base), cfg) is None  # type: ignore[arg-type]
    assert gate_reason(_obs(0.0, 1, good, **{**base, "reproj_px": 3.0}), cfg) == "reproj"  # type: ignore[arg-type]
    assert gate_reason(_obs(0.0, 1, good, **{**base, "tag_px": 10.0}), cfg) == "small"  # type: ignore[arg-type]
    assert gate_reason(_obs(0.0, 1, good, **{**base, "distance_m": 2.0}), cfg) == "far"  # type: ignore[arg-type]
    assert gate_reason(_obs(0.0, 1, good, **{**base, "view_angle_deg": 60.0}), cfg) == "oblique"  # type: ignore[arg-type]
    # Pixel-less wire observation: reproj/tag_px None -> those gates skip, kept.
    assert gate_reason(_obs(0.0, 1, good, distance_m=0.5, view_angle_deg=5.0), cfg) is None


def test_aggregate_visits_aggregates_and_drops_thin() -> None:
    """A dense visit (>= min_observations) yields one estimate; a thin visit is
    dropped and its glimpses tallied under 'thin'."""
    cfg = AggregationConfig()
    dense = [
        _obs(float(i) * 0.1, 5, _pose((1.0, 0.0, 0.5), Rotation.identity()), distance_m=0.5)
        for i in range(5)
    ]
    thin = [_obs(100.0, 5, _pose((1.0, 0.0, 0.5), Rotation.identity()), distance_m=0.5)]
    estimates, rejected = aggregate_visits([*dense, *thin], cfg)
    assert len(estimates) == 1
    assert estimates[0].n_observations == 5
    assert rejected.get("thin") == 1


def test_tag_aggregator_streaming_needs_min_observations() -> None:
    """Streaming: no estimate below min_observations; one once the window fills;
    a stale glimpse outside time_window_s is purged."""
    cfg = AggregationConfig(min_observations=3, time_window_s=5.0)
    agg = TagAggregator(cfg)
    pose = _pose((1.0, 0.0, 0.5), Rotation.identity())
    assert agg.observe(_obs(0.0, 8, pose, distance_m=0.5)) is None
    assert agg.robust_estimate(8) is None  # 1 < 3
    agg.observe(_obs(0.1, 8, pose, distance_m=0.5))
    agg.observe(_obs(0.2, 8, pose, distance_m=0.5))
    est = agg.robust_estimate(8)
    assert est is not None and est.n_observations == 3
    # A glimpse 10 s later purges the three 0.x-second ones (window 5 s).
    agg.observe(_obs(10.2, 8, pose, distance_m=0.5))
    assert agg.robust_estimate(8) is None  # only 1 in-window again


def test_robust_cluster_pose_single_observation_returns_medoid() -> None:
    """A one-glimpse cluster has nothing to aggregate: robust_cluster_pose short-circuits
    to the medoid (the sole pose itself) rather than running Huber IRLS on n=1."""
    pose = _pose((1.0, -2.0, 0.5), Rotation.from_euler("z", 20.0, degrees=True))
    aggregated = robust_cluster_pose(
        [_obs(0.0, 3, pose)], rotation_weight_m_per_rad=0.5, huber_delta_m=0.05
    )
    assert aggregated == pose


def test_aggregate_visits_tallies_gate_rejections_by_reason() -> None:
    """A batch glimpse that fails a per-glimpse gate (here distance past 1 m -> 'far')
    is dropped and counted under its gate name in the rejection tally, distinct from
    the 'thin'-cluster tally -- so a caller can log WHY glimpses were cut."""
    cfg = AggregationConfig()
    pose = _pose((1.0, 0.0, 0.5), Rotation.identity())
    dense = [_obs(float(i) * 0.1, 5, pose, distance_m=0.5) for i in range(3)]
    far_glimpse = _obs(0.35, 5, pose, distance_m=2.0)  # past the 1 m distance gate
    estimates, rejected = aggregate_visits([*dense, far_glimpse], cfg)
    assert len(estimates) == 1  # the three in-gate glimpses still aggregate
    assert rejected.get("far") == 1
    assert "thin" not in rejected  # the far glimpse was gated, not tallied as a thin cluster


def test_tag_aggregator_observe_returns_gate_reason_and_drops_glimpse() -> None:
    """Streaming observe() returns the gate name of a rejected glimpse (here 'far')
    and never appends it to the marker's window, so a gated sighting cannot pad the
    min_observations count toward a premature aggregated estimate."""
    agg = TagAggregator(AggregationConfig())
    pose = _pose((1.0, 0.0, 0.5), Rotation.identity())
    assert agg.observe(_obs(0.0, 8, pose, distance_m=2.0)) == "far"  # 2 m > 1 m gate
    assert agg.robust_estimate(8) is None  # nothing buffered from the rejected glimpse


@pytest.mark.parametrize(
    "distance_m, reproj_px, expected",
    [
        pytest.param(REF_DISTANCE_M, REF_REPROJ_PX, 1.0, id="reference_is_one"),
        pytest.param(None, None, 1.0, id="none_reads_as_reference"),
        pytest.param(2.0 * REF_DISTANCE_M, 1.5 * REF_REPROJ_PX, 9.0, id="quadratic_2x_and_1.5x"),
        pytest.param(0.0, 0.0, 0.25, id="inner_clamps_then_floor"),
        pytest.param(0.2, 0.5, 0.25, id="exactly_at_clamps"),
        pytest.param(0.1, 0.4, 0.25, id="below_clamps_never_lower"),
    ],
)
def test_tag_noise_scale(
    distance_m: float | None, reproj_px: float | None, expected: float
) -> None:
    """Planar-PnP error grows ~quadratically with range and reproj is a direct misfit
    proxy, so the terms multiply as squares (2x range * 1.5x reproj = 2^2 * 1.5^2 = 9).
    The reference glimpse scales by 1.0 and a None input reads as the reference (that term
    drops out, keeping a pixel-less estimate neutral); the inner clamps (0.2 m, 0.5 px) and
    the 0.25 floor bound how confident any tag may get."""
    assert tag_noise_scale(distance_m, reproj_px) == expected


@pytest.mark.parametrize(
    "pose, expected_diag",
    [
        # Identity: blocks readable directly -- index 2 is the loose range term, index 5 the spin.
        pytest.param(
            _pose((1.0, 2.0, 3.0), Rotation.identity()),
            [0.0025, 0.0025, 0.25, 0.04, 0.04, 0.0025],
            id="identity_ros_order",
        ),
        # 90 deg about y swings the tag normal onto world x: the range term moves to index 0.
        pytest.param(
            _pose((0.0, 0.0, 0.0), Rotation.from_euler("y", 90.0, degrees=True)),
            [0.25, 0.0025, 0.0025, 0.0025, 0.04, 0.04],
            id="rotated_weak_axis_follows_normal",
        ),
    ],
)
def test_tag_covariance_is_ros_translation_first_and_scales(
    pose: tuple[float, ...], expected_diag: list[float]
) -> None:
    """ROS PoseWithCovariance is (x, y, z, rot_x, rot_y, rot_z) -- TRANSLATION FIRST, the
    opposite of the GTSAM Pose3 tangent the diagonals are ported from: the loose 0.25 m^2
    range term (planar PnP's weak axis, along the tag normal) sits in a translation slot and
    the tight 0.0025 rad^2 spin in a rotation slot. The blocks are built in the tag frame and
    rotated by the pose's R, so a 90 deg-about-y pose swings the range term onto world x.
    Scale multiplies the whole matrix (off-diagonals stay zero here), nothing else."""
    covariance = tag_covariance(pose, 1.0)
    np.testing.assert_allclose(covariance, np.diag(expected_diag), atol=1e-15)
    np.testing.assert_allclose(tag_covariance(pose, 9.0), covariance * 9.0, atol=1e-15)


# ---------------------------------------------------------------------------
# End-to-end integration: the ambiguity gate + robust aggregation on the FULL
# published pixel path, on SIMULATED constructed-truth data (seeded rng, no
# hardware, no recording). A real recording has NO ground truth for a tag's pose
# -- that unknown IS the relocalization problem -- so correctness is proven on a
# scene we built: ONE marker at a KNOWN world_T_marker and a KNOWN map_T_marker,
# viewed by a pinhole camera from KNOWN world_T_optical poses. Each glimpse's
# corner pixels come from cv2.projectPoints of the true (or mirror-flipped)
# marker plus deterministic sub-pixel noise, driving corners -> ambiguity_gated_pose
# -> TagAggregator -> the map_T_world = map_T_marker @ inv(world_T_marker) the judge
# would receive. The flip is a 180 deg about the tag x-axis right-multiplied in the
# MARKER frame (the planar-PnP two-fold ambiguity): it leaves the marker POSITION
# unchanged and inverts only ORIENTATION.
# ---------------------------------------------------------------------------

_INT_MARKER_LENGTH_M = 0.15
_INT_MARKER_ID = 6
# Pinhole intrinsics (SIMULATED camera): fx=fy=600 px, 640x480 centre, no distortion.
_INT_K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
_INT_D = np.zeros((5, 1))
_INT_SIGMA_PX = 0.3  # per-corner pixel noise: enough to make weak-perspective views ambiguous

# --- KNOWN ground truth (the whole point of a constructed scene) ---
_WORLD_T_MARKER = np.eye(4)  # the marker's true pose in the world (LIO) frame
_WORLD_T_MARKER[:3, :3] = Rotation.from_euler("xyz", (5.0, -8.0, 12.0), degrees=True).as_matrix()
_WORLD_T_MARKER[:3, 3] = (0.4, -0.2, 0.9)
_MAP_T_MARKER = np.eye(4)  # the surveyed tag pose in the global map frame
_MAP_T_MARKER[:3, :3] = Rotation.from_euler("xyz", (2.0, 3.0, 90.0), degrees=True).as_matrix()
_MAP_T_MARKER[:3, 3] = (5.0, 3.0, 1.2)
_MARKER_FLIP = np.eye(4)  # 180 deg about the tag x-axis == the PnP mirror flip
_MARKER_FLIP[:3, :3] = Rotation.from_rotvec(math.pi * np.array([1.0, 0.0, 0.0])).as_matrix()

# The candidate map_T_world the judge SHOULD receive, and the one a swallowed flip produces.
_MAP_T_WORLD_TRUTH = _MAP_T_MARKER @ np.linalg.inv(_WORLD_T_MARKER)
_MAP_T_WORLD_FLIP = _MAP_T_MARKER @ np.linalg.inv(_WORLD_T_MARKER @ _MARKER_FLIP)


def _object_points(marker_length_m: float) -> np.ndarray:
    """The four planar tag corners in the marker frame (OpenCV ArUco order, Z=0)."""
    h = marker_length_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)


def _look_at(cam_pos: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    """frame_T_optical for a camera at cam_pos looking at target (optical convention:
    +Z forward toward the target, +X right, +Y down)."""
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
    """world_T_optical for a camera on a sphere AROUND the marker, placed in the MARKER
    frame so obliquity is controlled directly: elev_deg is measured off the tag plane
    toward the tag normal (+Z), so 90 deg is fronto-parallel (weak perspective,
    mirror-ambiguous) and lower is oblique (strong perspective)."""
    e, a = math.radians(elev_deg), math.radians(azim_deg)
    pos_marker = dist_m * np.array(
        [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)]
    )
    marker_t_optical = _look_at(pos_marker, np.zeros(3), up=np.array([0.0, 1.0, 0.0]))
    return np.asarray(_WORLD_T_MARKER @ marker_t_optical)


def _glimpse_corners(
    world_t_optical: np.ndarray, flipped: bool, rng: np.random.Generator
) -> np.ndarray:
    """The tag's corner pixels for one camera pose: project the true (or mirror-flipped)
    marker, then add deterministic sub-pixel noise."""
    optical_t_marker = np.linalg.inv(world_t_optical) @ _WORLD_T_MARKER
    if flipped:
        optical_t_marker = optical_t_marker @ _MARKER_FLIP
    rvec = cv2.Rodrigues(optical_t_marker[:3, :3])[0]
    tvec = optical_t_marker[:3, 3]
    projected, _ = cv2.projectPoints(
        _object_points(_INT_MARKER_LENGTH_M), rvec, tvec, _INT_K, _INT_D
    )
    noisy = projected.reshape(4, 2) + rng.normal(0.0, _INT_SIGMA_PX, (4, 2))
    return noisy.astype(np.float32)


def _run_full_path(
    glimpses: list[tuple[float, float, float, bool]], ambiguity_ratio_min: float, seed: int
) -> tuple[np.ndarray, int, int]:
    """Drive the whole published pipeline on the constructed glimpses and return
    (candidate map_T_world, n_kept, n_rejected). Per glimpse: corners ->
    ambiguity_gated_pose -> lift into the world with the KNOWN world_T_optical ->
    TagAggregator.observe; then robust_estimate and compose the candidate exactly as
    FiducialPrior does. ts steps 0.1 s so every glimpse lands in one 5 s window."""
    rng = np.random.default_rng(seed)
    aggregator = TagAggregator(AggregationConfig())
    ts, n_kept, n_rejected = 0.0, 0, 0
    for elev_deg, azim_deg, dist_m, flipped in glimpses:
        world_t_optical = _marker_frame_camera(elev_deg, azim_deg, dist_m)
        corners_px = _glimpse_corners(world_t_optical, flipped, rng)
        gated = ambiguity_gated_pose(
            corners_px,
            _INT_MARKER_LENGTH_M,
            _INT_K,
            _INT_D,
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
                marker_id=_INT_MARKER_ID,
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
    estimate = aggregator.robust_estimate(_INT_MARKER_ID)
    assert estimate is not None, "aggregation needs >= min_observations kept glimpses"
    world_t_marker_aggregated = matrix_from_pose7(estimate.pose)
    map_t_world = _MAP_T_MARKER @ np.linalg.inv(world_t_marker_aggregated)
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


def test_gate_plus_aggregation_recovers_known_pose() -> None:
    """Gate ON + aggregation recover the CONSTRUCTED truth on the full pixel path. A clean
    MAJORITY of oblique glimpses feeding the true marker pose, plus a MINORITY of
    fronto-parallel glimpses whose corners are the MIRROR-FLIPPED pose. At ratio 2 the
    flipped fronto-parallel views are rejected as mirror-ambiguous (and any that slip
    through are out-voted), so the aggregated candidate map_T_world recovers the truth to a
    fraction of a degree and a few mm -- and is ~180 deg from the flip."""
    clean = [
        (58.0 + 3.0 * (i % 4), float(az), 0.60, False) for i, az in enumerate(range(0, 360, 40))
    ]
    flips = [(88.0, float(az), 0.60, True) for az in (10, 100, 190, 280)]
    candidate, _n_kept, n_rejected = _run_full_path(clean + flips, ambiguity_ratio_min=2.0, seed=7)

    rot_truth_deg, trans_truth_m = _pose_error(candidate, _MAP_T_WORLD_TRUTH)
    rot_flip_deg, _ = _pose_error(candidate, _MAP_T_WORLD_FLIP)
    assert n_rejected >= 1  # the gate actually fired on the mirror-ambiguous flips
    assert rot_truth_deg < 3.0  # measured 0.19 deg
    assert trans_truth_m < 0.05  # measured 0.003 m
    assert rot_flip_deg > 150.0  # measured 179.8 deg: did NOT land on the flip

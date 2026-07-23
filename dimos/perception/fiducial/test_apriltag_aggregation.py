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

import numpy as np
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
    robust_cluster_pose,
    tag_covariance,
    tag_noise_scale,
    tag_side_px,
    view_quality,
)


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


def test_view_quality_head_on_is_zero_angle() -> None:
    """A tag 0.8 m straight ahead, facing the camera (normal along -Z toward the
    camera, i.e. the tag's +Z away from it) reads distance 0.8 m, angle ~0 deg."""
    pose = _pose((0.0, 0.0, 0.8), Rotation.identity())
    distance, view_angle = view_quality(pose)
    assert abs(distance - 0.8) < 1e-9
    # The line-of-sight uses distance + 1e-9, a ~0.003 deg head-on floor.
    assert view_angle < 0.01


def test_view_quality_oblique_grows() -> None:
    """Rotating the tag about Y off head-on increases the view angle."""
    straight = view_quality(_pose((0.0, 0.0, 1.0), Rotation.identity()))[1]
    tilted = view_quality(
        _pose((0.0, 0.0, 1.0), Rotation.from_euler("y", 40.0, degrees=True))
    )[1]
    assert straight < 0.01
    assert tilted > straight
    assert abs(tilted - 40.0) < 1e-4


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


def test_cluster_medoid_picks_central_over_outlier() -> None:
    """The medoid is the most central pose; a lone 5 m outlier is never it."""
    inliers = [_pose((1.0 + 0.01 * i, 2.0, 3.0), Rotation.identity()) for i in range(5)]
    outlier = _pose((6.0, 2.0, 3.0), Rotation.identity())
    cluster = [_obs(float(i), 1, p) for i, p in enumerate([*inliers, outlier])]
    medoid = cluster_medoid(cluster, rotation_weight_m_per_rad=0.5)
    assert medoid.pose[0] < 1.1  # a central inlier, not the 6 m outlier


def test_robust_cluster_pose_downweights_translation_outlier() -> None:
    """Huber IRLS pulls the fused translation to the inlier consensus (~1.0 m),
    NOT the naive mean (~1.7 m) a single 5 m outlier would drag it to."""
    rng = np.random.default_rng(3)
    inliers = [
        _pose((1.0 + rng.normal(0, 0.01), 2.0, 3.0), Rotation.identity()) for _ in range(6)
    ]
    outlier = _pose((6.0, 2.0, 3.0), Rotation.identity())
    cluster = [_obs(float(i), 1, p) for i, p in enumerate([*inliers, outlier])]
    fused = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    naive_mean_x = float(np.mean([p[0] for p in [*inliers, outlier]]))
    assert naive_mean_x > 1.5  # the outlier drags the plain mean well off
    assert abs(fused[0] - 1.0) < 0.05  # Huber holds the fused estimate at consensus


def test_robust_cluster_pose_markley_quaternion_mean() -> None:
    """The fused rotation is the Markley eigen-mean: symmetric small yaws about Z
    (-2..+2 deg) average to ~0 deg, and q/-q double-cover does not cancel it."""
    rots = [Rotation.from_euler("z", d, degrees=True) for d in (-2.0, -1.0, 0.0, 1.0, 2.0)]
    poses = [_pose((0.0, 0.0, 1.0), r) for r in rots]
    # Flip the sign of one quaternion (same rotation, q == -q) to exercise the
    # hemisphere alignment inside the fusion.
    poses[1] = tuple(-c for c in poses[1])  # type: ignore[assignment]
    cluster = [_obs(float(i), 1, p) for i, p in enumerate(poses)]
    fused = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    yaw = Rotation.from_quat(fused[3:7]).as_euler("xyz", degrees=True)[2]
    assert abs(yaw) < 0.2  # symmetric spread averages to ~0 deg


def test_robust_fusion_beats_median_single_sighting() -> None:
    """THE benefit mechanism, deterministically: 15 sightings of one fixed marker
    with ~2 deg per-sighting orientation noise. The Huber-fused orientation error
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
    fused = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    fused_err_deg = (Rotation.from_quat(fused[3:7]) * truth.inv()).magnitude() * 180.0 / math.pi
    assert fused_err_deg < float(np.median(single_errs_deg))


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


def test_robust_cluster_pose_translation_is_exact_mean_without_outlier() -> None:
    """With every translation inside huber_delta_m, no sample is down-weighted, so
    the Huber IRLS translation collapses to the plain arithmetic mean of the
    inliers -- IRLS must not spuriously discount a clean sighting."""
    translations = [
        (1.0, 2.0, 3.0),
        (1.02, 2.0, 3.0),
        (0.98, 2.0, 3.0),
        (1.0, 2.03, 3.0),
        (1.0, 1.97, 3.0),
    ]
    cluster = [
        _obs(float(i), 1, _pose(t, Rotation.identity())) for i, t in enumerate(translations)
    ]
    fused = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    mean_xyz = np.mean(np.array(translations), axis=0)
    assert np.max(np.abs(np.array(fused[:3]) - mean_xyz)) < 1e-9


def test_robust_cluster_pose_recovers_identical_rotation() -> None:
    """The Markley eigen-mean of N identical unit quaternions is that quaternion:
    a cluster all at one non-trivial 3D rotation fuses back to it exactly (down to
    quaternion double-cover, which the error metric absorbs)."""
    truth = Rotation.from_euler("xyz", (15.0, -20.0, 30.0), degrees=True)
    cluster = [_obs(float(i), 1, _pose((0.0, 0.0, 1.0), truth)) for i in range(4)]
    fused = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    err_deg = (Rotation.from_quat(fused[3:7]) * truth.inv()).magnitude() * 180.0 / math.pi
    assert err_deg < 1e-6


def test_robust_cluster_pose_clean_majority_rejects_mirror_flips() -> None:
    """Known-truth mirror ambiguity: 5 correct sightings + 2 genuinely flipped
    ones (180 deg about X, the kind of PnP mirror solution the ambiguity gate can
    miss). The flips' ~pi-rad rotation residual sits far past the Huber knee, so
    IRLS drives their weight down and the fused rotation lands on the clean
    majority -- distinct from the q==-q sign flip (same rotation) tested above."""
    flip = Rotation.from_rotvec((math.pi, 0.0, 0.0))  # 180 deg mirror, a different rotation
    poses = [_pose((0.0, 0.0, 1.0), Rotation.identity()) for _ in range(5)]
    poses += [_pose((0.0, 0.0, 1.0), flip) for _ in range(2)]
    cluster = [_obs(float(i), 1, p) for i, p in enumerate(poses)]
    fused = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    err_deg = Rotation.from_quat(fused[3:7]).magnitude() * 180.0 / math.pi
    assert err_deg < 0.5  # fused sits on the clean identity majority, not the 180 deg flip


def test_robust_cluster_pose_follows_flip_majority() -> None:
    """The converse bound, pinned as known-truth so the limit is explicit: the
    fusion is a robust majority vote, not an oracle. When the flips DOMINATE (5
    flipped, 2 clean), the medoid seeds inside the flip cluster and IRLS converges
    to the flipped rotation -- the estimator cannot recover truth once bad glimpses
    outnumber good ones."""
    flip = Rotation.from_rotvec((math.pi, 0.0, 0.0))
    poses = [_pose((0.0, 0.0, 1.0), flip) for _ in range(5)]
    poses += [_pose((0.0, 0.0, 1.0), Rotation.identity()) for _ in range(2)]
    cluster = [_obs(float(i), 1, p) for i, p in enumerate(poses)]
    fused = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)
    err_to_flip_deg = (
        Rotation.from_quat(fused[3:7]) * flip.inv()
    ).magnitude() * 180.0 / math.pi
    assert err_to_flip_deg < 0.5  # follows the flipped majority


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


def test_aggregate_visits_fuses_and_drops_thin() -> None:
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
    """A one-glimpse cluster has nothing to fuse: robust_cluster_pose short-circuits
    to the medoid (the sole pose itself) rather than running Huber IRLS on n=1."""
    pose = _pose((1.0, -2.0, 0.5), Rotation.from_euler("z", 20.0, degrees=True))
    fused = robust_cluster_pose(
        [_obs(0.0, 3, pose)], rotation_weight_m_per_rad=0.5, huber_delta_m=0.05
    )
    assert fused == pose


def test_aggregate_visits_tallies_gate_rejections_by_reason() -> None:
    """A batch glimpse that fails a per-glimpse gate (here distance past 1 m -> 'far')
    is dropped and counted under its gate name in the rejection tally, distinct from
    the 'thin'-cluster tally -- so a caller can log WHY glimpses were cut."""
    cfg = AggregationConfig()
    pose = _pose((1.0, 0.0, 0.5), Rotation.identity())
    dense = [_obs(float(i) * 0.1, 5, pose, distance_m=0.5) for i in range(3)]
    far_glimpse = _obs(0.35, 5, pose, distance_m=2.0)  # past the 1 m distance gate
    estimates, rejected = aggregate_visits([*dense, far_glimpse], cfg)
    assert len(estimates) == 1  # the three in-gate glimpses still fuse
    assert rejected.get("far") == 1
    assert "thin" not in rejected  # the far glimpse was gated, not tallied as a thin cluster


def test_tag_aggregator_observe_returns_gate_reason_and_drops_glimpse() -> None:
    """Streaming observe() returns the gate name of a rejected glimpse (here 'far')
    and never appends it to the marker's window, so a gated sighting cannot pad the
    min_observations count toward a premature fused estimate."""
    agg = TagAggregator(AggregationConfig())
    pose = _pose((1.0, 0.0, 0.5), Rotation.identity())
    assert agg.observe(_obs(0.0, 8, pose, distance_m=2.0)) == "far"  # 2 m > 1 m gate
    assert agg.robust_estimate(8) is None  # nothing buffered from the rejected glimpse


def test_tag_noise_scale_is_one_at_the_reference_glimpse() -> None:
    """The reference glimpse (REF_DISTANCE_M, REF_REPROJ_PX) scales by exactly 1.0,
    so the covariance a fused pose ships there is the documented base block and
    nothing else. A None input reads as the reference, i.e. that term drops out --
    that is what keeps a pixel-less estimate neutral instead of silently confident."""
    assert tag_noise_scale(REF_DISTANCE_M, REF_REPROJ_PX) == 1.0
    assert tag_noise_scale(None, None) == 1.0
    assert tag_noise_scale(None, REF_REPROJ_PX) == 1.0
    assert tag_noise_scale(REF_DISTANCE_M, None) == 1.0


def test_tag_noise_scale_grows_quadratically_and_clamps_at_both_ends() -> None:
    """Planar-PnP error grows ~quadratically with range and reproj is a direct misfit
    proxy, so the two terms multiply as squares: 2x range and 1.5x reproj is
    2^2 * 1.5^2 = 9. The inner clamps (0.2 m, 0.5 px) stop a suspiciously-perfect
    read claiming near-zero variance, and the 0.25 floor caps how confident any tag
    may get -- a glimpse at 0 m with 0 px misfit is still only 4x the reference."""
    assert tag_noise_scale(2.0 * REF_DISTANCE_M, 1.5 * REF_REPROJ_PX) == 9.0
    assert tag_noise_scale(0.0, 0.0) == 0.25  # both inner clamps, then the floor
    assert tag_noise_scale(0.2, 0.5) == 0.25  # exactly at the clamps: same value
    assert tag_noise_scale(0.1, 0.4) == 0.25  # below them: clamped, never lower


def test_tag_covariance_is_ros_translation_first_and_scales_linearly() -> None:
    """ROS PoseWithCovariance is (x, y, z, rot_x, rot_y, rot_z) -- TRANSLATION FIRST,
    the opposite of the GTSAM Pose3 tangent the diagonals are ported from. With an
    identity rotation the blocks are readable directly: index 2 must carry the loose
    0.25 m^2 range term (planar PnP's weak axis, along the tag normal) and index 5
    the tight 0.0025 rad^2 in-plane spin. Swapping the blocks would put 0.04 rad^2 in
    the metres slot. Scale multiplies the whole matrix, nothing else."""
    identity_pose = _pose((1.0, 2.0, 3.0), Rotation.identity())

    base = tag_covariance(identity_pose, 1.0)

    np.testing.assert_allclose(
        base, np.diag([0.0025, 0.0025, 0.25, 0.04, 0.04, 0.0025]), atol=1e-15
    )
    np.testing.assert_allclose(tag_covariance(identity_pose, 9.0), base * 9.0, atol=1e-15)


def test_tag_covariance_rotates_the_blocks_into_the_pose_frame() -> None:
    """Both blocks are built in the TAG frame and rotated by the pose's R, so the
    weak axis follows the tag's normal instead of pointing along world z forever.
    Rotating 90 deg about y swings the tag normal onto world x: the 0.25 m^2 range
    term moves to index 0, and the trace (rotation-invariant) is unchanged."""
    turned = _pose((0.0, 0.0, 0.0), Rotation.from_euler("y", 90.0, degrees=True))

    covariance = tag_covariance(turned, 1.0)

    np.testing.assert_allclose(
        np.diag(covariance), [0.25, 0.0025, 0.0025, 0.0025, 0.04, 0.04], atol=1e-15
    )
    assert abs(float(np.trace(covariance[:3, :3])) - 0.255) < 1e-15

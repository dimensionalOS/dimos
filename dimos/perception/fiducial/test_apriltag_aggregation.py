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
    AggregationConfig,
    TagAggregator,
    TagObservation,
    aggregate_visits,
    cluster_by_time,
    cluster_medoid,
    gate_reason,
    robust_cluster_pose,
    tag_pixel_size,
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
    # jnav's line-of-sight uses distance + 1e-9, a ~0.003 deg head-on floor.
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


def test_tag_pixel_size_of_known_square() -> None:
    """A 30x30 px axis-aligned corner quad has tag side length 30 px."""
    corners = np.array([[0.0, 0.0], [30.0, 0.0], [30.0, 30.0], [0.0, 30.0]])
    assert abs(tag_pixel_size(corners) - 30.0) < 1e-9


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


def test_gate_reason_each_gate_fires_and_none_skips() -> None:
    """Every gate rejects its own failure; a None field disables only its gate."""
    cfg = AggregationConfig()
    good = _pose((0.0, 0.0, 0.5), Rotation.identity())
    base = dict(distance_m=0.5, view_angle_deg=5.0, reproj_px=1.0, tag_px=40.0, speed_mps_dps=(0.1, 5.0))
    assert gate_reason(_obs(0.0, 1, good, **base), cfg) is None  # type: ignore[arg-type]
    assert gate_reason(_obs(0.0, 1, good, **{**base, "reproj_px": 3.0}), cfg) == "reproj"  # type: ignore[arg-type]
    assert gate_reason(_obs(0.0, 1, good, **{**base, "tag_px": 10.0}), cfg) == "small"  # type: ignore[arg-type]
    assert gate_reason(_obs(0.0, 1, good, **{**base, "distance_m": 2.0}), cfg) == "far"  # type: ignore[arg-type]
    assert gate_reason(_obs(0.0, 1, good, **{**base, "view_angle_deg": 60.0}), cfg) == "oblique"  # type: ignore[arg-type]
    assert gate_reason(_obs(0.0, 1, good, **{**base, "speed_mps_dps": (1.0, 5.0)}), cfg) == "motion"  # type: ignore[arg-type]
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

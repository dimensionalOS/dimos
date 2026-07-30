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

"""Rules and geometry of the reference generator, without the replay.

The sweep that produces ``Measurement`` objects needs a replay, a detector and
a GPU-shaped afternoon; everything that decides what those measurements *mean*
needs neither, and that is where the table's credibility comes from. Four
things are checked here, all on hand-built inputs or on the committed artifact:

* the **qualification rules** -- which views count as independent, how
  candidates cluster, which gate claims a drop, when a label is too ambiguous
  to ask about, and how one object collecting several names is caught;
* the **projection and point selection**, on synthetic points whose pixels and
  world median are computed by hand, so a frame, sign or axis-order mistake in
  the step that turns LiDAR into a position fails here rather than silently
  moving every reference. Both selection paths are covered: the shrunk bbox that
  the table is built with, and the mask path that is one flag away;
* the **two copies of the calibration**, which a recalibration would otherwise
  silently split, and the **funnel arithmetic**, which is the only thing making
  the manifest a description of one sweep rather than a pile of counters;
* the **committed tables themselves**, against invariants a future pipeline
  change must not break -- each reference sits about one measured depth from the
  nearest pose it was seen from, and every committed question is passable on
  that same evidence. These run once per dataset the reference tree ships
  (``reference_sets.dataset_names()``), so a second recording is held to the
  same invariants as the first without a test edit, and a pipeline change that
  only holds on the recording it was tuned against fails here.
"""

from __future__ import annotations

from collections import Counter
import json
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.agents.evals.questions import load_refs, nearest_viewpoint_m
from dimos.agents.evals.reference_sets import dataset_dir, dataset_names, load_question_set
from dimos.agents.evals.teacher import (
    _FALLBACK_D,
    _FALLBACK_K,
    _FALLBACK_SIZE,
    Camera,
    GateParams,
    Measurement,
    Projector,
    QualifyDiagnostics,
    Reference,
    ScanStats,
    _cluster_by_position,
    _one_view_per_frame,
    _xy_spread,
    apply_gates,
    assign_location_groups,
    build_funnel,
    independent_views,
    load_camera,
    measure_detection,
    measurement_drops,
    points_on_mask,
    qualify,
    shrink_bbox,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

GATES = GateParams()


def make_measurement(
    ts: float,
    world_xyz: tuple[float, float, float],
    robot_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    label: str = "houseplant",
    **overrides: Any,
) -> Measurement:
    """A detection that already survived projection, i.e. one gate input."""
    fields: dict[str, Any] = {
        "ts": ts,
        "label": label,
        "conf": 0.5,
        "bbox": (100.0, 100.0, 200.0, 260.0),
        "mean_gray": 80.0,
        "robot_xyz": robot_xyz,
        "world_xyz": world_xyz,
        "n_points": 40,
        "depth_m": 2.0,
        "depth_iqr_m": 0.2,
        "range_m": 2.0,
        # The audit trail the crops draw; no gate reads it.
        "point_source": "bbox",
        "mask_outline": (),
        "inlier_uv": ((150.0, 180.0),),
    }
    fields.update(overrides)
    return Measurement(**fields)


def make_reference(raw_label: str, x: float, y: float = 0.0, z: float = 0.0) -> Reference:
    return Reference(
        dataset="go2_bigoffice",
        raw_label=raw_label,
        location_group="",
        x=x,
        y=y,
        z=z,
        n_views_raw=2,
        n_independent_views=2,
        n_detections=2,
        spread_m=0.1,
        min_baseline_m=0.4,
        max_baseline_m=0.4,
        depth_median_m=2.0,
        mean_conf=0.5,
        frame_ts=[0.0, 1.0],
        robot_poses=[[0.0, 0.0, 0.3], [0.4, 0.0, 0.3]],
    )


# --- per-measurement gates --------------------------------------------------


def test_gate_params_derive_the_sampling_gap_from_the_rate() -> None:
    assert GateParams(sample_hz=4.0).min_frame_gap_s == 0.25


def test_shrink_bbox_pulls_each_edge_toward_the_center() -> None:
    """Corners of a box are mostly background, so the depth sample uses the core."""
    assert shrink_bbox((0.0, 0.0, 100.0, 200.0), 0.2) == (10.0, 20.0, 90.0, 180.0)
    assert shrink_bbox((0.0, 0.0, 100.0, 200.0), 0.0) == (0.0, 0.0, 100.0, 200.0)


@pytest.mark.parametrize(
    ("overrides", "expected"),
    [
        ({}, None),
        ({"n_points": 5}, "min_points"),
        ({"depth_iqr_m": 0.9}, "depth_iqr"),
        ({"range_m": 5.0}, "envelope_range"),
        ({"world_xyz": (1.0, 2.0, 2.4)}, "envelope_height"),
    ],
    ids=["kept", "too_few_points", "depth_spread", "too_far", "too_high"],
)
def test_each_gate_drops_what_it_is_for(overrides: dict[str, Any], expected: str | None) -> None:
    measurement = make_measurement(0.0, **{"world_xyz": (1.0, 2.0, 0.5), **overrides})
    kept, drops = apply_gates([measurement], GATES)
    assert kept == ([] if expected else [measurement])
    assert dict(drops) == ({expected: 1} if expected else {})


def test_the_first_failing_gate_claims_the_drop() -> None:
    """One measurement, one reason -- otherwise the funnel counts add up wrong."""
    kept, drops = apply_gates(
        [make_measurement(0.0, (1.0, 2.0, 0.5), n_points=5, depth_iqr_m=0.9, range_m=5.0)], GATES
    )
    assert kept == []
    assert dict(drops) == {"min_points": 1}


# --- view independence ------------------------------------------------------


def test_views_are_independent_when_the_robot_moved() -> None:
    views = [
        make_measurement(0.0, (1.0, 2.0, 0.5), robot_xyz=(0.0, 0.0, 0.3)),
        make_measurement(0.1, (1.0, 2.0, 0.5), robot_xyz=(0.4, 0.0, 0.3)),
    ]
    assert independent_views(views, GATES) == views


def test_time_alone_does_not_make_two_views_independent() -> None:
    """Independence is spatial: a parked robot has one line of sight, not two.

    The two views are 90 seconds apart from the same spot, which an earlier
    "or enough time elapsed" rule accepted. It should not be: what a second view
    is supposed to contribute is a second angle on the object, and a static
    re-observation contributes agreement with itself. On the reference recording
    the rule never actually decided anything -- rerunning the sweep without it
    produced a byte-identical table -- so it was a claim the numbers did not need.
    """
    views = [
        make_measurement(0.0, (1.0, 2.0, 0.5), robot_xyz=(0.0, 0.0, 0.3)),
        make_measurement(90.0, (1.0, 2.0, 0.5), robot_xyz=(0.05, 0.0, 0.3)),
    ]
    assert [v.ts for v in independent_views(views, GATES)] == [0.0]


def test_near_duplicate_frames_count_as_one_view() -> None:
    """A fast sampling rate must not manufacture agreement out of one glance."""
    views = [
        make_measurement(0.0, (1.0, 2.0, 0.5), robot_xyz=(0.0, 0.0, 0.3)),
        make_measurement(0.1, (1.0, 2.0, 0.5), robot_xyz=(0.05, 0.0, 0.3)),
        make_measurement(0.2, (1.0, 2.0, 0.5), robot_xyz=(0.10, 0.0, 0.3)),
    ]
    assert [v.ts for v in independent_views(views, GATES)] == [0.0]


def test_independent_views_is_greedy_in_timestamp_order() -> None:
    """Greedy is a lower bound on the largest independent set, i.e. conservative.

    The middle view is not far enough from the first, so it is skipped even
    though keeping it instead would also have yielded two views. Selection is by
    timestamp, so shuffling the input cannot change the answer.
    """
    views = [
        make_measurement(0.0, (1.0, 2.0, 0.5), robot_xyz=(0.0, 0.0, 0.3)),
        make_measurement(0.5, (1.0, 2.0, 0.5), robot_xyz=(0.2, 0.0, 0.3)),
        make_measurement(1.2, (1.0, 2.0, 0.5), robot_xyz=(0.5, 0.0, 0.3)),
    ]
    assert [v.ts for v in independent_views(views, GATES)] == [0.0, 1.2]
    assert [v.ts for v in independent_views(list(reversed(views)), GATES)] == [0.0, 1.2]


# --- clustering -------------------------------------------------------------


def test_clustering_separates_distinct_places() -> None:
    near = [make_measurement(0.0, (1.0, 2.0, 0.5)), make_measurement(1.0, (1.2, 2.0, 0.5))]
    far = [make_measurement(2.0, (8.0, 2.0, 0.5))]
    clusters = _cluster_by_position(near + far, GATES.cluster_radius_m)
    assert [len(cluster) for cluster in clusters] == [2, 1]
    assert clusters[1] == far


def test_clustering_follows_the_running_median() -> None:
    """Membership is judged against the cluster's median, not its first member.

    The third candidate is 1.05 m from the first one -- outside the 0.75 m
    radius -- but 0.7 m from the median of the two already in the cluster, so
    it joins. That is what lets a cluster track an object measured from
    several angles instead of splitting on the first outlier.
    """
    clusters = _cluster_by_position(
        [
            make_measurement(0.0, (0.0, 0.0, 0.5)),
            make_measurement(1.0, (0.7, 0.0, 0.5)),
            make_measurement(2.0, (1.05, 0.0, 0.5)),
        ],
        0.75,
    )
    assert len(clusters) == 1
    assert len(clusters[0]) == 3


def test_clustering_does_not_depend_on_input_order() -> None:
    measurements = [
        make_measurement(2.0, (8.0, 2.0, 0.5)),
        make_measurement(0.0, (1.0, 2.0, 0.5)),
        make_measurement(1.0, (1.2, 2.0, 0.5)),
    ]
    shuffled = [measurements[1], measurements[0], measurements[2]]
    as_timestamps = [
        [m.ts for m in cluster] for cluster in _cluster_by_position(measurements, 0.75)
    ]
    assert as_timestamps == [[0.0, 1.0], [2.0]]
    assert [[m.ts for m in c] for c in _cluster_by_position(shuffled, 0.75)] == as_timestamps


def test_one_view_per_frame_keeps_the_best_supported_detection() -> None:
    """Two detections in one frame are one view, and the denser one wins."""
    sparse = make_measurement(1.0, (1.0, 2.0, 0.5), n_points=25)
    dense = make_measurement(1.0, (1.05, 2.0, 0.5), n_points=90)
    assert _one_view_per_frame([sparse, dense]) == [dense]


def test_xy_spread_is_the_widest_disagreement_and_ignores_height() -> None:
    assert _xy_spread([make_measurement(0.0, (1.0, 2.0, 0.5))]) == 0.0
    spread = _xy_spread(
        [
            make_measurement(0.0, (0.0, 0.0, 0.0)),
            make_measurement(1.0, (0.3, 0.4, 9.0)),
            make_measurement(2.0, (0.0, 0.1, 0.0)),
        ]
    )
    assert spread == pytest.approx(0.5)


# --- label qualification ----------------------------------------------------


def test_a_label_seen_from_two_independent_views_qualifies() -> None:
    measurements = [
        make_measurement(0.0, (1.0, 2.0, 0.5), robot_xyz=(0.0, 0.0, 0.3), conf=0.4),
        make_measurement(0.5, (1.1, 2.0, 0.5), robot_xyz=(0.5, 0.0, 0.3), conf=0.6),
    ]
    references, diagnostics, members = qualify(measurements, GATES, "go2_bigoffice")

    assert diagnostics.label_verdicts["qualified"] == 1
    assert diagnostics.clusters_formed == 1
    assert members["houseplant"] == measurements
    assert len(references) == 1
    reference = references[0]
    assert (reference.x, reference.y, reference.z) == (1.05, 2.0, 0.5)
    assert reference.n_views_raw == 2
    assert reference.n_independent_views == 2
    assert reference.n_detections == 2
    assert reference.min_baseline_m == 0.5
    assert reference.mean_conf == 0.5
    assert reference.frame_ts == [0.0, 0.5]
    assert reference.location_group == ""  # assigned once every label exists


def test_a_label_with_two_qualified_places_is_unusable() -> None:
    """Two instances a question could not tell apart -- dropped, not guessed at."""
    measurements = [
        make_measurement(0.0, (1.0, 2.0, 0.5), robot_xyz=(0.0, 0.0, 0.3), label="shelf"),
        make_measurement(0.5, (1.0, 2.0, 0.5), robot_xyz=(0.5, 0.0, 0.3), label="shelf"),
        make_measurement(1.0, (9.0, 2.0, 0.5), robot_xyz=(8.0, 0.0, 0.3), label="shelf"),
        make_measurement(1.5, (9.0, 2.0, 0.5), robot_xyz=(8.5, 0.0, 0.3), label="shelf"),
    ]
    references, diagnostics, members = qualify(measurements, GATES, "go2_bigoffice")

    assert references == []
    assert members == {}
    assert diagnostics.clusters_formed == 2
    assert diagnostics.label_verdicts["ambiguous_multi_instance"] == 1


def test_a_label_seen_only_once_does_not_qualify() -> None:
    measurements = [
        make_measurement(0.0, (1.0, 2.0, 0.5), robot_xyz=(0.00, 0.0, 0.3)),
        make_measurement(0.1, (1.0, 2.0, 0.5), robot_xyz=(0.05, 0.0, 0.3)),
    ]
    references, diagnostics, _ = qualify(measurements, GATES, "go2_bigoffice")

    assert references == []
    assert diagnostics.cluster_drops["insufficient_independent_views"] == 1
    assert diagnostics.label_verdicts["no_qualified_cluster"] == 1


def test_a_label_whose_views_disagree_about_where_it_is_does_not_qualify() -> None:
    """Three independent views that cannot agree on a position are not a reference.

    The robot moves between all three, so the independence gate has no claim on
    the drop; what fails is the spread.
    """
    measurements = [
        make_measurement(0.0, (0.0, 0.0, 0.5), robot_xyz=(0.0, -2.0, 0.3)),
        make_measurement(1.1, (0.45, 0.0, 0.5), robot_xyz=(0.5, -2.0, 0.3)),
        make_measurement(2.2, (0.9, 0.0, 0.5), robot_xyz=(1.0, -2.0, 0.3)),
    ]
    references, diagnostics, _ = qualify(measurements, GATES, "go2_bigoffice")

    assert references == []
    assert diagnostics.cluster_drops["spread_too_large"] == 1
    assert diagnostics.label_verdicts["no_qualified_cluster"] == 1


# --- location groups --------------------------------------------------------


def test_location_groups_link_transitively() -> None:
    """A chain of near-misses is one object: the ends never have to be close.

    ``alpha``-``beta`` and ``beta``-``gamma`` are both inside the radius while
    ``alpha``-``gamma`` is not, and all three are the same shelf bay under
    three detector names.
    """
    references = [
        make_reference("alpha", 0.0),
        make_reference("beta", 0.6),
        make_reference("gamma", 1.2),
        make_reference("delta", 10.0),
    ]
    tagged, table = assign_location_groups(references, 0.75)

    groups = {reference.raw_label: reference.location_group for reference in tagged}
    assert groups == {
        "alpha": "loc-01",
        "beta": "loc-01",
        "gamma": "loc-01",
        "delta": "loc-02",
    }
    assert [reference.raw_label for reference in tagged] == ["alpha", "beta", "delta", "gamma"]
    assert table[0] == {
        "location_group": "loc-01",
        "raw_labels": ["alpha", "beta", "gamma"],
        "centroid": [0.6, 0.0, 0.0],
        "max_pairwise_m": 1.2,
    }
    assert table[1]["raw_labels"] == ["delta"]
    assert table[1]["max_pairwise_m"] == 0.0


def test_location_group_ids_do_not_depend_on_input_order() -> None:
    """Ids are numbered by first label, so they survive a re-run and diff cleanly."""
    references = [
        make_reference("alpha", 0.0),
        make_reference("beta", 0.6),
        make_reference("delta", 10.0),
    ]
    forward, _ = assign_location_groups(references, 0.75)
    backward, _ = assign_location_groups(list(reversed(references)), 0.75)
    assert forward == backward


def test_references_further_apart_than_the_radius_stay_separate() -> None:
    tagged, table = assign_location_groups(
        [make_reference("alpha", 0.0), make_reference("beta", 0.8)], 0.75
    )
    assert [reference.location_group for reference in tagged] == ["loc-01", "loc-02"]
    assert len(table) == 2


def test_labels_stacked_on_top_of_each_other_are_one_location() -> None:
    """Linking is planar: a shelf and the boxes on it are one "go to X" answer.

    The two references are 1.4 m apart in 3-D -- outside the radius -- and
    0.1 m apart on the floor. A 3-D link would let height split them, both
    names would become questions, and an agent that drove to the shelf would be
    marked wrong for whichever name it did not happen to match.
    """
    tagged, table = assign_location_groups(
        [make_reference("shelf", 0.0, z=0.2), make_reference("storage boxes", 0.1, z=1.6)], 0.75
    )
    assert [reference.location_group for reference in tagged] == ["loc-01", "loc-01"]
    assert table[0]["raw_labels"] == ["shelf", "storage boxes"]
    # The reported spread is XY too, i.e. the distance the decision was made on.
    assert table[0]["max_pairwise_m"] == pytest.approx(0.1)


# --- projection -------------------------------------------------------------
#
# The step that turns a LiDAR sweep into a pixel is where a frame, sign or
# axis-order mistake hides best: it moves every reference by a plausible-looking
# amount and nothing else complains. These place synthetic points at known
# offsets from a known pose and check the pixels against the geometry by hand.

#: ``dimos.robot.unitree.go2.connection.BASE_TO_OPTICAL``, rebuilt here rather
#: than imported: that module pulls in the whole robot stack (rerun, reactivex,
#: the WebRTC connection) for two quaternions, and this lane has to stay fast.
#: What is under test is the projection math, so pinning the mount it was
#: computed against is also what the tests want.
BASE_TO_OPTICAL = Transform(
    translation=Vector3(0.3, 0.0, 0.0),
    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
    frame_id="base_link",
    child_frame_id="camera_optical",
)

CAMERA = Camera(
    matrix=np.array(_FALLBACK_K, dtype=np.float64).reshape(3, 3),
    distortion=np.array(_FALLBACK_D, dtype=np.float64).reshape(-1, 1),
    width=_FALLBACK_SIZE[0],
    height=_FALLBACK_SIZE[1],
    source="front_camera_720.yaml (inlined fallback constants)",
)
PROJECTOR = Projector(CAMERA, BASE_TO_OPTICAL, GATES.front_z_m)

#: Principal point and focal lengths of :data:`CAMERA`, spelled out.
CX, CY = float(CAMERA.matrix[0, 2]), float(CAMERA.matrix[1, 2])
FX, FY = float(CAMERA.matrix[0, 0]), float(CAMERA.matrix[1, 1])

#: The camera sits this far ahead of ``base_link`` along the robot's heading.
CAMERA_FORWARD_M = 0.3


def pose_at(x: float, y: float, z: float = 0.3, yaw: float = 0.0) -> tuple[float, ...]:
    """A recorded pose ``(x, y, z, qx, qy, qz, qw)`` with only yaw applied."""
    return (x, y, z, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def expected_pixel(point_optical: tuple[float, float, float]) -> tuple[float, float]:
    """Project one optical-frame point with the equidistant model, by hand.

    Written out from the model's definition rather than delegating to
    ``cv2.fisheye`` -- the point is to have a second opinion about what the
    pixel should be, not to restate the call under test.
    """
    x, y, z = point_optical
    a, b = x / z, y / z
    r = math.hypot(a, b)
    theta = math.atan(r)
    k1, k2, k3, k4 = (float(k) for k in CAMERA.distortion.reshape(-1))
    theta_d = theta * (1.0 + k1 * theta**2 + k2 * theta**4 + k3 * theta**6 + k4 * theta**8)
    scale = theta_d / r if r > 0.0 else 1.0
    return FX * scale * a + CX, FY * scale * b + CY


def test_a_point_on_the_optical_axis_lands_on_the_principal_point() -> None:
    """The zero case: dead ahead, at the camera's height, is (cx, cy) exactly."""
    points = np.array([[2.3, 0.0, 0.3]])
    uv, depth, world, robot_xyz = PROJECTOR.project(points, pose_at(0.0, 0.0))

    assert robot_xyz == (0.0, 0.0, 0.3)
    assert uv[0] == pytest.approx((CX, CY))
    # Depth is measured from the camera, which is 0.3 m ahead of base_link.
    assert depth[0] == pytest.approx(2.3 - CAMERA_FORWARD_M)
    assert world[0] == pytest.approx([2.3, 0.0, 0.3])


def test_the_optical_axis_follows_the_robots_yaw() -> None:
    """The pose's rotation is applied, not just its translation.

    Turned 90 degrees left, "2 m ahead" is +y in the map frame, and the camera
    offset moves with it.
    """
    points = np.array([[1.0, 4.3, 0.3]])
    uv, depth, _, _ = PROJECTOR.project(points, pose_at(1.0, 2.0, yaw=math.pi / 2.0))

    assert uv[0] == pytest.approx((CX, CY))
    assert depth[0] == pytest.approx(2.0)


def test_offsets_move_the_pixel_the_way_the_world_does() -> None:
    """Left in the map frame is left in the image; up is up.

    A sign flip in the base->optical rotation would keep the magnitudes and
    mirror the picture, which no downstream gate could notice.
    """
    ahead = np.array([[2.3, 0.0, 0.3]])
    left = np.array([[2.3, 0.5, 0.3]])  # +y is to the robot's left
    up = np.array([[2.3, 0.0, 0.8]])  # +z is up

    (u_ahead, v_ahead) = PROJECTOR.project(ahead, pose_at(0.0, 0.0))[0][0]
    (u_left, v_left) = PROJECTOR.project(left, pose_at(0.0, 0.0))[0][0]
    (u_up, v_up) = PROJECTOR.project(up, pose_at(0.0, 0.0))[0][0]

    assert u_left < u_ahead and v_left == pytest.approx(v_ahead)
    assert v_up < v_ahead and u_up == pytest.approx(u_ahead)


@pytest.mark.parametrize(
    ("world_point", "optical_point"),
    [
        ((2.3, 0.5, 0.3), (-0.5, 0.0, 2.0)),
        ((2.3, -0.4, 0.9), (0.4, -0.6, 2.0)),
        ((1.3, 0.25, 0.0), (-0.25, 0.3, 1.0)),
    ],
    ids=["left", "right_and_up", "close_and_low"],
)
def test_projected_pixels_match_the_equidistant_model(
    world_point: tuple[float, float, float], optical_point: tuple[float, float, float]
) -> None:
    """map -> optical is ``(-dy, -dz, dx)``, and the pixel is the fisheye model.

    The optical points are spelled out in the parameters, so the test also pins
    the axis convention (x right, y down, z forward) rather than only the
    arithmetic.
    """
    uv, depth, _, _ = PROJECTOR.project(np.array([world_point]), pose_at(0.0, 0.0))

    assert depth[0] == pytest.approx(optical_point[2])
    assert uv[0] == pytest.approx(expected_pixel(optical_point))


def test_points_behind_the_camera_and_off_the_sensor_are_dropped() -> None:
    """Only points the camera could actually have seen survive projection.

    Without the ``front_z`` test a point behind the robot re-emerges mirrored
    in front of it; without the image bounds a point beside the robot lands on
    a pixel that no bbox can legitimately claim.
    """
    behind = np.array([[-1.0, 0.0, 0.3]])
    grazing = np.array([[0.30, 0.0, 0.3]])  # exactly at the camera: depth 0
    beside = np.array([[0.36, 5.0, 0.3]])  # in front, but ~89 degrees off-axis

    assert len(PROJECTOR.project(behind, pose_at(0.0, 0.0))[0]) == 0
    assert len(PROJECTOR.project(grazing, pose_at(0.0, 0.0))[0]) == 0
    assert len(PROJECTOR.project(beside, pose_at(0.0, 0.0))[0]) == 0


def test_projection_keeps_pixels_depths_and_world_points_index_aligned() -> None:
    """A pixel test also has to select the right map-frame point.

    The dropped point sits between the two survivors, so an off-by-one in the
    masking would leave the arrays plausibly shaped and wrongly paired.
    """
    points = np.array([[2.3, 0.5, 0.3], [-1.0, 0.0, 0.3], [3.3, 0.0, 0.3]])
    uv, depth, world, _ = PROJECTOR.project(points, pose_at(0.0, 0.0))

    assert len(uv) == len(depth) == len(world) == 2
    assert world[0] == pytest.approx([2.3, 0.5, 0.3])
    assert world[1] == pytest.approx([3.3, 0.0, 0.3])
    assert depth == pytest.approx([2.0, 3.0])
    assert uv[1] == pytest.approx((CX, CY))  # the second survivor is dead ahead


# --- measuring one detection ------------------------------------------------

BBOX = (100.0, 100.0, 300.0, 300.0)
#: The centre of ``BBOX``, i.e. inside it however hard it is shrunk.
INSIDE_UV = (200.0, 200.0)
#: Inside the raw ``BBOX``, outside the frozen 0.15 shrink of it -- (115, 115)
#: to (285, 285) -- because bbox corners are mostly background.
OUTSIDE_SHRUNK_UV = (110.0, 110.0)

#: Frame size the synthetic masks below are built at; only its bounds matter.
MASK_SHAPE = (400, 400)


def projection(
    rows: list[tuple[tuple[float, float], float, tuple[float, float, float]]],
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Build the ``(uv, depth, world)`` triple ``measure_detection`` consumes."""
    uv = np.array([row[0] for row in rows], dtype=np.float64)
    depth = np.array([row[1] for row in rows], dtype=np.float64)
    world = np.array([row[2] for row in rows], dtype=np.float64)
    return uv, depth, world


def mask_over(*regions: tuple[int, int, int, int]) -> NDArray[np.uint8]:
    """A 0/255 mask like ``Detection2DSeg`` carries, set over *regions* (x1,y1,x2,y2)."""
    mask = np.zeros(MASK_SHAPE, dtype=np.uint8)
    for x1, y1, x2, y2 in regions:
        mask[y1:y2, x1:x2] = 255
    return mask


def test_measure_detection_reports_the_depth_cluster_and_ignores_its_tails() -> None:
    """The position is the map-frame median of the [p25, p75] depth inliers.

    The detection holds a railing in front of the object and a wall behind it,
    both of which are legitimate in-detection points -- they count toward
    ``n_points`` -- but neither may move the position.
    """
    rows = (
        [(INSIDE_UV, 0.8, (0.8, 0.0, 0.5))] * 2  # foreground railing
        + [(INSIDE_UV, 2.0, (2.0, 0.0, 0.5))] * 10  # the object itself
        + [(INSIDE_UV, 6.0, (6.0, 0.0, 0.5))] * 2  # wall behind it
    )
    measured = measure_detection(*projection(rows), BBOX, None, GATES)

    assert measured is not None
    assert measured.n_points == 14  # every non-ground in-box point, not just the inliers
    assert measured.depth_median_m == pytest.approx(2.0)
    assert measured.depth_iqr_m == pytest.approx(0.0)
    assert measured.world_xyz == pytest.approx((2.0, 0.0, 0.5))
    # The pixels of exactly the inliers, for the audit crop -- the two tails are
    # in `n_points` but must not be shown as having voted.
    assert len(measured.inlier_uv) == 10
    assert set(measured.inlier_uv) == {INSIDE_UV}


def test_measure_detection_rejects_the_floor_before_it_can_vote() -> None:
    """The floor is the largest single source of in-detection points.

    Here it is also the *majority* of them, so a median that included it would
    put the reference on the ground several metres short of the object -- which
    is what the ground gate exists to prevent.
    """
    rows = [(INSIDE_UV, 2.0, (2.0, 0.0, 0.5))] * 3 + [(INSIDE_UV, 1.0, (1.0, 0.0, 0.02))] * 5
    measured = measure_detection(*projection(rows), BBOX, None, GATES)

    assert measured is not None
    assert measured.n_points == 3
    assert measured.world_xyz == pytest.approx((2.0, 0.0, 0.5))


def test_measure_detection_only_looks_inside_the_shrunk_box() -> None:
    rows = [(INSIDE_UV, 2.0, (2.0, 0.0, 0.5))] * 3 + [
        (OUTSIDE_SHRUNK_UV, 9.0, (9.0, 0.0, 0.5)),
        ((400.0, 400.0), 9.0, (9.0, 9.0, 0.5)),  # outside the raw box entirely
    ]
    measured = measure_detection(*projection(rows), BBOX, None, GATES)

    assert measured is not None
    assert measured.n_points == 3
    assert measured.source == "bbox"
    assert measured.world_xyz == pytest.approx((2.0, 0.0, 0.5))


@pytest.mark.parametrize(
    ("rows", "reason"),
    [
        ([(INSIDE_UV, 1.0, (1.0, 0.0, 0.02))] * 5, "only the floor is in the box"),
        ([(OUTSIDE_SHRUNK_UV, 2.0, (2.0, 0.0, 0.5))] * 5, "nothing is in the shrunk box"),
        ([], "the sweep put no point on the image at all"),
    ],
    ids=["all_ground", "all_outside", "empty"],
)
def test_measure_detection_reports_nothing_rather_than_guessing(
    rows: list[tuple[tuple[float, float], float, tuple[float, float, float]]], reason: str
) -> None:
    uv, depth, world = (
        projection(rows) if rows else (np.zeros((0, 2)), np.zeros(0), np.zeros((0, 3)))
    )
    assert measure_detection(uv, depth, world, BBOX, None, GATES) is None, reason


def test_a_detection_straddling_two_depths_reports_an_iqr_the_gate_drops() -> None:
    """The IQR is the "is this one object?" signal, and it has to reach the gate.

    A detection spanning a near shelf and a far wall has no honest single
    position; ``measure_detection`` still returns one, and ``apply_gates`` is
    what throws it away -- so the number it hands over must actually be wide.
    """
    # Enough points on each side that the earlier `min_points` gate has no
    # claim on the drop: the one under test is the depth spread.
    rows = [(INSIDE_UV, 1.0, (1.0, 0.0, 0.5))] * 12 + [(INSIDE_UV, 5.0, (5.0, 0.0, 0.5))] * 12
    measured = measure_detection(*projection(rows), BBOX, None, GATES)

    assert measured is not None
    assert measured.n_points >= GATES.min_points
    assert measured.depth_iqr_m > GATES.max_depth_iqr_m

    straddling = make_measurement(
        0.0,
        measured.world_xyz,
        n_points=measured.n_points,
        depth_m=measured.depth_median_m,
        depth_iqr_m=measured.depth_iqr_m,
    )
    kept, drops = apply_gates([straddling], GATES)
    assert kept == []
    assert dict(drops) == {"depth_iqr": 1}


# --- mask-based point selection ---------------------------------------------
#
# Off by default (`teacher.PointSelection` records the measurement that decided
# it: as shipped, `Detection2DSeg` masks are letterbox-misaligned with the frame
# by 12-19 px on this recording). The path is still exercised here, because the
# reason it is off is a property of that data, not of this code.

MASK_GATES = GateParams(point_selection="mask")


def test_points_on_mask_tests_the_pixel_the_point_landed_on() -> None:
    """Nearest-integer sample, and a coordinate that rounds off the sensor is clipped.

    The last point is 0.3 px inside the frame and rounds to ``(400, 400)``, one
    past the last row and column: projection guarantees a pixel is *within* the
    image, which is not the same as rounding to a valid index, and indexing on it
    unclipped would raise rather than answer.
    """
    mask = mask_over((100, 100, 200, 200), (390, 390, 400, 400))
    uv = np.array([[150.0, 150.0], [150.4, 199.4], [250.0, 150.0], [399.7, 399.7]])
    assert points_on_mask(uv, mask).tolist() == [True, True, False, True]


def test_mask_selection_ignores_points_the_box_would_have_admitted() -> None:
    """The point of a mask: a bbox around an L-shaped object is mostly not it.

    Both points sit inside the shrunk bbox, so bbox selection cannot tell them
    apart; the mask covers only the left half, and the wall point behind the
    object's right half is exactly the kind of point that quietly moves a
    reference.
    """
    rows = [((150.0, 200.0), 2.0, (2.0, 0.0, 0.5))] * 4 + [
        ((250.0, 200.0), 9.0, (9.0, 0.0, 0.5))
    ] * 4
    mask = mask_over((100, 100, 200, 300))  # left half of BBOX only

    masked = measure_detection(*projection(rows), BBOX, mask, MASK_GATES)
    boxed = measure_detection(*projection(rows), BBOX, None, GATES)

    assert masked is not None and boxed is not None
    assert masked.source == "mask"
    assert masked.n_points == 4
    assert masked.world_xyz == pytest.approx((2.0, 0.0, 0.5))
    assert boxed.source == "bbox"
    assert boxed.n_points == 8
    assert boxed.world_xyz != pytest.approx(masked.world_xyz)


def test_a_detection_without_a_mask_falls_back_to_the_shrunk_box() -> None:
    """Mask mode is per detection: a plain ``Detection2DBBox`` still gets measured.

    The fallback has to be visible in the result rather than inferred, because
    the manifest reports how many measurements each path produced -- a mask-mode
    run that quietly measured everything by bbox would otherwise look like a
    mask-mode run.
    """
    rows = [(INSIDE_UV, 2.0, (2.0, 0.0, 0.5))] * 4
    measured = measure_detection(*projection(rows), BBOX, None, MASK_GATES)

    assert measured is not None
    assert measured.source == "bbox"
    assert measured.n_points == 4


def test_bbox_mode_ignores_a_mask_that_is_there() -> None:
    """``--point-selection bbox`` means bbox, not "bbox unless a mask turns up"."""
    rows = [((150.0, 200.0), 2.0, (2.0, 0.0, 0.5)), ((250.0, 200.0), 9.0, (9.0, 0.0, 0.5))]
    mask = mask_over((100, 100, 200, 300))

    measured = measure_detection(*projection(rows), BBOX, mask, GATES)
    assert measured is not None
    assert measured.source == "bbox"
    assert measured.n_points == 2


def test_an_empty_mask_reports_nothing_rather_than_falling_back() -> None:
    """A mask the detector left blank is an answer, not a missing one.

    Falling back to the box here would silently measure the background of a
    detection whose own segmentation claimed nothing, which is the opposite of
    what mask selection is for.
    """
    rows = [(INSIDE_UV, 2.0, (2.0, 0.0, 0.5))] * 4
    assert measure_detection(*projection(rows), BBOX, mask_over(), MASK_GATES) is None


# --- the calibration ---------------------------------------------------------


def test_the_inlined_fallback_calibration_matches_the_packaged_yaml() -> None:
    """The two copies of the Go2 front-camera calibration must not diverge.

    ``load_camera`` reads ``dimos/robot/unitree/go2/front_camera_720.yaml`` and
    silently falls back to constants copied out of it, so an install without
    package data still runs. A copy is a copy: if the yaml is ever recalibrated,
    the fallback would keep projecting with the old intrinsics and the only
    symptom would be reference positions that moved on machines where the
    package data was missing.
    """
    camera = load_camera()
    if "fallback" in camera.source:
        pytest.skip(f"packaged calibration is unreachable here ({camera.source})")

    assert tuple(camera.matrix.reshape(-1).tolist()) == pytest.approx(_FALLBACK_K)
    assert tuple(camera.distortion.reshape(-1).tolist()) == pytest.approx(_FALLBACK_D)
    assert (camera.width, camera.height) == _FALLBACK_SIZE


# --- the funnel --------------------------------------------------------------


def test_the_funnel_accounts_for_every_counted_detection() -> None:
    """``raw_detections`` minus the drop buckets is ``surviving_measurements``.

    Built on numbers chosen so every bucket is non-empty, including
    ``dropped_no_lidar_match``: detections on a frame whose LiDAR message was
    missing were already counted as raw and can never be placed, so before they
    had a bucket the identity quietly stopped holding on any recording with a
    gap -- and the reference recording has none, so nothing said so.
    """
    stats = ScanStats(
        raw_detections=100,
        below_conf=3,
        no_lidar_match=7,
        no_points_in_detection=20,
        measured_from_bbox=70,
    )
    drops: Counter[str] = Counter(
        {"min_points": 11, "depth_iqr": 5, "envelope_range": 2, "envelope_height": 1}
    )
    funnel = build_funnel(stats, drops, QualifyDiagnostics(), 4)

    assert sum(measurement_drops(stats, drops).values()) == 49
    assert funnel["surviving_measurements"] == 100 - 49 == 51
    assert funnel["dropped_no_lidar_match"] == 7


@pytest.mark.parametrize("dataset", dataset_names())
def test_the_committed_manifests_funnel_adds_up(dataset: str) -> None:
    """The same identity, on the artifacts rather than on hand-built counters.

    ``measurement_drops`` names the buckets, so this reads them back out of each
    committed manifest instead of re-listing them -- a new drop path that forgets
    to update the manifest fails here.
    """
    funnel = json.loads((dataset_dir(dataset) / "manifest.json").read_text())["funnel"]
    buckets = measurement_drops(ScanStats(), Counter())

    dropped = sum(funnel[key] for key in buckets)
    assert funnel["raw_detections"] - dropped == funnel["surviving_measurements"]
    # Everything that got a position was measured by one path or the other.
    placed = funnel["measured_from_mask"] + funnel["measured_from_bbox"]
    assert (
        placed
        == funnel["raw_detections"]
        - funnel["dropped_below_conf"]
        - funnel["dropped_no_lidar_match"]
        - funnel["dropped_no_points_in_detection"]
    )


# --- the committed tables ---------------------------------------------------

#: How far ``nearest_viewpoint_m`` may sit from the depth the same row reports.
#: The two are measured differently -- one is the closest of several poses, the
#: other the median optical depth over the cluster, from a camera 0.3 m ahead of
#: base_link -- so they are only ever approximately equal. The widest gap in the
#: committed tables is 0.79 m, on ``go2_bigoffice``'s ``dollhouse``: the white
#: service robot that moves between sightings, which is exactly why review
#: dropped it. ``go2_short``'s widest is 0.69 m, on its ``houseplant``.
#:
#: What this tolerance can and cannot catch is worth being exact about, because
#: 0.9 m sounds like a calibration check and is not one. It catches metre-scale
#: mistakes -- projecting in the wrong frame, a dropped sign, a swapped axis
#: order, metres read as something else -- each of which moves this by metres. It
#: does **not** catch the 0.3 m camera offset: forgetting ``BASE_TO_OPTICAL``'s
#: translation shifts every depth by 0.3 m, which stays comfortably inside the
#: tolerance and inside the honest spread of the table. That case is covered by
#: ``test_a_point_on_the_optical_axis_lands_on_the_principal_point`` instead,
#: which asserts the offset exactly.
MAX_VIEWPOINT_DEPTH_MISMATCH_M = 0.9


@pytest.mark.parametrize("dataset", dataset_names())
def test_every_reference_sits_about_one_measured_depth_from_its_nearest_viewpoint(
    dataset: str,
) -> None:
    """``x``/``y``, ``robot_poses`` and ``depth_median_m`` have to agree.

    They come from three different places -- the LiDAR sweep, the recorded pose
    and the optical-frame depths -- so a future change that projects into the
    wrong frame, drops a sign or swaps an axis order breaks their agreement long
    before it produces a table anyone would question by eye.
    """
    for ref in load_refs(dataset_dir(dataset) / "refs.jsonl"):
        mismatch = nearest_viewpoint_m(ref) - ref["depth_median_m"]
        assert abs(mismatch) <= MAX_VIEWPOINT_DEPTH_MISMATCH_M, (
            f"{dataset}/{ref['raw_label']!r}: nearest viewpoint "
            f"{nearest_viewpoint_m(ref):.2f} m vs depth median "
            f"{ref['depth_median_m']:.2f} m ({mismatch:+.2f} m apart)"
        )


@pytest.mark.parametrize("dataset", dataset_names())
def test_every_committed_question_is_passable_on_the_teachers_own_evidence(dataset: str) -> None:
    """The invariant ``questions.build_questions`` enforces, asserted on the artifact.

    The scored goal is a viewpoint, so a question whose object was never seen
    from closer than its own threshold could not be passed however well the
    agent behaved. This is the same check as the generator's, run against the
    committed files rather than against whatever the generator was last fed.
    """
    refs = {ref["raw_label"]: ref for ref in load_refs(dataset_dir(dataset) / "refs.jsonl")}
    questions = load_question_set(dataset)

    assert questions
    for question in questions:
        nearest = nearest_viewpoint_m(refs[question.raw_label])
        assert nearest < question.threshold_m, (
            f"{question.question_id!r} is unpassable by construction: nearest teacher "
            f"viewpoint {nearest:.2f} m >= threshold {question.threshold_m} m"
        )

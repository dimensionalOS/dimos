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

"""Qualification rules of the reference generator, on hand-built measurements.

The sweep that produces ``Measurement`` objects needs a replay, a detector and
a GPU-shaped afternoon; the rules that decide which of them become reference
rows need neither, and they are where the table's credibility comes from. So
these build measurements directly and check the decisions: which views count
as independent, how candidates cluster, which gate claims a drop, when a label
is too ambiguous to ask about, and how one object collecting several names is
caught.
"""

from __future__ import annotations

from typing import Any

import pytest

from dimos.agents.evals.teacher import (
    GateParams,
    Measurement,
    Reference,
    _cluster_by_position,
    _one_view_per_frame,
    _xy_spread,
    apply_gates,
    assign_location_groups,
    independent_views,
    qualify,
    shrink_bbox,
)

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


def test_views_are_independent_when_enough_time_passed() -> None:
    views = [
        make_measurement(0.0, (1.0, 2.0, 0.5)),
        make_measurement(1.5, (1.0, 2.0, 0.5)),
    ]
    assert independent_views(views, GATES) == views


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

    The middle view is neither far enough nor late enough from the first, so it
    is skipped even though keeping it instead would also have yielded two views.
    Selection is by timestamp, so shuffling the input cannot change the answer.
    """
    views = [
        make_measurement(0.0, (1.0, 2.0, 0.5), robot_xyz=(0.0, 0.0, 0.3)),
        make_measurement(0.5, (1.0, 2.0, 0.5), robot_xyz=(0.2, 0.0, 0.3)),
        make_measurement(1.2, (1.0, 2.0, 0.5), robot_xyz=(0.25, 0.0, 0.3)),
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
    measurements = [
        make_measurement(0.0, (0.0, 0.0, 0.5)),
        make_measurement(1.1, (0.45, 0.0, 0.5)),
        make_measurement(2.2, (0.9, 0.0, 0.5)),
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

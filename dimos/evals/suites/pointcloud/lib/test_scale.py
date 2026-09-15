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

"""Check analytic labels against the actual float32 observations, without an LLM."""

from __future__ import annotations

from pathlib import Path

from langchain_core.language_models.fake_chat_models import FakeListChatModel
import numpy as np
import pytest

from dimos.evals.agents.question_answer import QuestionAnswer
from dimos.evals.runner import EvalRunner
from dimos.evals.suites.pointcloud.dataset.pointcloud_scale import build_suite
from dimos.evals.suites.pointcloud.lib.scale import (
    LOCAL_REGION,
    TIMESTAMP,
    CloudSpec,
    GeometryQuestion,
    cloud_specs,
    parse_number,
    points_for,
    prepare_recordings,
    questions,
    score_number,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.mark.parametrize("q", [q for q in questions() if q.quantity == "span"], ids=lambda q: q.id)
def test_span_labels_match_float32_points(q: GeometryQuestion) -> None:
    points = points_for(q.cloud).astype(np.float64)
    measured = np.ptp(points[:, q.axis])
    assert measured == pytest.approx(q.expected, rel=1e-6)


@pytest.mark.parametrize(
    "q", [q for q in questions() if q.quantity == "maximum"], ids=lambda q: q.id
)
def test_coordinate_labels_match_float32_points(q: GeometryQuestion) -> None:
    measured = float(points_for(q.cloud)[:, q.axis].max())
    assert measured == pytest.approx(q.expected, rel=0, abs=q.tolerance * 0.001)


@pytest.mark.parametrize("q", [q for q in questions() if q.quantity == "gap"], ids=lambda q: q.id)
def test_gap_labels_match_largest_projected_point_gap(q: GeometryQuestion) -> None:
    points = points_for(q.cloud).astype(np.float64)
    # Select the local pair by its overall bounding region. The measurement below
    # uses only sampled coordinates, not the analytic facing endpoints.
    local = np.array(q.cloud.boxes[:2])
    lo, hi = local[:, 0].min(axis=0), local[:, 1].max(axis=0)
    selected = points[np.all((points >= lo - q.tolerance) & (points <= hi + q.tolerance), axis=1)]
    measured = np.diff(np.unique(selected[:, q.axis])).max()
    assert measured == pytest.approx(q.expected, rel=1e-6)


@pytest.mark.parametrize("layout", ["wide", "narrow"])
def test_prepared_crop_is_an_exact_subset_and_preserves_local_gap(layout: str) -> None:
    specs = {s.id: s for s in cloud_specs()}
    overview = points_for(specs[f"overview_{layout}"])
    crop = points_for(specs[f"crop_{layout}"])
    lo, hi = LOCAL_REGION
    selected = overview[np.all((overview >= lo) & (overview <= hi), axis=1)]

    np.testing.assert_array_equal(crop, selected)
    np.testing.assert_array_equal(crop, points_for(specs[f"room_{layout}"]))
    assert len(crop) < len(overview)
    assert np.ptp(overview[:, 0]) == 1000.0


def test_optical_fixture_is_a_rigid_rotation_of_the_room() -> None:
    specs = {s.id: s for s in cloud_specs()}
    room = points_for(specs["room_wide"]).astype(np.float64)
    rotation = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    expected = np.unique((room @ rotation.T + [0, 0, 12]).astype(np.float32), axis=0)

    np.testing.assert_allclose(points_for(specs["optical"]), expected, atol=1e-6)
    assert np.linalg.det(rotation) == 1.0


@pytest.mark.parametrize(
    ("reply", "expected", "tolerance", "score"),
    [
        ("0.002", 0.002, 0.00002, 1.0),
        ("  -1250\n", -1250, 0.01, 1.0),
        ("2e-3", 0.002, 0.00002, 0.0),
        ("-1.25E+3", -1250, 0.01, 0.0),
        ("0.002 m", 0.002, 0.00002, 0.0),
        ("The answer is 0.002", 0.002, 0.00002, 0.0),
        ("0", 0, 0.05, 1.0),
        ("3000003", 3000003, 0.04, 1.0),
        ("3000000", 3000003, 0.04, 0.0),
        ("0", 0.002, 0.00002, 0.0),
        ("2", 0.002, 0.00002, 0.0),
        ("NaN", 2, 0.02, 0.0),
        ("inf", 2, 0.02, 0.0),
        ("unknown", 2, 0.02, 0.0),
        ("1 or 2", 2, 0.02, 0.0),
    ],
)
def test_numeric_grading_handles_scales_and_invalid_answers(
    reply: str, expected: float, tolerance: float, score: float
) -> None:
    assert score_number(expected, parse_number(reply), tolerance=tolerance) == score


@pytest.fixture
def recordings(tmp_path: Path) -> Path:
    prepare_recordings(tmp_path)
    return tmp_path


@pytest.mark.parametrize("spec", cloud_specs(), ids=lambda s: s.id)
def test_recordings_contain_exact_points_and_frame_without_label_streams(
    recordings: Path, spec: CloudSpec
) -> None:
    with SqliteStore(path=str(spec.path(recordings)), must_exist=True) as store:
        observations = store.stream("pointcloud", PointCloud2).to_list()
        assert store.list_streams() == ["pointcloud"]
        assert len(observations) == 1
        assert observations[0].ts == TIMESTAMP
        assert observations[0].data.frame_id == spec.frame_id
        np.testing.assert_array_equal(observations[0].data.points_f32(), points_for(spec))


def test_preparing_twice_does_not_append_duplicate_observations(recordings: Path) -> None:
    prepare_recordings(recordings)
    for spec in cloud_specs():
        with SqliteStore(path=str(spec.path(recordings)), must_exist=True) as store:
            assert store.stream("pointcloud", PointCloud2).count() == 1


def test_all_cases_run_through_existing_encoder_and_grader(
    recordings: Path, tmp_path: Path
) -> None:
    cases = build_suite(recordings)
    model = FakeListChatModel(
        responses=[f"{q.expected:.12f}".rstrip("0").rstrip(".") for q in questions()]
    )
    results = EvalRunner(out_dir=tmp_path / "results").run(cases, QuestionAnswer(chat_model=model))

    assert len(results) == 35
    assert all(r.passed and not r.error for r in results)
    assert len({r.case_id for r in results}) == len(cases)

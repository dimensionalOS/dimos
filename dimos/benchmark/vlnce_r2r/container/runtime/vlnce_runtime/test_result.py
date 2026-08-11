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

import json

import pytest

from dimos.benchmark.vlnce_r2r.container.runtime.vlnce_runtime.result import (
    ResultError,
    build_result,
    publish_result,
)


def _private_case():
    return {
        "attempt_id": "attempt-1",
        "case_id": "case-1",
        "case_fingerprint": "a" * 64,
        "benchmark": "vlnce_r2r",
        "dataset_revision": "R2R_VLNCE_v1-3",
        "split": "train",
        "episode_id": "515",
        "scene_id": "mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",
        "upstream_revision": "b" * 40,
        "runtime_image_digest": "c" * 64,
        "protocol_revision": "vlnce-public.v1",
        "result_schema_revision": "vlnce-result.v1",
        "condition_label": "dimos_geometry_training_scene_development",
    }


def _metrics(success=0.0):
    return {
        "distance_to_goal": 6.8,
        "success": success,
        "spl": 0.0,
        "ndtw": 0.3,
        "path_length": 0.0,
        "oracle_success": 0.0,
        "steps_taken": 1.0,
    }


def test_result_preserves_official_values_and_binds_trajectory() -> None:
    result = build_result(
        _private_case(),
        "submitted",
        [[0.0, 0.0, 0.0]],
        _metrics(),
        {"habitat_sim": "0.1.7"},
        1.5,
    )

    assert result["metrics"] == {
        "DISTANCE_TO_GOAL": 6.8,
        "SUCCESS": 0.0,
        "SPL": 0.0,
        "NDTW": 0.3,
        "PATH_LENGTH": 0.0,
        "ORACLE_SUCCESS": 0.0,
        "STEPS_TAKEN": 1.0,
    }
    assert result["trajectory"] == {
        "sha256": "f2ca8ff4fda02d669a4ab05f5eaf6677db92acaf806baabf9c49b994cee36f96",
        "points": 1,
    }


def test_result_publishes_atomically_exactly_once(tmp_path) -> None:
    result = build_result(_private_case(), "timeout", [[0.0, 0.0, 0.0]], _metrics(), {}, 10.0)
    path = tmp_path / "vlnce-result.v1.json"

    publish_result(path, result)

    assert json.loads(path.read_text()) == result
    assert list(tmp_path.iterdir()) == [path]
    with pytest.raises(ResultError, match="already published"):
        publish_result(path, result)


def test_result_rejects_missing_or_non_finite_official_metrics() -> None:
    metrics = _metrics()
    del metrics["ndtw"]
    with pytest.raises(ResultError, match="metric set"):
        build_result(_private_case(), "submitted", [[0.0, 0.0, 0.0]], metrics, {}, 1.0)

    metrics = _metrics()
    metrics["spl"] = float("nan")
    with pytest.raises(ResultError, match="must be finite"):
        build_result(_private_case(), "submitted", [[0.0, 0.0, 0.0]], metrics, {}, 1.0)

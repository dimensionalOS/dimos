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
from pathlib import Path

import pytest

from dimos.benchmark.vlnce_r2r.models import VlnceTaskManifest
from dimos.benchmark.vlnce_r2r.native_result import validate_native_result


def _case() -> VlnceTaskManifest:
    path = Path(__file__).parent / "cases/mp3d-example-episode-515/task.json"
    return VlnceTaskManifest.model_validate_json(path.read_bytes())


def _result(case: VlnceTaskManifest, attempt_id: str) -> bytes:
    source = case.source
    return json.dumps(
        {
            "schema_version": "vlnce-result.v1",
            "attempt_id": attempt_id,
            "case_id": case.case_id,
            "case_fingerprint": case.fingerprint,
            "benchmark": source.benchmark,
            "dataset_revision": source.dataset_revision,
            "split": source.split,
            "episode_id": source.episode_id,
            "scene_id": source.scene_id,
            "upstream_revision": source.upstream_revision,
            "runtime_image_digest": source.preparation.image.image_digest,
            "protocol_revision": source.protocol_revision,
            "result_schema_revision": source.result_schema_revision,
            "condition_label": source.condition_label,
            "terminal_reason": "submitted",
            "duration_seconds": 12.0,
            "trajectory": {"sha256": "a" * 64, "points": 4},
            "metrics": {
                "DISTANCE_TO_GOAL": 0.8,
                "SUCCESS": 1.0,
                "SPL": 0.7,
                "NDTW": 0.9,
                "PATH_LENGTH": 4.2,
                "ORACLE_SUCCESS": 1.0,
                "STEPS_TAKEN": 42.0,
            },
            "runtime": {"habitat_sim": "0.1.7"},
        }
    ).encode()


def test_validates_pinned_schema_and_exact_attempt_identity() -> None:
    case = _case()

    result = validate_native_result(_result(case, "attempt-1"), case=case, attempt_id="attempt-1")

    assert result.metrics.SUCCESS == 1.0
    assert result.terminal_reason == "submitted"


def test_rejects_a_changed_schema_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    schema = tmp_path / "result-schema.v1.json"
    schema.write_text("{}\n", encoding="utf-8")
    monkeypatch.setattr("dimos.benchmark.vlnce_r2r.native_result.RESULT_SCHEMA_PATH", schema)
    case = _case()

    with pytest.raises(ValueError, match="pinned schema"):
        validate_native_result(_result(case, "attempt-1"), case=case, attempt_id="attempt-1")

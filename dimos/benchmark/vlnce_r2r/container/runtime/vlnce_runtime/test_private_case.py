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

import gzip
import hashlib
import json

import pytest

from dimos.benchmark.vlnce_r2r.container.runtime.vlnce_runtime.private_case import (
    IDENTITY_FIELDS,
    PrivateCaseError,
    load_private_case,
)


def _write_binding(tmp_path):
    episode = {
        "episode_id": "515",
        "scene_id": "mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",
        "instruction": {"instruction_text": "Wait at the toilet. "},
        "reference_path": [[0.0, 0.0, 0.0], [1.0, 0.0, 1.0]],
    }
    episode_sha256 = hashlib.sha256(
        json.dumps(episode, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    instruction = episode["instruction"]["instruction_text"]
    private_case = {
        "schema_version": "vlnce-private-case.v1",
        "benchmark": "vlnce_r2r",
        "attempt_id": "attempt-1",
        "case_id": "17DRP5sb8fy-smoke",
        "case_fingerprint": "a" * 64,
        "upstream_revision": "729d141b2ee10628061ada74dd3a5b9f70faeba5",
        "dataset_revision": "R2R_VLNCE_v1-3",
        "split": "train",
        "episode_id": "515",
        "episode_sha256": episode_sha256,
        "scene_id": episode["scene_id"],
        "instruction": instruction,
        "instruction_sha256": hashlib.sha256(instruction.encode()).hexdigest(),
        "runtime_image_digest": "b" * 64,
        "protocol_revision": "vlnce-public.v1",
        "result_schema_revision": "vlnce-result.v1",
        "condition_label": "dimos_geometry_training_scene_development",
        "timeout_seconds": 300.0,
    }
    private_path = tmp_path / "private-case.json"
    private_path.write_text(json.dumps(private_case))
    dataset_path = tmp_path / "train.json.gz"
    with gzip.open(dataset_path, "wt", encoding="utf-8") as handle:
        json.dump({"episodes": [episode]}, handle)
    expected = {field: private_case[field] for field in IDENTITY_FIELDS}
    return private_case, private_path, dataset_path, expected


def test_private_case_verifies_all_identities_and_mounted_episode(tmp_path):
    private_case, private_path, dataset_path, expected = _write_binding(tmp_path)

    loaded, episode = load_private_case(private_path, dataset_path, expected)

    assert loaded == private_case
    assert episode["episode_id"] == "515"


@pytest.mark.parametrize(
    "field",
    [
        "attempt_id",
        "case_id",
        "dataset_revision",
        "split",
        "episode_id",
        "scene_id",
        "instruction",
        "runtime_image_digest",
        "protocol_revision",
        "result_schema_revision",
    ],
)
def test_private_case_rejects_each_foreign_identity(tmp_path, field):
    _, private_path, dataset_path, expected = _write_binding(tmp_path)
    expected[field] = "c" * 64 if field.endswith("digest") else "foreign"

    with pytest.raises(PrivateCaseError, match="does not match the attempt"):
        load_private_case(private_path, dataset_path, expected)


def test_private_case_rejects_substituted_episode_payload(tmp_path):
    _, private_path, dataset_path, expected = _write_binding(tmp_path)
    with gzip.open(dataset_path, "wt", encoding="utf-8") as handle:
        json.dump(
            {
                "episodes": [
                    {
                        "episode_id": "515",
                        "scene_id": "mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",
                        "instruction": {"instruction_text": "Wait at the toilet. "},
                        "reference_path": [[9.0, 9.0, 9.0]],
                    }
                ]
            },
            handle,
        )

    with pytest.raises(PrivateCaseError, match="payload does not match"):
        load_private_case(private_path, dataset_path, expected)

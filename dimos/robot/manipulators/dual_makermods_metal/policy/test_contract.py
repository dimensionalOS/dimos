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

"""The manifest is the deployment contract: unknown or mismatched input fails closed."""

import json

import numpy as np
import pytest

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    HOME,
    JOINT_ORDER,
    SCHEMA,
    CheckpointSpec,
    ModelSpec,
    PolicyContractError,
    PolicyManifest,
    check_feedback,
    load_manifest,
    load_normalizer,
    sha256_file,
    validate_manifest,
    write_manifest,
)


def _manifest(**overrides) -> PolicyManifest:
    return PolicyManifest(
        **{
            "name": "metal_arm_policy",
            "checkpoint": CheckpointSpec(
                sha256="a" * 64,
                iteration=38000,
                source_sha256="b" * 64,
                recipe_variant="cmdlp005-ori10-rawex001-v1",
            ),
            "model": ModelSpec(onnx_sha256="c" * 64, normalizer_sha256="d" * 64),
            **overrides,
        }
    )


def test_manifest_round_trips_through_json(tmp_path):
    manifest = _manifest()
    path = write_manifest(manifest, tmp_path / "manifest.json")
    loaded = load_manifest(path)
    assert loaded == manifest


def test_manifest_rejects_unknown_schema():
    payload = _manifest().model_dump(by_alias=True)
    payload["schema"] = "dimos.metal_arm_policy.manifest/0"
    with pytest.raises(PolicyContractError, match="schema"):
        validate_manifest(payload)


def test_manifest_rejects_unknown_keys():
    payload = _manifest().model_dump(by_alias=True)
    payload["controller"] = {"gain": 1.0}
    with pytest.raises(PolicyContractError, match="Extra inputs"):
        validate_manifest(payload)


def test_manifest_rejects_reordered_joints():
    payload = _manifest().model_dump(by_alias=True)
    payload["observation"]["joint_order"] = list(reversed(JOINT_ORDER))
    with pytest.raises(PolicyContractError, match="joint order"):
        validate_manifest(payload)


def test_manifest_rejects_foreign_frame_layout():
    payload = _manifest().model_dump(by_alias=True)
    payload["observation"]["frame_fields"] = [{"name": "q", "width": 576}]
    with pytest.raises(PolicyContractError, match="frame layout"):
        validate_manifest(payload)


def test_manifest_rejects_home_outside_limits():
    payload = _manifest().model_dump(by_alias=True)
    payload["action"]["lower"] = [-0.1] * 12
    payload["action"]["upper"] = [0.1] * 12
    with pytest.raises(PolicyContractError, match="home pose"):
        validate_manifest(payload)


def test_manifest_rejects_inconsistent_observation_dimensions():
    payload = _manifest().model_dump(by_alias=True)
    payload["model"]["obs_dim"] = 96
    with pytest.raises(PolicyContractError, match="obs_dim"):
        validate_manifest(payload)


def test_artifact_hashes_are_verified(tmp_path):
    manifest = _manifest()
    onnx = tmp_path / "metal_arm_policy.onnx"
    normalizer = tmp_path / "metal_arm_policy.normalizer.npz"
    onnx.write_bytes(b"not-a-real-graph")
    np.savez(normalizer, mean=np.zeros(576, np.float32), std=np.ones(576, np.float32))
    manifest = _manifest(
        model=ModelSpec(onnx_sha256=sha256_file(onnx), normalizer_sha256=sha256_file(normalizer))
    )
    assert manifest.verify_artifacts(tmp_path) == (onnx, normalizer)
    onnx.write_bytes(b"tampered")
    with pytest.raises(PolicyContractError, match="hash mismatch"):
        manifest.verify_artifacts(tmp_path)


def test_missing_artifact_fails_before_inference(tmp_path):
    with pytest.raises(PolicyContractError, match="not found"):
        _manifest().verify_artifacts(tmp_path)


def test_normalizer_rejects_non_finite_and_wrong_width(tmp_path):
    bad = tmp_path / "bad.npz"
    np.savez(bad, mean=np.zeros(576, np.float32), std=np.ones(576, np.float32))
    mean, std = load_normalizer(bad)
    assert mean.shape == (1, 576) and std.shape == (1, 576)
    np.savez(bad, mean=np.zeros(96, np.float32), std=np.ones(96, np.float32))
    with pytest.raises(PolicyContractError, match="width"):
        load_normalizer(bad)
    mean = np.zeros(576, np.float32)
    mean[7] = np.nan
    np.savez(bad, mean=mean, std=np.ones(576, np.float32))
    with pytest.raises(PolicyContractError, match="non-finite"):
        load_normalizer(bad)


def test_feedback_outside_training_limits_is_rejected():
    manifest = _manifest()
    q = np.asarray(HOME, dtype=np.float32)
    check_feedback(q, np.zeros(12, np.float32), manifest)
    q[2] = -1.0  # joint3's trained range starts at 0 rad
    with pytest.raises(PolicyContractError, match="outside the trained limits"):
        check_feedback(q, np.zeros(12, np.float32), manifest)
    with pytest.raises(PolicyContractError, match="non-finite"):
        check_feedback(np.full(12, np.nan, np.float32), np.zeros(12, np.float32), manifest)


def test_manifest_json_matches_published_schema_name(tmp_path):
    path = write_manifest(_manifest(), tmp_path / "manifest.json")
    assert json.loads(path.read_text())["schema"] == SCHEMA

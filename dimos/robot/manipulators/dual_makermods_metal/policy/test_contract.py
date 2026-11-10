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
from pydantic import ValidationError
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
            "schema": SCHEMA,
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


def test_manifest_round_trips_and_publishes_the_schema(tmp_path):
    manifest = _manifest()
    path = write_manifest(manifest, tmp_path / "manifest.json")
    assert json.loads(path.read_text())["schema"] == SCHEMA
    assert load_manifest(path) == manifest
    assert PolicyManifest.model_validate(manifest.model_dump(by_alias=True)) == manifest


def _unknown_schema(payload):
    payload["schema"] = "dimos.metal_arm_policy.manifest/0"


def _unknown_key(payload):
    payload["controller"] = {"gain": 1.0}


def _reordered_joints(payload):
    payload["observation"]["joint_order"] = list(reversed(JOINT_ORDER))


def _foreign_frame_layout(payload):
    payload["observation"]["frame_fields"] = [{"name": "q", "width": 576}]


def _home_outside_limits(payload):
    payload["action"]["lower"] = [-0.1] * 12
    payload["action"]["upper"] = [0.1] * 12


def _inconsistent_obs_dim(payload):
    payload["model"]["obs_dim"] = 96


@pytest.mark.parametrize(
    ("mutate", "match"),
    [
        (_unknown_schema, "schema"),
        (_unknown_key, "Extra inputs"),
        (_reordered_joints, "joint order"),
        (_foreign_frame_layout, "frame layout"),
        (_home_outside_limits, "home pose"),
        (_inconsistent_obs_dim, "obs_dim"),
    ],
    ids=["schema", "unknown-key", "joint-order", "frame-layout", "home-limits", "obs-dim"],
)
def test_manifest_rejects_invalid_payloads(mutate, match):
    payload = _manifest().model_dump(by_alias=True)
    mutate(payload)
    with pytest.raises(PolicyContractError, match=match):
        validate_manifest(payload)


def test_artifacts_are_verified_before_inference(tmp_path):
    with pytest.raises(PolicyContractError, match="not found"):
        _manifest().verify_artifacts(tmp_path)

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


@pytest.mark.parametrize("missing", ["omitted", "null"], ids=["omitted", "null"])
def test_manifest_requires_schema_on_every_validation_surface(tmp_path, missing):
    payload = _manifest().model_dump(by_alias=True)
    if missing == "omitted":
        payload.pop("schema")
    else:
        payload["schema"] = None
    with pytest.raises(PolicyContractError, match="schema"):
        validate_manifest(payload)
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload))
    with pytest.raises(PolicyContractError, match="schema"):
        load_manifest(path)
    with pytest.raises(ValidationError, match="schema"):
        PolicyManifest.model_validate(payload)


NON_FINITE_SCALARS = (
    ("timing", "policy_hz"),
    ("timing", "physics_dt"),
    ("action", "slew_limit"),
    ("action", "action_scale"),
    ("action", "clamp_margin"),
    ("action", "low_pass_alpha"),
    ("observation", "dq_scale"),
    ("normalizer", "eps"),
)


@pytest.mark.parametrize(("group", "key"), NON_FINITE_SCALARS)
@pytest.mark.parametrize(
    "value", [float("inf"), float("-inf"), float("nan")], ids=["inf", "neg_inf", "nan"]
)
def test_manifest_rejects_non_finite_scalars(group, key, value):
    payload = _manifest().model_dump(by_alias=True)
    payload[group][key] = value
    with pytest.raises(PolicyContractError, match=f"{group}.{key}"):
        validate_manifest(payload)


def test_manifest_rejects_non_finite_vector_elements():
    payload = _manifest().model_dump(by_alias=True)
    payload["action"]["home"] = [float("inf"), *payload["action"]["home"][1:]]
    with pytest.raises(PolicyContractError, match="action.home"):
        validate_manifest(payload)
    payload = _manifest().model_dump(by_alias=True)
    payload["station"]["left_mount_xyz"] = [0.0, float("nan"), 0.0]
    with pytest.raises(PolicyContractError, match="station.left_mount_xyz"):
        validate_manifest(payload)


def test_manifest_accepts_finite_scalar_overrides():
    payload = _manifest().model_dump(by_alias=True)
    payload["timing"]["policy_hz"] = 19.5
    payload["action"]["slew_limit"] = 0.05
    payload["observation"]["dq_scale"] = 4.0
    manifest = validate_manifest(payload)
    assert manifest.timing.policy_hz == 19.5
    assert manifest.action.slew_limit == 0.05
    assert manifest.observation.dq_scale == 4.0

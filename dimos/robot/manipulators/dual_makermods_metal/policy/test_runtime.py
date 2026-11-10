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

"""Action decode order and the causal observation history.

The decoder is the part of the trained controller that must be reproduced
exactly; these tests pin every step of it against hand-computed values.
"""

import numpy as np
import pytest

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    HOME,
    SCHEMA,
    CheckpointSpec,
    ModelSpec,
    PolicyContractError,
    PolicyManifest,
)
from dimos.robot.manipulators.dual_makermods_metal.policy.runtime import ActionDecoder


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


def test_zero_action_holds_home():
    decoder = ActionDecoder(_manifest())
    command = decoder.decode(np.zeros(12, np.float32))
    assert np.allclose(command, HOME)
    assert np.allclose(decoder.previous_action, 0.0)


def test_action_scale_and_low_pass_are_applied_in_order():
    manifest = _manifest()
    decoder = ActionDecoder(manifest)
    command = decoder.decode(np.ones(12, np.float32))
    desired_step = np.clip(
        manifest.action.low_pass_alpha * manifest.action.action_scale,
        -manifest.action.slew_limit,
        manifest.action.slew_limit,
    )
    expected = np.asarray(HOME) + desired_step
    assert np.allclose(command, expected, atol=1e-6)


def test_saturation_clips_action_before_decoding():
    manifest = _manifest()
    decoder = ActionDecoder(manifest)
    saturated = decoder.decode(np.full(12, 10.0, np.float32))
    clipped = ActionDecoder(manifest).decode(np.full(12, 1.0, np.float32))
    assert np.allclose(saturated, clipped)
    assert np.allclose(decoder.previous_action, 1.0)


def test_per_tick_slew_limit_caps_every_joint():
    manifest = _manifest()
    decoder = ActionDecoder(manifest)
    previous = decoder.issued
    command = decoder.decode(np.ones(12, np.float32))
    assert np.all(np.abs(command - previous) <= manifest.action.slew_limit + 1e-6)


def test_desired_target_stays_inside_training_limits_with_margin():
    manifest = _manifest()
    decoder = ActionDecoder(manifest)
    for _ in range(400):
        decoder.decode(np.ones(12, np.float32))
    command = decoder.issued
    assert np.all(
        command <= np.asarray(manifest.action.upper) - manifest.action.clamp_margin + 1e-5
    )
    assert np.all(
        command >= np.asarray(manifest.action.lower) + manifest.action.clamp_margin - 1e-5
    )


def test_reset_seeds_from_measured_pose_not_home():
    manifest = _manifest()
    decoder = ActionDecoder(manifest)
    measured = np.asarray(HOME, np.float32) + 0.05
    decoder.reset(measured)
    assert np.allclose(decoder.issued, measured)
    assert np.allclose(decoder.previous_action, 0.0)


def test_reset_rejects_measured_pose_outside_training_limits():
    decoder = ActionDecoder(_manifest())
    invalid = np.asarray(HOME, np.float32)
    invalid[2] = -0.5
    with pytest.raises(PolicyContractError, match="outside the trained limits"):
        decoder.reset(invalid)


def test_decode_rejects_non_finite_actions():
    decoder = ActionDecoder(_manifest())
    with pytest.raises(PolicyContractError, match="non-finite"):
        decoder.decode(np.full(12, np.nan, np.float32))


def test_decode_rejects_wrong_shape():
    decoder = ActionDecoder(_manifest())
    with pytest.raises(PolicyContractError, match="shape"):
        decoder.decode(np.zeros(11, np.float32))

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

"""R1Pro feature validation at the actual LeRobot checkpoint boundary."""

from dimos_lerobot.prepare_r1pro_dataset import stable_joint_statistics
from dimos_lerobot.runtime import (
    R1ProLeRobotPolicyRuntime,
    R1ProPickPlacePolicyRuntime,
    _validate_features,
)
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
import numpy as np
import pytest

from dimos.experimental.isolated_python.bootstrap import validate_runtime
from dimos.imitation.policy.lerobot.module import R1ProLeRobotPolicy, R1ProPickPlacePolicy
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_IO, R1PRO_SIM_ACT_IO


@pytest.mark.parametrize("state_width,action_width", [(14, 18), (18, 14)])
def test_openyam_joint_width_is_rejected_even_when_camera_keys_match(state_width, action_width):
    config = ACTConfig(
        device="cpu",
        pretrained_backbone_weights=None,
        input_features={
            "observation.images.overview": PolicyFeature(
                type=FeatureType.VISUAL, shape=(3, 240, 320)
            ),
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(state_width,)),
        },
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(action_width,))},
    )
    with pytest.raises(ValueError, match="does not match"):
        _validate_features(config, R1PRO_SIM_ACT_IO)


def test_r1pro_isolated_runtime_matches_declared_rpc_contract():
    assert validate_runtime(R1ProLeRobotPolicy, R1ProLeRobotPolicyRuntime) is None


def test_pick_place_runtime_matches_declared_rpc_contract():
    assert validate_runtime(R1ProPickPlacePolicy, R1ProPickPlacePolicyRuntime) is None


def test_grasping_checkpoint_requires_both_cameras_and_gripper_actions():
    config = ACTConfig(
        device="cpu",
        pretrained_backbone_weights=None,
        input_features={
            "observation.images.head": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 160, 160)),
            "observation.images.right_wrist": PolicyFeature(
                type=FeatureType.VISUAL, shape=(3, 160, 160)
            ),
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(20,)),
        },
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(20,))},
    )
    _validate_features(config, R1PRO_PICK_PLACE_IO)
    del config.input_features["observation.images.right_wrist"]
    with pytest.raises(ValueError, match="right_wrist"):
        _validate_features(config, R1PRO_PICK_PLACE_IO)


def test_constant_action_normalization_does_not_amplify_rounding_or_passive_vibration():
    actions = np.tile(np.array([0.4, 0.05, 0.0], dtype=np.float32), (222, 1))
    actions[:, 2] = np.linspace(-0.5, 0.5, 222)
    states = actions.copy()
    states[:, 0] += np.linspace(-1e-5, 1e-5, 222, dtype=np.float32)
    stats = stable_joint_statistics(states, actions)
    action = stats["action"]
    state = stats["observation.state"]
    normalized_actions = (actions - np.asarray(action["mean"])) / action["std"]
    normalized_states = (states - np.asarray(state["mean"])) / state["std"]
    np.testing.assert_allclose(normalized_actions[:, :2], 0.0, atol=1e-10)
    assert np.max(np.abs(normalized_states[:, :2])) < 0.0001
    assert np.std(normalized_actions[:, 2]) == pytest.approx(1)
    assert np.all(np.asarray(action["mean"]) >= action["min"])
    assert np.all(np.asarray(action["mean"]) <= action["max"])

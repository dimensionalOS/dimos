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

from dimos_lerobot.runtime import R1ProLeRobotPolicyRuntime, _validate_features
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
import pytest

from dimos.experimental.isolated_python.bootstrap import validate_runtime
from dimos.imitation.policy.lerobot.module import R1ProLeRobotPolicy
from dimos.robot.galaxea.r1pro.learning import R1PRO_SIM_ACT_IO


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

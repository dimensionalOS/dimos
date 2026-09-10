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

"""Robot profile bindings for the isolated LeRobot backend."""

from dimos.imitation.policy.module import PolicyRolloutConfig, declare_policy_module
from dimos.robot.galaxea.r1pro.learning import (
    R1PRO_PACKING_IO,
    R1PRO_PICK_PLACE_IO,
    R1PRO_SIM_ACT_IO,
)
from dimos.robot.manipulators.dual_openyam.learning import DUAL_OPENYAM_LEROBOT_IO
from dimos.robot.manipulators.openyam.learning import OPENYAM_QUEST_IO


class LeRobotPolicyConfig(PolicyRolloutConfig):
    """LeRobot checkpoint and common rollout settings."""


OpenYamLeRobotPolicy = declare_policy_module(
    "OpenYamLeRobotPolicy",
    __name__,
    OPENYAM_QUEST_IO,
    LeRobotPolicyConfig,
    "dimos_lerobot.runtime:LeRobotPolicyRuntime",
)


DualOpenYamLeRobotPolicy = declare_policy_module(
    "DualOpenYamLeRobotPolicy",
    __name__,
    DUAL_OPENYAM_LEROBOT_IO,
    LeRobotPolicyConfig,
    "dimos_lerobot.runtime:DualOpenYamLeRobotPolicyRuntime",
)


R1ProLeRobotPolicy = declare_policy_module(
    "R1ProLeRobotPolicy",
    __name__,
    R1PRO_SIM_ACT_IO,
    LeRobotPolicyConfig,
    "dimos_lerobot.runtime:R1ProLeRobotPolicyRuntime",
)


R1ProPickPlacePolicy = declare_policy_module(
    "R1ProPickPlacePolicy",
    __name__,
    R1PRO_PICK_PLACE_IO,
    LeRobotPolicyConfig,
    "dimos_lerobot.runtime:R1ProPickPlacePolicyRuntime",
)


R1ProPackingPolicy = declare_policy_module(
    "R1ProPackingPolicy",
    __name__,
    R1PRO_PACKING_IO,
    LeRobotPolicyConfig,
    "dimos_lerobot.runtime:R1ProPackingPolicyRuntime",
)

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

import pytest

from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.robot.manipulators.openarm.config import openarm_bimanual_model_config
from dimos.robot.manipulators.openarm.connection import OpenArmConnection


@pytest.mark.parametrize("deployed", [False, True])
def test_openarm_dual_model_contains_every_canonical_joint_and_group_frame(deployed: bool) -> None:
    config = OpenArmConnection.models().planning if deployed else openarm_bimanual_model_config()

    description = prepare_robot_model(config).description

    names = [
        f"{'arm/' if deployed else ''}openarm_{side}_joint{index}"
        for side in ("left", "right")
        for index in range(1, 8)
    ]
    assert config.joint_names == names
    assert [joint.name for joint in description.joints if joint.type != "fixed"] == names
    assert [group.name for group in config.planning_groups] == [
        "left_arm",
        "right_arm",
        "both_arms",
    ]

    assert [group.joint_names for group in config.planning_groups] == [
        tuple(names[:7]),
        tuple(names[7:]),
        tuple(names),
    ]

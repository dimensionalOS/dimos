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

"""Panda prepared-model configuration tests."""

import pytest

from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.robot.manipulators.panda.config import (
    PANDA_GRIPPER_RANGE,
    make_panda_hardware,
    make_panda_model_config,
)


def test_panda_model_config_uses_canonical_names() -> None:
    config = make_panda_model_config(gripper_hardware_id="arm")
    assert config.joint_names == [f"joint{i}" for i in range(1, 8)]
    assert config.planning_groups[0].tip_link == "hand_tcp"
    assert config.gripper_hardware_id == "arm"


def test_panda_hardware_reports_the_finger_opening() -> None:
    hardware = make_panda_hardware("arm")
    assert hardware.joints[-1] == "arm/gripper"
    assert hardware.adapter_kwargs["gripper_range"] == PANDA_GRIPPER_RANGE


@pytest.mark.self_hosted  # clones franka_description
def test_panda_model_asset_has_the_arm_and_fixed_fingers() -> None:
    config = make_panda_model_config()
    model = prepare_robot_model(config).description
    assert [joint.name for joint in model.joints if joint.type != "fixed"] == config.joint_names
    assert 'name="hand_tcp"' in model.xml

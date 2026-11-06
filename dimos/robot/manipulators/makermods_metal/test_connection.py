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

"""Literal Metal firmware topology and lazy deployment contracts."""

import pytest

from dimos.robot.manipulators.makermods_metal.connection import (
    MetalConnection,
    MetalConnectionConfig,
)


@pytest.mark.parametrize("selector", ["", "  "])
def test_metal_rejects_empty_can_selector_before_opening_hardware(selector):
    with pytest.raises(ValueError, match="CAN selector must not be empty"):
        MetalConnectionConfig(can_port=selector)


def test_metal_model_schema_is_lazy_and_namespaces_both_planner_and_gripper(mocker):
    load = mocker.patch(
        "dimos.robot.assets.model.RobotModel.load", side_effect=AssertionError("must stay lazy")
    )
    deployed = MetalConnection.planning_model("arm")
    assert [
        *deployed.joint_names,
        *[f"arm/{name}" for name in MetalConnection.gripper_joints.values()],
    ] == [
        "arm/joint1",
        "arm/joint2",
        "arm/joint3",
        "arm/joint4",
        "arm/joint5",
        "arm/joint6",
        "arm/gripper",
    ]
    assert deployed.planning_groups[0].joint_names == tuple(deployed.joint_names)
    assert "gripper_hardware_id" not in deployed.model_dump()
    load.assert_not_called()

# Copyright 2025-2026 Dimensional Inc.
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

"""xArm prepared-model configuration tests."""

from dimos.manipulation.planning.spec.validation import validate_robot_model_config
from dimos.robot.manipulators.xarm.config import (
    make_dual_xarm6_model_config,
    make_xarm6_model_config,
)


def test_xarm_gripper_geometry_does_not_imply_control_hardware() -> None:
    model_only = make_xarm6_model_config(add_gripper=True)
    controlled = make_xarm6_model_config(
        add_gripper=True,
        gripper_hardware_id="arm",
    )

    assert model_only.planning_groups[0].tip_link == "link_tcp"
    assert model_only.gripper_hardware_id is None
    assert controlled.planning_groups[0].tip_link == "link_tcp"
    assert controlled.gripper_hardware_id == "arm"


def test_dual_xarm6_is_one_prepared_model_with_canonical_groups() -> None:
    config = make_dual_xarm6_model_config()

    model = validate_robot_model_config(config)

    assert model.root_link == "world"
    assert [joint.name for joint in model.joints if joint.type != "fixed"] == config.joint_names
    assert [
        (group.name, group.joint_names, group.base_link, group.tip_link)
        for group in config.planning_groups
    ] == [
        ("left_arm", tuple(config.joint_names[:6]), "left/link_base", "left/link6"),
        ("right_arm", tuple(config.joint_names[6:]), "right/link_base", "right/link6"),
        ("both_arms", tuple(config.joint_names), "world", None),
    ]


def test_prefixed_xarm_model_uses_coordinator_facing_names_in_asset() -> None:
    config = make_xarm6_model_config(add_gripper=False, prefix="xarm_arm/")

    model = validate_robot_model_config(config)

    assert [joint.name for joint in model.joints if joint.type != "fixed"] == config.joint_names
    assert config.joint_names == [f"xarm_arm/joint{i}" for i in range(1, 7)]
    assert config.planning_groups[0].tip_link == "xarm_arm/link6"

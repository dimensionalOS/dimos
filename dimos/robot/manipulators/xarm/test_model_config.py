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

import pytest

from dimos.manipulation.planning.spec.validation import validate_robot_model_config
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_BASE_POSE,
    XARM7_SIM_PATH,
    XARM_GRIPPER_COLLISION_EXCLUSIONS,
    make_dual_xarm6_model_config,
    make_xarm6_model_config,
    make_xarm7_sim_robot_config,
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


def test_dual_xarm6_configures_canonical_groups() -> None:
    config = make_dual_xarm6_model_config()

    assert [
        (group.name, group.joint_names, group.base_link, group.tip_link)
        for group in config.planning_groups
    ] == [
        ("left_arm", tuple(config.joint_names[:6]), "left/link_base", "left/link_tcp"),
        ("right_arm", tuple(config.joint_names[6:]), "right/link_base", "right/link_tcp"),
        ("both_arms", tuple(config.joint_names), "world", None),
    ]
    assert config.collision_exclusion_pairs == [
        (f"{prefix}{left}", f"{prefix}{right}")
        for prefix in ("left/", "right/")
        for left, right in XARM_GRIPPER_COLLISION_EXCLUSIONS
    ]


@pytest.mark.self_hosted
def test_dual_xarm6_model_contains_canonical_joint_topology() -> None:
    config = make_dual_xarm6_model_config()
    model = validate_robot_model_config(config)

    assert model.root_link == "world"
    assert [joint.name for joint in model.joints if joint.name in config.joint_names] == (
        config.joint_names
    )
    assert {"left/drive_joint", "right/drive_joint"} <= {joint.name for joint in model.joints}


def test_prefixed_xarm_model_config_uses_coordinator_facing_names() -> None:
    config = make_xarm6_model_config(add_gripper=False, prefix="xarm_arm/")

    assert config.joint_names == [f"xarm_arm/joint{i}" for i in range(1, 7)]
    assert config.planning_groups[0].tip_link == "xarm_arm/link6"


@pytest.mark.self_hosted
def test_prefixed_xarm_model_asset_uses_coordinator_facing_names() -> None:
    config = make_xarm6_model_config(add_gripper=False, prefix="xarm_arm/")
    model = validate_robot_model_config(config)

    assert [joint.name for joint in model.joints if joint.type != "fixed"] == config.joint_names


@pytest.mark.self_hosted
def test_sim_model_stands_on_the_same_pedestal_as_the_sim_scene() -> None:
    """The planner and MuJoCo must agree on where the arm is bolted down.

    xarm7.xml mounts link_base on a pedestal. A model placed at the origin
    instead solves every pose that far below the arm it is driving, so grasps
    close on air with no error anywhere to explain it.
    """
    import re

    mjcf = (XARM7_SIM_PATH.parent / "xarm7.xml").read_text()
    match = re.search(r'<body name="link_base" pos="([\d.\- ]+)"', mjcf)
    assert match is not None, "xarm7.xml no longer declares link_base with a pos"
    scene_base_z = float(match.group(1).split()[2])

    assert scene_base_z == XARM7_SIM_BASE_POSE.position.z
    assert make_xarm7_sim_robot_config().base_pose == XARM7_SIM_BASE_POSE

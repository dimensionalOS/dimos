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

from typing import cast

import pinocchio
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.robot.manipulators.openarm.blueprints.basic import openarm_planner_coordinator
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    OPENARM_BIMANUAL_MODEL,
    OPENARM_BIMANUAL_XACRO,
    OPENARM_DESCRIPTION_REF,
    OPENARM_DESCRIPTION_SOURCE,
    OPENARM_DESCRIPTION_URL,
    OPENARM_JOINTS,
    OPENARM_SIDES,
    openarm_bimanual_model_config,
    openarm_urdf_joints,
)
from dimos.robot.manipulators.openarm.connection import OpenArmConnection, OpenArmConnectionConfig


def test_openarm_model_uses_pinned_official_bimanual_xacro() -> None:
    assert OPENARM_DESCRIPTION_URL == "https://github.com/enactic/openarm_description"
    assert OPENARM_DESCRIPTION_REF == "1fba2cbc05001f05b4514120b70130b4ac06f409"
    assert OPENARM_DESCRIPTION_SOURCE.url == OPENARM_DESCRIPTION_URL
    assert OPENARM_DESCRIPTION_SOURCE.ref == OPENARM_DESCRIPTION_REF
    assert cast("tuple[str, ...]", OPENARM_BIMANUAL_XACRO.parts)[-5:] == (
        "assets",
        "robot",
        "openarm_v2.0",
        "urdf",
        "openarm_v20.urdf.xacro",
    )


def test_openarm_config_exposes_one_bimanual_model() -> None:
    config = openarm_bimanual_model_config()
    registry = PlanningGroupRegistry(config.planning_groups)

    assert config.model is OPENARM_BIMANUAL_MODEL
    assert config.joint_names == OPENARM_ARM_JOINTS
    assert [group.name for group in config.planning_groups] == [
        "left_arm",
        "right_arm",
        "both_arms",
    ]
    assert [group.joint_names for group in config.planning_groups] == [
        tuple(openarm_urdf_joints(side)) for side in OPENARM_SIDES
    ] + [tuple(OPENARM_ARM_JOINTS)]
    assert config.collision_exclusion_pairs == [
        ("openarm_left_ee_link1", "openarm_left_ee_link2"),
        ("openarm_right_ee_link1", "openarm_right_ee_link2"),
    ]
    assert [group.id for group in registry.list()] == [
        "left_arm",
        "right_arm",
        "both_arms",
    ]


@pytest.mark.self_hosted
def test_openarm_model_fixes_fingers_without_removing_grasp_frames() -> None:
    loaded = OPENARM_BIMANUAL_MODEL.load()
    finger_joint_names = {
        f"openarm_{side}_finger_joint{index}" for side in OPENARM_SIDES for index in (1, 2)
    }

    assert {joint.name for joint in loaded.joints if joint.type == "fixed"}.issuperset(
        finger_joint_names
    )

    model = pinocchio.buildModelFromXML(loaded.xml)

    assert model.nq == model.nv == len(OPENARM_ARM_JOINTS)
    assert tuple(str(name) for name in model.names[1:]) == tuple(
        name for side in OPENARM_SIDES for name in openarm_urdf_joints(side)
    )
    frame_names = {str(frame.name) for frame in model.frames}
    assert {f"openarm_{side}_grasp_frame" for side in OPENARM_SIDES} <= frame_names


@pytest.mark.parametrize(
    ("left_can_port", "right_can_port", "message"),
    [
        ("can1", None, "requires both CAN selectors"),
        (None, "can0", "requires both CAN selectors"),
        ("can0", "can0", "requires distinct CAN selectors"),
        ("", "can0", "CAN selector must not be empty"),
        ("can1", "  ", "CAN selector must not be empty"),
    ],
)
def test_openarm_connection_rejects_incomplete_or_ambiguous_bus_selection(
    left_can_port: str | None,
    right_can_port: str | None,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        OpenArmConnectionConfig(left_can_port=left_can_port, right_can_port=right_can_port)


def test_openarm_topology_preserves_motor_addresses_gains_and_buses() -> None:
    robot = OpenArmConnection.robot

    assert robot.buses == ("left", "right")
    assert [(arm.name, arm.bus) for arm in robot.arms] == [
        ("left_arm", "left"),
        ("right_arm", "right"),
    ]
    for side, arm in zip(("left", "right"), robot.arms, strict=True):
        assert [
            (joint.name, joint.motor, joint.send_id, joint.recv_id, joint.kp, joint.kd)
            for joint in arm.joints
        ] == [
            (f"openarm_{side}_joint1", "DM8009", 1, 17, 100.0, 1.5),
            (f"openarm_{side}_joint2", "DM8009", 2, 18, 100.0, 1.5),
            (f"openarm_{side}_joint3", "DM4340", 3, 19, 80.0, 1.0),
            (f"openarm_{side}_joint4", "DM4340", 4, 20, 80.0, 1.0),
            (f"openarm_{side}_joint5", "DM4310", 5, 21, 60.0, 0.8),
            (f"openarm_{side}_joint6", "DM4310", 6, 22, 60.0, 0.8),
            (f"openarm_{side}_joint7", "DM4310", 7, 23, 60.0, 0.8),
        ]
    assert [
        (
            gripper.name,
            gripper.bus,
            gripper.motor,
            gripper.send_id,
            gripper.recv_id,
            gripper.direction,
            gripper.current,
        )
        for gripper in robot.grippers
    ] == [
        ("left_gripper", "left", "DM4310", 8, 24, "decreasing_position", 0.15),
        ("right_gripper", "right", "DM4310", 8, 24, "decreasing_position", 0.15),
    ]
    assert robot.joint_names == tuple(OPENARM_JOINTS)
    assert robot.mock_positions == (0.0,) * 16


def test_openarm_can_ports_are_blueprint_cli_options() -> None:
    parsed = BlueprintConfigParser(openarm_planner_coordinator).parse(
        ["--left-can-port", "can1", "--right-can-port", "can0"],
        environ={},
    )

    connection = parsed.module_kwargs("openarmconnection")
    assert connection["left_can_port"] == "can1"
    assert connection["right_can_port"] == "can0"

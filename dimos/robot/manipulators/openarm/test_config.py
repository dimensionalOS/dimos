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

from pathlib import Path
from typing import cast

import pinocchio
import pytest

from dimos.control.components import HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
)
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.mock.adapter import MockWholeBodyAdapter
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.robot.manipulators.openarm.blueprints.basic import (
    _OpenArmCoordinator,
    openarm_planner_coordinator,
)
from dimos.robot.manipulators.openarm.blueprints.teleop import OpenArmTeleopCoordinator
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    OPENARM_BIMANUAL_MODEL,
    OPENARM_BIMANUAL_XACRO,
    OPENARM_DESCRIPTION_REF,
    OPENARM_DESCRIPTION_SOURCE,
    OPENARM_DESCRIPTION_URL,
    OPENARM_HARDWARE_ID,
    OPENARM_JOINTS,
    OPENARM_SIDES,
    openarm_bimanual_model_config,
    openarm_hardware,
    openarm_urdf_joints,
)


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


@pytest.fixture
def model_bounds(mocker):
    bounds = {name: (-float(i + 1), float(i + 2)) for i, name in enumerate(OPENARM_ARM_JOINTS)}
    xml = (
        '<robot name="test"><link name="base"/>'
        + "".join(
            f'<link name="{name}_link"/><joint name="{name}" type="revolute">'
            f'<parent link="base"/><child link="{name}_link"/>'
            f'<limit lower="{bounds[name][0]}" upper="{bounds[name][1]}"/></joint>'
            for name in reversed(OPENARM_ARM_JOINTS)
        )
        + "</robot>"
    )
    mocker.patch.object(
        RobotModel, "load", return_value=LoadedRobotModel(xml, Path("model.urdf"), {})
    )
    return bounds


def test_openarm_hardware_defaults_to_mock_without_can_ports(model_bounds) -> None:
    hardware = openarm_hardware()

    assert (hardware.hardware_id, hardware.hardware_type, hardware.adapter_type) == (
        OPENARM_HARDWARE_ID,
        HardwareType.WHOLE_BODY,
        "mock_whole_body",
    )
    limits = hardware.limits
    assert limits is not None
    assert limits.position_lower == [
        *[model_bounds[name][0] for name in OPENARM_ARM_JOINTS],
        0.0,
        0.0,
    ]
    assert limits.position_upper == [
        *[model_bounds[name][1] for name in OPENARM_ARM_JOINTS],
        1.0,
        1.0,
    ]
    assert limits.velocity_max == [None] * len(OPENARM_JOINTS)


def test_openarm_hardware_uses_physical_adapter_with_explicit_can_ports() -> None:
    hardware = openarm_hardware(left_can_port="can1", right_can_port="can0")

    assert hardware.adapter_type == "openarm_damiao"
    runtime_config = hardware.adapter_kwargs["runtime_config"]
    assert isinstance(runtime_config, DamiaoRuntimeConfig)
    assert runtime_config.bus_devices == {"left": "can1", "right": "can0"}


@pytest.mark.parametrize(
    "limit_xml",
    [
        "",
        '<limit lower="-1"/>',
        '<limit lower="nan" upper="1"/>',
        '<limit lower="-1" upper="inf"/>',
        '<limit lower="1" upper="1"/>',
        '<limit lower="2" upper="1"/>',
    ],
)
def test_mock_hardware_rejects_invalid_model_limits(mocker, limit_xml):
    name = OPENARM_ARM_JOINTS[0]
    xml = f'<robot name="test"><joint name="{name}" type="revolute">{limit_xml}</joint></robot>'
    mocker.patch.object(
        RobotModel, "load", return_value=LoadedRobotModel(xml, Path("model.urdf"), {})
    )
    with pytest.raises(ValueError, match=f"invalid position limits for '{name}'"):
        openarm_hardware()


@pytest.mark.parametrize(
    ("left_can_port", "right_can_port"),
    [("can1", None), (None, "can0")],
)
def test_openarm_hardware_rejects_partial_can_configuration(
    left_can_port: str | None,
    right_can_port: str | None,
) -> None:
    with pytest.raises(ValueError, match="requires both left and right CAN ports"):
        openarm_hardware(
            left_can_port=left_can_port,
            right_can_port=right_can_port,
        )


def test_openarm_can_ports_are_blueprint_cli_options(mocker) -> None:
    load = mocker.patch.object(RobotModel, "load", side_effect=AssertionError("Must stay lazy"))
    parsed = BlueprintConfigParser(openarm_planner_coordinator).parse(
        ["--left-can-port", "can1", "--right-can-port", "can0"],
        environ={},
    )

    coordinator = parsed.module_kwargs("ControlCoordinator")
    assert coordinator["left_can_port"] == "can1"
    assert coordinator["right_can_port"] == "can0"
    load.assert_not_called()


@pytest.mark.parametrize("coordinator_type", [_OpenArmCoordinator, OpenArmTeleopCoordinator])
def test_startup_supplies_model_limits_to_trajectory_execution(
    mocker, model_bounds, coordinator_type
):
    mocker.patch.object(ControlCoordinator, "_setup_from_config")
    coordinator = coordinator_type()
    try:
        coordinator._setup_from_config()
        [component] = coordinator.config.hardware
        hardware = ConnectedWholeBody(MockWholeBodyAdapter(dof=len(component.joints)), component)
        task = JointTrajectoryTask(
            JointTrajectoryTaskConfig(joint_names=OPENARM_ARM_JOINTS), {"openarm": hardware}
        )
        for name in (OPENARM_ARM_JOINTS[0], OPENARM_ARM_JOINTS[7]):
            valid = JointTrajectory(joint_names=[name], points=[TrajectoryPoint(positions=[0.0])])
            assert task.execute(valid, {}).status is TrajectoryExecutionStatus.ACCEPTED
            invalid = JointTrajectory(
                joint_names=[name],
                points=[TrajectoryPoint(positions=[model_bounds[name][1] + 0.1])],
            )
            assert (
                task.execute(invalid, {}).status
                is TrajectoryExecutionStatus.POSITION_LIMIT_VIOLATION
            )
    finally:
        coordinator.stop()

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
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.manipulators.spec import ControlMode
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import CommandStatus
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.robot.manipulators.dual_makermods_metal import model as model_module
from dimos.robot.manipulators.dual_makermods_metal.blueprints.basic import (
    DualMetalManipulationConfig,
    dual_makermods_metal_planner_coordinator,
)
from dimos.robot.manipulators.dual_makermods_metal.config import ALL_ARM_JOINTS, dual_metal_hardware
from dimos.robot.manipulators.dual_makermods_metal.coordinator import (
    DualMetalCoordinator,
    DualMetalCoordinatorConfig,
)
from dimos.robot.manipulators.dual_makermods_metal.model import (
    DualMetalDescription,
    dual_metal_model_config,
)


@pytest.mark.parametrize(
    "ports",
    [
        dict(left_can_port="left"),
        dict(right_can_port="right"),
        dict(left_can_port="same", right_can_port="same"),
        dict(left_can_port="", right_can_port="right"),
    ],
)
def test_rejects_incomplete_or_ambiguous_can_selection(ports) -> None:
    with pytest.raises(ValueError):
        dual_metal_hardware(**ports)


def test_two_explicit_ports_preserve_single_arm_adapter_and_current_defaults() -> None:
    arms = dual_metal_hardware(
        left_can_port="left",
        right_can_port="right",
    )
    assert [arm.adapter_type for arm in arms] == ["makermods_metal"] * 2
    assert [arm.adapter_kwargs["runtime_config"].bus_devices for arm in arms] == [
        {"metal": "left"},
        {"metal": "right"},
    ]
    assert [arm.adapter_kwargs["gripper_current"] for arm in arms] == [0.1, 0.1]
    assert len({name for arm in arms for name in arm.joints}) == 14


def test_mock_grippers_and_arms_are_independent() -> None:
    parsed = BlueprintConfigParser(dual_makermods_metal_planner_coordinator).parse([], environ={})
    c = DualMetalCoordinator(**parsed.module_kwargs("ControlCoordinator"))
    try:
        c._setup_from_config()
        assert list(c._tasks) == ["joint_trajectory", "left_arm_gripper", "right_arm_gripper"]
        left, right = c._hardware["left_arm"], c._hardware["right_arm"]
        assert left.write_command(
            {"left_joint2": -0.2, "left_arm/gripper": 0.8}, ControlMode.POSITION
        )
        assert right.read_state()["right_joint2"].position == 0.0
        assert right.read_state()["right_arm/gripper"].position == 0.0
        assert left.read_state()["left_arm/gripper"].position == 0.8
        assert c.config.tasks[1].stream_bind != c.config.tasks[2].stream_bind
    finally:
        c.stop()


def test_blueprint_parses_mounts_and_can_selectors() -> None:
    parsed = BlueprintConfigParser(dual_makermods_metal_planner_coordinator).parse(
        [
            "--left-can-port",
            "left",
            "--right-can-port",
            "right",
            "--left-base-xyz",
            "[0,0.25,0]",
            "--right-base-xyz",
            "[0,-0.25,0]",
        ],
        environ={},
    )
    hw = DualMetalCoordinatorConfig.model_validate(parsed.module_kwargs("ControlCoordinator"))
    assert (hw.left_can_port, hw.right_can_port) == ("left", "right")
    planning = DualMetalManipulationConfig.model_validate(
        parsed.module_kwargs("dualmetalmanipulationmodule")
    )
    assert planning.left_base_xyz == (0.0, 0.25, 0.0)
    assert planning.right_base_xyz == (0.0, -0.25, 0.0)
    assert planning.model.joint_names == list(ALL_ARM_JOINTS)


def test_derived_model_prefixes_frames_and_keeps_individual_mounts(tmp_path: Path, mocker) -> None:
    xml = '<robot name="metal"><link name="base_link"/><link name="tip"/><joint name="joint1" type="revolute"><parent link="base_link"/><child link="tip"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>'
    mocker.patch.object(
        RobotModel, "load", return_value=LoadedRobotModel(xml, tmp_path / "single.urdf", {})
    )
    mocker.patch.object(model_module, "DEFAULT_ROBOT_ASSET_CACHE_ROOT", tmp_path)
    description = DualMetalDescription((0, 0.25, 0), (0, -0.25, 0), (0, 0, 0), (0, 0, 1.0))
    root = ET.parse(description).getroot()
    assert {j.attrib["name"] for j in root.findall("joint")} == {
        "left_joint1",
        "right_joint1",
        "left_mount",
        "right_mount",
    }
    assert root.find("joint[@name='left_mount']/origin").attrib["xyz"] == "0 0.25 0"
    assert root.find("joint[@name='right_mount']/origin").attrib["rpy"] == "0 0 1.0"
    assert root.find("joint[@name='right_joint1']/parent").attrib["link"] == "right_base_link"


def test_gripper_commands_route_to_selected_group_only() -> None:
    config = dual_metal_model_config()
    m = ManipulationModule(model=config)
    m._control_coordinator = Mock()
    m._world_monitor = Mock(planning_groups=PlanningGroupRegistry(config.planning_groups))
    try:
        m._control_coordinator.task_invoke.return_value = True
        assert m.set_gripper_position(0.8, "left_arm").status is CommandStatus.SUCCEEDED
        m._control_coordinator.task_invoke.assert_called_once_with(
            "left_arm_gripper", "set_normalized", {"values": [0.8]}
        )
        m._control_coordinator.task_invoke.reset_mock()
        assert m.set_gripper_position(0.2, "right_arm").status is CommandStatus.SUCCEEDED
        m._control_coordinator.task_invoke.assert_called_once_with(
            "right_arm_gripper", "set_normalized", {"values": [0.2]}
        )
        m._control_coordinator.task_invoke.reset_mock()
        m.set_gripper_position(0.5)
        m._control_coordinator.task_invoke.assert_not_called()
        m._control_coordinator.task_invoke.return_value = [0.8]
        assert m._get_group_gripper_position("left_arm") == 0.8
        m._control_coordinator.task_invoke.assert_called_once_with(
            "left_arm_gripper", "get_normalized", {}
        )
        m._control_coordinator.task_invoke.reset_mock()
        m._control_coordinator.task_invoke.return_value = [0.2]
        assert m._get_group_gripper_position("right_arm") == 0.2
        m._control_coordinator.task_invoke.assert_called_once_with(
            "right_arm_gripper", "get_normalized", {}
        )
    finally:
        m._world_monitor = None
        m.stop()


@pytest.mark.parametrize("position", [(0.0, float("nan"), 0.0), (0.0, float("inf"), 0.0)])
def test_rejects_nonfinite_mounts(position) -> None:
    with pytest.raises(ValueError, match="finite"):
        dual_metal_model_config(left_xyz=position)


def test_rejects_ambiguous_mount_positions() -> None:
    with pytest.raises(ValueError, match="distinct"):
        dual_metal_model_config(left_xyz=(0, 0, 0), right_xyz=(0, 0, 0))


def test_gripper_mapping_rejects_unknown_planning_groups() -> None:
    config = dual_metal_model_config()
    with pytest.raises(ValueError, match="declared groups"):
        type(config)(**{**dict(config), "gripper_hardware_ids": {"missing_arm": "left_arm"}})

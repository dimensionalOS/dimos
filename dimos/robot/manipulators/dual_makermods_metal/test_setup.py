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
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.manipulators.spec import ControlMode
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import CommandStatus
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.robot.assets import composition
from dimos.robot.assets.model import RobotModel
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
    source = tmp_path / "single.urdf"
    source.write_text(xml)
    mocker.patch.object(
        model_module,
        "make_metal_model_config",
        return_value=SimpleNamespace(model=RobotModel.from_file(source)),
    )
    mocker.patch.object(composition, "DEFAULT_ROBOT_ASSET_CACHE_ROOT", tmp_path)
    model = dual_metal_model_config(
        left_xyz=(0, 0.25, 0), right_xyz=(0, -0.25, 0), right_rpy=(0, 0, 1.0)
    ).model.load()
    root = ET.fromstring(model.xml)
    assert {j.attrib["name"] for j in root.findall("joint")} == {
        "left_joint1",
        "right_joint1",
        "left_mount",
        "right_mount",
    }
    assert model.get_joint("left_mount").origin_xyz == (0, 0.25, 0)
    assert model.get_joint("right_mount").origin_rpy == (0, 0, 1)
    assert model.get_joint("right_joint1").parent_link == "right_base_link"


def test_manipulation_grippers_remain_unbound_until_multigripper_pr() -> None:
    config = dual_metal_model_config()
    module = ManipulationModule(model=config)
    module._control_coordinator = Mock()
    module._world_monitor = Mock(planning_groups=PlanningGroupRegistry(config.planning_groups))
    try:
        assert not any(group.has_gripper for group in module.list_planning_groups())
        for group_id in ("left_arm", "right_arm", None):
            assert module.set_gripper_position(0.5, group_id).status is CommandStatus.REJECTED
        module._control_coordinator.task_invoke.assert_not_called()
    finally:
        module._world_monitor = None
        module.stop()


@pytest.mark.parametrize("position", [(0.0, float("nan"), 0.0), (0.0, float("inf"), 0.0)])
def test_rejects_nonfinite_mounts(position) -> None:
    with pytest.raises(ValueError, match="finite"):
        dual_metal_model_config(left_xyz=position)


def test_rejects_ambiguous_mount_positions() -> None:
    with pytest.raises(ValueError, match="distinct"):
        dual_metal_model_config(left_xyz=(0, 0, 0), right_xyz=(0, 0, 0))

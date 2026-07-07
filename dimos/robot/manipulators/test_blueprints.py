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

from typing import Any

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.visualization.config import NoManipulationVisualizationConfig
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.blueprints import teleop as openarm_teleop_blueprints
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    openarm_mini_dual_teleop_viser,
    openarm_mini_left_teleop_viser,
    openarm_mini_right_teleop_viser,
    openarm_mini_teleop_openarm,
)
from dimos.robot.manipulators.xarm.blueprints.basic import (
    dual_xarm6_planner,
    xarm6_planner_only,
    xarm7_planner_coordinator,
)
from dimos.robot.manipulators.xarm.config import make_xarm7_model_config
from dimos.teleop.openarm_mini.adapter import OpenArmMiniTeleopAdapter
from dimos.teleop.openarm_mini.teleop_module import OpenArmMiniTeleopModule
from dimos.teleop.openarm_mini.viser_visualizer import OpenArmJointStateViserModule
from dimos.teleop.quest.blueprints import teleop_quest_go2, teleop_quest_xarm7
from dimos.teleop.quest.quest_extensions import ArmTeleopModule, Go2TeleopModule
from dimos.teleop.runtime.teleop_module import TeleopModule


def _manipulation_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is ManipulationModule)


def _manipulation_config(blueprint: Blueprint) -> ManipulationModuleConfig:
    return ManipulationModuleConfig(**_manipulation_kwargs(blueprint))


def _right_teleop_global_joint_names() -> list[str]:
    return [f"right_arm/openarm_right_joint{i}" for i in range(1, 8)]


def _left_teleop_global_joint_names() -> list[str]:
    return [f"left_arm/openarm_left_joint{i}" for i in range(1, 8)]


def _dual_gain_kp() -> list[float]:
    return [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0]


def _dual_gain_kd() -> list[float]:
    return [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5]


def test_planner_helper_defaults_to_no_visualization() -> None:
    blueprint = planner(robots=[make_xarm7_model_config(name="arm", add_gripper=True)])

    kwargs = _manipulation_kwargs(blueprint)
    config = ManipulationModuleConfig(**kwargs)

    assert "visualization" not in kwargs
    assert isinstance(config.visualization, NoManipulationVisualizationConfig)


def test_planner_helper_preserves_explicit_visualization() -> None:
    blueprint = planner(
        robots=[make_xarm7_model_config(name="arm", add_gripper=True)],
        visualization={"backend": "meshcat"},
    )

    assert _manipulation_kwargs(blueprint)["visualization"] == {"backend": "meshcat"}


def test_xarm_planner_blueprints_default_to_no_visualization() -> None:
    for blueprint in (xarm6_planner_only, dual_xarm6_planner, xarm7_planner_coordinator):
        config = _manipulation_config(blueprint)

        assert isinstance(config.visualization, NoManipulationVisualizationConfig)


def test_openarm_mini_teleop_blueprint_wires_joint_commands() -> None:
    teleop_atom = next(
        atom for atom in openarm_mini_teleop_openarm.blueprints if atom.module is TeleopModule
    )
    coordinator_atom = next(
        atom for atom in openarm_mini_teleop_openarm.blueprints if atom.module is ControlCoordinator
    )

    assert isinstance(teleop_atom.kwargs["adapter"], OpenArmMiniTeleopAdapter)
    assert any(
        stream.name == "joint_command" and stream.direction == "out"
        for stream in teleop_atom.streams
    )
    assert any(
        stream.name == "joint_command" and stream.direction == "in"
        for stream in coordinator_atom.streams
    )
    assert [task.type for task in coordinator_atom.kwargs["tasks"]] == ["servo", "servo"]
    assert [hardware.hardware_id for hardware in coordinator_atom.kwargs["hardware"]] == [
        "left_arm",
        "right_arm",
    ]


def test_openarm_mini_left_teleop_viser_blueprint_is_visualization_only() -> None:
    teleop_atom = next(
        atom
        for atom in openarm_mini_left_teleop_viser.blueprints
        if atom.module is OpenArmMiniTeleopModule
    )
    visualizer_atom = next(
        atom
        for atom in openarm_mini_left_teleop_viser.blueprints
        if atom.module is OpenArmJointStateViserModule
    )

    assert teleop_atom.kwargs["openarm_mini"].enabled_sides == ("left",)
    assert visualizer_atom.kwargs["robot_id"] == "openarm_left"
    assert visualizer_atom.kwargs["robot"].joint_names == [
        f"openarm_left_joint{i}" for i in range(1, 8)
    ]
    assert any(
        stream.name == "joint_command" and stream.direction == "out"
        for stream in teleop_atom.streams
    )
    assert any(
        stream.name == "joint_command" and stream.direction == "in"
        for stream in visualizer_atom.streams
    )
    assert all(
        atom.module is not ControlCoordinator for atom in openarm_mini_left_teleop_viser.blueprints
    )
    assert all("hardware" not in atom.kwargs for atom in openarm_mini_left_teleop_viser.blueprints)

    for atom in (teleop_atom, visualizer_atom):
        atom.module.resolve_config({**atom.kwargs, "g": global_config})

    resolved = OpenArmMiniTeleopModule.resolve_config(
        {
            **teleop_atom.kwargs,
            "openarm_mini": {
                "port_left": "/dev/ttyACM0",
            },
            "g": global_config,
        }
    )
    assert resolved.openarm_mini.port_left == "/dev/ttyACM0"
    assert resolved.openarm_mini.enabled_sides == ("left",)


def test_openarm_mini_right_teleop_viser_blueprint_wires_mock_follower() -> None:
    teleop_atom = next(
        atom
        for atom in openarm_mini_right_teleop_viser.blueprints
        if atom.module is OpenArmMiniTeleopModule
    )
    coordinator_atom = next(
        atom
        for atom in openarm_mini_right_teleop_viser.blueprints
        if atom.module is ControlCoordinator
    )
    manipulation_atom = next(
        atom
        for atom in openarm_mini_right_teleop_viser.blueprints
        if atom.module is ManipulationModule
    )
    expected_joint_names = _right_teleop_global_joint_names()

    assert all(
        atom.module is not OpenArmJointStateViserModule
        for atom in openarm_mini_right_teleop_viser.blueprints
    )
    assert teleop_atom.kwargs["openarm_mini"].enabled_sides == ("right",)
    assert teleop_atom.kwargs["openarm_mini"].port_right == "/dev/ttyACM0"
    assert tuple(teleop_atom.kwargs["openarm_mini"].target_joint_names("right")) == tuple(
        expected_joint_names
    )

    hardware = coordinator_atom.kwargs["hardware"][0]
    assert hardware.hardware_id == "right_arm"
    assert hardware.adapter_type == "mock"
    assert hardware.address is None
    assert hardware.joints == expected_joint_names
    assert coordinator_atom.kwargs["tasks"][0].joint_names == expected_joint_names

    manipulation_config = ManipulationModuleConfig(**manipulation_atom.kwargs)
    assert manipulation_config.robots[0].name == "right_arm"
    assert manipulation_config.robots[0].joint_names == [
        f"openarm_right_joint{i}" for i in range(1, 8)
    ]
    assert isinstance(manipulation_config.visualization, ViserVisualizationConfig)

    assert any(
        stream.name == "joint_command" and stream.direction == "out"
        for stream in teleop_atom.streams
    )
    assert any(
        stream.name == "joint_command" and stream.direction == "in"
        for stream in coordinator_atom.streams
    )
    assert any(
        stream.name == "coordinator_joint_state" and stream.direction == "out"
        for stream in coordinator_atom.streams
    )
    assert any(
        stream.name == "coordinator_joint_state" and stream.direction == "in"
        for stream in manipulation_atom.streams
    )

    for atom in (teleop_atom, coordinator_atom, manipulation_atom):
        atom.module.resolve_config({**atom.kwargs, "g": global_config})


def test_openarm_mini_right_teleop_partial_override_preserves_right_defaults() -> None:
    teleop_atom = next(
        atom
        for atom in openarm_mini_right_teleop_viser.blueprints
        if atom.module is OpenArmMiniTeleopModule
    )

    resolved = OpenArmMiniTeleopModule.resolve_config(
        {
            **teleop_atom.kwargs,
            "openarm_mini": {
                "port_right": "/dev/ttyUSB0",
            },
            "g": global_config,
        }
    )

    assert resolved.openarm_mini.port_right == "/dev/ttyUSB0"
    assert resolved.openarm_mini.enabled_sides == ("right",)
    assert resolved.openarm_mini.target_joint_names("right") == tuple(
        _right_teleop_global_joint_names()
    )


def test_openarm_mini_right_teleop_uses_real_follower_when_can_port_is_set() -> None:
    original_can_port = global_config.can_port
    try:
        global_config.update(can_port="can-test")
        blueprint = openarm_teleop_blueprints._openarm_mini_right_teleop_viser_blueprint()
    finally:
        global_config.update(can_port=original_can_port)

    coordinator_atom = next(
        atom for atom in blueprint.blueprints if atom.module is ControlCoordinator
    )
    hardware = coordinator_atom.kwargs["hardware"][0]

    assert hardware.adapter_type == "openarm"
    assert hardware.address == "can-test"
    assert hardware.adapter_kwargs["side"] == "right"
    assert hardware.adapter_kwargs["auto_set_mit_mode"] is True
    assert hardware.adapter_kwargs["kp"] == _dual_gain_kp()
    assert hardware.adapter_kwargs["kd"] == _dual_gain_kd()


def test_openarm_mini_dual_teleop_viser_blueprint_wires_mock_followers() -> None:
    teleop_atom = next(
        atom
        for atom in openarm_mini_dual_teleop_viser.blueprints
        if atom.module is OpenArmMiniTeleopModule
    )
    coordinator_atom = next(
        atom
        for atom in openarm_mini_dual_teleop_viser.blueprints
        if atom.module is ControlCoordinator
    )
    manipulation_atom = next(
        atom
        for atom in openarm_mini_dual_teleop_viser.blueprints
        if atom.module is ManipulationModule
    )
    expected_left_joint_names = _left_teleop_global_joint_names()
    expected_right_joint_names = _right_teleop_global_joint_names()

    assert all(
        atom.module is not OpenArmJointStateViserModule
        for atom in openarm_mini_dual_teleop_viser.blueprints
    )
    assert teleop_atom.kwargs["openarm_mini"].enabled_sides == ("left", "right")
    assert teleop_atom.kwargs["openarm_mini"].port_left == "/dev/ttyUSB1"
    assert teleop_atom.kwargs["openarm_mini"].port_right == "/dev/ttyACM0"
    assert teleop_atom.kwargs["openarm_mini"].target_joint_names("left") == tuple(
        expected_left_joint_names
    )
    assert teleop_atom.kwargs["openarm_mini"].target_joint_names("right") == tuple(
        expected_right_joint_names
    )

    hardware_by_id = {
        hardware.hardware_id: hardware for hardware in coordinator_atom.kwargs["hardware"]
    }
    assert set(hardware_by_id) == {"left_arm", "right_arm"}
    assert hardware_by_id["left_arm"].adapter_type == "mock"
    assert hardware_by_id["right_arm"].adapter_type == "mock"
    assert hardware_by_id["left_arm"].address is None
    assert hardware_by_id["right_arm"].address is None
    assert hardware_by_id["left_arm"].joints == expected_left_joint_names
    assert hardware_by_id["right_arm"].joints == expected_right_joint_names

    assert [task.name for task in coordinator_atom.kwargs["tasks"]] == [
        "servo_left_arm",
        "servo_right_arm",
    ]
    assert [task.joint_names for task in coordinator_atom.kwargs["tasks"]] == [
        expected_left_joint_names,
        expected_right_joint_names,
    ]

    manipulation_config = ManipulationModuleConfig(**manipulation_atom.kwargs)
    assert [robot.name for robot in manipulation_config.robots] == ["left_arm", "right_arm"]
    assert [robot.joint_names for robot in manipulation_config.robots] == [
        [f"openarm_left_joint{i}" for i in range(1, 8)],
        [f"openarm_right_joint{i}" for i in range(1, 8)],
    ]
    assert isinstance(manipulation_config.visualization, ViserVisualizationConfig)

    assert any(
        stream.name == "joint_command" and stream.direction == "out"
        for stream in teleop_atom.streams
    )
    assert any(
        stream.name == "joint_command" and stream.direction == "in"
        for stream in coordinator_atom.streams
    )
    assert any(
        stream.name == "coordinator_joint_state" and stream.direction == "out"
        for stream in coordinator_atom.streams
    )
    assert any(
        stream.name == "coordinator_joint_state" and stream.direction == "in"
        for stream in manipulation_atom.streams
    )

    for atom in (teleop_atom, coordinator_atom, manipulation_atom):
        atom.module.resolve_config({**atom.kwargs, "g": global_config})


def test_openarm_mini_dual_teleop_partial_override_preserves_dual_defaults() -> None:
    teleop_atom = next(
        atom
        for atom in openarm_mini_dual_teleop_viser.blueprints
        if atom.module is OpenArmMiniTeleopModule
    )

    resolved = OpenArmMiniTeleopModule.resolve_config(
        {
            **teleop_atom.kwargs,
            "openarm_mini": {
                "port_left": "/dev/ttyUSB2",
                "port_right": "/dev/ttyUSB3",
            },
            "g": global_config,
        }
    )

    assert resolved.openarm_mini.port_left == "/dev/ttyUSB2"
    assert resolved.openarm_mini.port_right == "/dev/ttyUSB3"
    assert resolved.openarm_mini.enabled_sides == ("left", "right")
    assert resolved.openarm_mini.target_joint_names("left") == tuple(
        _left_teleop_global_joint_names()
    )
    assert resolved.openarm_mini.target_joint_names("right") == tuple(
        _right_teleop_global_joint_names()
    )


def test_openarm_mini_dual_teleop_uses_real_followers_when_can_ports_are_set() -> None:
    original_left_can_port = global_config.left_can_port
    original_right_can_port = global_config.right_can_port
    try:
        global_config.update(left_can_port="can-left", right_can_port="can-right")
        blueprint = openarm_teleop_blueprints._openarm_mini_dual_teleop_viser_blueprint()
    finally:
        global_config.update(
            left_can_port=original_left_can_port,
            right_can_port=original_right_can_port,
        )

    coordinator_atom = next(
        atom for atom in blueprint.blueprints if atom.module is ControlCoordinator
    )
    hardware_by_id = {
        hardware.hardware_id: hardware for hardware in coordinator_atom.kwargs["hardware"]
    }

    assert hardware_by_id["left_arm"].adapter_type == "openarm"
    assert hardware_by_id["left_arm"].address == "can-left"
    assert hardware_by_id["left_arm"].adapter_kwargs["side"] == "left"
    assert hardware_by_id["left_arm"].adapter_kwargs["auto_set_mit_mode"] is True
    assert hardware_by_id["left_arm"].adapter_kwargs["kp"] == _dual_gain_kp()
    assert hardware_by_id["left_arm"].adapter_kwargs["kd"] == _dual_gain_kd()

    assert hardware_by_id["right_arm"].adapter_type == "openarm"
    assert hardware_by_id["right_arm"].address == "can-right"
    assert hardware_by_id["right_arm"].adapter_kwargs["side"] == "right"
    assert hardware_by_id["right_arm"].adapter_kwargs["auto_set_mit_mode"] is True
    assert hardware_by_id["right_arm"].adapter_kwargs["kp"] == _dual_gain_kp()
    assert hardware_by_id["right_arm"].adapter_kwargs["kd"] == _dual_gain_kd()


def test_existing_quest_teleop_blueprints_still_use_quest_modules() -> None:
    assert any(atom.module is ArmTeleopModule for atom in teleop_quest_xarm7.blueprints)
    assert any(atom.module is Go2TeleopModule for atom in teleop_quest_go2.blueprints)
    assert all(atom.module is not TeleopModule for atom in teleop_quest_xarm7.blueprints)

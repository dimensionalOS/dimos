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

import builtins
from pathlib import Path
from typing import Any

import pytest

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.visualization.config import NoManipulationVisualizationConfig
from dimos.robot.manipulators.a1z.blueprints.teleop import (
    _build_a1z_keyboard_components,
    _convert_camera_info,
    _convert_depth_camera_info,
)
from dimos.robot.manipulators.a1z.config import A1Z_DOF, make_a1z_hardware
from dimos.robot.manipulators.a1z.simulation import (
    A1Z_SCENE_PATH,
    A1Z_SIM_HOME,
    _A1ZMujocoSimModule,
)
from dimos.robot.manipulators.a750.blueprints.teleop import keyboard_teleop_a750
from dimos.robot.manipulators.common.blueprints import eef_twist_task, planner
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    keyboard_teleop_openarm,
    keyboard_teleop_openarm_mock,
)
from dimos.robot.manipulators.piper.blueprints.teleop import keyboard_teleop_piper
from dimos.robot.manipulators.xarm.blueprints.basic import (
    dual_xarm6_planner,
    xarm6_planner_only,
    xarm7_planner_coordinator,
)
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    _build_xarm6_keyboard_components,
    keyboard_teleop_xarm6,
    keyboard_teleop_xarm7,
)
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_PATH,
    make_xarm7_model_config,
    make_xarm_hardware,
)
from dimos.robot.manipulators.xarm.simulation import _XArm6MujocoSimModule
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopConfig, KeyboardTeleopModule
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _manipulation_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return _module_kwargs(blueprint, ManipulationModule)


def _manipulation_config(blueprint: Blueprint) -> ManipulationModuleConfig:
    return ManipulationModuleConfig(**_manipulation_kwargs(blueprint))


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    return _module_kwargs(blueprint, ControlCoordinator)["tasks"]


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


def test_eef_twist_task_helper_uses_hardware_joints_and_default_name() -> None:
    hardware = make_xarm_hardware("arm", 6, adapter_type="mock")

    task = eef_twist_task(hardware, model_path=Path("fake.urdf"), ee_joint_id=6)

    assert task.name == EEF_TWIST_TASK_NAME
    assert task.type == "eef_twist"
    assert task.joint_names == hardware.joints
    assert task.params == {"model_path": Path("fake.urdf"), "ee_joint_id": 6}


def test_a1z_keyboard_blueprint_modes_are_explicit() -> None:
    hardware_blueprint = autoconnect(*_build_a1z_keyboard_components(simulation="dimsim"))
    simulation_blueprint = autoconnect(*_build_a1z_keyboard_components(simulation="mujoco"))

    hardware_tasks = _coordinator_tasks(hardware_blueprint)
    simulation_tasks = _coordinator_tasks(simulation_blueprint)
    assert not any(atom.module is ManipulationModule for atom in hardware_blueprint.blueprints)
    assert not any(atom.module is ManipulationModule for atom in simulation_blueprint.blueprints)
    assert all(task.type != "servo" for task in hardware_tasks)

    servo_tasks = [task for task in simulation_tasks if task.type == "servo"]

    assert len(servo_tasks) == 1
    assert servo_tasks[0].name == "servo_gripper"
    assert servo_tasks[0].joint_names == ["arm/gripper"]
    assert servo_tasks[0].priority != 10
    assert servo_tasks[0].params == {"timeout": 0.0, "default_positions": [0.0]}


def test_a1z_keyboard_wires_directly_in_both_modes() -> None:
    simulation = autoconnect(*_build_a1z_keyboard_components(simulation="mujoco"))
    hardware = autoconnect(*_build_a1z_keyboard_components(simulation="dimsim"))

    assert (KeyboardTeleopModule, "coordinator_ee_twist_command") not in simulation.remapping_map
    assert (KeyboardTeleopModule, "coordinator_ee_twist_command") not in hardware.remapping_map


def test_a1z_mujoco_includes_rerun_camera_and_world_views() -> None:
    blueprint = autoconnect(*_build_a1z_keyboard_components(simulation="mujoco"))

    rerun_atoms = [atom for atom in blueprint.blueprints if atom.module is RerunBridgeModule]
    assert len(rerun_atoms) == 1
    rerun_kwargs = rerun_atoms[0].kwargs
    assert rerun_kwargs["blueprint"] is not None
    assert rerun_kwargs["rerun_open"] == "web"
    assert rerun_kwargs["rerun_web"] is True
    assert "world/camera_info" in rerun_kwargs["visual_override"]
    assert "world/depth_camera_info" in rerun_kwargs["visual_override"]


def test_a1z_camera_info_overrides_use_message_optical_frame() -> None:
    class CameraInfo:
        def __init__(self, frame_id: str) -> None:
            self.frame_id = frame_id

        def to_rerun(self, **kwargs: Any) -> dict[str, Any]:
            return kwargs

    color_info = CameraInfo("wrist_camera_color_optical_frame")
    depth_info = CameraInfo("wrist_camera_depth_optical_frame")

    color = _convert_camera_info(color_info)
    depth = _convert_depth_camera_info(depth_info)

    assert color == {
        "image_topic": "/world/color_image",
        "optical_frame": "wrist_camera_color_optical_frame",
    }
    assert depth == {
        "image_topic": "/world/depth_image",
        "optical_frame": "wrist_camera_depth_optical_frame",
    }


def test_a1z_non_mujoco_excludes_visualization_stack() -> None:
    blueprint = autoconnect(*_build_a1z_keyboard_components(simulation="dimsim"))

    modules = {atom.module for atom in blueprint.blueprints}
    assert RerunBridgeModule not in modules
    assert WebsocketVisModule not in modules


@pytest.mark.parametrize("simulation", ["dimsim", "mujoco"])
def test_a1z_keyboard_speed_overrides_are_bounded_to_a1z(
    simulation: str,
) -> None:
    blueprint = autoconnect(*_build_a1z_keyboard_components(simulation))

    keyboard_kwargs = _module_kwargs(blueprint, KeyboardTeleopModule)
    config = KeyboardTeleopConfig(**keyboard_kwargs)

    assert config.linear_speed == 0.05
    assert config.angular_speed == 0.5


def test_a1z_non_mujoco_mode_does_not_import_simulator(monkeypatch: pytest.MonkeyPatch) -> None:
    real_import = builtins.__import__

    def reject_simulator_import(name: str, *args: Any, **kwargs: Any) -> Any:
        if name == "dimos.robot.manipulators.a1z.simulation":
            raise AssertionError("non-MuJoCo mode imported the optional simulator")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_simulator_import)
    blueprint = autoconnect(*_build_a1z_keyboard_components(simulation="dimsim"))
    hardware = _module_kwargs(blueprint, ControlCoordinator)["hardware"][0]

    assert hardware.adapter_type == "mock"
    assert not any("Mujoco" in atom.module.__name__ for atom in blueprint.blueprints)


def test_a1z_hardware_configuration_is_explicit() -> None:
    hardware = make_a1z_hardware("arm")
    simulation_hardware = make_a1z_hardware(
        "arm",
        adapter_type="sim_mujoco",
        address=str(A1Z_SCENE_PATH),
        home_joints=list(A1Z_SIM_HOME),
    )

    assert hardware.adapter_type == "mock"
    assert hardware.address is None
    assert "initial_positions" not in hardware.adapter_kwargs
    assert simulation_hardware.adapter_type == "sim_mujoco"
    assert simulation_hardware.address == str(A1Z_SCENE_PATH)
    assert simulation_hardware.adapter_kwargs["initial_positions"] == list(A1Z_SIM_HOME)


def test_a1z_simulation_blueprint_wires_scene_and_camera_contract() -> None:
    blueprint = autoconnect(*_build_a1z_keyboard_components(simulation="mujoco"))
    sim_kwargs = _module_kwargs(blueprint, _A1ZMujocoSimModule)
    sim_hardware = _module_kwargs(blueprint, ControlCoordinator)["hardware"][0]

    assert sim_kwargs["address"] == sim_hardware.address == str(A1Z_SCENE_PATH)
    assert sim_kwargs["dof"] == A1Z_DOF
    assert sim_kwargs["reset_joint_positions"] == list(A1Z_SIM_HOME)
    assert sim_kwargs["fps"] == 30
    assert sim_kwargs["camera_name"] == "wrist_camera"
    assert sim_kwargs["gripper_control_mapping"] == "identity"


def test_a1z_simulator_is_dedicated_and_absent_from_hardware_blueprint() -> None:
    assert _A1ZMujocoSimModule.dedicated_worker is True
    assert not any(
        atom.module is _A1ZMujocoSimModule
        for atom in autoconnect(*_build_a1z_keyboard_components("dimsim")).blueprints
    )
    assert any(
        atom.module is _A1ZMujocoSimModule
        for atom in autoconnect(*_build_a1z_keyboard_components("mujoco")).blueprints
    )


def test_xarm6_keyboard_builder_composes_non_mujoco_mode() -> None:
    blueprint = autoconnect(*_build_xarm6_keyboard_components("dimsim"))

    hardware = _module_kwargs(blueprint, ControlCoordinator)["hardware"][0]
    assert hardware.adapter_type == ("xarm" if global_config.xarm6_ip else "mock")
    assert hardware.address == global_config.xarm6_ip
    assert not any(atom.module is _XArm6MujocoSimModule for atom in blueprint.blueprints)


def test_xarm6_keyboard_builder_wires_mujoco_scene_and_camera() -> None:
    blueprint = autoconnect(*_build_xarm6_keyboard_components("mujoco"))

    sim_kwargs = _module_kwargs(blueprint, _XArm6MujocoSimModule)
    hardware = _module_kwargs(blueprint, ControlCoordinator)["hardware"][0]
    assert hardware.adapter_type == "sim_mujoco"
    assert hardware.address == sim_kwargs["address"] == str(XARM6_SIM_PATH)
    assert sim_kwargs["dof"] == 6
    assert sim_kwargs["camera_name"] == "wrist_camera"
    assert sim_kwargs["base_frame_id"] == "link6"
    assert sim_kwargs["width"] == 640
    assert sim_kwargs["height"] == 480
    assert sim_kwargs["fps"] == 15


def test_xarm6_simulator_is_dedicated_and_registry_export_is_direct() -> None:
    assert _XArm6MujocoSimModule.dedicated_worker is True
    assert isinstance(keyboard_teleop_xarm6, Blueprint)
    assert any(
        atom.module is _XArm6MujocoSimModule
        for atom in autoconnect(*_build_xarm6_keyboard_components("mujoco")).blueprints
    )


@pytest.mark.parametrize(
    "blueprint",
    [
        pytest.param(keyboard_teleop_xarm6, id="xarm6"),
        pytest.param(keyboard_teleop_xarm7, id="xarm7"),
        pytest.param(keyboard_teleop_piper, id="piper"),
        pytest.param(keyboard_teleop_openarm_mock, id="openarm-mock"),
        pytest.param(keyboard_teleop_openarm, id="openarm"),
        pytest.param(keyboard_teleop_a750, id="a750"),
    ],
)
def test_manipulator_keyboard_blueprint_uses_eef_twist_and_light_keyboard_kwargs(
    blueprint: Blueprint,
) -> None:
    keyboard_kwargs = _module_kwargs(blueprint, KeyboardTeleopModule)
    coordinator_tasks = _coordinator_tasks(blueprint)
    eef_twist_tasks = [task for task in coordinator_tasks if task.type == "eef_twist"]

    assert keyboard_kwargs == {}
    assert [task.name for task in eef_twist_tasks] == [EEF_TWIST_TASK_NAME]
    assert all(task.type != "cartesian_ik" for task in coordinator_tasks)

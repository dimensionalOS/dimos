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
from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.visualization.config import (
    NoManipulationVisualizationConfig,
)
from dimos.robot.manipulators.common.blueprints import eef_twist_task, planner
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.xarm.config import (
    make_xarm7_model_config,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
    make_xarm_hardware,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModuleConfig


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _manipulation_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return _module_kwargs(blueprint, ManipulationModule)


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


def test_xarm_perception_sim_uses_aligned_camera_frame() -> None:
    sim_robot = make_xarm7_sim_robot_config()
    sim_config = MujocoSimModuleConfig(
        **make_xarm7_sim_module_kwargs("test-xarm7-scene.xml"),
    )

    assert sim_robot.xacro_args["attach_rpy"] == "0 0 0"
    assert sim_config.base_frame_id == "link7"
    assert sim_config.reset_joint_positions == sim_robot.home_joints


def test_eef_twist_task_helper_uses_hardware_joints_and_default_name() -> None:
    hardware = make_xarm_hardware("arm", 6, adapter_type="mock")

    task = eef_twist_task(hardware, model_path=Path("fake.urdf"), ee_joint_id=6)

    assert task.name == EEF_TWIST_TASK_NAME
    assert task.type == "eef_twist"
    assert task.joint_names == hardware.joints
    assert task.params == {"model_path": Path("fake.urdf"), "ee_joint_id": 6}

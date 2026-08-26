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

"""Construction and objective tests for shared G1 Quest teleoperation."""

from typing import Any, cast

import numpy as np
import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import g1_arms
from dimos.control.tasks.pose_target_ik import PoseTargetIKTaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.planning.factory import create_world
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import (
    _G1_ARM_JOINT_NAME_MAPPING,
    _G1_TELEOP_MODEL,
    _G1_TELEOP_PINK,
    G1_TELEOP_TASK_NAME,
    _G1GrootCoordinator,
    unitree_g1_groot_wbc,
)
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_teleop import (
    G1CollectionRecorder,
    G1ManipulationModule,
    unitree_g1_teleop,
)
from dimos.robot.unitree.g1.manip_config import (
    G1_LEFT_ARM_JOINTS,
    G1_RIGHT_ARM_JOINTS,
    G1_UPPER_BODY_JOINT_NAME_MAPPING,
    G1_WAIST_JOINTS,
    g1_upper_body_model_config,
)
from dimos.robot.unitree.g1.teleop_ik import G1PinkPoseTargetSolver
from dimos.teleop.quest.quest_extensions import MobileVideoArmTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _teleop_task() -> TaskConfig:
    coordinator = next(
        atom
        for atom in unitree_g1_groot_wbc.blueprints
        if issubclass(atom.module, TeleopControlCoordinator)
    )
    return cast(
        "TaskConfig",
        next(task for task in coordinator.kwargs["tasks"] if task.type == "teleop_ik"),
    )


def test_g1_blueprint_uses_shared_bimanual_teleop_task() -> None:
    task = _teleop_task()

    assert task.name == G1_TELEOP_TASK_NAME
    assert task.joint_names == g1_arms
    assert task.priority == 20
    assert task.params["robot_model"] is _G1_TELEOP_MODEL
    assert task.params["solver_type"] is G1PinkPoseTargetSolver
    assert task.params["bindings"] == [
        {"hand": "left", "target_frame": "left_rubber_hand"},
        {"hand": "right", "target_frame": "right_rubber_hand"},
    ]
    assert _G1_TELEOP_MODEL.base_link == "pelvis"
    assert _G1_TELEOP_MODEL.joint_name_mapping == _G1_ARM_JOINT_NAME_MAPPING
    assert task.params["max_joint_velocity_rad_s"] == pytest.approx(np.deg2rad(120.0))


def test_g1_blueprint_keeps_bounded_trajectory_path_below_teleop() -> None:
    coordinator = next(
        atom
        for atom in unitree_g1_groot_wbc.blueprints
        if issubclass(atom.module, TeleopControlCoordinator)
    )

    arm_tasks = [
        task for task in coordinator.kwargs["tasks"] if set(task.joint_names) & set(g1_arms)
    ]

    assert [(task.name, task.type, task.priority) for task in arm_tasks] == [
        (JOINT_TRAJECTORY_TASK_NAME, "trajectory", 10),
        (G1_TELEOP_TASK_NAME, "teleop_ik", 20),
    ]


def test_g1_teleop_wires_arm_velocity_and_recording_streams() -> None:
    teleop_kwargs = _module_kwargs(unitree_g1_teleop, MobileVideoArmTeleopModule)

    assert "task_names" not in teleop_kwargs
    assert (
        unitree_g1_teleop.remapping_map[(MobileVideoArmTeleopModule.name, "left_controller_output")]
        == "left_cartesian_command"
    )
    assert (
        unitree_g1_teleop.remapping_map[
            (MobileVideoArmTeleopModule.name, "right_controller_output")
        ]
        == "right_cartesian_command"
    )
    assert (
        unitree_g1_teleop.remapping_map[(MobileVideoArmTeleopModule.name, "cmd_vel")] == "cmd_vel"
    )
    assert "left_cartesian_command" in G1CollectionRecorder.__annotations__
    assert "right_cartesian_command" in G1CollectionRecorder.__annotations__


def test_g1_teleop_excludes_navigation_and_legacy_visualization() -> None:
    module_names = {atom.module.__name__ for atom in unitree_g1_teleop.active_blueprints}

    assert module_names.isdisjoint(
        {
            "PointLio",
            "RayTracingVoxelMap",
            "VoxelGridMapper",
            "CostMapper",
            "ReplanningAStarPlanner",
            "MovementManager",
            "WebsocketVisModule",
            "RerunBridgeModule",
            "RerunWebSocketServer",
        }
    )


def test_g1_collection_streams_do_not_require_world_poses() -> None:
    recorder_kwargs = _module_kwargs(unitree_g1_teleop, G1CollectionRecorder)

    assert recorder_kwargs["poseless_streams"] == [
        "color_image",
        "status",
        "left_cartesian_command",
        "right_cartesian_command",
        "coordinator_joint_state",
    ]


def test_g1_upper_body_model_keeps_waist_and_arms_but_removes_legs() -> None:
    config = g1_upper_body_model_config()
    loaded = config.model.load()
    joint_names = {joint.name for joint in loaded.joints}

    assert loaded.root_link == "pelvis"
    assert "left_hip_pitch_joint" not in joint_names
    assert "right_hip_pitch_joint" not in joint_names
    assert {_G1_ARM_JOINT_NAME_MAPPING[name] for name in g1_arms} <= joint_names
    assert {G1_UPPER_BODY_JOINT_NAME_MAPPING[name] for name in G1_WAIST_JOINTS} <= joint_names
    assert config.max_velocity == 1.0
    assert config.max_acceleration == 2.0


def test_g1_upper_body_plans_arms_without_owning_waist() -> None:
    config = g1_upper_body_model_config()

    assert config.joint_names == [
        *(G1_UPPER_BODY_JOINT_NAME_MAPPING[name] for name in G1_WAIST_JOINTS),
        *(G1_UPPER_BODY_JOINT_NAME_MAPPING[name] for name in g1_arms),
    ]
    assert [group.name for group in config.planning_groups] == ["left_arm", "right_arm"]
    assert config.planning_groups[0].joint_names == tuple(
        G1_UPPER_BODY_JOINT_NAME_MAPPING[name] for name in G1_LEFT_ARM_JOINTS
    )
    assert config.planning_groups[1].joint_names == tuple(
        G1_UPPER_BODY_JOINT_NAME_MAPPING[name] for name in G1_RIGHT_ARM_JOINTS
    )


def test_g1_teleop_wires_manipulation_to_existing_coordinator() -> None:
    manipulation_kwargs = _module_kwargs(unitree_g1_teleop, G1ManipulationModule)

    assert manipulation_kwargs["instance_name"] == "G1Manipulation"
    assert [robot.name for robot in manipulation_kwargs["robots"]] == ["g1_upper_body"]
    assert manipulation_kwargs["visualization"] == ViserVisualizationConfig(host="0.0.0.0")
    assert (
        unitree_g1_teleop.remapping_map[("G1Manipulation", "_control_coordinator")]
        is _G1GrootCoordinator
    )


@pytest.mark.self_hosted
def test_g1_upper_body_model_builds_a_roboplan_scene() -> None:
    world = create_world()

    robot_id = world.add_robot(g1_upper_body_model_config())
    world.finalize()

    assert world.get_robot_ids() == [robot_id]


@pytest.mark.self_hosted
def test_g1_pink_solver_reduces_model_and_uses_g1_objective() -> None:
    frames = ("left_rubber_hand", "right_rubber_hand")
    config = PoseTargetIKTaskConfig(
        joint_names=tuple(g1_arms),
        robot_model=_G1_TELEOP_MODEL,
        target_frames=frames,
        pink=_G1_TELEOP_PINK,
    )
    solver = G1PinkPoseTargetSolver(config)
    seed = JointState(name=list(g1_arms), position=[0.0] * len(g1_arms))
    targets = solver.frame_poses(seed, frames)

    command = solver.step(targets, seed, 0.01)

    assert command is not None
    assert command.name == g1_arms
    context = next(iter(solver._control_contexts.values()))
    assert context.robot.model.nq == len(g1_arms)
    assert context.tasks is not None
    for frame_name in frames:
        frame_task = context.tasks[f"frame/{frame_name}"]
        assert frame_task.position_cost == pytest.approx([8.0, 8.0, 8.0])
        assert frame_task.orientation_cost == pytest.approx([2.0, 2.0, 2.0])
    posture = context.tasks["posture/current"]
    assert posture.cost == pytest.approx(np.tile([4.0, 3.0, 0.1, 3.0, 1.0, 1.0, 0.1], 2) * 0.01)
    assert posture.target_q == pytest.approx(np.zeros(len(g1_arms)))

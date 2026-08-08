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

"""Construction and component tests for safe OpenArm Quest teleoperation."""

from typing import Any

import numpy as np
import pink
import pytest
from pytest_mock import MockerFixture

from dimos.control.tasks.pose_target_ik import PoseTargetIKTaskConfig
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    OPENARM_QUEST_TASK_NAME,
    OpenArmTeleopCoordinator,
    _openarm_quest_hardware,
    _OpenArmManipulationModule,
    teleop_quest_openarm,
)
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    OPENARM_HOME_JOINTS,
    OPENARM_JOINTS,
    openarm_bimanual_model_config,
    openarm_mock_hardware,
)
from dimos.robot.manipulators.openarm.teleop_ik import OpenArmPinkPoseTargetSolver
from dimos.teleop.quest.quest_extensions import ArmTeleopModule
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils import data


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _solver_config(
    model: Any,
    frames: tuple[str, ...],
    pink_config: PinkKinematicsConfig,
) -> PoseTargetIKTaskConfig:
    return PoseTargetIKTaskConfig(
        joint_names=tuple(OPENARM_ARM_JOINTS),
        robot_model=model,
        target_frames=frames,
        pink=pink_config,
    )


def test_openarm_quest_hardware_defaults_to_mock() -> None:
    hardware = _openarm_quest_hardware(None, None)

    assert hardware.adapter_type == "mock_whole_body"


def test_openarm_quest_hardware_uses_explicit_can_ports() -> None:
    hardware = _openarm_quest_hardware("can8", "can9")

    assert hardware.adapter_type == "openarm_damiao"
    assert hardware.adapter_kwargs["runtime_config"].bus_addresses == {
        "left": "can8",
        "right": "can9",
    }


@pytest.mark.parametrize(
    ("left_can_port", "right_can_port"),
    [("can8", None), (None, "can9")],
)
def test_openarm_quest_hardware_requires_both_can_ports(
    left_can_port: str | None,
    right_can_port: str | None,
) -> None:
    with pytest.raises(ValueError, match="requires both left and right CAN ports"):
        _openarm_quest_hardware(left_can_port, right_can_port)


def test_openarm_can_ports_are_blueprint_cli_options(mocker: MockerFixture) -> None:
    get_data = mocker.patch.object(data, "get_data")

    parsed = BlueprintConfigParser(teleop_quest_openarm).parse(
        ["--left-can-port", "can8", "--right-can-port", "can9"],
        environ={},
    )

    coordinator = parsed.module_kwargs("ControlCoordinator")
    assert coordinator["left_can_port"] == "can8"
    assert coordinator["right_can_port"] == "can9"
    get_data.assert_not_called()


def test_openarm_mock_hardware_and_model_use_canonical_zero_start() -> None:
    hardware = openarm_mock_hardware()
    model = openarm_bimanual_model_config()

    assert model.home_joints == OPENARM_HOME_JOINTS
    assert hardware.adapter_kwargs["initial_positions"] == [*OPENARM_HOME_JOINTS, 0.0, 0.0]
    assert OPENARM_HOME_JOINTS == [0.0] * len(OPENARM_ARM_JOINTS)


def test_openarm_quest_blueprint_has_one_bimanual_mock_task() -> None:
    coordinator_kwargs = _module_kwargs(teleop_quest_openarm, OpenArmTeleopCoordinator)
    teleop_kwargs = _module_kwargs(teleop_quest_openarm, ArmTeleopModule)
    manipulation_kwargs = _module_kwargs(teleop_quest_openarm, _OpenArmManipulationModule)
    tasks = coordinator_kwargs["tasks"]

    assert "hardware" not in coordinator_kwargs
    assert "left_can_port" not in coordinator_kwargs
    assert "right_can_port" not in coordinator_kwargs
    assert len(tasks) == 2

    task = next(task for task in tasks if task.type == "teleop_ik")
    trajectory = next(task for task in tasks if task.type == "trajectory")
    bindings = task.params["bindings"]
    assert task.name == OPENARM_QUEST_TASK_NAME
    assert task.type == "teleop_ik"
    assert task.joint_names == OPENARM_ARM_JOINTS
    assert {binding["hand"] for binding in bindings} == {"left", "right"}
    assert {binding["target_frame"] for binding in bindings} == {
        "openarm_left_grasp_frame",
        "openarm_right_grasp_frame",
    }
    assert set(task.joint_names) | {binding["gripper_joint"] for binding in bindings} == set(
        OPENARM_JOINTS
    )
    assert isinstance(task.params["pink"], PinkKinematicsConfig)
    assert "robot_model" not in task.params
    assert task.params["solver_type"] is OpenArmPinkPoseTargetSolver
    assert task.params["pink"].joint_limit_posture_margin == 0.3
    assert task.params["max_command_tracking_error_deg"] == 10.0
    expected_velocity_limits = {
        joint_name: limit
        for side_offset in (0, 7)
        for joint_name, limit in zip(
            OPENARM_ARM_JOINTS[side_offset : side_offset + 7],
            (1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0),
            strict=True,
        )
    }
    assert task.params["joint_velocity_limits_rad_s"] == expected_velocity_limits
    assert task.params["joint_command_filter_cutoff_hz"] == 5.0
    assert task.priority == 10
    assert trajectory.joint_names == OPENARM_ARM_JOINTS
    assert trajectory.priority == 20
    assert "robots" not in manipulation_kwargs
    assert manipulation_kwargs["kinematics"] == task.params["pink"]
    assert manipulation_kwargs["visualization"] == {"backend": "viser"}
    assert teleop_kwargs["task_names"] == {
        "left": OPENARM_QUEST_TASK_NAME,
        "right": OPENARM_QUEST_TASK_NAME,
    }
    assert teleop_quest_openarm.remapping_map == {
        (ArmTeleopModule.name, "left_controller_output"): "left_cartesian_command",
        (ArmTeleopModule.name, "right_controller_output"): "right_cartesian_command",
    }


def test_openarm_quest_commands_both_arms_and_grippers_through_coordinator(
    mocker: MockerFixture,
) -> None:
    coordinator_kwargs = _module_kwargs(teleop_quest_openarm, OpenArmTeleopCoordinator)
    mocker.patch.object(OpenArmPinkPoseTargetSolver, "_validate_frame_targets")
    frame_poses = mocker.patch.object(
        OpenArmPinkPoseTargetSolver,
        "frame_poses",
        return_value={
            "openarm_left_grasp_frame": PoseStamped(position=[0.5, 0.2, 0.4]),
            "openarm_right_grasp_frame": PoseStamped(position=[0.5, -0.2, 0.4]),
        },
    )
    step = mocker.patch.object(
        OpenArmPinkPoseTargetSolver,
        "step",
        return_value=JointState(
            name=list(OPENARM_ARM_JOINTS),
            position=[0.01] * len(OPENARM_ARM_JOINTS),
        ),
    )
    mocker.patch.object(TickLoop, "start")
    coordinator = OpenArmTeleopCoordinator(publish_joint_state=False, **coordinator_kwargs)

    try:
        coordinator.start()
        task = coordinator._tasks[OPENARM_QUEST_TASK_NAME]
        assert task._teleop_config.robot_model.name == "openarm"
        assert task._teleop_config.max_joint_velocity_rad_s == 2.0
        assert task._teleop_config.joint_velocity_limits_rad_s == {
            joint_name: limit
            for side_offset in (0, 7)
            for joint_name, limit in zip(
                OPENARM_ARM_JOINTS[side_offset : side_offset + 7],
                (1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0),
                strict=True,
            )
        }
        assert task._teleop_config.joint_command_filter_cutoff_hz == 5.0
        buttons = Buttons()
        buttons.left_primary = True
        buttons.right_primary = True
        buttons.pack_analog_triggers(left=0.25, right=0.75)
        coordinator._dispatch("teleop_buttons", buttons)
        coordinator._dispatch(
            "left_cartesian_command",
            PoseStamped(frame_id=OPENARM_QUEST_TASK_NAME, position=[1.0, 0.0, 0.0]),
        )
        coordinator._dispatch(
            "right_cartesian_command",
            PoseStamped(frame_id=OPENARM_QUEST_TASK_NAME, position=[-1.0, 0.0, 0.0]),
        )

        assert coordinator._tick_loop is not None
        coordinator._tick_loop._tick()

        connected = coordinator._hardware["openarm"]
        states = connected.adapter.read_motor_states()
        assert [state.q for state in states[: len(OPENARM_ARM_JOINTS)]] == [0.01] * len(
            OPENARM_ARM_JOINTS
        )
        assert [state.q for state in states[-2:]] == [
            1.0 - buttons.left_trigger_analog,
            1.0 - buttons.right_trigger_analog,
        ]
        frame_poses.assert_called_once()
        step.assert_called_once()

        released = Buttons()
        released.right_primary = True
        coordinator._dispatch("teleop_buttons", released)
        coordinator._tick_loop._tick()
        step.assert_called_once()
    finally:
        coordinator.stop()


@pytest.mark.self_hosted
def test_openarm_teleop_pink_objective_uses_robot_specific_tuning() -> None:
    model = openarm_bimanual_model_config()
    frames = ("openarm_left_grasp_frame", "openarm_right_grasp_frame")
    config = PinkKinematicsConfig(
        dt=0.01,
        posture_cost=1e-3,
        joint_limit_posture_margin=0.3,
        gain=0.25,
    )
    seed = JointState(name=OPENARM_ARM_JOINTS, position=[0.0] * len(OPENARM_ARM_JOINTS))
    solver = OpenArmPinkPoseTargetSolver(_solver_config(model, frames, config))
    targets = solver.frame_poses(seed, frames)

    solver.step(targets, seed, 0.01)

    tasks = next(iter(solver._control_contexts.values())).tasks
    assert tasks is not None
    for frame_name in frames:
        frame_task = tasks[f"frame/{frame_name}"]
        assert frame_task.position_cost == pytest.approx([1.0, 1.0, 1.0])
        assert frame_task.orientation_cost == pytest.approx([0.2, 0.2, 0.2])
        manipulability = tasks[f"manipulability/{frame_name}"]
        assert isinstance(manipulability, pink.tasks.ManipulabilityTask)
        assert manipulability.cost == 0.005
        assert manipulability.manipulability_rate == 0.05
        assert manipulability.mask == pytest.approx([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
    assert tasks["posture/current"].cost == pytest.approx(
        np.tile([4.0, 3.0, 0.1, 3.0, 1.0, 1.0, 0.1], 2) * 1e-3
    )


@pytest.mark.self_hosted
def test_openarm_bimanual_pink_steps_from_canonical_zero_with_bounded_updates() -> None:
    model = openarm_bimanual_model_config()
    frames = ("openarm_left_grasp_frame", "openarm_right_grasp_frame")
    config = PinkKinematicsConfig(
        dt=0.01,
        position_cost=1.0,
        orientation_cost=1.0,
        posture_cost=1e-3,
        joint_limit_posture_margin=0.3,
        lm_damping=1e-6,
        gain=0.25,
    )
    ik = OpenArmPinkPoseTargetSolver(_solver_config(model, frames, config))
    seed = JointState(name=OPENARM_ARM_JOINTS, position=[0.0] * len(OPENARM_ARM_JOINTS))
    initial = ik.frame_poses(seed, frames)
    targets = {
        name: PoseStamped(
            frame_id=pose.frame_id,
            position=[pose.position.x, pose.position.y, pose.position.z + 0.01],
            orientation=pose.orientation,
        )
        for name, pose in initial.items()
    }

    max_delta = 0.0
    for _ in range(100):
        result = ik.step(targets, seed, 0.01)
        assert result is not None
        max_delta = max(
            max_delta,
            float(np.max(np.abs(np.asarray(result.position) - np.asarray(seed.position)))),
        )
        seed = result

    final = ik.frame_poses(seed, frames)
    errors = [
        np.linalg.norm(
            np.array([target.position.x, target.position.y, target.position.z])
            - np.array(
                [
                    final[name].position.x,
                    final[name].position.y,
                    final[name].position.z,
                ]
            )
        )
        for name, target in targets.items()
    ]
    assert seed.position[3] > 0.1
    assert seed.position[10] > 0.1
    assert max(errors) < 1e-3
    assert np.rad2deg(max_delta) < 5.0

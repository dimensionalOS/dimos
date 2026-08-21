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

"""Behavior tests for unified single- and two-hand Quest teleoperation."""

from pathlib import Path
from types import SimpleNamespace
from typing import cast

import pytest
from pytest_mock import MockerFixture

from dimos.control.coordinator import TaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver, PoseTargetIKTaskConfig
from dimos.control.tasks.teleop_ik_task.teleop_ik_task import (
    OperatorHand,
    TeleopHandBinding,
    TeleopIKTask,
    TeleopIKTaskConfig,
    create_task,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.teleop.quest.quest_types import Buttons


def _robot_model() -> RobotModelConfig:
    return RobotModelConfig(
        name="robot",
        model=RobotModel.from_file(Path("fake.urdf")),
        joint_names=["model_left", "model_right"],
        joint_name_mapping={
            "robot/left": "model_left",
            "robot/right": "model_right",
        },
    )


def _binding(
    hand: str,
    frame: str,
    gripper_joint: str | None = None,
) -> TeleopHandBinding:
    return TeleopHandBinding(
        hand=cast("OperatorHand", hand),
        target_frame=frame,
        gripper_joint=gripper_joint,
        gripper_open_position=1.0,
        gripper_closed_position=0.0,
    )


def _config(bindings: tuple[TeleopHandBinding, ...], *, timeout: float = 0.5) -> TeleopIKTaskConfig:
    return TeleopIKTaskConfig(
        joint_names=("robot/left", "robot/right"),
        robot_model=_robot_model(),
        bindings=bindings,
        timeout=timeout,
    )


def _solver(mocker: MockerFixture) -> PinkPoseTargetSolver:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    solver.frame_poses.return_value = {
        "left_tool": PoseStamped(
            position=Vector3(1.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        "right_tool": PoseStamped(
            position=Vector3(-1.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
    }
    solver.step.return_value = JointState(
        name=["robot/left", "robot/right"], position=[0.01, -0.01]
    )
    return solver


def _state(t_now: float = 1.0) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"robot/left": 0.0, "robot/right": 0.0}),
        t_now=t_now,
        dt=0.01,
    )


def _buttons(
    *,
    left: bool = False,
    right: bool = False,
    left_trigger: float = 0.0,
    right_trigger: float = 0.0,
) -> Buttons:
    buttons = Buttons()
    buttons.left_primary = left
    buttons.right_primary = right
    buttons.pack_analog_triggers(left_trigger, right_trigger)
    return buttons


def _pose(x: float) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, 0.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


class _CustomPoseTargetSolver(PinkPoseTargetSolver):
    instances: list["_CustomPoseTargetSolver"] = []

    def __init__(self, config: PoseTargetIKTaskConfig) -> None:
        self.received_config = config
        self.instances.append(self)


@pytest.mark.parametrize(
    ("bindings", "message"),
    [
        ((), "exactly one or two"),
        (
            (
                _binding("left", "left_tool"),
                _binding("right", "right_tool"),
                _binding("left", "third_tool"),
            ),
            "exactly one or two",
        ),
        (
            (_binding("left", "left_tool"), _binding("left", "right_tool")),
            "unique operator hands",
        ),
        (
            (_binding("left", "tool"), _binding("right", "tool")),
            "unique target frames",
        ),
        (
            (
                _binding("left", "left_tool", "robot/gripper"),
                _binding("right", "right_tool", "robot/gripper"),
            ),
            "unique gripper joints",
        ),
    ],
)
def test_binding_configuration_rejects_invalid_collections(
    mocker: MockerFixture,
    bindings: tuple[TeleopHandBinding, ...],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        TeleopIKTask("quest", _config(bindings), solver=_solver(mocker))


def test_single_binding_tracks_relative_controller_motion(mocker: MockerFixture) -> None:
    solver = _solver(mocker)
    task = TeleopIKTask(
        "quest",
        _config((_binding("right", "right_tool"),)),
        solver=solver,
    )
    task.on_teleop_buttons(_buttons(right=True), 1.0)
    task.on_right_cartesian_command(_pose(0.5), 1.0)
    assert task.compute(_state()) is not None

    task.on_right_cartesian_command(_pose(0.7), 1.1)
    output = task.compute(_state(1.1))

    assert output is not None
    target = solver.step.call_args.args[0]["right_tool"]
    assert target.position.x == pytest.approx(-0.8)


def test_bimanual_task_requires_both_hands_and_releases_atomically(
    mocker: MockerFixture,
) -> None:
    solver = _solver(mocker)
    task = TeleopIKTask(
        "quest",
        _config(
            (
                _binding("left", "left_tool"),
                _binding("right", "right_tool"),
            )
        ),
        solver=solver,
    )
    task.on_teleop_buttons(_buttons(left=True), 1.0)
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    task.on_right_cartesian_command(_pose(-0.1), 1.0)
    assert task.compute(_state()) is None

    task.on_teleop_buttons(_buttons(left=True, right=True), 1.1)
    task.on_left_cartesian_command(_pose(0.1), 1.1)
    task.on_right_cartesian_command(_pose(-0.1), 1.1)
    assert task.compute(_state(1.1)) is not None
    assert solver.frame_poses.call_count == 1

    task.on_teleop_buttons(_buttons(left=True, right=False), 1.2)
    assert task.compute(_state(1.2)) is None
    assert not task.is_active()


def test_deadman_reengagement_reseeds_command_from_feedback(
    mocker: MockerFixture,
) -> None:
    solver = _solver(mocker)
    task = TeleopIKTask(
        "quest",
        _config((_binding("left", "left_tool"),)),
        solver=solver,
    )
    task.on_teleop_buttons(_buttons(left=True), 1.0)
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    solver.step.return_value = JointState(name=["robot/left", "robot/right"], position=[0.1, -0.1])
    assert task.compute(_state()) is not None

    task.on_teleop_buttons(_buttons(), 1.1)
    task.on_teleop_buttons(_buttons(left=True), 1.2)
    task.on_left_cartesian_command(_pose(0.2), 1.2)
    solver.step.return_value = JointState(
        name=["robot/left", "robot/right"], position=[0.01, -0.01]
    )
    assert task.compute(_state(1.2)) is not None

    assert solver.reset.call_count >= 1


def test_estop_and_preemption_clear_command_session(mocker: MockerFixture) -> None:
    solver = _solver(mocker)
    task = TeleopIKTask(
        "quest",
        _config((_binding("left", "left_tool"),)),
        solver=solver,
    )
    task.on_teleop_buttons(_buttons(left=True), 1.0)
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    assert task.compute(_state()) is not None

    task.set_estop(True)
    assert not task.is_active()
    task.set_estop(False)
    task.on_teleop_buttons(_buttons(left=True), 1.1)
    task.on_left_cartesian_command(_pose(0.2), 1.1)
    assert task.compute(_state(1.1)) is not None
    assert solver.reset.call_count >= 1

    task.on_preempted("trajectory", frozenset({"robot/left"}))
    assert not task.is_active()


def test_bimanual_timeout_clears_both_sides_and_reengagement_recaptures(
    mocker: MockerFixture,
) -> None:
    solver = _solver(mocker)
    task = TeleopIKTask(
        "quest",
        _config(
            (
                _binding("left", "left_tool"),
                _binding("right", "right_tool"),
            ),
            timeout=0.2,
        ),
        solver=solver,
    )
    task.on_teleop_buttons(_buttons(left=True, right=True), 1.0)
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    task.on_right_cartesian_command(_pose(-0.1), 1.0)
    assert task.compute(_state(1.0)) is not None

    task.on_left_cartesian_command(_pose(0.2), 1.25)
    assert task.compute(_state(1.25)) is None
    assert not task.is_active()

    task.on_teleop_buttons(_buttons(), 1.3)
    task.on_teleop_buttons(_buttons(left=True, right=True), 1.4)
    task.on_left_cartesian_command(_pose(0.2), 1.4)
    task.on_right_cartesian_command(_pose(-0.2), 1.4)
    assert task.compute(_state(1.4)) is not None
    assert solver.frame_poses.call_count == 2
    assert solver.reset.call_count >= 1


@pytest.mark.parametrize(
    ("bindings", "buttons"),
    [
        ((_binding("left", "left_tool"),), _buttons(left=True)),
        (
            (_binding("left", "left_tool"), _binding("right", "right_tool")),
            _buttons(left=True, right=True),
        ),
    ],
)
def test_stale_deadman_stops_fresh_pose_streams(
    mocker: MockerFixture,
    bindings: tuple[TeleopHandBinding, ...],
    buttons: Buttons,
) -> None:
    solver = _solver(mocker)
    task = TeleopIKTask("quest", _config(bindings, timeout=0.2), solver=solver)
    task.on_teleop_buttons(buttons, 1.0)
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    if len(bindings) == 2:
        task.on_right_cartesian_command(_pose(-0.1), 1.0)
    assert task.compute(_state(1.0)) is not None

    task.on_left_cartesian_command(_pose(0.2), 1.25)
    if len(bindings) == 2:
        task.on_right_cartesian_command(_pose(-0.2), 1.25)

    assert task.compute(_state(1.25)) is None
    assert not task.is_active()
    assert solver.step.call_count == 1


def test_fresh_deadman_keeps_pose_stream_active(mocker: MockerFixture) -> None:
    solver = _solver(mocker)
    task = TeleopIKTask(
        "quest",
        _config((_binding("left", "left_tool"),), timeout=0.2),
        solver=solver,
    )
    task.on_teleop_buttons(_buttons(left=True), 1.0)
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    assert task.compute(_state(1.0)) is not None

    task.on_teleop_buttons(_buttons(left=True), 1.15)
    task.on_left_cartesian_command(_pose(0.2), 1.25)

    assert task.compute(_state(1.25)) is not None
    assert task.is_active()


def test_bimanual_step_contains_both_targets_and_grippers(
    mocker: MockerFixture,
) -> None:
    solver = _solver(mocker)
    task = TeleopIKTask(
        "quest",
        _config(
            (
                _binding("left", "left_tool", "robot/left_gripper"),
                _binding("right", "right_tool", "robot/right_gripper"),
            )
        ),
        solver=solver,
    )
    task.on_teleop_buttons(
        _buttons(left=True, right=True, left_trigger=0.25, right_trigger=0.75),
        1.0,
    )
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    task.on_right_cartesian_command(_pose(-0.1), 1.0)

    output = task.compute(_state())

    assert output is not None
    assert set(solver.step.call_args.args[0]) == {
        "left_tool",
        "right_tool",
    }
    assert output.joint_names == [
        "robot/left",
        "robot/right",
        "robot/left_gripper",
        "robot/right_gripper",
    ]
    assert output.positions is not None
    assert output.positions[2:] == pytest.approx([0.75, 0.25], abs=0.01)


def test_factory_rejects_gripper_missing_from_hardware() -> None:
    cfg = TaskConfig(
        name="quest",
        type="teleop_ik",
        joint_names=["robot/left", "robot/right"],
        params={
            "robot_model": _robot_model(),
            "bindings": [
                {
                    "hand": "left",
                    "target_frame": "left_tool",
                    "gripper_joint": "robot/missing_gripper",
                }
            ],
        },
    )

    with pytest.raises(ValueError, match="unknown gripper"):
        create_task(
            cfg,
            hardware={"robot": SimpleNamespace(joint_names=["robot/left", "robot/right"])},
        )


def test_factory_constructs_plain_pose_target_solver_by_default(
    mocker: MockerFixture,
) -> None:
    init = mocker.patch.object(PinkPoseTargetSolver, "__init__", return_value=None)
    cfg = TaskConfig(
        name="quest",
        type="teleop_ik",
        joint_names=["robot/left", "robot/right"],
        params={
            "robot_model": _robot_model(),
            "bindings": [{"hand": "left", "target_frame": "left_tool"}],
        },
    )

    task = create_task(cfg, hardware={})

    assert type(task._solver) is PinkPoseTargetSolver
    assert task._config.max_joint_velocity_rad_s == 5.0
    assert task._config.joint_velocity_limits_rad_s == {}
    assert task._config.joint_command_filter_cutoff_hz == 5.0
    assert task._config.max_command_tracking_error_deg == 10.0
    assert init.call_args.args[0].pink == task._config.pink


def test_factory_constructs_fresh_custom_solver_for_each_task() -> None:
    _CustomPoseTargetSolver.instances.clear()
    cfg = TaskConfig(
        name="quest",
        type="teleop_ik",
        joint_names=["robot/left", "robot/right"],
        params={
            "robot_model": _robot_model(),
            "bindings": [{"hand": "left", "target_frame": "left_tool"}],
            "solver_type": _CustomPoseTargetSolver,
        },
    )

    first = create_task(cfg, hardware={})
    second = create_task(cfg, hardware={})

    assert len(_CustomPoseTargetSolver.instances) == 2
    assert first._solver is _CustomPoseTargetSolver.instances[0]
    assert second._solver is _CustomPoseTargetSolver.instances[1]
    assert first._solver is not second._solver
    assert _CustomPoseTargetSolver.instances[0].received_config == first._config

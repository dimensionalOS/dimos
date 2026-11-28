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

import math
from pathlib import Path

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_twist_task.eef_twist_task import (
    EEFTwistTask,
    EEFTwistTaskConfig,
)
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel


def _robot_model() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("fake.urdf")),
        joint_names=["arm/joint1", "arm/joint2"],
    )


def _solver(mocker: MockerFixture) -> PinkPoseTargetSolver:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    solver.frame_poses.return_value = {
        "tool": PoseStamped(frame_id="world", position=[0.0, 0.0, 0.0])
    }
    solver.step.return_value = JointState(
        name=["arm/joint1", "arm/joint2"],
        position=[0.01, 0.02],
    )
    return solver


def _config(
    *,
    target_frames: tuple[str, ...] = ("tool",),
    command_timeout: float = 0.3,
) -> EEFTwistTaskConfig:
    return EEFTwistTaskConfig(
        joint_names=("arm/joint1", "arm/joint2"),
        robot_model=_robot_model(),
        target_frames=target_frames,
        timeout=0.0,
        command_timeout=command_timeout,
    )


def _task(
    mocker: MockerFixture, *, command_timeout: float = 0.3
) -> tuple[EEFTwistTask, PinkPoseTargetSolver]:
    solver = _solver(mocker)
    task = EEFTwistTask(
        "eef",
        _config(command_timeout=command_timeout),
        solver=solver,
    )
    return task, solver


@pytest.mark.parametrize("target_frames", [(), ("tool", "other")])
def test_eef_twist_config_requires_exactly_one_target_frame(
    target_frames: tuple[str, ...],
) -> None:
    with pytest.raises(ValueError):
        _config(target_frames=target_frames)


def _state(
    t_now: float = 1.0,
    *,
    dt: float = 0.01,
    measured: tuple[float, float] = (0.0, 0.0),
    held: tuple[float, float] | None = None,
) -> CoordinatorState:
    names = ("arm/joint1", "arm/joint2")
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=dict(zip(names, measured, strict=True)),
            joint_held_positions={} if held is None else dict(zip(names, held, strict=True)),
        ),
        t_now=t_now,
        dt=dt,
    )


def _restarted_at(solver: PinkPoseTargetSolver) -> list[float]:
    """The joints the task last restarted the solver's command at."""
    return list(solver.reset.call_args.args[0].position)


def _twist(x: float = 0.1) -> TwistStamped:
    return TwistStamped(frame_id="tool", linear=[x, 0.0, 0.0], angular=[0.0, 0.0, 0.0])


def test_twist_integrates_target_and_uses_shared_persistent_command(
    mocker: MockerFixture,
) -> None:
    task, solver = _task(mocker)
    assert task.on_ee_twist_command(_twist(), t_now=1.0)

    first = task.compute(_state(1.01))
    second = task.compute(_state(1.02))

    assert first is not None
    assert second is not None
    first_target = solver.step.call_args_list[0].args[0]["tool"]
    second_target = solver.step.call_args_list[1].args[0]["tool"]
    assert first_target.position.x == pytest.approx(0.001)
    assert second_target.position.x == pytest.approx(0.002)


def test_zero_twist_reanchors_target_at_measured_pose(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))
    resets = solver.reset.call_count
    task.on_ee_twist_command(_twist(0.0), t_now=1.02)

    task.compute(_state(1.03))

    target = solver.step.call_args.args[0]["tool"]
    assert target.position.x == pytest.approx(0.0)
    assert solver.reset.call_count == resets + 1


def test_release_restarts_at_the_held_joints_not_the_measured_ones(
    mocker: MockerFixture,
) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01, measured=(0.02, -0.03), held=(0.0, 0.0)))
    task.on_ee_twist_command(_twist(0.0), t_now=1.02)

    task.compute(_state(1.03, measured=(0.12, 0.07), held=(0.1, 0.1)))

    assert _restarted_at(solver) == pytest.approx([0.1, 0.1])


def test_jog_after_idle_starts_where_another_task_left_the_arm_held(
    mocker: MockerFixture,
) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01, held=(0.0, 0.0)))
    task.compute(_state(1.32, held=(0.0, 0.0)))
    assert not task.is_active()

    task.on_ee_twist_command(_twist(), t_now=2.0)
    task.compute(_state(2.01, measured=(0.51, 0.49), held=(0.5, 0.5)))

    assert _restarted_at(solver) == pytest.approx([0.5, 0.5])


def test_jog_starts_at_the_measured_joints_when_none_is_held(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)

    task.compute(_state(1.01, measured=(0.02, -0.03)))

    assert _restarted_at(solver) == pytest.approx([0.02, -0.03])


def test_a_hold_far_from_the_arm_is_brought_within_the_tracking_limit(
    mocker: MockerFixture,
) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)

    task.compute(_state(1.01, measured=(0.0, 0.0), held=(1.0, -1.0)))

    limit = math.radians(10.0)
    assert _restarted_at(solver) == pytest.approx([limit, -limit])


def test_twist_back_soon_after_an_unreachable_push_moves_at_once(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)  # the measured pose stays at x=0
    for tick in range(100):
        task.on_ee_twist_command(_twist(1.0), t_now=1.0 + tick * 0.01)
        task.compute(_state(1.0 + tick * 0.01))
    task.on_ee_twist_command(_twist(0.0), t_now=2.0)
    task.on_ee_twist_command(_twist(-1.0), t_now=2.1)  # within command_timeout

    task.compute(_state(2.11))

    target = solver.step.call_args.args[0]["tool"]
    assert target.position.x == pytest.approx(-0.01)


def test_stale_twist_holds_one_tick_then_goes_idle(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))

    output = task.compute(_state(1.31))

    assert output is not None
    target = solver.step.call_args.args[0]["tool"]
    assert target.position.x == pytest.approx(0.001)
    assert not task.is_active()


def test_task_is_active_only_while_twists_arrive(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    assert not task.is_active()

    task.on_ee_twist_command(_twist(), t_now=1.0)
    assert task.is_active()
    assert task.compute(_state(1.01)) is not None
    assert task.compute(_state(1.2)) is not None
    assert task.is_active()

    task.compute(_state(1.31))
    assert not task.is_active()

    resets = solver.reset.call_count
    task.on_ee_twist_command(_twist(), t_now=2.0)
    assert task.is_active()
    assert task.compute(_state(2.01)) is not None
    assert solver.reset.call_count == resets + 1
    # Re-anchored at the measured pose, not the target held before idling.
    assert solver.frame_poses.call_count == 2
    target = solver.step.call_args.args[0]["tool"]
    assert target.position.x == pytest.approx(0.001)


def test_zero_twist_does_not_keep_task_active(mocker: MockerFixture) -> None:
    task, _ = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))
    task.on_ee_twist_command(_twist(0.0), t_now=1.25)

    task.compute(_state(1.31))
    assert not task.is_active()

    task.on_ee_twist_command(_twist(0.0), t_now=1.4)
    assert not task.is_active()


def test_zero_timeout_never_goes_idle(mocker: MockerFixture) -> None:
    task, solver = _task(mocker, command_timeout=0.0)
    assert task.is_active()
    assert task.compute(_state(1.0)) is not None

    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.on_ee_twist_command(_twist(0.0), t_now=1.01)
    assert task.compute(_state(1.02)) is not None
    assert task.compute(_state(10.0)) is not None
    assert task.is_active()
    assert solver.frame_poses.call_count == 2  # re-anchored once, when the twist stopped


def test_estop_rejects_input_and_reanchors_after_clear(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))

    task.set_estop(True)
    assert not task.on_ee_twist_command(_twist(), t_now=1.02)
    assert task.compute(_state(1.02)) is None

    task.set_estop(False)
    assert task.compute(_state(1.03)) is not None
    assert solver.frame_poses.call_count == 2


def test_preemption_discards_twist_target_and_command_state(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))

    task.on_preempted("trajectory", frozenset({"arm/joint1"}))
    task.compute(_state(1.02))

    assert solver.frame_poses.call_count == 2
    assert solver.reset.call_count >= 1


def test_invalid_twist_is_rejected(mocker: MockerFixture) -> None:
    task, _ = _task(mocker)
    invalid = _twist()
    invalid.linear.x = np.nan

    assert not task.on_ee_twist_command(invalid, t_now=1.0)

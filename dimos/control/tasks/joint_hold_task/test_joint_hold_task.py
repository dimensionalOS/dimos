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

from __future__ import annotations

import pytest

from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.joint_hold_task.joint_hold_task import (
    JointHoldTask,
    JointHoldTaskConfig,
)

ARMS = ["arm/joint1", "arm/joint2"]
BASE = ["base/vx", "base/vy", "base/wz"]
AT_REST = {"base/vx": 0.0, "base/vy": 0.0, "base/wz": 0.0}


def _task(**overrides: object) -> JointHoldTask:
    config = JointHoldTaskConfig(
        joint_names=list(ARMS),
        base_joint_names=list(BASE),
        **overrides,  # type: ignore[arg-type]
    )
    return JointHoldTask("hold_arms", config)


def _state(arms: dict[str, float], base: dict[str, float], t_now: float) -> CoordinatorState:
    """A tick where the arms are at `arms` and the base is commanded at `base`."""
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=dict(arms),
            joint_velocities=dict(base),
        ),
        t_now=t_now,
    )


def _arms(j1: float = 0.1, j2: float = 0.2) -> dict[str, float]:
    return {"arm/joint1": j1, "arm/joint2": j2}


def _driving(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> dict[str, float]:
    return {"base/vx": vx, "base/vy": vy, "base/wz": wz}


def test_commands_nothing_while_the_base_rests() -> None:
    task = _task()
    assert task.compute(_state(_arms(), AT_REST, 1.0)) is None


def test_a_slow_crawl_is_not_movement() -> None:
    # Below both thresholds: command noise must not pin the arms.
    task = _task()
    assert task.compute(_state(_arms(), _driving(vx=0.001, wz=0.001), 1.0)) is None


def test_holds_the_pose_the_arm_was_in_when_the_base_started() -> None:
    task = _task()
    out = task.compute(_state(_arms(0.1, 0.2), _driving(vx=0.3), 1.0))
    assert out is not None
    assert out.mode is ControlMode.SERVO_POSITION
    assert out.joint_names == ARMS
    assert out.positions == [0.1, 0.2]

    # The arm is swinging; the command stays on the latched pose, not the new one.
    out = task.compute(_state(_arms(0.4, -0.3), _driving(vx=0.3), 1.05))
    assert out is not None
    assert out.positions == [0.1, 0.2]


def test_sees_motion_from_any_writer_not_just_teleop() -> None:
    """The point of reading the base off the coordinator: the virtual joints carry
    the aggregate of teleop, the route follower and a plan's base segment alike."""
    task = _task()
    # A pure strafe, as a whole-body plan's base segment would command.
    assert task.compute(_state(_arms(), _driving(vy=0.25), 1.0)) is not None


def test_yaw_alone_counts_as_movement() -> None:
    task = _task()
    assert task.compute(_state(_arms(), _driving(wz=0.4), 1.0)) is not None


def test_releases_once_the_base_has_been_quiet() -> None:
    task = _task(release_after_s=0.5)
    assert task.compute(_state(_arms(), _driving(vx=0.3), 1.0)) is not None

    # Still inside the release window.
    assert task.compute(_state(_arms(), AT_REST, 1.2)) is not None
    # Past it.
    assert task.compute(_state(_arms(), AT_REST, 1.6)) is None


def test_continued_driving_extends_the_hold() -> None:
    task = _task(release_after_s=0.5)
    task.compute(_state(_arms(), _driving(vx=0.3), 1.0))
    task.compute(_state(_arms(), _driving(vx=0.3), 1.4))
    assert task.compute(_state(_arms(), AT_REST, 1.6)) is not None


def test_preemption_drops_the_latch_and_relatches_where_the_arm_ended_up() -> None:
    task = _task()
    task.compute(_state(_arms(0.1, 0.2), _driving(vx=0.3), 1.0))

    # A plan takes the joints and moves the arm somewhere else.
    task.on_preempted("joint_trajectory", frozenset({"arm/joint1"}))

    out = task.compute(_state(_arms(0.9, -0.4), _driving(vx=0.3), 2.0))
    assert out is not None
    assert out.positions == [0.9, -0.4]


def test_preemption_of_unrelated_joints_keeps_the_latch() -> None:
    task = _task()
    task.compute(_state(_arms(0.1, 0.2), _driving(vx=0.3), 1.0))

    task.on_preempted("base_trajectory", frozenset(BASE))

    out = task.compute(_state(_arms(0.9, -0.4), _driving(vx=0.3), 1.1))
    assert out is not None
    assert out.positions == [0.1, 0.2]


def test_a_missing_arm_joint_holds_nothing() -> None:
    # Pinning a subset would leave the rest to swing into the held joints,
    # which is the collision this task exists to prevent.
    task = _task()
    assert task.compute(_state({"arm/joint1": 0.1}, _driving(vx=0.3), 1.0)) is None


def test_latches_on_a_later_tick_once_the_arm_state_is_complete() -> None:
    task = _task()
    assert task.compute(_state({"arm/joint1": 0.1}, _driving(vx=0.3), 1.0)) is None

    out = task.compute(_state(_arms(0.15, 0.25), _driving(vx=0.3), 1.02))
    assert out is not None
    assert out.positions == [0.15, 0.25]


def test_a_base_joint_absent_from_the_tick_is_not_motion() -> None:
    """A base that has not reported yet must not be read as driving."""
    task = _task()
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions=_arms()),
        t_now=1.0,
    )
    assert task.compute(state) is None


def test_without_base_joints_only_the_explicit_hold_fires() -> None:
    task = JointHoldTask(
        "hold_arms", JointHoldTaskConfig(joint_names=list(ARMS), base_joint_names=[])
    )
    assert task.compute(_state(_arms(), _driving(vx=0.9), 1.0)) is None

    assert task.hold(t_now=1.0) is True
    out = task.compute(_state(_arms(), _driving(vx=0.9), 1.0))
    assert out is not None
    assert out.positions == [0.1, 0.2]


def test_release_and_disable() -> None:
    task = _task()
    task.compute(_state(_arms(), _driving(vx=0.3), 1.0))
    task.release()
    assert task.compute(_state(_arms(), AT_REST, 1.1)) is None

    task.set_enabled(False)
    assert task.compute(_state(_arms(), _driving(vx=0.3), 2.0)) is None
    assert task.get_status()["enabled"] is False

    task.set_enabled(True)
    assert task.compute(_state(_arms(), _driving(vx=0.3), 3.0)) is not None


def test_start_and_stop_drive_the_enable_flag() -> None:
    # auto_start=True makes the coordinator call start(); without it the task is
    # added and then immediately torn down again.
    task = _task()
    task.stop()
    assert task.compute(_state(_arms(), _driving(vx=0.3), 1.0)) is None

    task.start()
    assert task.compute(_state(_arms(), _driving(vx=0.3), 2.0)) is not None


def test_claim_sits_below_the_trajectory_task() -> None:
    from dimos.control.tasks.trajectory_task.trajectory_task import JointTrajectoryTaskConfig

    claim = _task().claim()
    assert claim.mode is ControlMode.SERVO_POSITION
    # It claims only the arms; the base joints it reads belong to other tasks.
    assert claim.joints == frozenset(ARMS)
    assert claim.priority < JointTrajectoryTaskConfig(joint_names=list(ARMS)).priority


def test_requires_joints() -> None:
    with pytest.raises(ValueError, match="at least one joint"):
        JointHoldTask("hold_arms", JointHoldTaskConfig(joint_names=[]))

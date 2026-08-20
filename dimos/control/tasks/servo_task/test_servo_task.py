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

"""Behavioral tests for the uniform ``(msg, t_now)`` joint_command handler."""

from __future__ import annotations

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.servo_task.servo_task import JointServoTask, JointServoTaskConfig
from dimos.msgs.sensor_msgs.JointState import JointState


def _task() -> JointServoTask:
    return JointServoTask("servo", JointServoTaskConfig(joint_names=["a/j1", "a/j2"]))


def test_on_joint_command_sets_position_targets() -> None:
    task = _task()
    assert task.on_joint_command(JointState(name=["a/j1", "a/j2"], position=[0.1, 0.2]), 1.0)
    out = task.compute(CoordinatorState(joints=JointStateSnapshot(), t_now=1.0))
    assert out is not None
    assert out.positions == [0.1, 0.2]


def test_on_joint_command_ignores_messages_without_positions() -> None:
    task = _task()
    assert not task.on_joint_command(JointState(name=["a/j1", "a/j2"], velocity=[0.1, 0.2]), 1.0)
    assert not task.is_active()


def test_on_joint_command_requires_all_claimed_joints() -> None:
    task = _task()
    assert not task.on_joint_command(JointState(name=["a/j1"], position=[0.1]), 1.0)
    assert not task.is_active()


def _state(t_now: float, positions: dict[str, float] | None = None) -> CoordinatorState:
    return CoordinatorState(joints=JointStateSnapshot(joint_positions=positions or {}), t_now=t_now)


def test_preempted_joints_track_measured_positions() -> None:
    task = _task()
    task.on_joint_command(JointState(name=["a/j1", "a/j2"], position=[0.0, 0.0]), 1.0)

    task.on_preempted("traj", frozenset({"a/j2"}))
    out = task.compute(_state(1.1, {"a/j1": 0.5, "a/j2": 0.8}))
    assert out is not None
    assert out.positions == [0.0, 0.8]

    out = task.compute(_state(1.2, {"a/j1": 0.5, "a/j2": 0.0}))
    assert out is not None
    assert out.positions == [0.0, 0.8]


def test_hold_resumes_at_handoff_position_after_release() -> None:
    task = _task()
    task.on_joint_command(JointState(name=["a/j1", "a/j2"], position=[0.0, 0.0]), 1.0)

    for step in range(5):
        task.on_preempted("traj", frozenset({"a/j1", "a/j2"}))
        task.compute(_state(1.0 + 0.1 * step, {"a/j1": 0.2 * step, "a/j2": 0.1 * step}))

    out = task.compute(_state(1.5, {"a/j1": 0.81, "a/j2": 0.41}))
    assert out is not None
    assert out.positions == [0.8, 0.4]


def test_hold_measured_on_start_latches_the_real_pose() -> None:
    # A configured pose would snap the joints there from wherever the robot
    # is, at full stiffness, on the first tick — the G1 arms did exactly that.
    task = JointServoTask(
        "servo_arms",
        JointServoTaskConfig(joint_names=["a", "b"], timeout=0.0, hold_measured_on_start=True),
    )
    task.start()

    # Stays active while waiting so it gets a chance to latch.
    assert task.is_active()
    assert task.compute(_state(1.0)) is None, "must not command before state arrives"

    out = task.compute(_state(1.1, {"a": 0.4, "b": -0.25}))
    assert out is not None
    assert out.positions == [0.4, -0.25]

    # Once latched it holds that pose, not whatever the joints drift to.
    held = task.compute(_state(1.2, {"a": 0.9, "b": -0.9}))
    assert held is not None
    assert held.positions == [0.4, -0.25]


def test_default_positions_still_command_the_configured_pose() -> None:
    task = JointServoTask(
        "servo",
        JointServoTaskConfig(joint_names=["a"], timeout=0.0, default_positions=[0.0]),
    )
    task.start()
    out = task.compute(_state(1.0, {"a": 0.7}))
    assert out is not None
    assert out.positions == [0.0]

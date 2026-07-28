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

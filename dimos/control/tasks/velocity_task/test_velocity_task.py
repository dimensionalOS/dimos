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

import time

import numpy as np

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.velocity_task.velocity_task import (
    JointVelocityTask,
    JointVelocityTaskConfig,
)
from dimos.message_utils import time as message_time
from dimos.messages.sensor_msgs.msg import JointState
from dimos.messages.std_msgs.msg import Header


def _task() -> JointVelocityTask:
    return JointVelocityTask("velocity", JointVelocityTaskConfig(joint_names=["a/j1", "a/j2"]))


def test_on_joint_command_sets_velocities() -> None:
    task = _task()
    assert task.on_joint_command(
        JointState(
            header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
            name=["a/j1", "a/j2"],
            position=np.asarray([], dtype=np.float64),
            velocity=np.asarray([0.3, -0.1], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        ),
        1.0,
    )
    out = task.compute(CoordinatorState(joints=JointStateSnapshot(), t_now=1.0))
    assert out is not None
    assert list(out.velocities) == [0.3, -0.1]


def test_on_joint_command_ignores_position_bearing_messages() -> None:
    # Mirrors the coordinator's if-position-elif-velocity split: a message
    # carrying positions must never drive the velocity task.
    task = _task()
    both = JointState(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
        name=["a/j1", "a/j2"],
        position=np.asarray([0.1, 0.2], dtype=np.float64),
        velocity=np.asarray([0.3, -0.1], dtype=np.float64),
        effort=np.asarray([], dtype=np.float64),
    )
    assert not task.on_joint_command(both, 1.0)
    positions = JointState(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
        name=["a/j1", "a/j2"],
        position=np.asarray([0.1, 0.2], dtype=np.float64),
        velocity=np.asarray([], dtype=np.float64),
        effort=np.asarray([], dtype=np.float64),
    )
    assert not task.on_joint_command(positions, 1.0)
    assert not task.is_active()

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

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.spatial_control import R1TorsoHeightTask, height_step


def state(t_now: float = 10.0) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={
                f"r1pro/torso_joint{i}": q for i, q in enumerate([0.4, -0.8, -0.4, 0.3], 1)
            }
        ),
        t_now=t_now,
        dt=0.02,
    )


def test_height_compensates_pitch_and_expires() -> None:
    task = R1TorsoHeightTask(np.array([[-3, 3]] * 4))
    assert task.compute(state()) is None
    task.on_torso_height_command(Float32(data=-1), 10)
    output = task.compute(state())
    assert output is not None
    assert output.joint_names == [f"r1pro/torso_joint{i}" for i in range(1, 4)]
    assert set(output.joint_names) == task.claim().joints
    assert output.positions is not None
    assert np.max(np.abs(np.array(output.positions) - [0.4, -0.8, -0.4])) <= 0.003 + 1e-12
    assert task.compute(state(10.31)) is None


def test_height_estop_and_preemption_discard_motion() -> None:
    task = R1TorsoHeightTask(np.array([[-3, 3]] * 4))
    task.on_torso_height_command(Float32(data=-1), 10)
    task.set_estop(True)
    task.on_torso_height_command(Float32(data=-1), 10)
    assert task.compute(state()) is None
    task.set_estop(False)
    assert task.compute(state()) is None
    task.on_torso_height_command(Float32(data=-1), 10)
    task.on_preempted("hold", task.claim().joints)
    assert task.compute(state()) is None


@pytest.mark.parametrize("axis", [float("nan"), float("inf"), 2.0])
def test_invalid_height_command_cancels_previous_motion(axis: float) -> None:
    task = R1TorsoHeightTask(np.array([[-3, 3]] * 4))
    task.on_torso_height_command(Float32(data=-1), 10)
    task.on_torso_height_command(Float32(data=axis), 10)
    assert task.compute(state()) is None


def torso_pose(q):
    return (
        0.35 * (math.sin(q[0]) + math.sin(q[0] + q[1])),
        0.35 * (math.cos(q[0]) + math.cos(q[0] + q[1])),
        q[0] + q[1] - q[2],
        q[3],
    )


@pytest.mark.parametrize("axis", [-1, 1])
def test_torso_vertical_step_preserves_reach_pitch_and_yaw(axis):
    q = np.array([0.4, -0.8, -0.4, 0.3])
    initial = torso_pose(q)
    for _ in range(20):
        next_q = height_step(q, axis, 0.02, np.array([[-3, 3]] * 4))
        assert np.max(np.abs(next_q - q)) <= 0.15 * 0.02 + 1e-12
        q = next_q
    final = torso_pose(q)
    assert (final[1] - initial[1]) * axis > 0
    np.testing.assert_allclose(
        [final[0], final[2], final[3]], [initial[0], initial[2], initial[3]], atol=1e-8
    )


def test_active_arm_ik_can_take_pitch_without_interrupting_height() -> None:
    task = R1TorsoHeightTask(np.array([[-3, 3]] * 4))
    task.on_torso_height_command(Float32(data=-1), 10)
    task.on_preempted("teleop", frozenset({"r1pro/torso_joint3"}))
    assert task.compute(state()) is not None

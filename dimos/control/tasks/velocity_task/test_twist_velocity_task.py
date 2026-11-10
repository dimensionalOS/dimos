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

import math

import pytest

from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.velocity_task.twist_velocity_task import TwistVelocityTask
from dimos.control.tasks.velocity_task.velocity_task import JointVelocityTaskConfig
from dimos.msgs.geometry_msgs.Twist import Twist

JOINTS = ["robot/base_x", "robot/base_y", "robot/base_yaw"]


def task() -> TwistVelocityTask:
    return TwistVelocityTask(
        "input",
        JointVelocityTaskConfig(JOINTS),
        ["linear.x", "linear.y", "angular.z"],
        yaw_joint=JOINTS[2],
    )


def state(t: float, yaw: float | None) -> CoordinatorState:
    positions = {} if yaw is None else {JOINTS[2]: yaw}
    return CoordinatorState(joints=JointStateSnapshot(joint_positions=positions), t_now=t)


def test_body_intent_rotates_each_tick_without_refreshing_input_timeout() -> None:
    t = task()
    assert t.claim().joints == frozenset(JOINTS)
    assert t.claim().mode == ControlMode.VELOCITY
    assert t.on_twist_command(Twist(linear=[0.4, 0.0, 0.0], angular=[0.0, 0.0, 0.3]), 1.0)
    out = t.compute(state(1.0, 0.0))
    assert out is not None and out.velocities == [0.4, 0.0, 0.3]
    out = t.compute(state(1.1, math.pi / 2))
    assert out is not None and out.velocities == pytest.approx([0.0, 0.4, 0.3])
    out = t.compute(state(1.3, math.pi / 2))
    assert out is not None and out.velocities == [0.0, 0.0, 0.0]


@pytest.mark.parametrize("yaw", [None, math.nan, math.inf])
def test_missing_or_invalid_measured_yaw_fails_closed(yaw: float | None) -> None:
    t = task()
    t.on_twist_command(Twist(linear=[0.4, 0.0, 0.0]), 1.0)
    assert t.compute(state(1.0, yaw)) is None


def test_estop_clears_raw_intent_and_requires_fresh_post_clear_input() -> None:
    t = task()
    msg = Twist(linear=[0.4, 0.0, 0.0])
    assert t.on_twist_command(msg, 1.0)
    t.set_estop(True)
    assert not t.on_twist_command(msg, 1.1)
    t.set_estop(False)
    t.start()
    assert t.compute(state(1.1, 0.0)) is None
    assert t.on_twist_command(msg, 1.2)
    assert t.compute(state(1.2, 0.0)) is not None
    t.clear()
    t.start()
    assert t.compute(state(1.2, 0.0)) is None


def test_nonfinite_input_never_activates_task() -> None:
    t = task()
    assert not t.on_twist_command(Twist(linear=[math.nan, 0.0, 0.0]), 1.0)
    assert not t.is_active()

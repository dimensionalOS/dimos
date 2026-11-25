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
from pytest_mock import MockerFixture

from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.velocity_task.twist_velocity_task import TwistVelocityTask
from dimos.control.tasks.velocity_task.velocity_task import JointVelocityTaskConfig
from dimos.msgs.geometry_msgs.Twist import Twist

JOINTS = ["robot/base_forward", "robot/base_lateral", "robot/base_turn"]


def task() -> TwistVelocityTask:
    return TwistVelocityTask(
        "input",
        JointVelocityTaskConfig(JOINTS),
        ["linear.x", "linear.y", "angular.z"],
    )


def state(t: float) -> CoordinatorState:
    return CoordinatorState(joints=JointStateSnapshot(), t_now=t)


def test_body_axes_need_no_pose_and_do_not_refresh_input_timeout() -> None:
    t = task()
    assert t.claim().joints == frozenset(JOINTS)
    assert t.claim().mode == ControlMode.VELOCITY
    assert t.on_twist_command(Twist(linear=[0.4, -0.2, 0.0], angular=[0.0, 0.0, 0.3]), 1.0)
    for now in (1.0, 1.1):
        out = t.compute(state(now))
        assert out is not None and out.velocities == [0.4, -0.2, 0.3]
    out = t.compute(state(1.3))
    assert out is not None and out.velocities == [0.0, 0.0, 0.0]


def test_explicit_mapping_can_select_other_axes_in_joint_order() -> None:
    t = TwistVelocityTask("input", JointVelocityTaskConfig(JOINTS[:2]), ["angular.x", "linear.z"])
    assert t.on_twist_command(Twist(linear=[0.0, 0.0, 0.2], angular=[-0.3, 0.0, 0.0]), 1.0)
    out = t.compute(state(1.0))
    assert out is not None and out.velocities == [-0.3, 0.2]


def test_estop_clears_raw_intent_and_requires_fresh_post_clear_input() -> None:
    t = task()
    msg = Twist(linear=[0.4, 0.0, 0.0])
    assert t.on_twist_command(msg, 1.0)
    t.set_estop(True)
    assert not t.on_twist_command(msg, 1.1)
    t.set_estop(False)
    t.start()
    assert t.compute(state(1.1)) is None
    assert t.on_twist_command(msg, 1.2)
    assert t.compute(state(1.2)) is not None
    t.clear()
    t.start()
    assert t.compute(state(1.2)) is None


def test_nonfinite_input_never_activates_task() -> None:
    t = task()
    assert not t.on_twist_command(Twist(linear=[math.nan, 0.0, 0.0]), 1.0)
    assert not t.is_active()


def test_key_release_stops_quietly_through_repeated_idle_periods(mocker: MockerFixture) -> None:
    warning = mocker.patch("dimos.control.tasks.velocity_task.velocity_task.logger.warning")
    t = task()
    t.start()
    assert t.compute(state(1.0)) is None

    for start in (2.0, 5.0, 8.0):
        assert t.on_twist_command(Twist(linear=[0.4, 0.0, 0.0]), start)
        out = t.compute(state(start))
        assert out is not None and out.velocities == [0.4, 0.0, 0.0]
        assert t.is_streaming()

        # Keyboard teleop sends one zero Twist on release, then stays silent.
        assert t.on_twist_command(Twist(), start + 0.1)
        for elapsed in (0.1, 0.4, 1.0, 2.0):
            out = t.compute(state(start + elapsed))
            assert out is not None and out.velocities == [0.0, 0.0, 0.0]
        assert not t.is_streaming()

    warning.assert_not_called()


@pytest.mark.parametrize(
    "command",
    [
        Twist(linear=[0.4, 0.0, 0.0]),
        Twist(linear=[0.0, -0.2, 0.0]),
        Twist(angular=[0.0, 0.0, 0.3]),
        Twist(linear=[1e-10, 0.0, 0.0]),
    ],
)
def test_missing_motion_updates_warn_once_and_stop_after_idle(
    command: Twist, mocker: MockerFixture
) -> None:
    warning = mocker.patch("dimos.control.tasks.velocity_task.velocity_task.logger.warning")
    t = task()
    assert t.on_twist_command(Twist(), 1.0)
    t.compute(state(1.5))
    warning.assert_not_called()

    for start in (2.0, 4.0):
        warning.reset_mock()
        assert t.on_twist_command(command, start)
        for elapsed in (0.5, 1.0):
            out = t.compute(state(start + elapsed))
            assert out is not None and out.velocities == [0.0, 0.0, 0.0]
        assert not t.is_streaming()
        warning.assert_called_once_with("JointVelocityTask input timed out (no update for 0.500s)")

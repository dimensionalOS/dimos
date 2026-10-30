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

"""The coherent control clock: exact replies, delays, reset and arbitration."""

from dataclasses import replace

import pytest

from dimos.control._control_test_helpers import fake_arm_model
import dimos.control.coordinator as coordinator_module
from dimos.control.coordinator import ControlCoordinator
from dimos.control.feedback import ControlFeedback
from dimos.control.task import (
    ControlTask,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
    ResourceClaim,
)
from dimos.core.module import Module
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.testing.cbor import roundtrip


@pytest.fixture
def controller(mocker):
    model = fake_arm_model().with_prefix("arm/")
    names = tuple(j.name for j in model.load().joints)
    control = ControlCoordinator(
        feedback_robot="arm",
        model=model,
        motor_gains={n: (40.0, 3.0) for n in names},
    )
    mocker.patch.object(Module, "start")
    sent = mocker.patch.object(control.motor_control, "publish")
    task = mocker.Mock(spec=ControlTask)
    task.name = "test_positions"
    task.claim.return_value = ResourceClaim(
        joints=frozenset(names[:1]), priority=10, mode=ControlMode.SERVO_POSITION
    )
    task.is_active.return_value = True
    task.compute.return_value = JointCommandOutput(
        [names[0]], positions=[0.4], mode=ControlMode.SERVO_POSITION
    )
    control.add_task(task)
    control.start()
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions=dict.fromkeys(names, 0.1), timestamp=100.0),
        t_now=7.0,
        dt=0.02,
    )
    try:
        yield control, task, sent, ControlFeedback("episode-a", 350, state)
    finally:
        control.stop()


def test_duplicate_feedback_retransmits_without_advancing_tasks(controller):
    control, task, sent, sample = controller
    assert control.receive_feedback(roundtrip(sample))
    reply = sent.call_args.args[0]
    assert not control.receive_feedback(sample)
    assert control.get_tick_count() == 1
    task.compute.assert_called_once_with(sample.state)
    assert sent.call_args.args[0] is reply
    decoded = roundtrip(reply)
    assert (decoded.epoch, decoded.tick) == ("episode-a", 350)
    assert decoded.motors.q == [0.4] + [0.1] * 6  # unclaimed joints are held
    assert decoded.motors.kp == [40.0] * 7
    assert decoded.motors.kd == [3.0] * 7


def test_slow_inference_keeps_sample_clock_and_identity(controller, mocker):
    control, task, sent, sample = controller
    now = [100.0]
    mocker.patch.object(coordinator_module.time, "monotonic", side_effect=lambda: now[0])

    def slow_compute(state):
        now[0] += 2.0  # wall time passes while the supplied state remains fixed
        assert state.t_now == 7.0 and state.dt == 0.02
        return JointCommandOutput(list(state.joints.joint_positions)[:1], positions=[0.2])

    task.compute.side_effect = slow_compute
    assert control.receive_feedback(sample)
    reply = sent.call_args.args[0]
    assert (reply.epoch, reply.tick) == (sample.epoch, sample.tick)
    assert reply.motors.q == [0.2] + [0.1] * 6
    assert control.control_status()["max_compute_ms"] == pytest.approx(2000.0)


def test_reset_retires_old_feedback_and_releases_previous_holds(controller):
    control, task, sent, sample = controller
    control.receive_feedback(sample)
    task.compute.return_value = None
    reset = replace(
        sample,
        epoch="episode-b",
        tick=0,
        state=replace(
            sample.state,
            t_now=0.0,
            joints=replace(
                sample.state.joints,
                joint_positions=dict.fromkeys(sample.state.joints.joint_positions, -0.2),
            ),
        ),
    )
    assert control.receive_feedback(reset)
    assert sent.call_args.args[0].motors.q == [-0.2] * 7
    assert not control.receive_feedback(sample)
    assert control.get_tick_count() == 2


def test_controller_failure_leaves_no_motor_reply(controller):
    control, task, sent, sample = controller
    task.compute.side_effect = RuntimeError("inference failed")
    with pytest.raises(RuntimeError, match="inference failed"):
        control.receive_feedback(sample)
    sent.assert_not_called()
    assert "inference failed" in control.control_status()["error"]
    assert not control.receive_feedback(sample)
    sent.assert_not_called()


def test_velocity_deadman_uses_wall_time(controller, mocker):
    control, task, sent, sample = controller
    control.receive_feedback(sample)
    now = [100.0]
    mocker.patch.object(coordinator_module.time, "monotonic", side_effect=lambda: now[0])
    dispatch = mocker.spy(control, "_dispatch")
    control.set_velocity_command(Twist(linear=Vector3(0.2, 0, 0)))
    next_sample = replace(sample, tick=351, state=replace(sample.state, t_now=7.02))
    control.receive_feedback(next_sample)
    assert dispatch.call_args.args[1].linear.x == 0.2
    now[0] = 100.31
    control.receive_feedback(
        replace(next_sample, tick=352, state=replace(sample.state, t_now=7.04))
    )
    assert dispatch.call_args.args[1].linear.x == 0.0


def test_estop_clear_recomputes_the_frozen_sample_once(controller):
    control, task, sent, sample = controller
    control.receive_feedback(sample)
    control.set_estop(True)
    waiting = replace(sample, tick=351, state=replace(sample.state, t_now=7.02))
    assert not control.receive_feedback(waiting)
    assert sent.call_count == 1
    assert control.get_tick_count() == 1
    control.set_estop(False)
    assert control.receive_feedback(waiting)
    assert (sent.call_args.args[0].epoch, sent.call_args.args[0].tick) == (
        waiting.epoch,
        waiting.tick,
    )
    assert not control.receive_feedback(waiting)
    assert control.get_tick_count() == 2


def test_reset_while_stopped_retires_the_previous_epoch(controller):
    control, task, sent, sample = controller
    control.receive_feedback(sample)
    control.set_estop(True)
    reset = replace(sample, epoch="episode-b", tick=0, state=replace(sample.state, t_now=0.0))
    assert not control.receive_feedback(reset)
    control.set_estop(False)
    assert control.receive_feedback(reset)
    assert not control.receive_feedback(sample)
    assert control.get_tick_count() == 2
    assert sent.call_args.args[0].epoch == "episode-b"

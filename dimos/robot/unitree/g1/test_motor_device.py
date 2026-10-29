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

from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace

import pytest

from dimos.control.contract.convert import motor_command_to_values
from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.motor_device import (
    G1_JOINT_NAMES,
    G1LowStateSnapshot,
    G1MotorDevice,
)
from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnectionConfig


@pytest.fixture
def rig(mocker):
    feedback = mocker.Mock()
    config = G1WholeBodyConnectionConfig(soft_start_seconds=0.0)
    device = G1MotorDevice(config, feedback)
    frames = []
    publisher = mocker.Mock()
    publisher.Write.side_effect = lambda frame: frames.append(deepcopy(frame))
    subscriber = mocker.Mock()
    device._publisher = publisher
    device._subscriber = subscriber
    device._low_cmd = SimpleNamespace(
        mode_machine=5,
        crc=0,
        motor_cmd=[
            SimpleNamespace(mode=1, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0) for _ in range(35)
        ],
    )
    device._crc = mocker.Mock()
    device._crc.Crc.return_value = 123
    device._mode_machine = 5
    try:
        yield SimpleNamespace(
            device=device,
            config=config,
            feedback=feedback,
            publisher=publisher,
            subscriber=subscriber,
            frames=frames,
        )
    finally:
        device.stop()


def _command():
    n = len(G1_JOINT_NAMES)
    return motor_command_to_values(
        MotorCommandArray(
            q=[1.0] * n,
            dq=[0.5] * n,
            kp=[100.0] * n,
            kd=[5.0] * n,
            tau=[8.0] * n,
        ),
        G1_JOINT_NAMES,
    )


def test_soft_start_is_damping_first(rig, mocker):
    rig.config.soft_start_seconds = 3.0
    mocker.patch("dimos.robot.unitree.g1.motor_device.time.perf_counter", return_value=10.0)

    rig.device.write(_command())

    motor = rig.frames[0].motor_cmd[0]
    assert (motor.q, motor.dq, motor.kp, motor.kd, motor.tau) == (1.0, 0.5, 0.0, 5.0, 0.0)


def test_stiffness_and_feedforward_ramp_to_full(rig, mocker):
    rig.config.soft_start_seconds = 2.0
    mocker.patch("dimos.robot.unitree.g1.motor_device.time.perf_counter", side_effect=[10, 11, 12])

    for _ in range(3):
        rig.device.write(_command())

    assert [(f.motor_cmd[0].kp, f.motor_cmd[0].kd, f.motor_cmd[0].tau) for f in rig.frames] == [
        (0.0, 5.0, 0.0),
        (50.0, 5.0, 4.0),
        (100.0, 5.0, 8.0),
    ]


def test_disabled_soft_start_passes_the_pd_command_through(rig):
    rig.device.write(_command())

    motor = rig.frames[0].motor_cmd[0]
    assert (motor.q, motor.dq, motor.kp, motor.kd, motor.tau) == (1.0, 0.5, 100.0, 5.0, 8.0)
    assert rig.frames[0].crc == 123


def test_halt_sends_damping_immediately_and_restarts_soft_start(rig, mocker):
    rig.config.soft_start_seconds = 2.0
    rig.config.halt_damping = 7.0
    mocker.patch("dimos.robot.unitree.g1.motor_device.time.perf_counter", side_effect=[10, 12, 13])
    rig.device.write(_command())
    rig.device.write(_command())

    rig.device.halt()
    rig.device.write(_command())

    halted = rig.frames[-2].motor_cmd[:29]
    assert [(m.q, m.dq, m.kp, m.kd, m.tau) for m in halted] == [(0.0, 0.0, 0.0, 7.0, 0.0)] * 29
    assert rig.frames[-1].motor_cmd[0].kp == 0.0
    rig.publisher.Close.assert_not_called()


def test_stop_disables_all_motor_slots_and_closes_dds_endpoints(rig):
    rig.device.write(_command())

    rig.device.stop()

    assert [(m.mode, m.q, m.dq, m.kp, m.kd, m.tau) for m in rig.frames[-1].motor_cmd] == [
        (0, POS_STOP, VEL_STOP, 0, 0, 0),
    ] * 35
    rig.publisher.Close.assert_called_once_with()
    rig.subscriber.Close.assert_called_once_with()


def test_first_firmware_sample_supplies_mode_machine_for_commands(rig):
    rig.device._verify_mode_machine_once(SimpleNamespace(mode_machine=7))
    rig.device._verify_mode_machine_once(SimpleNamespace(mode_machine=9))

    rig.device.write(_command())

    assert rig.frames[0].mode_machine == 7


def test_motor_and_imu_feedback_share_timestamp_and_preserve_axis_order(rig):
    sample = G1LowStateSnapshot(
        positions=[0.1] * 29,
        velocities=[0.2] * 29,
        efforts=[0.3] * 29,
        quaternion=(1.0, 0.0, 0.0, 0.0),
        gyroscope=(1.0, 2.0, 3.0),
        accelerometer=(4.0, 5.0, 6.0),
    )

    rig.device._publish_motor_state_and_imu(now=42.0, frame_id="pelvis", sample=sample)

    state_port, state = rig.feedback.call_args_list[0].args
    imu_port, imu = rig.feedback.call_args_list[1].args
    assert (state_port, imu_port, state.ts, imu.ts) == ("motor_states", "imu", 42.0, 42.0)
    assert state.name == G1_JOINT_NAMES
    assert state.position == [0.1] * 29
    assert (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) == (
        0,
        0,
        0,
        1,
    )
    assert (imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z) == (1, 2, 3)
    assert (imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z) == (
        4,
        5,
        6,
    )

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

from collections.abc import Iterator
from types import SimpleNamespace

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import MotorCommandArray
from dimos_generated.std_msgs.msg import Header
from pydantic import ValidationError
import pytest

from dimos.robot.unitree.g1.wholebody_connection import (
    _NUM_MOTOR_SLOTS,
    _NUM_MOTORS,
    G1LowStateSnapshot,
    G1WholeBodyConnection,
    G1WholeBodyConnectionConfig,
)


@pytest.fixture
def connection() -> Iterator[G1WholeBodyConnection]:
    connection = G1WholeBodyConnection()
    try:
        yield connection
    finally:
        connection._publisher = None  # keep stop() away from the fake DDS state
        connection._low_cmd = None
        connection.stop()


class _FakePublisher:
    def __init__(self):
        self.frames = []

    def Write(self, low_cmd):
        self.frames.append(
            [(m.q, m.dq, m.kp, m.kd, m.tau) for m in low_cmd.motor_cmd[:_NUM_MOTORS]]
        )


def _wire(connection, soft_start_seconds):
    """Give the connection just enough fake DDS state to accept commands."""
    connection.config.soft_start_seconds = soft_start_seconds
    connection._publisher = _FakePublisher()
    connection._low_cmd = SimpleNamespace(
        mode_machine=0,
        crc=0,
        motor_cmd=[
            SimpleNamespace(mode=1, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0)
            for _ in range(_NUM_MOTOR_SLOTS)
        ],
    )
    connection._crc = SimpleNamespace(Crc=lambda _cmd: 0)
    connection._mode_machine = 5
    return connection._publisher


def _command():
    return MotorCommandArray(
        q=[1.0] * _NUM_MOTORS,
        dq=[0.0] * _NUM_MOTORS,
        kp=[100.0] * _NUM_MOTORS,
        kd=[5.0] * _NUM_MOTORS,
        tau=[8.0] * _NUM_MOTORS,
    )


def test_soft_start_is_damping_first(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=1000.0)

    connection._on_motor_command(_command())

    q, dq, kp, kd, tau = publisher.frames[0][0]
    # First frame: target and damping pass through, stiffness and tau do not —
    # this is what keeps taking control from slamming the robot.
    assert q == 1.0
    assert kd == 5.0
    assert kp < 1.0
    assert abs(tau) < 0.1


def test_stiffness_ramps_to_full(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=0.05)

    connection._on_motor_command(_command())
    # Rewind the clock instead of sleeping through the window.
    connection._soft_start_t0 -= 1.0
    connection._on_motor_command(_command())

    _q, _dq, kp, kd, tau = publisher.frames[-1][0]
    assert kp == 100.0
    assert kd == 5.0
    assert tau == 8.0


def test_soft_start_disabled_passes_through(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=0.0)

    connection._on_motor_command(_command())

    _q, _dq, kp, _kd, tau = publisher.frames[0][0]
    assert kp == 100.0
    assert tau == 8.0


def test_wrong_joint_count_is_dropped(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=0.0)

    connection._on_motor_command(MotorCommandArray(q=[0.0] * 5))

    assert publisher.frames == []


@pytest.mark.parametrize("value", [float("inf"), float("-inf"), float("nan")])
def test_non_finite_soft_start_is_rejected(value):
    # inf satisfies a bare ge=0.0, and every finite elapsed time over inf is
    # zero, so the scale would pin at 0 forever: full damping, no stiffness,
    # and no way back to the commanded gains.
    with pytest.raises(ValidationError):
        G1WholeBodyConnectionConfig(soft_start_seconds=value)


def test_generated_feedback_has_one_exact_header_and_ros_quaternion(connection, monkeypatch):
    joints, imu = [], []
    monkeypatch.setattr(connection.motor_states, "publish", joints.append)
    monkeypatch.setattr(connection.imu, "publish", imu.append)
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="g1_pelvis")
    sample = G1LowStateSnapshot(
        positions=[0.1] * 29,
        velocities=[0.2] * 29,
        efforts=[0.3] * 29,
        quaternion=(0.8, 0.0, 0.0, 0.6),
        gyroscope=(1.0, 2.0, 3.0),
        accelerometer=(4.0, 5.0, 6.0),
    )
    connection._publish_motor_state_and_imu(header=header, sample=sample)
    assert joints[0].header == header == imu[0].header
    assert list(joints[0].position) == [0.1] * 29
    assert imu[0].orientation.w == 0.8
    assert imu[0].orientation.z == 0.6
    assert imu[0].angular_velocity.x == 1.0
    assert imu[0].linear_acceleration.z == 6.0
    for message in [joints[0], imu[0]]:
        assert type(message).decode(message.encode()) == message
    header.stamp.nanosec = 0
    assert joints[0].header.stamp.nanosec == 123456789


@pytest.mark.parametrize("field", ["dq", "kp", "kd", "tau"])
def test_inconsistent_motor_array_is_dropped(connection, field):
    publisher = _wire(connection, soft_start_seconds=0.0)
    command = _command()
    setattr(command, field, [0.0])
    connection._on_motor_command(command)
    assert publisher.frames == []

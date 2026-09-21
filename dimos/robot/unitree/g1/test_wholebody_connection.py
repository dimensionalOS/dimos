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

from pydantic import ValidationError
import pytest

from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.wholebody_connection import (
    _NUM_MOTOR_SLOTS,
    _NUM_MOTORS,
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

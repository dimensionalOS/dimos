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
import time
from types import SimpleNamespace

from pydantic import ValidationError
import pytest

from dimos.control.connection_source import ConnectionSource
from dimos.control.contract.keys import Interface
from dimos.hardware.manipulators.spec import ControlMode
from dimos.hardware.whole_body.spec import POS_STOP
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.config import G1_JOINT_NAMES
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
        self.modes = [motor.mode for motor in low_cmd.motor_cmd]
        self.frames.append(
            [(m.q, m.dq, m.kp, m.kd, m.tau) for m in low_cmd.motor_cmd[:_NUM_MOTORS]]
        )

    def Close(self):
        pass


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
    connection._low_state = SimpleNamespace()
    connection._state_received_at = time.monotonic()
    return connection._publisher


def _command():
    values = {f"{name}/position": 0.0 for name in G1_JOINT_NAMES}
    values[f"{G1_JOINT_NAMES[0]}/position"] = 1.0
    return values


def test_soft_start_is_damping_first(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=1000.0)

    connection.write(_command())

    q, dq, kp, kd, tau = publisher.frames[0][0]
    # First frame: target and damping pass through, stiffness does not —
    # this is what keeps taking control from slamming the robot.
    assert q == 1.0
    assert kd == 3.0
    assert kp < 1.0
    assert abs(tau) < 0.1


def test_stiffness_ramps_to_full(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=0.05)

    connection.write(_command())
    # Rewind the clock instead of sleeping through the window.
    connection._soft_start_t0 -= 1.0
    connection.write(_command())

    _q, _dq, kp, kd, tau = publisher.frames[-1][0]
    assert kp == 40.0
    assert kd == 3.0
    assert tau == 0.0


def test_soft_start_disabled_passes_through(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=0.0)

    connection.write(_command())

    _q, _dq, kp, _kd, tau = publisher.frames[0][0]
    assert kp == 40.0
    assert tau == 0.0


def test_incomplete_joint_positions_are_rejected(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=0.0)

    with pytest.raises(ValueError, match="complete 29-joint"):
        connection.write({f"{G1_JOINT_NAMES[0]}/position": 0.0})

    assert publisher.frames == []


def test_canonical_source_routes_model_order_and_keeps_limits(connection):
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection._connected = True
    connection._low_state = SimpleNamespace()
    connection._state_received_at = time.monotonic()
    source = ConnectionSource(connection.config, ["position_command"], ["joint_state"], 0.2)
    # A real model-valid pose, including asymmetric knee/ankle limits.
    pose = {
        name: max(limit.lo or 0.0, min(0.0, limit.hi or 0.0))
        for name in G1_JOINT_NAMES
        for limit in [source.position_limit(name)]
    }
    source.on_message(
        "joint_state",
        JointState(
            name=list(reversed(G1_JOINT_NAMES)),
            position=[pose[name] for name in reversed(G1_JOINT_NAMES)],
        ),
    )
    assert source.ready_for_control()
    values = source.command({name: (q, ControlMode.POSITION) for name, q in pose.items()})
    connection.command(values)
    assert [frame[0] for frame in publisher.frames[-1]] == [pose[n] for n in G1_JOINT_NAMES]
    assert all(frame[2:4] == (40.0, 3.0) for frame in publisher.frames[-1])
    assert all(
        interfaces == frozenset({Interface.POSITION})
        for interfaces in source.joint_command_interfaces.values()
    )
    before = len(publisher.frames)
    connection.command({f"{G1_JOINT_NAMES[0]}/position": POS_STOP})
    assert len(publisher.frames) == before
    assert connection.status().last_rejection
    connection.halt()
    assert all(
        q == POS_STOP and kp == 0.0 and kd == 3.0 and tau == 0.0
        for q, _, kp, kd, tau in publisher.frames[-1]
    )
    assert all(m.mode == 1 for m in connection._low_cmd.motor_cmd[:29])
    connection._state_received_at -= 1.0
    with pytest.raises(ValueError, match="fresh measured"):
        connection.write(values)


def test_canonical_port_deadman_damping_and_final_firmware_stop(connection, monkeypatch):
    from dimos.control.test_coordinator_connections import LocalTransport

    publisher = _wire(connection, soft_start_seconds=0.0)
    connection.config.deadman_timeout_s = 0.03
    monkeypatch.setattr(connection, "connect", lambda: None)
    commands = LocalTransport()
    connection.set_transport("position_command", commands)
    connection.start()
    connection._low_state = SimpleNamespace()
    connection._state_received_at = time.monotonic()
    source = ConnectionSource(connection.config, ["position_command"], ["joint_state"], 0.2)
    pose = [max(source.position_limit(n).lo or 0.0, 0.0) for n in G1_JOINT_NAMES]
    command = JointState(name=G1_JOINT_NAMES, position=pose)
    commands.publish(command)
    assert len(publisher.frames) == 1
    deadline = time.monotonic() + 2.0
    while not connection.status().deadman_fired and time.monotonic() < deadline:
        time.sleep(0.005)
    assert connection.status().deadman_fired
    assert all(mode == 1 for mode in publisher.modes)
    assert all(q == POS_STOP and kp == 0.0 for q, _, kp, _, _ in publisher.frames[-1])
    connection.stop()
    assert all(mode == 0 for mode in publisher.modes)
    assert all(
        q == POS_STOP and kp == kd == tau == 0.0 for q, _, kp, kd, tau in publisher.frames[-1]
    )
    count = len(publisher.frames)
    commands.publish(command)
    assert len(publisher.frames) == count


def test_lowstate_snapshot_motor_order_and_imu_quaternion(connection, mocker):
    sample = SimpleNamespace(
        mode_machine=7,
        motor_state=[
            SimpleNamespace(q=float(i), dq=float(i + 1), tau_est=float(i + 2)) for i in range(29)
        ],
        imu_state=SimpleNamespace(
            quaternion=[1.0, 0.0, 0.0, 0.0],
            gyroscope=[1.0, 2.0, 3.0],
            accelerometer=[4.0, 5.0, 6.0],
        ),
    )
    connection._subscriber = SimpleNamespace(Read=lambda: sample)
    joint_state = mocker.patch.object(connection.joint_state, "publish")
    imu = mocker.patch.object(connection.imu, "publish")
    assert connection._drain_low_state()
    snapshot = connection._snapshot_motor_imu()
    connection._publish_motor_state_and_imu(10.0, "g1_pelvis", snapshot)
    assert joint_state.call_args.args[0].name == G1_JOINT_NAMES
    assert joint_state.call_args.args[0].position == list(range(29))
    assert imu.call_args.args[0].orientation.w == 1.0
    assert imu.call_args.args[0].orientation.x == 0.0
    assert list(imu.call_args.args[0].angular_velocity) == [1.0, 2.0, 3.0]
    assert connection._mode_machine == 7
    connection._subscriber = None


def test_lowstate_is_not_republished_as_fresh(connection, mocker):
    connection._subscriber = SimpleNamespace(Read=lambda: None)
    connection._low_state = SimpleNamespace()
    assert not connection._drain_low_state()
    assert connection._state_received_at == float("-inf")
    connection._subscriber = None


@pytest.mark.parametrize("value", [float("inf"), float("-inf"), float("nan")])
def test_non_finite_soft_start_is_rejected(value):
    # inf satisfies a bare ge=0.0, and every finite elapsed time over inf is
    # zero, so the scale would pin at 0 forever: full damping, no stiffness,
    # and no way back to the commanded gains.
    with pytest.raises(ValidationError):
        G1WholeBodyConnectionConfig(soft_start_seconds=value)

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

from dimos.control.tasks.g1_sonic_wbc_task.sonic_safety import (
    COMMAND_TIMEOUT_SECONDS,
    DAMPING_KD,
)
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.std_msgs.String import String
from dimos.robot.unitree.g1.wholebody_connection import (
    _NUM_MOTOR_SLOTS,
    _NUM_MOTORS,
    G1WholeBodyConnection,
    G1WholeBodyConnectionConfig,
)


@pytest.fixture
def clock(mocker):
    clock = SimpleNamespace(now=10.0)
    mocker.patch(
        "dimos.robot.unitree.g1.wholebody_connection.time.perf_counter",
        side_effect=lambda: clock.now,
    )
    return clock


@pytest.fixture
def connection(clock) -> Iterator[G1WholeBodyConnection]:
    connection = G1WholeBodyConnection(command_timeout_seconds=COMMAND_TIMEOUT_SECONDS)
    try:
        yield connection
    finally:
        connection._publisher = None  # keep stop() away from the fake DDS state
        connection._low_cmd = None
        connection._subscriber = None
        connection.stop()


class _FakePublisher:
    def __init__(self):
        self.frames = []
        self.modes = []

    def Write(self, low_cmd):
        self.modes.append([m.mode for m in low_cmd.motor_cmd[:_NUM_MOTORS]])
        self.frames.append(
            [(m.q, m.dq, m.kp, m.kd, m.tau) for m in low_cmd.motor_cmd[:_NUM_MOTORS]]
        )

    def Close(self):
        pass


def _feedback(connection, *, tick=1, motor=0, velocity=0.0):
    sample = SimpleNamespace(
        tick=tick,
        mode_machine=5,
        motor_state=[SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0) for _ in range(_NUM_MOTORS)],
        imu_state=SimpleNamespace(
            quaternion=[1.0, 0.0, 0.0, 0.0],
            gyroscope=[0.0, 0.0, 0.0],
            accelerometer=[0.0, 0.0, 0.0],
        ),
    )
    sample.motor_state[motor].dq = velocity
    connection._subscriber = SimpleNamespace(Read=lambda: sample, Close=lambda: None)
    connection._drain_low_state()


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
    connection._sport_mode_released = True
    _feedback(connection)
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
    connection._publish_latest_command(10.0)

    q, dq, kp, kd, tau = publisher.frames[0][0]
    # First frame: target and damping pass through, stiffness and tau do not —
    # this is what keeps taking control from slamming the robot.
    assert q == 1.0
    assert kd == 5.0
    assert kp < 1.0
    assert abs(tau) < 0.1


def test_legacy_controller_can_hold_without_refreshing_commands(connection, clock):
    connection.config.command_timeout_seconds = (
        G1WholeBodyConnectionConfig().command_timeout_seconds
    )
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection._on_motor_command(_command())

    clock.now += 10.0
    _feedback(connection, tick=2)
    connection._publish_latest_command(clock.now)

    assert connection.command_stream_status()["fault_reason"] is None
    assert publisher.frames[-1][0] == (1.0, 0.0, 100.0, 5.0, 8.0)


def test_stiffness_ramps_to_full(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=0.05)

    connection._on_motor_command(_command())
    connection._publish_latest_command(10.0)
    # Rewind the clock instead of sleeping through the window.
    connection._soft_start_t0 -= 1.0
    connection._publish_latest_command(10.0)

    _q, _dq, kp, kd, tau = publisher.frames[-1][0]
    assert kp == 100.0
    assert kd == 5.0
    assert tau == 8.0


def test_soft_start_disabled_passes_through(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=0.0)

    connection._on_motor_command(_command())
    connection._publish_latest_command(10.0)

    _q, _dq, kp, _kd, tau = publisher.frames[0][0]
    assert kp == 100.0
    assert tau == 8.0


def test_wrong_joint_count_is_dropped(connection: G1WholeBodyConnection):
    publisher = _wire(connection, soft_start_seconds=0.0)

    connection._on_motor_command(MotorCommandArray(q=[0.0] * 5))

    assert publisher.frames == []


@pytest.mark.parametrize("release_sport_mode, expected_releases", [(True, 1), (False, 0)])
def test_sport_mode_handoff_waits_for_first_complete_command(
    connection: G1WholeBodyConnection,
    mocker,
    release_sport_mode,
    expected_releases,
):
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection.config.release_sport_mode = release_sport_mode
    release = mocker.patch.object(connection, "_release_sport_mode")
    connection._sport_mode_released = False

    connection._on_motor_command(MotorCommandArray(q=[0.0] * 5))
    release.assert_not_called()

    connection._on_motor_command(_command())
    connection._on_motor_command(_command())
    connection._publish_latest_command(10.0)
    connection._publish_latest_command(10.002)

    assert release.call_args_list == [mocker.call()] * expected_releases
    assert len(publisher.frames) == 2


def test_latest_policy_target_is_republished_on_each_dds_tick(
    connection: G1WholeBodyConnection,
) -> None:
    publisher = _wire(connection, soft_start_seconds=0.0)

    connection._on_motor_command(_command())

    assert publisher.frames == []
    assert connection._publish_latest_command(10.0)
    assert connection._publish_latest_command(10.002)
    assert len(publisher.frames) == 2
    assert publisher.frames[0] == publisher.frames[1]


@pytest.mark.parametrize("value", [float("inf"), float("-inf"), float("nan")])
def test_non_finite_soft_start_is_rejected(value):
    # inf satisfies a bare ge=0.0, and every finite elapsed time over inf is
    # zero, so the scale would pin at 0 forever: full damping, no stiffness,
    # and no way back to the commanded gains.
    with pytest.raises(ValidationError):
        G1WholeBodyConnectionConfig(soft_start_seconds=value)


@pytest.mark.parametrize("velocity", [-36.0, 36.0])
@pytest.mark.parametrize("motor", [0, 14, 28])
def test_overspeed_latches_damping_despite_fresh_commands(connection, velocity, motor):
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection._on_motor_command(_command())
    connection._publish_latest_command(10.0)

    _feedback(connection, tick=2, motor=motor, velocity=velocity)
    connection._publish_latest_command(10.002)
    _feedback(connection, tick=3)
    connection._on_motor_command(_command())
    connection._publish_latest_command(10.004)

    assert publisher.frames[-2:] == [[(0.0, 0.0, 0.0, DAMPING_KD, 0.0)] * 29] * 2
    assert publisher.modes[-1] == [1] * 29
    assert "joint overspeed" in connection.command_stream_status()["fault_reason"]
    with pytest.raises(RuntimeError, match="restart"):
        connection.set_estop(False)


@pytest.mark.parametrize("velocity", [-35.0, 35.0])
def test_joint_velocity_at_limit_does_not_trip(connection, velocity):
    publisher = _wire(connection, soft_start_seconds=0.0)
    _feedback(connection, tick=2, velocity=velocity)
    connection._on_motor_command(_command())
    connection._publish_latest_command(10.0)
    assert publisher.frames[-1][0][2] == 100.0
    assert connection.command_stream_status()["fault_reason"] is None


def test_feedback_timeout_overrides_fresh_policy_commands(connection, clock):
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection._on_motor_command(_command())
    connection._publish_latest_command(clock.now)
    clock.now += 0.101
    connection._on_motor_command(_command())

    connection._publish_latest_command(clock.now)

    assert publisher.frames[-1] == [(0.0, 0.0, 0.0, DAMPING_KD, 0.0)] * 29
    assert connection.command_stream_status()["fault_reason"] == "robot feedback timeout"


def test_command_timeout_overrides_fresh_robot_feedback(connection, clock):
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection._on_motor_command(_command())
    connection._publish_latest_command(clock.now)
    clock.now += 0.101
    _feedback(connection, tick=2)

    connection._publish_latest_command(clock.now)

    assert publisher.frames[-1] == [(0.0, 0.0, 0.0, DAMPING_KD, 0.0)] * 29
    assert connection.command_stream_status()["fault_reason"] == "policy command timeout"


def test_repeated_firmware_tick_does_not_refresh_feedback_age(connection, clock):
    _wire(connection, soft_start_seconds=0.0)
    captured_at = connection._feedback_wall_time
    clock.now += 0.101
    _feedback(connection, tick=1)

    assert connection.command_stream_status()["feedback_age_ms"] == pytest.approx(101.0)
    assert connection._feedback_wall_time == captured_at


def test_stop_before_handoff_does_not_take_control(connection, mocker):
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection._sport_mode_released = False
    release = mocker.patch.object(connection, "_release_sport_mode")

    connection.set_estop(True)
    connection._on_motor_command(_command())
    connection._publish_latest_command(10.0)

    release.assert_not_called()
    assert publisher.frames == []
    assert connection.command_stream_status()["fault_reason"] == "operator stop"


def test_stop_during_handoff_cannot_be_overwritten_by_prepared_command(connection, mocker):
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection._sport_mode_released = False
    mocker.patch.object(
        connection, "_release_sport_mode", side_effect=lambda: connection.set_estop(True)
    )

    connection._on_motor_command(_command())
    connection._publish_latest_command(10.0)

    assert publisher.frames == [[(0.0, 0.0, 0.0, DAMPING_KD, 0.0)] * 29]


@pytest.mark.parametrize("source", ["rpc", "policy"])
def test_all_stop_sources_use_same_latched_damping(connection, source):
    publisher = _wire(connection, soft_start_seconds=1000.0)
    connection._on_motor_command(_command())
    operations = {
        "rpc": lambda: connection.set_estop(True),
        "policy": lambda: connection._on_sonic_fault(String("inference failed")),
    }

    operations[source]()
    connection._on_motor_command(_command())
    connection._publish_latest_command(10.0)

    assert publisher.frames == [[(0.0, 0.0, 0.0, DAMPING_KD, 0.0)] * 29]


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_command_trips_damping(connection, value):
    publisher = _wire(connection, soft_start_seconds=0.0)
    command = _command()
    command.q[0] = value

    connection._on_motor_command(command)
    connection._publish_latest_command(10.0)

    assert publisher.frames == [[(0.0, 0.0, 0.0, DAMPING_KD, 0.0)] * 29]
    assert connection.command_stream_status()["fault_reason"] == "non-finite motor command"


def test_shutdown_preserves_latched_damping(connection):
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection.set_estop(True)

    connection.stop()

    assert publisher.frames[-1] == [(0.0, 0.0, 0.0, DAMPING_KD, 0.0)] * 29
    assert publisher.modes[-1] == [1] * 29


@pytest.mark.timeout(2)
def test_writer_recovers_to_damping_even_when_fault_reporting_fails(connection, mocker):
    publisher = _wire(connection, soft_start_seconds=0.0)
    connection._on_motor_command(_command())
    write_frame = publisher.Write
    calls = []

    def write(low_cmd):
        calls.append(low_cmd)
        if len(calls) == 1:
            raise RuntimeError("DDS publication failed")
        write_frame(low_cmd)
        connection._stop_event.set()

    mocker.patch.object(publisher, "Write", side_effect=write)
    mocker.patch.object(
        connection.g1_fault, "publish", side_effect=RuntimeError("status unavailable")
    )

    connection._command_loop()

    assert publisher.frames == [[(0.0, 0.0, 0.0, DAMPING_KD, 0.0)] * 29]
    assert connection.command_stream_status()["fault_reason"] == "motor command publication failed"


@pytest.mark.parametrize(
    "field",
    [
        "feedback_timeout_seconds",
        "command_timeout_seconds",
        "joint_velocity_limit",
        "publish_rate_hz",
    ],
)
@pytest.mark.parametrize("value", [0.0, float("nan"), float("inf")])
def test_invalid_safety_configuration_is_rejected(field, value):
    with pytest.raises(ValidationError):
        G1WholeBodyConnectionConfig(**{field: value})

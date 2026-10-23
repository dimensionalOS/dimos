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

from collections.abc import Callable, Iterator
from dataclasses import replace
import math
import struct
import time
from typing import Any
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.hardware_interface import ConnectedHardware
from dimos.hardware.manipulators.registry import adapter_registry
from dimos.hardware.manipulators.seeedstudio import adapter as adapter_module
from dimos.hardware.manipulators.seeedstudio.adapter import (
    ARM_VELOCITY_MAX,
    GRIPPER_CLOSED_RAD,
    GRIPPER_MAX_OPENING_M,
    GRIPPER_OPEN_RAD,
    GRIPPER_RAD_PER_M,
    GRIPPER_STALL_EFFORT_NM,
    GRIPPER_VELOCITY_RAD_S,
    SeeedStudioAdapter,
    gripper_motor_to_opening,
    gripper_opening_to_motor,
)
from dimos.hardware.manipulators.seeedstudio.protocol import (
    DISABLE,
    ENABLE,
    Feedback,
    MotorParameters,
)
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter

ARM = [0.1, -0.5, -0.6, 0.2, 0.3, 0.4]
OPENING = 0.02
MOTOR_POSITIONS = [*ARM, gripper_opening_to_motor(OPENING)]
POSITIONS = [*ARM, OPENING]


def parameters(mid: int, fid: int) -> MotorParameters:
    return MotorParameters(mid, fid, 2, 12.5, 10, 28, 0)


def feedback(params: MotorParameters) -> Feedback:
    return Feedback(
        MOTOR_POSITIONS[params.motor_id - 1],
        params.motor_id * 0.01,
        params.motor_id * 0.1,
        0,
        25,
        26,
        time.monotonic(),
    )


@pytest.fixture
def bus(mocker: MockerFixture) -> MagicMock:
    transport: MagicMock = mocker.patch(
        "dimos.hardware.manipulators.seeedstudio.adapter.DmSerialTransport"
    ).return_value
    transport.is_open.return_value = True
    transport.parameters.side_effect = parameters
    transport.feedback.side_effect = feedback
    transport.read_register.return_value = 2
    return transport


@pytest.fixture
def uncached(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make every read sweep the bus instead of reusing the 20 ms cache."""
    monkeypatch.setattr(adapter_module, "READ_CACHE_SECONDS", -1.0)


@pytest.fixture
def adapter(bus: MagicMock) -> Iterator[SeeedStudioAdapter]:
    adapter = SeeedStudioAdapter("test-only")
    assert adapter.connect()
    yield adapter
    adapter.disconnect()


def activate(adapter: SeeedStudioAdapter, bus: MagicMock, **overrides: Any) -> None:
    """Enable against a bus whose motors acknowledge ENABLE frames."""
    enabled: set[int] = set()

    def send(mid: int, payload: bytes, *_: float) -> None:
        if payload == ENABLE:
            enabled.add(mid)
        elif payload == DISABLE:
            enabled.discard(mid)

    def state(params: MotorParameters) -> Feedback:
        return replace(feedback(params), status=int(params.motor_id in enabled), **overrides)

    bus.send.side_effect = send
    bus.feedback.side_effect = state
    assert adapter.activate()
    bus.send.reset_mock()


def sent_targets(bus: MagicMock) -> list[tuple[float, float]]:
    return [struct.unpack("<ff", c.args[1]) for c in bus.send.call_args_list]


def test_registry_and_constructor_have_no_hardware_side_effects(bus: MagicMock) -> None:
    adapter = adapter_registry.create(
        "seeedstudio_b601_dm", address="test-only", dof=7, hardware_id="arm"
    )
    assert isinstance(adapter, ManipulatorAdapter)
    bus.open.assert_not_called()
    bus.send.assert_not_called()


def test_constructor_rejects_arm_without_gripper(bus: MagicMock) -> None:
    with pytest.raises(ValueError, match="dof=7"):
        SeeedStudioAdapter("test-only", dof=6)


def test_connect_reads_state_and_disconnect_writes_nothing(bus: MagicMock) -> None:
    adapter = SeeedStudioAdapter("test-only")
    try:
        assert adapter.connect()
        assert adapter.read_joint_positions() == pytest.approx(POSITIONS)
        velocities = adapter.read_joint_velocities()
        assert velocities[:6] == pytest.approx([i * 0.01 for i in range(1, 7)])
        assert velocities[6] == pytest.approx(-0.07 / GRIPPER_RAD_PER_M)
        assert adapter.read_joint_efforts() == pytest.approx([i * 0.1 for i in range(1, 8)])
        limits = adapter.get_limits()
        assert (limits.position_lower[-1], limits.position_upper[-1]) == (
            0.0,
            GRIPPER_MAX_OPENING_M,
        )
    finally:
        adapter.disconnect()
    bus.send.assert_not_called()
    bus.close.assert_called_once()


def test_partial_connection_closes_port(bus: MagicMock) -> None:
    bus.parameters.side_effect = [parameters(1, 17), TimeoutError("motor 2 missing")]
    adapter = SeeedStudioAdapter("test-only")
    assert not adapter.connect()
    assert not adapter.is_connected()
    assert "motor 2" in adapter.read_error()[1]
    bus.close.assert_called_once()
    bus.send.assert_not_called()


@pytest.mark.parametrize(
    "opening,motor",
    [(0.0, GRIPPER_CLOSED_RAD), (GRIPPER_MAX_OPENING_M, GRIPPER_OPEN_RAD)],
)
def test_gripper_opening_maps_linearly_to_motor_travel(opening: float, motor: float) -> None:
    assert gripper_opening_to_motor(opening) == pytest.approx(motor)
    assert gripper_motor_to_opening(motor) == pytest.approx(opening)


def test_gripper_conversion_clamps_to_travel() -> None:
    assert gripper_opening_to_motor(1.0) == gripper_opening_to_motor(GRIPPER_MAX_OPENING_M)
    assert gripper_opening_to_motor(-1.0) == GRIPPER_CLOSED_RAD
    # The hand-measured closed stop sits past the 0 m reference.
    assert gripper_motor_to_opening(-0.15) == 0.0
    assert gripper_motor_to_opening(-5.93) == GRIPPER_MAX_OPENING_M


def test_activation_seeds_measured_pose_then_enables(bus: MagicMock) -> None:
    adapter = SeeedStudioAdapter("test-only")
    assert adapter.connect()
    enabled: set[int] = set()

    def send(mid: int, payload: bytes, *_: float) -> None:
        if payload == ENABLE:
            enabled.add(mid)

    bus.send.side_effect = send
    bus.feedback.side_effect = lambda p: replace(feedback(p), status=int(p.motor_id in enabled))
    assert adapter.activate()
    calls = bus.send.call_args_list
    assert [c.args[0] for c in calls[:7]] == list(range(0x101, 0x108))
    assert [struct.unpack("<ff", c.args[1])[0] for c in calls[:7]] == pytest.approx(MOTOR_POSITIONS)
    assert [c.args[:2] for c in calls[7:14]] == [(i, ENABLE) for i in range(1, 8)]
    assert adapter.read_enabled()


def test_wrong_mode_does_not_enable(adapter: SeeedStudioAdapter, bus: MagicMock) -> None:
    bus.read_register.return_value = 1
    assert not adapter.activate()
    assert "POS_VEL" in adapter.read_error()[1]
    bus.send.assert_not_called()


def test_joint2_rest_reading_is_within_limits(adapter: SeeedStudioAdapter, bus: MagicMock) -> None:
    rest = [0.1, 0.0055, *MOTOR_POSITIONS[2:]]
    enabled: set[int] = set()

    def send(mid: int, payload: bytes, *_: float) -> None:
        if payload == ENABLE:
            enabled.add(mid)

    bus.send.side_effect = send
    bus.feedback.side_effect = lambda p: replace(
        feedback(p), position=rest[p.motor_id - 1], status=int(p.motor_id in enabled)
    )
    assert adapter.activate()


def test_start_pose_outside_limits_does_not_enable(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    bus.feedback.side_effect = lambda p: replace(
        feedback(p), position=0.05 if p.motor_id == 2 else MOTOR_POSITIONS[p.motor_id - 1]
    )
    assert not adapter.activate()
    assert "joint2=0.050" in adapter.read_error()[1]
    bus.send.assert_not_called()


def test_unacknowledged_enable_rolls_back_all_motors(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    # Firmware never acknowledges enabling; serial-write success is not sufficient.
    assert not adapter.activate()
    assert [c.args for c in bus.send.call_args_list[-7:]] == [(i, DISABLE) for i in range(1, 8)]
    assert not adapter.write_joint_positions(POSITIONS)


@pytest.mark.parametrize(
    "positions,velocity",
    [
        (ARM, 1.0),
        ([math.nan] * 7, 1.0),
        ([math.inf] * 7, 1.0),
        ([3.0, *POSITIONS[1:]], 1.0),
        ([0.1, 0.05, *POSITIONS[2:]], 1.0),
        (POSITIONS, 0.0),
        (POSITIONS, -0.1),
        (POSITIONS, math.nan),
        (POSITIONS, 1.1),
    ],
)
def test_invalid_commands_send_nothing(
    adapter: SeeedStudioAdapter, bus: MagicMock, positions: list[float], velocity: float
) -> None:
    activate(adapter, bus)
    assert not adapter.write_joint_positions(positions, velocity)
    bus.send.assert_not_called()


def test_commands_convert_gripper_and_scale_velocity(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    activate(adapter, bus)
    assert adapter.write_joint_positions([*ARM, 0.05], velocity=0.5)
    assert [c.args[0] for c in bus.send.call_args_list] == list(range(0x101, 0x108))
    targets = sent_targets(bus)
    assert [t[0] for t in targets] == pytest.approx([*ARM, gripper_opening_to_motor(0.05)])
    assert [t[1] for t in targets] == pytest.approx(
        [0.5 * ARM_VELOCITY_MAX] * 6 + [GRIPPER_VELOCITY_RAD_S]
    )


def test_gripper_opening_beyond_travel_is_clamped(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    activate(adapter, bus)
    assert adapter.write_joint_positions([*ARM, 0.5])
    assert sent_targets(bus)[-1][0] == pytest.approx(
        gripper_opening_to_motor(GRIPPER_MAX_OPENING_M)
    )


def test_stalled_close_holds_until_a_more_open_target(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    activate(adapter, bus, effort=GRIPPER_STALL_EFFORT_NM + 0.5)
    measured = MOTOR_POSITIONS[6]
    assert adapter.write_joint_positions([*ARM, 0.0])
    assert adapter.write_joint_positions([*ARM, 0.0])
    assert [t[0] for t in sent_targets(bus)[6::7]] == pytest.approx([measured, measured])

    bus.send.reset_mock()
    assert adapter.write_joint_positions([*ARM, 0.04])
    assert sent_targets(bus)[-1][0] == pytest.approx(gripper_opening_to_motor(0.04))


def test_close_without_resistance_is_not_held(adapter: SeeedStudioAdapter, bus: MagicMock) -> None:
    activate(adapter, bus)
    assert adapter.write_joint_positions([*ARM, 0.0])
    assert sent_targets(bus)[-1][0] == pytest.approx(GRIPPER_CLOSED_RAD)


def assert_held(adapter: SeeedStudioAdapter, bus: MagicMock) -> None:
    """Commands are refused and no motor was disabled."""
    assert not any(c.args[1] == DISABLE for c in bus.send.call_args_list)
    bus.send.reset_mock()
    assert not adapter.write_joint_positions(POSITIONS)
    assert not adapter.read_enabled()
    bus.send.assert_not_called()


@pytest.mark.usefixtures("uncached")
def test_stale_feedback_holds_instead_of_moving(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    activate(adapter, bus)
    bus.feedback.side_effect = lambda p: replace(feedback(p), status=1, received_at=0)
    assert not adapter.write_joint_positions(POSITIONS)
    assert "expired" in adapter.read_error()[1]
    assert_held(adapter, bus)


@pytest.mark.usefixtures("uncached")
def test_read_timeout_while_enabled_holds(adapter: SeeedStudioAdapter, bus: MagicMock) -> None:
    activate(adapter, bus)
    bus.feedback.side_effect = TimeoutError("missing motor 2")
    with pytest.raises(TimeoutError, match="motor 2"):
        adapter.read_joint_positions()
    assert bus.feedback.call_count >= 3
    assert_held(adapter, bus)


def test_failed_write_holds(adapter: SeeedStudioAdapter, bus: MagicMock) -> None:
    activate(adapter, bus)
    bus.send.side_effect = [None, OSError("write failed")]
    assert not adapter.write_joint_positions(POSITIONS)
    assert [c.args[0] for c in bus.send.call_args_list] == [0x101, 0x102]
    bus.send.side_effect = None
    assert_held(adapter, bus)


def test_lost_port_holds(adapter: SeeedStudioAdapter, bus: MagicMock) -> None:
    activate(adapter, bus)
    bus.is_open.return_value = False
    assert not adapter.write_joint_positions(POSITIONS)
    assert "connection lost" in adapter.read_error()[1]
    bus.send.assert_not_called()


@pytest.mark.usefixtures("uncached")
def test_held_arm_disables_only_on_explicit_deactivate(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    activate(adapter, bus)
    enabled_state: Callable[[MotorParameters], Feedback] = bus.feedback.side_effect
    bus.feedback.side_effect = TimeoutError("missing motor 2")
    with pytest.raises(TimeoutError):
        adapter.read_joint_positions()
    bus.feedback.side_effect = enabled_state
    assert not adapter.activate()
    assert adapter.deactivate()
    assert [c.args for c in bus.send.call_args_list] == [(i, DISABLE) for i in range(1, 8)]


@pytest.mark.usefixtures("uncached")
def test_single_missed_reply_is_retried_without_holding(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    activate(adapter, bus)
    enabled_state: Callable[[MotorParameters], Feedback] = bus.feedback.side_effect
    missed = iter([TimeoutError("dropped reply")])

    def flaky(params: MotorParameters) -> Feedback:
        if params.motor_id == 6 and (error := next(missed, None)) is not None:
            raise error
        return enabled_state(params)

    bus.feedback.side_effect = flaky
    assert adapter.write_joint_positions(POSITIONS)
    assert adapter.read_enabled()
    assert adapter.read_error() == (0, "")


@pytest.mark.usefixtures("uncached")
def test_fault_disables_all_and_blocks_later_commands(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    activate(adapter, bus)
    bus.feedback.side_effect = lambda p: replace(feedback(p), status=10)
    assert not adapter.write_joint_positions(POSITIONS)
    assert [c.args for c in bus.send.call_args_list] == [(i, DISABLE) for i in range(1, 8)]
    assert "over_current" in adapter.read_error()[1]
    assert not adapter.write_joint_positions(POSITIONS)


def test_disable_continues_after_one_write_fails(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    activate(adapter, bus)
    bus.send.side_effect = [OSError("lost USB"), *([None] * 6)]
    bus.feedback.side_effect = feedback
    assert not adapter.deactivate()
    assert [c.args[0] for c in bus.send.call_args_list] == list(range(1, 8))


def test_optional_unsupported_features(adapter: SeeedStudioAdapter, bus: MagicMock) -> None:
    assert not adapter.set_control_mode(ControlMode.TORQUE)
    assert adapter.set_control_mode(ControlMode.SERVO_POSITION)
    assert not adapter.write_joint_velocities([0.0] * 7)
    assert not adapter.write_clear_errors()
    assert adapter.read_cartesian_position() is None
    assert not adapter.write_cartesian_position({"x": 0.0})
    assert adapter.read_force_torque() is None
    bus.send.assert_not_called()


@pytest.mark.usefixtures("uncached")
def test_coordinator_joint_state_and_partial_gripper_command(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    component = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*(f"joint{i}" for i in range(1, 7)), "arm/gripper"],
        adapter_type="seeedstudio_b601_dm",
        address="test-only",
    )
    hardware = ConnectedHardware(adapter, component)
    state = hardware.read_state()
    assert state["arm/gripper"].position == pytest.approx(OPENING)
    activate(adapter, bus)
    assert hardware.write_command({"arm/gripper": 0.04}, ControlMode.SERVO_POSITION)
    targets = [t[0] for t in sent_targets(bus)]
    assert targets == pytest.approx([*ARM, gripper_opening_to_motor(0.04)])


def test_never_disables_motors_another_process_enabled(bus: MagicMock) -> None:
    bus.feedback.side_effect = lambda p: replace(feedback(p), status=1)
    adapter = SeeedStudioAdapter("test-only")
    try:
        assert adapter.connect()
        assert not adapter.activate()
        assert adapter.deactivate()
        assert not adapter.write_stop()
    finally:
        adapter.disconnect()
    bus.send.assert_not_called()


def test_joint_reads_share_one_sweep(bus: MagicMock) -> None:
    adapter = SeeedStudioAdapter("test-only")
    try:
        assert adapter.connect()
        bus.feedback.reset_mock()
        adapter.read_joint_positions()
        adapter.read_joint_velocities()
        adapter.read_joint_efforts()
        assert bus.feedback.call_count == 0
    finally:
        adapter.disconnect()


def test_enable_waits_for_sensor_acknowledgement(
    adapter: SeeedStudioAdapter, bus: MagicMock
) -> None:
    enabled: set[int] = set()
    enable_reads = 0

    def send(mid: int, payload: bytes, *_: float) -> None:
        if payload == ENABLE:
            enabled.add(mid)

    def delayed_feedback(params: MotorParameters) -> Feedback:
        nonlocal enable_reads
        if params.motor_id in enabled:
            enable_reads += 1
        return replace(feedback(params), status=int(bool(enabled) and enable_reads > 7))

    bus.send.side_effect = send
    bus.feedback.side_effect = delayed_feedback
    assert adapter.activate()
    assert enable_reads == 14
    assert sum(call.args[1] == ENABLE for call in bus.send.call_args_list) == 7


def test_stop_holds_measured_pose(adapter: SeeedStudioAdapter, bus: MagicMock) -> None:
    activate(adapter, bus)
    assert adapter.write_stop()
    assert [t[0] for t in sent_targets(bus)] == pytest.approx(MOTOR_POSITIONS)


@pytest.mark.parametrize("reported", [-1.0, -1.0 + 2 * math.pi, -1.0 + 4 * math.pi])
def test_wrapped_gripper_angle_is_read_and_commanded_in_travel(
    bus: MagicMock, reported: float
) -> None:
    # After power-up the motor reports one turn modulo 2*pi; -1.0 rad is in travel.
    bus.feedback.side_effect = lambda p: replace(
        feedback(p), position=reported if p.motor_id == 7 else MOTOR_POSITIONS[p.motor_id - 1]
    )
    adapter = SeeedStudioAdapter("test-only")
    try:
        assert adapter.connect()
        assert adapter.read_joint_positions()[-1] == pytest.approx(gripper_motor_to_opening(-1.0))
        enabled: set[int] = set()

        def send(mid: int, payload: bytes, *_: float) -> None:
            if payload == ENABLE:
                enabled.add(mid)

        bus.send.side_effect = send
        bus.feedback.side_effect = lambda p: replace(
            feedback(p),
            position=reported if p.motor_id == 7 else MOTOR_POSITIONS[p.motor_id - 1],
            status=int(p.motor_id in enabled),
        )
        assert adapter.activate()
        # The seed holds the motor exactly where it reports itself.
        assert sent_targets(bus)[6][0] == pytest.approx(reported)
        bus.send.reset_mock()
        assert adapter.write_joint_positions([*ARM, 0.02])
        assert sent_targets(bus)[-1][0] == pytest.approx(
            gripper_opening_to_motor(0.02) + (reported - -1.0)
        )
    finally:
        adapter.disconnect()

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

from collections.abc import Generator
from dataclasses import dataclass, field
from queue import Empty, Queue
import threading

import pytest

from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_MAX_POSITION_M,
    PILLAR_MIN_POSITION_M,
    PillarFeedback,
    PillarPhase,
    PillarSerialDriver,
)


class ManualClock:
    def __init__(self, initial: float = 10.0) -> None:
        self.now = initial

    def __call__(self) -> float:
        return self.now


class FakeSerial:
    def __init__(self) -> None:
        self._incoming: Queue[bytes | None] = Queue()
        self._writes_changed = threading.Condition()
        self.writes: list[bytes] = []
        self.closed = False

    def read(self, size: int = 1) -> bytes:
        try:
            data = self._incoming.get(timeout=0.05)
        except Empty:
            return b""
        if data is None:
            return b""
        assert len(data) <= size
        return data

    def write(self, data: bytes) -> int:
        with self._writes_changed:
            self.writes.append(data)
            self._writes_changed.notify_all()
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True
        self._incoming.put(None)

    def wait_for_write(self, payload: bytes, timeout: float = 1.0) -> None:
        with self._writes_changed:
            assert self._writes_changed.wait_for(lambda: payload in self.writes, timeout=timeout)


@dataclass
class FakeSerialFactory:
    serial: FakeSerial
    calls: list[dict[str, object]] = field(default_factory=list)

    def __call__(
        self,
        *,
        port: str,
        baudrate: int,
        timeout: float,
        write_timeout: float,
    ) -> FakeSerial:
        self.calls.append(
            {
                "port": port,
                "baudrate": baudrate,
                "timeout": timeout,
                "write_timeout": write_timeout,
            }
        )
        return self.serial


@dataclass
class DriverHarness:
    driver: PillarSerialDriver
    serial: FakeSerial
    factory: FakeSerialFactory
    clock: ManualClock
    feedback: list[PillarFeedback]

    def set_status(self, *, position_mm: float, homed: bool, state: str = "IDLE") -> None:
        marker = (
            "HOMED, soft limits -500..-2 mm | start -50 mm | 0 = top switch"
            if homed
            else "NOT HOMED ('home'), nominal zero, soft limits would be "
            "-500..-2 mm | start -50 mm | 0 = top switch"
        )
        self.driver._feed_bytes(
            (
                f"pos {position_mm:.2f} mm ({round(position_mm * 80)} steps)\r\n"
                f"state {state} | brake ENGAGED (SSR off) | auto-brake on\r\n"
                f"{marker}\r\n"
            ).encode()
        )

    def clear_writes(self) -> None:
        with self.serial._writes_changed:
            self.serial.writes.clear()


@pytest.fixture
def harness() -> Generator[DriverHarness, None, None]:
    serial = FakeSerial()
    factory = FakeSerialFactory(serial)
    clock = ManualClock()
    feedback: list[PillarFeedback] = []
    driver = PillarSerialDriver(
        device_path="/dev/fake-pillar",
        boot_wait_s=0.0,
        status_poll_interval_s=60.0,
        feedback_stale_after_s=2.0,
        shutdown_stop_timeout_s=0.0,
        serial_factory=factory,
        feedback_callback=feedback.append,
        clock=clock,
    )
    driver.connect()
    serial.wait_for_write(b"p\n")
    serial.writes.clear()
    try:
        yield DriverHarness(driver, serial, factory, clock, feedback)
    finally:
        driver.disconnect()


def test_connect_uses_nano_serial_settings_and_disconnects_idle(harness: DriverHarness) -> None:
    assert harness.factory.calls == [
        {
            "port": "/dev/fake-pillar",
            "baudrate": 115_200,
            "timeout": 0.05,
            "write_timeout": 0.5,
        }
    ]
    assert harness.driver.is_connected()

    harness.driver.disconnect()

    assert harness.serial.writes == []
    assert harness.serial.closed
    assert not harness.driver.is_connected()


def test_fragmented_feedback_is_unavailable_until_homed(harness: DriverHarness) -> None:
    harness.driver._feed_bytes(b"pos -50.0")
    assert harness.driver.status().position_m is None

    harness.driver._feed_bytes(
        b"0 mm (-4000 steps)\r\nstate IDLE | brake ENGAGED (SSR off) | auto-brake on\r\n"
        b"NOT HOMED ('home'), nominal zero, soft limits would be -500..-2 mm | "
        b"start -50 mm | 0 = top switch\r\n"
    )
    assert harness.driver.status().position_m == pytest.approx(-0.05)
    assert harness.driver.latest_feedback() is None
    assert harness.feedback == []

    harness.driver._feed_bytes(
        b"HOMED, soft limits -500..-2 mm | start -50 mm | 0 = top switch\r\n"
    )

    assert harness.driver.latest_feedback() == PillarFeedback(-0.05, harness.clock.now)
    assert harness.feedback == [PillarFeedback(-0.05, harness.clock.now)]


def test_home_stays_homing_across_internal_moves_until_final_success(
    harness: DriverHarness,
) -> None:
    harness.set_status(position_mm=-50.0, homed=False)

    assert harness.driver.home()
    assert harness.serial.writes == [b"home\n"]
    assert harness.driver.status().phase is PillarPhase.HOMING

    harness.driver._feed_bytes(
        b"TOP LIMIT hit - stopped. pos 0.00 mm (0 steps)\r\n"
        b"homing: backing off\r\n"
        b"move -5.00 mm DOWN\r\n"
        b"done. pos -5.00 mm (-400 steps)\r\n"
    )
    assert harness.driver.status().phase is PillarPhase.HOMING
    assert harness.driver.status().fault is None

    harness.driver._feed_bytes(
        b"done. pos -50.00 mm (-4000 steps)\r\nHOMED - at start position\r\n"
    )
    status = harness.driver.status()
    assert status.phase is PillarPhase.IDLE
    assert status.homed
    assert status.position_m == pytest.approx(-0.05)


def test_home_failure_blocks_position_commands(harness: DriverHarness) -> None:
    harness.set_status(position_mm=-50.0, homed=False)
    assert harness.driver.home()

    harness.driver._feed_bytes(
        b"HOMING FAILED: switch never tripped - wrong direction ('n') or switch missing\r\n"
    )

    status = harness.driver.status()
    assert status.phase is PillarPhase.FAULT
    assert not status.homed
    assert status.fault is not None
    assert not harness.driver.queue_position(-0.1)


def test_position_command_uses_metres_and_coalesces_latest_target(
    harness: DriverHarness,
) -> None:
    harness.set_status(position_mm=-50.0, homed=True)
    harness.clear_writes()

    assert harness.driver.queue_position(-0.100)
    harness.driver._service()
    assert harness.serial.writes == [b"g -100.000\n"]

    harness.driver._feed_bytes(
        b"move -50.00 mm DOWN\r\nstate MOVING | brake RELEASED (SSR on) | auto-brake on\r\n"
    )
    assert harness.driver.queue_position(-0.100)
    assert harness.driver.queue_position(-0.200)
    harness.driver._service()
    assert harness.serial.writes == [b"g -100.000\n"]

    harness.driver._feed_bytes(b"done. pos -100.00 mm (-8000 steps)\r\n")
    assert harness.serial.writes == [b"g -100.000\n", b"g -200.000\n"]


def test_status_polling_is_suspended_while_pillar_is_moving(harness: DriverHarness) -> None:
    harness.set_status(position_mm=-50.0, homed=True)
    harness.clear_writes()
    assert harness.driver.queue_position(-0.100)
    harness.driver._service()
    harness.driver._feed_bytes(b"move -50.00 mm DOWN\r\n")

    harness.clock.now += 60.01
    harness.driver._service()
    assert harness.serial.writes == [b"g -100.000\n"]

    harness.driver._feed_bytes(b"done. pos -100.00 mm (-8000 steps)\r\n")
    harness.driver._service()
    assert harness.serial.writes == [b"g -100.000\n", b"p\n"]


def test_long_move_keeps_latest_target_without_motion_status_poll(
    harness: DriverHarness,
) -> None:
    harness.set_status(position_mm=-50.0, homed=True)
    harness.clear_writes()
    assert harness.driver.queue_position(-0.100)
    harness.driver._service()
    harness.driver._feed_bytes(b"move -50.00 mm DOWN\r\n")

    harness.clock.now += 2.01
    assert harness.driver.queue_position(-0.200)
    harness.driver._service()
    assert harness.serial.writes == [b"g -100.000\n"]

    harness.driver._feed_bytes(b"done. pos -100.00 mm (-8000 steps)\r\n")
    assert harness.serial.writes == [b"g -100.000\n", b"g -200.000\n"]


def test_latest_command_can_cancel_an_obsolete_queued_target(harness: DriverHarness) -> None:
    harness.set_status(position_mm=-50.0, homed=True)
    harness.clear_writes()
    assert harness.driver.queue_position(-0.100)
    harness.driver._service()
    harness.driver._feed_bytes(b"move -50.00 mm DOWN\r\n")

    assert harness.driver.queue_position(-0.200)
    assert harness.driver.queue_position(-0.100)
    harness.driver._feed_bytes(b"done. pos -100.00 mm (-8000 steps)\r\n")

    assert harness.serial.writes == [b"g -100.000\n"]


def test_stopped_target_can_be_commanded_again(harness: DriverHarness) -> None:
    harness.set_status(position_mm=-50.0, homed=True)
    harness.clear_writes()
    assert harness.driver.queue_position(-0.200)
    harness.driver._service()
    harness.driver._feed_bytes(b"move -150.00 mm DOWN\r\n")

    assert harness.driver.stop_motion()
    harness.driver._feed_bytes(b"stopped. pos -100.00 mm (-8000 steps)\r\n")
    assert harness.driver.queue_position(-0.200)
    harness.driver._service()

    assert harness.serial.writes == [b"g -200.000\n", b"x\n", b"g -200.000\n"]


def test_fault_remains_latched_across_periodic_status(harness: DriverHarness) -> None:
    harness.set_status(position_mm=-50.0, homed=True)
    harness.driver._feed_bytes(
        b"move -20.00 mm DOWN\r\nwarning: brake is engaged\r\n"
        b"state IDLE | brake ENGAGED (SSR off) | auto-brake off\r\n"
    )

    status = harness.driver.status()
    assert status.phase is PillarPhase.FAULT
    assert status.fault == "warning: brake is engaged"
    assert not harness.driver.queue_position(-0.1)


@pytest.mark.parametrize(
    ("line", "expected"),
    [
        ("top limit: D2 HIGH -> TRIGGERED (contact open)", True),
        ("top limit: D2 LOW -> clear (NC contact closed to GND)", False),
    ],
)
def test_status_query_reports_top_limit(harness: DriverHarness, line: str, expected: bool) -> None:
    harness.driver._feed_bytes(f"{line}\r\n".encode())
    assert harness.driver.status().top_limit_triggered is expected


@pytest.mark.parametrize(
    "target",
    [
        float("nan"),
        float("inf"),
        float("-inf"),
        PILLAR_MIN_POSITION_M - 0.001,
        PILLAR_MAX_POSITION_M + 0.001,
    ],
)
def test_position_command_rejects_nonfinite_and_out_of_range_targets(
    harness: DriverHarness,
    target: float,
) -> None:
    harness.set_status(position_mm=-50.0, homed=True)
    harness.clear_writes()

    assert not harness.driver.queue_position(target)
    harness.driver._service()
    assert harness.serial.writes == []


def test_position_command_rejects_stale_feedback(harness: DriverHarness) -> None:
    harness.set_status(position_mm=-50.0, homed=True)
    harness.clock.now += 2.01
    harness.clear_writes()

    assert harness.driver.latest_feedback() is None
    assert not harness.driver.queue_position(-0.1)
    harness.driver._service()
    assert harness.serial.writes == []


def test_controller_reboot_invalidates_homed_state(harness: DriverHarness) -> None:
    harness.set_status(position_mm=-50.0, homed=True)
    assert harness.driver.latest_feedback() is not None

    harness.driver._feed_bytes(b"openFT pillar bench test\r\n")

    status = harness.driver.status()
    assert not status.homed
    assert status.phase is PillarPhase.FAULT
    assert status.fault == "pillar controller rebooted; home is required"
    assert harness.driver.latest_feedback() is None
    assert not harness.driver.queue_position(-0.1)

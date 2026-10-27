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

"""The emulated Nano speaks the pillar_driver protocol, and PillarConnection drives it as the rail."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from itertools import pairwise
import sys
import time
from typing import Any, cast
from unittest.mock import MagicMock

import pytest

from dimos.core.module import Module
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_FIRMWARE_JOINT,
    PILLAR_HOME_POSITION_M,
    PILLAR_LIFT_JOINT,
    PILLAR_SIM_DEVICE,
    PillarConnection,
    PillarConnectionConfig,
    PillarPhase,
)
from dimos.robot.diy.alfred.pillar_emulator import (
    FIRMWARE_ACCEL_MM_S2,
    FIRMWARE_BOOT_S,
    FIRMWARE_SPEED_MM_S,
    PillarEmulator,
    PillarFirmware,
)

SPEED = FIRMWARE_SPEED_MM_S / 1000.0
ACCEL = FIRMWARE_ACCEL_MM_S2 / 1000.0


class _Board:
    """The firmware on a clock the test turns: everything it said, and when."""

    def __init__(self, position_m: float = PILLAR_HOME_POSITION_M) -> None:
        self.now = 0.0
        self.firmware = PillarFirmware(now=0.0, position_m=position_m)
        self.lines: list[str] = []
        self.track: list[tuple[float, float]] = []

    def send(self, line: str) -> str:
        """The single reply to ``line`` (events that fell due before it are kept too)."""
        said = self.firmware.receive(line, self.now)
        self.lines += said
        replies = [s for s in said if s == "ok" or s.startswith("ok ") or s.startswith("err ")]
        assert len(replies) == 1, said
        return replies[0]

    def run(self, seconds: float, dt: float = 0.005) -> list[str]:
        said: list[str] = []
        end = self.now + seconds
        while self.now < end - 1e-12:
            self.now = min(end, self.now + dt)
            said += self.firmware.advance(self.now)
            self.track.append((self.now, self.firmware.position_m))
        self.lines += said
        return said

    def run_until(self, event: str, timeout: float = 60.0) -> float:
        """Seconds until ``<STATUS,..,event>``."""
        start = self.now
        while self.now - start < timeout:
            if any(line.endswith(f",{event}>") for line in self.run(0.005)):
                return self.now - start
        raise AssertionError(f"no {event} within {timeout} s; said {self.lines[-5:]}")

    def booted(self) -> _Board:
        self.run(FIRMWARE_BOOT_S + 0.01)
        assert self.send("set echo 0") == "ok"
        return self

    def homed(self) -> _Board:
        self.booted()
        assert self.send("home") == "ok"
        self.run_until("homed")
        return self


def _events(lines: list[str]) -> list[str]:
    return [line[1:-1].split(",")[2] for line in lines if line.startswith("<STATUS,")]


def _frame(position_m: float) -> str:
    return f"<0,{PILLAR_FIRMWARE_JOINT},{position_m:.6f}>"


def test_the_board_boots_unhomed_and_loses_what_was_sent_during_the_reset() -> None:
    board = _Board()
    assert board.firmware.receive("home", 0.1) == [], "nothing answers before ready"
    said = board.run(FIRMWARE_BOOT_S)
    assert _events(said) == ["ready"]
    assert said[0] == "pillar_driver (emulated)", "prose, which the connection logs and drops"
    assert not board.firmware.homed
    assert board.firmware.position_m == PILLAR_HOME_POSITION_M, "the restored position"


def test_settings_answer_ok_and_telemetry_streams_only_once_rate_is_set() -> None:
    board = _Board().booted()
    assert not [line for line in board.run(0.5) if line.startswith("<0")]
    assert board.send("set rate 50") == "ok"
    frames = [line for line in board.run(1.0) if not line.startswith("<STATUS")]
    assert 49 <= len(frames) <= 51
    stamp, joint, value = frames[-1][1:-1].split(",")
    assert joint == PILLAR_FIRMWARE_JOINT and float(value) == PILLAR_HOME_POSITION_M
    assert int(stamp) == pytest.approx(board.now * 1000.0, abs=25)
    assert board.send("set speed 60") == board.send("set accel 400") == "ok"
    assert board.send("set rate 0") == "ok"
    assert not [line for line in board.run(0.5) if not line.startswith("<STATUS")]
    assert board.send("set rate 500") == "err bad_arg"


def test_echo_is_prose_until_it_is_turned_off() -> None:
    board = _Board()
    board.run(FIRMWARE_BOOT_S + 0.01)
    assert board.firmware.receive("set echo 0", board.now) == ["> set echo 0", "ok"]
    assert board.firmware.receive("set rate 10", board.now) == ["ok"]


def test_home_answers_at_once_then_touches_the_switch_and_parks() -> None:
    board = _Board(position_m=-0.20).booted()
    assert board.send(_frame(-0.10)) == "err not_homed"
    assert board.send("home") == "ok"
    assert board.send("home") == "err busy", "home cannot restart itself mid-run"
    took = board.run_until("homed")
    assert _events(board.lines)[-3:] == ["limit_triggered", "limit_clear", "homed"]
    assert board.firmware.homed and not board.firmware.moving
    assert board.firmware.position_m == PILLAR_HOME_POSITION_M
    assert -1e-3 < max(position for _, position in board.track) <= 0.0, "to the switch, no further"
    # 0.20 m up and 0.05 m down at 40 mm/s, plus a ramp at each end of each leg.
    travel = (0.20 + 0.05) / SPEED
    assert travel < took < travel + 2 * SPEED / ACCEL + 0.1


def test_a_target_ramps_there_within_speed_and_reports_move_done() -> None:
    board = _Board().homed()
    start = board.now
    assert board.send(_frame(-0.25)) == "ok"
    took = board.run_until("move_done")
    assert board.firmware.position_m == -0.25 and not board.firmware.moving
    assert took == pytest.approx(0.20 / SPEED + SPEED / ACCEL, abs=0.05), "a trapezoid"
    moved = [(t, p) for t, p in board.track if t >= start]
    speeds = [abs(b[1] - a[1]) / (b[0] - a[0]) for a, b in pairwise(moved)]
    assert max(speeds) <= SPEED * 1.001
    assert board.send(_frame(-0.25)) == "err zero_move", "already there"


def test_a_reversal_rides_the_ramp_down_first() -> None:
    board = _Board().homed()
    board.send(_frame(-0.40))
    board.run(2.0)  # cruising down at 40 mm/s
    turned_at = board.firmware.position_m
    assert board.send(_frame(-0.05)) == "ok", "a new target preempts the move"
    board.run_until("move_done")
    lowest = min(position for _, position in board.track[-2000:])
    # v^2/2a: 2.7 mm at the default 40 mm/s and 300 mm/s^2, as the README says.
    assert turned_at - lowest == pytest.approx(SPEED**2 / (2 * ACCEL), abs=2e-4)
    assert board.firmware.position_m == -0.05


def test_stop_ramps_down_where_it_is() -> None:
    board = _Board().homed()
    board.send(_frame(-0.40))
    board.run(2.0)
    at = board.firmware.position_m
    assert board.send("stop") == "ok"
    board.run_until("move_done")
    assert at - board.firmware.position_m == pytest.approx(SPEED**2 / (2 * ACCEL), abs=2e-4)
    assert not board.firmware.moving and board.firmware.homed
    assert board.send("stop") == "ok", "a stop at rest is accepted and says nothing more"
    assert not _events(board.run(0.2))


def test_a_stop_mid_home_aborts_the_homing() -> None:
    board = _Board(position_m=-0.30).booted()
    board.send("home")
    board.run(1.0)
    board.send("stop")
    board.run_until("homing_aborted")
    assert not board.firmware.homed
    assert board.send(_frame(-0.10)) == "err not_homed"


@pytest.mark.parametrize(
    ("line", "reply"),
    [
        ("jump", "err unknown_cmd"),
        ("set colour 3", "err unknown_key"),
        ("set rate", "err needs_arg"),
        ("set speed fast", "err bad_arg"),
        ("<0,lift,-0.1>", "err bad_frame"),
        ("<0,pillar_platform_joint,low>", "err bad_frame"),
        (_frame(-0.6), "err too_far"),
        (_frame(0.0), "err too_far"),
        ("set rate " + "1" * 64, "err line_too_long"),
    ],
)
def test_what_the_connection_must_never_send_is_refused(line: str, reply: str) -> None:
    board = _Board().homed()
    assert board.send(line).startswith(reply)


def test_a_target_past_the_limit_the_rail_is_on_means_already_there() -> None:
    board = _Board().homed()
    board.send(_frame(-0.5))
    board.run_until("move_done")
    assert board.send(_frame(-0.6)) == "err at_soft_limit"


# ---------------------------------------------------------------------- the serial port


def test_the_port_reads_like_pyserial() -> None:
    port = PillarEmulator(timeout=0.05)
    try:
        assert port.read(256) == b"", "silent until the board has booted"
        deadline = time.monotonic() + 2.0
        received = b""
        while b"ready>" not in received and time.monotonic() < deadline:
            received += port.read(256)
        assert received.endswith(b",ready>\r\n")
        assert port.write(b"set echo 0\nset rate 20\n") == 23
        time.sleep(0.2)
        lines = port.read(4096).decode().splitlines()
        assert lines[:3] == ["> set echo 0", "ok", "ok"], "echo was on until the first reply"
        assert lines[3].startswith("<") and lines[3].endswith(f",{PILLAR_HOME_POSITION_M:.6f}>")
    finally:
        port.close()
    assert not port.is_open
    with pytest.raises(OSError):
        port.read(1)


# ---------------------------------------------------------------------- the connection on it


@pytest.fixture
def pillar(monkeypatch: pytest.MonkeyPatch) -> Iterator[PillarConnection]:
    """PillarConnection with the framework's Module.__init__ skipped, on the emulated rail, set
    fast so a test homes in under a second; pyserial is made unimportable to show it is unused."""
    monkeypatch.setattr(Module, "__init__", lambda self, **kwargs: None)
    monkeypatch.setitem(sys.modules, "serial", None)
    module = PillarConnection()
    module.config = PillarConnectionConfig(
        device_path=PILLAR_SIM_DEVICE, speed_mm_s=250.0, accel_mm_s2=2500.0
    )
    module.motor_states = MagicMock()
    module._connect()
    try:
        yield module
    finally:
        module._disconnect()


def _wait(condition: Callable[[], Any], timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.02)


def test_the_connection_homes_jogs_and_stops_the_emulated_rail(pillar: PillarConnection) -> None:
    _wait(lambda: pillar.get_status()["ready"])
    status = pillar.get_status()
    assert status["phase"] == PillarPhase.IDLE.value and not status["homed"]
    assert pillar.set_position(-0.10) is False, "refused until homed, as on the robot"

    assert pillar.home() is True
    _wait(lambda: pillar.get_status()["homed"])
    status = pillar.get_status()
    assert status["phase"] == PillarPhase.IDLE.value and status["fault"] is None
    assert status["position_m"] == pytest.approx(PILLAR_HOME_POSITION_M, abs=0.003)
    assert status["top_limit_triggered"] is False, "it backed off the switch"

    assert pillar.set_position(-0.15) is True
    _wait(lambda: pillar.get_status()["phase"] == PillarPhase.MOVING.value, timeout=1.0)
    _wait(lambda: pillar.get_status()["phase"] == PillarPhase.IDLE.value)
    assert pillar.get_status()["position_m"] == pytest.approx(-0.15, abs=0.003)

    assert pillar.set_position(-0.45) is True
    _wait(lambda: pillar.get_status()["position_m"] < -0.25)
    assert pillar.stop_motion() is True
    _wait(lambda: pillar.get_status()["phase"] == PillarPhase.IDLE.value)
    stopped = pillar.get_status()
    assert -0.45 < stopped["position_m"] < -0.25 and stopped["fault"] is None

    publish = cast("MagicMock", pillar.motor_states.publish)
    published = [call.args[0] for call in publish.call_args_list]
    assert published and all(isinstance(msg, JointState) for msg in published)
    assert published[-1].name == [PILLAR_LIFT_JOINT]
    assert published[-1].position[0] == pytest.approx(stopped["position_m"], abs=0.01)

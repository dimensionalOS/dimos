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
import json
import math
import socket
import struct
from threading import Event, Thread
import time
from typing import Any
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.robot.deeprobotics.m20 import connection as m20_connection
from dimos.robot.deeprobotics.m20.connection import (
    _BASIC_SERVER_HEADER,
    _BASIC_SERVER_MAGIC,
    CHARGE_CHARGING,
    CHARGE_COMMAND_START,
    CHARGE_COMMAND_STOP,
    CHARGE_ENTERING,
    CHARGE_ERROR,
    CHARGE_EXITING,
    CHARGE_IDLE,
    CHARGE_ON_DOCK,
    DOCK_CHARGING,
    GAIT_BASIC,
    GAIT_STAIR_AGILE,
    MOTION_RL_CONTROL,
    MOTION_SIT,
    MOTION_STAND,
    USAGE_MODE_NAVIGATION,
    M20Connection,
    dock_verdict,
)

INPUTS = ("requested_cmd_vel", "command_ready", "motion_state", "gait_state", "odometry")
OUTPUTS = ("cmd_vel", "motion_state_cmd", "gait_cmd", "odom", "goal")
HEADER = struct.Struct("<4sHHB7s")


class FakeTransport:
    def __init__(self) -> None:
        self.subscribers: list[Callable[[Any], None]] = []
        self.sent: list[Any] = []

    def subscribe(self, cb: Callable[[Any], None], stream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def publish(self, msg: Any) -> None:
        for cb in list(self.subscribers):
            cb(msg)

    def broadcast(self, stream: Any, msg: Any) -> None:
        self.sent.append(msg)
        self.publish(msg)

    def stop(self) -> None:
        self.subscribers.clear()


class FakeBasicServer:
    """Loopback basic_server that reports to whoever sends it heartbeats."""

    def __init__(self) -> None:
        self.charge = CHARGE_IDLE
        self.usage_mode = USAGE_MODE_NAVIGATION
        self.errors: list[int] = []
        self.received: list[dict[str, Any]] = []
        self.raw: list[bytes] = []
        self.reporting = True
        self._stop = Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", 0))
        self._socket.settimeout(0.02)
        self.port = int(self._socket.getsockname()[1])
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join()
        self._socket.close()

    def charge_commands(self) -> list[int]:
        return [
            frame["Items"]["Charge"]
            for frame in list(self.received)
            if (frame["Type"], frame["Command"]) == (2, 24)
        ]

    def _run(self) -> None:
        peer = None
        while not self._stop.is_set():
            try:
                data, peer = self._socket.recvfrom(0xFFFF)
                self.raw.append(data)
                self.received.append(json.loads(data[_BASIC_SERVER_HEADER.size :])["PatrolDevice"])
            except TimeoutError:
                pass
            if peer is None or not self.reporting:
                continue
            for frame in (
                b"garbage",
                _BASIC_SERVER_MAGIC + b"\0" * 12 + b"{not json",
                self._report(4, {"MotionStatus": {"Height": 0.5}}),
                self._report(
                    6,
                    {
                        "BasicStatus": {
                            "MotionState": 17,
                            "Charge": self.charge,
                            "ControlUsageMode": self.usage_mode,
                        }
                    },
                ),
                self._report(3, {"ErrorList": [{"errorCode": []}]}),
                self._report(3, {"ErrorList": [{"errorCode": c} for c in [0, *self.errors]]}),
            ):
                self._socket.sendto(frame, peer)

    @staticmethod
    def _report(command: int, items: dict[str, Any]) -> bytes:
        body = json.dumps(
            {"PatrolDevice": {"Type": 1002, "Command": command, "Items": items}}
        ).encode()
        return _BASIC_SERVER_HEADER.pack(_BASIC_SERVER_MAGIC, len(body), 1, 1, b"\0" * 7) + body


@pytest.fixture
def server() -> Iterator[FakeBasicServer]:
    fake = FakeBasicServer()
    try:
        yield fake
    finally:
        fake.close()


@pytest.fixture
def connection(server: FakeBasicServer) -> Iterator[M20Connection]:
    module = M20Connection(
        basic_server_host="127.0.0.1",
        basic_server_udp_port=server.port,
        dock_settle_s=0.05,
        dock_start_timeout_s=0.4,
        dock_timeout_s=1.5,
        undock_timeout_s=1.5,
    )
    for stream in ("cmd_vel", "goal", "motion_state_cmd", "gait_cmd", "odom"):
        setattr(module, stream, MagicMock())
    for tool in ("start_tool", "tool_update", "stop_tool"):
        setattr(module, tool, MagicMock())
    module._stand = MagicMock(return_value=True)  # type: ignore[method-assign]
    module._motion_state = MOTION_RL_CONTROL
    module._start_basic_server_link()
    try:
        yield module
    finally:
        module.stop()


@pytest.fixture
def started(
    server: FakeBasicServer, mocker: MockerFixture
) -> Iterator[Callable[..., M20Connection]]:
    """Start M20Connections on fake streams with the given methods stubbed, and stop the ones a test left running."""
    built: list[M20Connection] = []

    def start(stubs: dict[str, Callable[..., Any]] | None = None, **config: Any) -> M20Connection:
        conn = M20Connection(
            basic_server_host="127.0.0.1",
            basic_server_udp_port=server.port,
            **config,
        )
        for name in INPUTS + OUTPUTS:
            getattr(conn, name).transport = FakeTransport()
        for name, stub in (stubs or {"standup": lambda: True}).items():
            mocker.patch.object(conn, name, side_effect=stub)
        conn.start()
        built.append(conn)
        return conn

    yield start
    for conn in built:
        if not conn._link_stop.is_set():
            conn.stop()


def _wait_until(condition: Callable[[], bool], timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not condition():
        assert time.monotonic() < deadline, "condition not met in time"
        time.sleep(0.01)


def _wait_for_status(module: M20Connection, status: int) -> None:
    _wait_until(lambda: module._fresh_charge_status() == status)


def _updates(module: M20Connection) -> list[str]:
    return [call.args[1] for call in module.tool_update.call_args_list]  # type: ignore[attr-defined]


@pytest.mark.parametrize(
    "status,entered,elapsed_s,expected",
    [
        (CHARGE_CHARGING, True, 1.0, DOCK_CHARGING),
        (CHARGE_ERROR, True, 1.0, "the robot reported a charging error"),
        (CHARGE_EXITING, True, 1.0, "the robot gave up and is backing off the dock"),
        (CHARGE_IDLE, True, 1.0, "the robot stopped docking and returned to idle"),
        (CHARGE_IDLE, False, 1.0, None),
        (CHARGE_IDLE, False, 5.0, "the robot did not accept the dock command"),
        (CHARGE_ENTERING, True, 149.0, None),
        (CHARGE_ENTERING, True, 150.0, "timed out after 150 s"),
        (CHARGE_ON_DOCK, True, 149.0, None),
        (CHARGE_ON_DOCK, True, 150.0, "the robot is on the dock but not charging"),
    ],
)
def test_dock_verdict(status: int, entered: bool, elapsed_s: float, expected: str | None) -> None:
    assert dock_verdict(status, entered, elapsed_s, 5.0, 150.0) == expected


def test_reports_update_the_status_past_malformed_frames(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    server.charge = CHARGE_CHARGING
    _wait_for_status(connection, CHARGE_CHARGING)
    assert connection.charge_status() == "charging"

    server.errors = [0x8506]
    _wait_until(lambda: connection._charge_error == 0x8506)
    assert connection.charge_status() == "charging, error 0x8506 no current from the dock"

    server.errors = []
    _wait_until(lambda: connection._charge_error is None)
    assert connection.charge_status() == "charging"


def test_charge_status_is_unknown_when_reports_go_stale(
    connection: M20Connection, server: FakeBasicServer, monkeypatch: pytest.MonkeyPatch
) -> None:
    _wait_for_status(connection, CHARGE_IDLE)
    server.reporting = False
    monkeypatch.setattr("dimos.robot.deeprobotics.m20.connection._CHARGE_STATUS_MAX_AGE_S", 0.05)
    _wait_until(lambda: connection.charge_status() == "unknown")


def test_dock_cancels_nav_then_sends_the_vendor_command_and_reports_charging(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    _wait_for_status(connection, CHARGE_IDLE)

    assert "Docking" in connection.dock()
    _wait_until(lambda: server.charge_commands() == [1])

    connection.goal.publish.assert_called_once()  # type: ignore[attr-defined]
    goal = connection.goal.publish.call_args.args[0]  # type: ignore[attr-defined]
    assert math.isnan(goal.x) and math.isnan(goal.y) and math.isnan(goal.z)
    connection.cmd_vel.publish.assert_not_called()  # type: ignore[attr-defined]

    apdu = next(frame for frame in server.raw if b'"Command":24' in frame)
    magic, length, _message_id, encoding, _reserved = _BASIC_SERVER_HEADER.unpack(
        apdu[: _BASIC_SERVER_HEADER.size]
    )
    assert (magic, encoding, length) == (
        bytes.fromhex("eb91eb90"),
        1,
        len(apdu) - _BASIC_SERVER_HEADER.size,
    )

    server.charge = CHARGE_ENTERING
    _wait_until(lambda: "Charge status: entering the dock." in _updates(connection))
    server.charge = CHARGE_CHARGING
    _wait_until(lambda: "Docked and charging." in _updates(connection))

    connection.start_tool.assert_called_once_with("dock")  # type: ignore[attr-defined]
    connection.stop_tool.assert_called_once_with("dock")  # type: ignore[attr-defined]
    assert connection._charge_task is None


@pytest.mark.parametrize(
    "prepare,status,reason",
    [
        (
            lambda m, s: setattr(s, "charge", CHARGE_CHARGING),
            CHARGE_CHARGING,
            "charge status is charging",
        ),
        (lambda m, s: setattr(m, "_motion_state", 4), CHARGE_IDLE, "not standing"),
    ],
)
def test_dock_refuses_from_a_bad_state(
    connection: M20Connection,
    server: FakeBasicServer,
    prepare: Callable[[M20Connection, FakeBasicServer], None],
    status: int,
    reason: str,
) -> None:
    _wait_for_status(connection, CHARGE_IDLE)
    prepare(connection, server)
    _wait_for_status(connection, status)

    message = connection.dock()

    assert reason in message
    assert server.charge_commands() == []
    connection.goal.publish.assert_not_called()  # type: ignore[attr-defined]
    connection.stop_tool.assert_called_once_with("dock")  # type: ignore[attr-defined]


def test_dock_refuses_without_status_reports(server: FakeBasicServer) -> None:
    server.reporting = False
    module = M20Connection(basic_server_host="127.0.0.1", basic_server_udp_port=server.port)
    module.cmd_vel = MagicMock()  # type: ignore[assignment]
    for tool in ("start_tool", "stop_tool"):
        setattr(module, tool, MagicMock())
    module._start_basic_server_link()
    try:
        assert "not reporting" in module.dock()
    finally:
        module.stop()
    assert server.charge_commands() == []


def test_dock_failure_names_the_vendor_error_even_after_it_clears(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    _wait_for_status(connection, CHARGE_IDLE)
    connection.dock()
    _wait_until(lambda: server.charge_commands() == [1])
    server.charge = CHARGE_ENTERING
    _wait_for_status(connection, CHARGE_ENTERING)
    server.errors = [0x8503]
    _wait_until(lambda: connection._charge_error == 0x8503)
    server.errors = []
    server.charge = CHARGE_IDLE

    _wait_until(lambda: connection._charge_task is None)

    assert _updates(connection)[-1] == (
        "Docking failed: the robot stopped docking and returned to idle "
        "(0x8503 no location, reflectors not found). The robot is standing ready to navigate."
    )
    assert server.charge_commands() == [1]
    connection._stand.assert_called_once()  # type: ignore[attr-defined]


def test_dock_failure_leaves_a_sitting_robot_sitting(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    _wait_for_status(connection, CHARGE_IDLE)
    connection.dock()
    _wait_until(lambda: server.charge_commands() == [1])
    server.charge = CHARGE_ERROR
    _wait_for_status(connection, CHARGE_ERROR)
    connection._on_motion_state(Int32(MOTION_SIT))
    server.charge = CHARGE_IDLE

    _wait_until(lambda: connection._charge_task is None)

    assert _updates(connection)[-1].endswith("Call standup() before navigating.")
    connection._stand.assert_not_called()  # type: ignore[attr-defined]


def test_gait_is_resent_when_the_robot_drops_it(connection: M20Connection) -> None:
    connection.config.gait_timeout_s = 0.1
    sends: list[int] = []

    def publish(msg: UInt32) -> None:
        sends.append(msg.data)
        if len(sends) == 2:
            connection._on_gait_state(msg)

    connection.gait_cmd.publish.side_effect = publish  # type: ignore[attr-defined]

    assert connection._set_gait_and_wait(GAIT_STAIR_AGILE) is True
    assert sends == [GAIT_STAIR_AGILE, GAIT_STAIR_AGILE]


def test_dock_timeout_sends_stop(connection: M20Connection, server: FakeBasicServer) -> None:
    _wait_for_status(connection, CHARGE_IDLE)
    connection.dock()
    _wait_until(lambda: server.charge_commands() == [1])
    server.charge = CHARGE_ENTERING
    _wait_until(lambda: server.charge_commands() == [1, 0])
    server.charge = CHARGE_IDLE

    _wait_until(lambda: connection._charge_task is None)

    assert "timed out" in _updates(connection)[-1]


def test_undock_leaves_the_dock_and_stands(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    server.charge = CHARGE_CHARGING
    _wait_for_status(connection, CHARGE_CHARGING)

    assert "Undocking" in connection.undock()
    _wait_until(lambda: server.charge_commands() == [0])
    server.charge = CHARGE_EXITING
    connection._stand.assert_not_called()  # type: ignore[attr-defined]
    server.charge = CHARGE_IDLE

    _wait_until(lambda: "Off the dock and ready to navigate." in _updates(connection))
    connection._stand.assert_called_once()  # type: ignore[attr-defined]
    connection.stop_tool.assert_called_once_with("undock")  # type: ignore[attr-defined]


def test_undock_cancels_a_running_dock(connection: M20Connection, server: FakeBasicServer) -> None:
    _wait_for_status(connection, CHARGE_IDLE)
    connection.dock()
    _wait_until(lambda: server.charge_commands() == [1])
    server.charge = CHARGE_ENTERING
    _wait_for_status(connection, CHARGE_ENTERING)

    connection.undock()
    _wait_until(lambda: server.charge_commands() == [1, 0])
    server.charge = CHARGE_IDLE

    _wait_until(lambda: connection._charge_task is None)
    updates = _updates(connection)
    assert "Docking cancelled by undock." in updates
    assert "Off the dock and ready to navigate." in updates


def test_stop_during_the_settle_cancels_without_commanding_the_robot(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    connection.config.dock_settle_s = 5.0
    _wait_for_status(connection, CHARGE_IDLE)
    connection.dock()

    connection.stop_movement()

    _wait_until(lambda: "Docking cancelled before it started." in _updates(connection))
    assert connection._charge_task is None
    assert server.charge_commands() == []
    connection._stand.assert_not_called()  # type: ignore[attr-defined]

    assert connection._charge_thread is not None
    _wait_until(lambda: not connection._charge_thread.is_alive())  # type: ignore[union-attr]
    connection.config.dock_settle_s = 0.05
    assert "Docking." in connection.dock()
    _wait_until(lambda: server.charge_commands() == [CHARGE_COMMAND_START])


def test_undock_racing_the_dock_start_stops_after_it(
    connection: M20Connection, server: FakeBasicServer, mocker: MockerFixture
) -> None:
    send = connection._send_charge_command

    def undock_while_starting(command: int) -> bool:
        if command == CHARGE_COMMAND_START:
            racer = Thread(target=connection.undock, daemon=True)
            racer.start()
            racer.join(timeout=0.2)
        return send(command)

    mocker.patch.object(connection, "_send_charge_command", side_effect=undock_while_starting)
    _wait_for_status(connection, CHARGE_IDLE)
    connection.dock()

    _wait_until(lambda: connection._charge_task is None)
    assert server.charge_commands() == [CHARGE_COMMAND_START, CHARGE_COMMAND_STOP]
    assert "Docking cancelled by undock." in _updates(connection)


def test_stop_movement_cancels_a_running_dock(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    _wait_for_status(connection, CHARGE_IDLE)
    connection.dock()
    _wait_until(lambda: server.charge_commands() == [1])
    server.charge = CHARGE_ENTERING
    _wait_for_status(connection, CHARGE_ENTERING)

    connection.stop_movement()
    _wait_until(lambda: server.charge_commands() == [1, 0])
    server.charge = CHARGE_IDLE

    _wait_until(lambda: connection._charge_task is None)
    assert "Off the dock and ready to navigate." in _updates(connection)
    connection.cmd_vel.publish.assert_not_called()  # type: ignore[attr-defined]


def test_undock_refuses_when_not_on_the_dock(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    _wait_for_status(connection, CHARGE_IDLE)

    assert "not on the dock" in connection.undock()

    assert server.charge_commands() == []


def test_undock_stays_off_when_the_vendor_reports_an_exit_timeout(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    server.charge = CHARGE_CHARGING
    _wait_for_status(connection, CHARGE_CHARGING)
    connection.undock()
    _wait_until(lambda: server.charge_commands() == [0])
    server.errors = [0x8510]
    server.charge = CHARGE_IDLE

    _wait_until(lambda: connection._charge_task is None)

    assert _updates(connection)[-1] == "Undocking failed: 0x8510 dock exit timeout."
    connection._stand.assert_not_called()  # type: ignore[attr-defined]


def test_commands_are_refused_on_the_dock(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    server.charge = CHARGE_CHARGING
    _wait_for_status(connection, CHARGE_CHARGING)

    assert connection.move(Twist.zero()) is False
    connection.stop_movement()
    assert connection.liedown() is False
    assert connection.standup() is False
    assert connection.set_navigation_terrain("flat") is False

    connection.cmd_vel.publish.assert_not_called()  # type: ignore[attr-defined]
    connection.motion_state_cmd.publish.assert_not_called()  # type: ignore[attr-defined]
    connection.gait_cmd.publish.assert_not_called()  # type: ignore[attr-defined]
    connection._stand.assert_not_called()  # type: ignore[attr-defined]


def test_stop_does_not_sit_the_robot_on_the_dock(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    server.charge = CHARGE_CHARGING
    connection.config.auto_stand = True
    _wait_for_status(connection, CHARGE_CHARGING)

    connection.stop()

    connection.motion_state_cmd.publish.assert_not_called()  # type: ignore[attr-defined]
    connection.cmd_vel.publish.assert_not_called()  # type: ignore[attr-defined]


def test_requested_velocity_reaches_the_robot_off_the_dock(
    started: Callable[..., M20Connection],
) -> None:
    conn = started()
    _wait_for_status(conn, CHARGE_IDLE)
    twist = Twist(linear=Vector3(0.3, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.1))

    conn.requested_cmd_vel.transport.publish(twist)  # type: ignore[attr-defined]

    assert conn.cmd_vel.transport.sent == [twist]  # type: ignore[attr-defined]
    assert conn.goal.transport.sent == []  # type: ignore[attr-defined]


def test_requested_velocity_is_dropped_on_the_dock_and_cancels_the_goal_once(
    started: Callable[..., M20Connection], server: FakeBasicServer
) -> None:
    server.charge = CHARGE_CHARGING
    conn = started()
    _wait_for_status(conn, CHARGE_CHARGING)

    for _ in range(5):
        conn.requested_cmd_vel.transport.publish(Twist(linear=Vector3(0.3, 0.0, 0.0)))  # type: ignore[attr-defined]

    assert conn.cmd_vel.transport.sent == []  # type: ignore[attr-defined]
    (goal,) = conn.goal.transport.sent  # type: ignore[attr-defined]
    assert math.isnan(goal.x)


def test_the_dock_guards_hold_when_reports_go_stale(
    connection: M20Connection, server: FakeBasicServer, monkeypatch: pytest.MonkeyPatch
) -> None:
    server.charge = CHARGE_CHARGING
    connection.config.auto_stand = True
    _wait_for_status(connection, CHARGE_CHARGING)
    server.reporting = False
    monkeypatch.setattr("dimos.robot.deeprobotics.m20.connection._CHARGE_STATUS_MAX_AGE_S", 0.05)
    _wait_until(lambda: connection.charge_status() == "unknown")

    assert connection.move(Twist.zero()) is False
    connection.stop()

    connection.motion_state_cmd.publish.assert_not_called()  # type: ignore[attr-defined]
    connection.cmd_vel.publish.assert_not_called()  # type: ignore[attr-defined]


def test_auto_stand_undocks_when_on_the_dock(
    connection: M20Connection, server: FakeBasicServer
) -> None:
    server.charge = CHARGE_CHARGING
    _wait_for_status(connection, CHARGE_CHARGING)

    connection._auto_stand()

    _wait_until(lambda: server.charge_commands() == [0])
    connection._stand.assert_not_called()  # type: ignore[attr-defined]
    server.charge = CHARGE_IDLE
    _wait_until(lambda: "Off the dock and ready to navigate." in _updates(connection))
    connection._stand.assert_called_once()  # type: ignore[attr-defined]


def test_auto_stand_stands_when_idle(connection: M20Connection, server: FakeBasicServer) -> None:
    _wait_for_status(connection, CHARGE_IDLE)

    connection._auto_stand()

    connection._stand.assert_called_once()  # type: ignore[attr-defined]
    assert server.charge_commands() == []


def test_auto_stand_stands_without_status(
    server: FakeBasicServer, monkeypatch: pytest.MonkeyPatch
) -> None:
    server.reporting = False
    monkeypatch.setattr("dimos.robot.deeprobotics.m20.connection._CHARGE_STATUS_WAIT_S", 0.05)
    module = M20Connection(basic_server_host="127.0.0.1", basic_server_udp_port=server.port)
    module.cmd_vel = MagicMock()  # type: ignore[assignment]
    module._stand = MagicMock()  # type: ignore[method-assign]
    try:
        module._auto_stand()
    finally:
        module.stop()

    module._stand.assert_called_once()  # type: ignore[attr-defined]


def test_heartbeat_sends_a_type_100_apdu_to_the_udp_port(
    started: Callable[..., M20Connection], server: FakeBasicServer
) -> None:
    started()
    _wait_until(lambda: len(server.raw) > 0)
    datagram = server.raw[0]
    magic, length, _message_id, encoding, _reserved = HEADER.unpack_from(datagram)
    payload = datagram[HEADER.size :]
    device = json.loads(payload)["PatrolDevice"]
    assert (magic, length, encoding) == (bytes.fromhex("eb91eb90"), len(payload), 1)
    assert (device["Type"], device["Command"]) == (100, 100)


@pytest.mark.parametrize("robot_sits", [True, False])
def test_stop_lies_the_robot_down_it_stood_up_and_waits_for_sit(
    started: Callable[..., M20Connection], mocker: MockerFixture, robot_sits: bool
) -> None:
    conn = started(auto_stand=True, sit_timeout_s=0.2)
    motion_state = conn.motion_state.transport

    def sit() -> bool:
        if robot_sits:
            motion_state.publish(Int32(MOTION_SIT))
        return True

    liedown = mocker.patch.object(conn, "liedown", side_effect=sit)
    stop_movement = mocker.patch.object(conn, "stop_movement")
    warning = mocker.patch.object(m20_connection.logger, "warning")
    conn.stop()
    liedown.assert_called_once()
    stop_movement.assert_not_called()
    assert warning.called is not robot_sits


def test_stop_only_zeroes_velocity_without_auto_stand(
    started: Callable[..., M20Connection], mocker: MockerFixture
) -> None:
    conn = started()
    liedown = mocker.patch.object(conn, "liedown")
    stop_movement = mocker.patch.object(conn, "stop_movement")
    conn.stop()
    stop_movement.assert_called_once()
    liedown.assert_not_called()


def test_a_stop_during_standup_leaves_sit_as_the_last_posture_command(
    started: Callable[..., M20Connection],
) -> None:
    conn = started({"_enter_navigation_mode": lambda: True}, auto_stand=True, sit_timeout_s=0.1)
    sent = conn.motion_state_cmd.transport.sent  # type: ignore[attr-defined]
    _wait_until(lambda: [msg.data for msg in sent] == [MOTION_STAND])
    conn.stop()
    assert [msg.data for msg in sent] == [MOTION_STAND, MOTION_SIT]
    assert conn._auto_stand_thread is not None and not conn._auto_stand_thread.is_alive()


class FakeRobot:
    """Answers posture and gait commands, and resets the gait to Basic `late_resets` times after a stand."""

    def __init__(self, conn: M20Connection, late_resets: int) -> None:
        self.conn = conn
        self.late_resets = late_resets
        self.gaits: list[int] = []
        self.modes = 0
        conn.motion_state_cmd.publish.side_effect = lambda msg: conn._on_motion_state(msg)  # type: ignore[attr-defined]
        conn.gait_cmd.publish.side_effect = self._gait  # type: ignore[attr-defined]

    def mode(self) -> bool:
        self.modes += 1
        return True

    def _gait(self, msg: UInt32) -> None:
        self.gaits.append(msg.data)
        self.conn._on_gait_state(msg)
        if msg.data != GAIT_BASIC and self.late_resets > 0:
            self.late_resets -= 1
            Thread(target=self._reset, daemon=True).start()

    def _reset(self) -> None:
        time.sleep(0.05)
        self.conn._on_gait_state(UInt32(GAIT_BASIC))


@pytest.mark.parametrize("late_resets,stood", [(0, True), (2, True), (3, False)])
def test_stand_resends_navigation_after_the_late_gait_reset(
    connection: M20Connection,
    monkeypatch: pytest.MonkeyPatch,
    late_resets: int,
    stood: bool,
) -> None:
    monkeypatch.setattr("dimos.robot.deeprobotics.m20.connection._VENDOR_GAIT_WAIT_S", 0.3)
    connection.config.gait_timeout_s = 0.3
    connection.config.control_ready_timeout_s = 0.1
    connection._command_ready = True
    connection._motion_state = MOTION_SIT
    robot = FakeRobot(connection, late_resets)
    connection._enter_navigation_mode = robot.mode  # type: ignore[method-assign]

    assert M20Connection._stand(connection) is stood

    assert robot.modes == min(late_resets + 1, 3)
    assert robot.gaits == [GAIT_BASIC, GAIT_STAIR_AGILE] * robot.modes

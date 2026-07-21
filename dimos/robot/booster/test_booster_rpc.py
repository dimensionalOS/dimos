# Copyright 2025-2026 Dimensional Inc.
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

"""Unit tests for BoosterRPCConnection's command sender and mode transitions.

Drives the production `run_sender()` coroutine and `stop()` with the SDK mocked
out, so no robot or `booster_rpc` runtime behavior is needed.
"""

import asyncio
import time
from unittest.mock import patch

import pytest

# booster_rpc is an optional extra, skip cleanly if it isn't installed.
pytest.importorskip("booster_rpc")

from booster_rpc import RobotMode
import grpc

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.booster.booster_rpc import BoosterRPCConnection


def _twist(vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> Twist:
    return Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, vyaw))


def _sent(c: BoosterRPCConnection) -> list[tuple[float, float, float]]:
    """The (vx, vy, vyaw) tuples handed to the underlying gRPC move()."""
    return [tuple(call.args) for call in c._conn.move.call_args_list]


@pytest.fixture
def conn():
    """A BoosterRPCConnection with the gRPC SDK patched out (`_conn` is a mock)."""
    with patch("dimos.robot.booster.booster_rpc.BoosterConnection"):
        yield BoosterRPCConnection(ip="mock")


@pytest.fixture
async def start_sender(conn):
    """Runs the production `run_sender()`, always torn down via the production `stop()`."""
    tasks: list[asyncio.Task] = []

    def start() -> None:
        tasks.append(asyncio.create_task(conn.run_sender()))

    try:
        yield start
    finally:
        # stop() blocks until the sender exits, so call it off the loop (as the Module does).
        await asyncio.to_thread(conn.stop)
        await asyncio.gather(*tasks, return_exceptions=True)


class TestMoveIsNonBlocking:
    def test_move_returns_immediately(self, conn):
        # The caller (e.g. the 100 Hz coordinator) must not block on gRPC.
        start = time.perf_counter()
        assert conn.move(_twist(vx=0.5)) is True
        assert time.perf_counter() - start < 0.05

    def test_latest_command_wins_no_queue(self, conn):
        # Commands coalesce to the latest, not queued.
        conn.move(_twist(vx=0.1))
        conn.move(_twist(vx=0.9, vyaw=0.3))
        assert conn._latest == (0.9, 0.0, 0.3)


class TestSenderLoop:
    async def test_sends_latest_while_active(self, conn, start_sender):
        conn.cmd_vel_timeout = 0.5
        conn.send_hz = 200.0
        conn.move(_twist(vx=0.5, vyaw=-0.2))
        start_sender()
        await asyncio.sleep(0.1)  # < cmd_vel_timeout, still active
        sent = _sent(conn)
        assert (0.5, 0.0, -0.2) in sent  # the latest command reaches the robot
        assert all(s == (0.5, 0.0, -0.2) for s in sent)  # only the latest, never stale

    async def test_deadman_sends_one_zero_then_goes_quiet(self, conn, start_sender):
        conn.cmd_vel_timeout = 0.05
        conn.send_hz = 200.0
        conn.move(_twist(vx=0.5))
        start_sender()
        await asyncio.sleep(0.25)  # well past cmd_vel_timeout -> idle
        sent = _sent(conn)
        assert (0.5, 0.0, 0.0) in sent  # sent while active
        assert sent[-1] == (0.0, 0.0, 0.0)  # one dead-man stop on active->idle
        assert sent.count((0.0, 0.0, 0.0)) == 1  # then quiet, not a flood of zeros

    async def test_idle_sender_sends_nothing(self, conn, start_sender):
        conn.send_hz = 200.0
        start_sender()  # never issue a command
        await asyncio.sleep(0.1)
        assert _sent(conn) == []  # no command -> never active -> nothing sent

    async def test_confirm_stop_reports_delivered_zero(self, conn, start_sender):
        conn.send_hz = 200.0
        start_sender()
        conn.move(_twist())  # queue a zero command
        assert await asyncio.to_thread(conn.confirm_stop, 1.0) is True
        assert (0.0, 0.0, 0.0) in _sent(conn)

    def test_confirm_stop_false_when_sender_not_running(self, conn):
        conn.move(_twist())
        assert conn.confirm_stop(timeout=0.1) is False


class TestStandup:
    def test_returns_true_when_already_walking(self, conn):
        conn._conn.get_mode.return_value = RobotMode.WALKING
        assert conn.standup() is True
        conn._conn.change_mode.assert_not_called()  # no transition needed

    def test_refuses_unexpected_mode(self, conn):
        conn._conn.get_mode.return_value = RobotMode.CUSTOM
        assert conn.standup() is False
        conn._conn.change_mode.assert_not_called()  # refuses rather than forcing WALKING

    def test_arms_from_damping_once_each_mode_is_confirmed(self, conn):
        conn._conn.get_mode.side_effect = [
            RobotMode.DAMPING,  # standup() reads the starting mode
            RobotMode.PREPARE,  # PREPARE confirmed
            RobotMode.WALKING,  # WALKING confirmed
        ]
        assert conn.standup() is True
        requested = [call.args[0] for call in conn._conn.change_mode.call_args_list]
        assert requested == [RobotMode.PREPARE, RobotMode.WALKING]

    def test_fails_when_mode_is_never_confirmed(self, conn):
        conn.mode_transition_timeout = 0.2
        conn._conn.get_mode.return_value = RobotMode.PREPARE  # never reaches WALKING
        assert conn.standup() is False
        conn._conn.change_mode.assert_called_once_with(RobotMode.WALKING)

    def test_rerequests_mode_until_the_robot_accepts(self, conn):
        # The robot ignores requests while mid-motion, accepting only a later re-request.
        conn.mode_request_retry = 0.05
        conn._conn.get_mode.side_effect = lambda: (
            RobotMode.WALKING if conn._conn.change_mode.call_count >= 3 else RobotMode.PREPARE
        )
        assert conn.standup() is True
        assert conn._conn.change_mode.call_count >= 3
        assert all(c.args[0] == RobotMode.WALKING for c in conn._conn.change_mode.call_args_list)

    def test_transport_error_returns_false_instead_of_raising(self, conn):
        conn._conn.get_mode.side_effect = grpc.RpcError("transient")
        assert conn.standup() is False

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

"""Unit tests for BoosterRPCConnection's non-blocking command sender.

Mirrors the intent of unitree/b1/test_connection.py: exercise the fixed-rate
sender + dead-man timer (the logic that lets the 100 Hz ControlCoordinator drive
booster-rpc's blocking ~58/sec gRPC `move` without backing up) with the SDK
mocked out, so no robot or `booster_rpc` runtime behavior is needed.
"""

import threading
import time
from unittest.mock import patch

import pytest

# booster_rpc is an optional extra; skip cleanly if it isn't installed.
pytest.importorskip("booster_rpc")

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.booster.k1.connection import BoosterRPCConnection


def _twist(vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> Twist:
    return Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, vyaw))


@pytest.fixture
def conn():
    """A BoosterRPCConnection with the gRPC SDK patched out (`_conn` is a mock)."""
    with patch("dimos.robot.booster.k1.connection.BoosterConnection"):
        c = BoosterRPCConnection(ip="mock")
    yield c
    c._sender_stop.set()
    if c._sender_thread is not None:
        c._sender_thread.join(timeout=1.0)


def _run_sender(c: BoosterRPCConnection) -> None:
    """Start only the fixed-rate sender thread (skip the asyncio video loop)."""
    c._sender_stop.clear()
    c._sender_thread = threading.Thread(target=c._sender_loop, daemon=True)
    c._sender_thread.start()


def _stop_sender(c: BoosterRPCConnection) -> None:
    c._sender_stop.set()
    if c._sender_thread is not None:
        c._sender_thread.join(timeout=1.0)


def _sent(c: BoosterRPCConnection) -> list[tuple[float, float, float]]:
    """The (vx, vy, vyaw) tuples handed to the underlying gRPC move()."""
    return [tuple(call.args) for call in c._conn.move.call_args_list]


class TestMoveIsNonBlocking:
    def test_move_returns_immediately(self, conn):
        # The caller (e.g. the 100 Hz coordinator) must not block on gRPC.
        start = time.perf_counter()
        assert conn.move(_twist(vx=0.5)) is True
        assert time.perf_counter() - start < 0.05

    def test_latest_command_wins_no_queue(self, conn):
        # Commands coalesce to the latest; they are not queued.
        conn.move(_twist(vx=0.1))
        conn.move(_twist(vx=0.9, vyaw=0.3))
        assert conn._latest == (0.9, 0.0, 0.3)

    def test_duration_move_blocks_then_goes_stale(self, conn):
        # The discrete "move for N seconds" path (walk skill) blocks the caller,
        # then lets the command expire.
        start = time.perf_counter()
        conn.move(_twist(vx=0.4), duration=0.1)
        assert time.perf_counter() - start >= 0.1
        assert conn._latest == (0.0, 0.0, 0.0)


class TestSenderLoop:
    def test_sends_latest_while_active(self, conn):
        conn.cmd_vel_timeout = 0.5
        conn.send_hz = 200.0
        conn.move(_twist(vx=0.5, vyaw=-0.2))
        _run_sender(conn)
        time.sleep(0.1)  # < cmd_vel_timeout, still active
        _stop_sender(conn)
        sent = _sent(conn)
        assert (0.5, 0.0, -0.2) in sent  # the latest command reaches the robot
        assert all(s == (0.5, 0.0, -0.2) for s in sent)  # only the latest, never stale

    def test_deadman_sends_one_zero_then_goes_quiet(self, conn):
        conn.cmd_vel_timeout = 0.05
        conn.send_hz = 200.0
        conn.move(_twist(vx=0.5))
        _run_sender(conn)
        time.sleep(0.25)  # well past cmd_vel_timeout -> idle
        _stop_sender(conn)
        sent = _sent(conn)
        assert (0.5, 0.0, 0.0) in sent  # sent while active
        assert sent[-1] == (0.0, 0.0, 0.0)  # one dead-man stop on active->idle
        assert sent.count((0.0, 0.0, 0.0)) == 1  # then quiet, not a flood of zeros

    def test_idle_sender_sends_nothing(self, conn):
        conn.send_hz = 200.0
        _run_sender(conn)  # never issue a command
        time.sleep(0.1)
        _stop_sender(conn)
        assert _sent(conn) == []  # no command -> never active -> nothing sent

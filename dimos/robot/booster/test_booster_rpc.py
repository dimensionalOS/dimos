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
import threading
import time
from unittest.mock import MagicMock, patch

import pytest

# booster_rpc is an optional extra, skip cleanly if it isn't installed.
pytest.importorskip("booster_rpc")

from booster_rpc import RobotMode
import grpc

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.booster.booster_rpc import BoosterRPCConnection
from dimos.robot.booster.k1.connection import (
    WALK_MAX_LINEAR_MPS,
    K1Connection,
    _clamp,
    _clamp_linear,
)


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

    def test_confirm_stop_ignores_nonzero_sends(self, conn):
        # A concurrent publisher overwriting the stop must not count as a stop.
        result: dict[str, bool] = {}
        t = threading.Thread(target=lambda: result.update(ok=conn.confirm_stop(timeout=0.3)))
        t.start()
        time.sleep(0.05)
        conn._send(0.4, 0.0, 0.0)
        conn._send(0.4, 0.0, 0.0)
        t.join()
        assert result["ok"] is False

    def test_stop_is_idempotent(self, conn):
        # dimos shutdown stops modules twice, the second call must be a no-op.
        conn.stop()
        close_calls = conn._conn.close.call_count
        conn.stop()
        assert conn._conn.close.call_count == close_calls

    def test_confirm_stop_confirms_the_zero_itself(self, conn):
        result: dict[str, bool] = {}
        t = threading.Thread(target=lambda: result.update(ok=conn.confirm_stop(timeout=0.5)))
        t.start()
        time.sleep(0.05)
        conn._send(0.0, 0.0, 0.0)
        t.join()
        assert result["ok"] is True


class TestStandup:
    def test_returns_true_when_already_walking(self, conn):
        conn._conn.get_mode.return_value = RobotMode.WALKING
        assert conn.standup() is True
        conn._conn.change_mode.assert_not_called()  # no transition needed

    def test_refuses_unexpected_mode(self, conn):
        conn._conn.get_mode.return_value = RobotMode.CUSTOM
        assert conn.standup() is False
        conn._conn.change_mode.assert_not_called()  # refuses rather than forcing WALKING

    def test_arms_from_damping_with_one_request_per_transition(self, conn):
        conn.mode_settle = 0.0
        conn._conn.get_mode.side_effect = [
            RobotMode.DAMPING,  # standup() reads the starting mode
            RobotMode.PREPARE,  # PREPARE confirmed
            RobotMode.WALKING,  # WALKING confirmed
        ]
        assert conn.standup() is True
        requested = [call.args[0] for call in conn._conn.change_mode.call_args_list]
        assert requested == [RobotMode.PREPARE, RobotMode.WALKING]  # exactly one each

    def test_settles_after_each_confirmed_transition(self, conn):
        # The mode flag leads the physical motion: the next request must wait it out.
        conn.mode_settle = 0.1
        conn._conn.get_mode.side_effect = [RobotMode.PREPARE, RobotMode.WALKING]
        start = time.perf_counter()
        assert conn._arm(RobotMode.DAMPING) is True
        assert time.perf_counter() - start >= 0.2  # one settle per transition

    def test_fails_when_mode_is_never_confirmed(self, conn):
        conn.mode_transition_timeout = 0.2
        conn._conn.get_mode.return_value = RobotMode.PREPARE  # never reaches WALKING
        assert conn.standup() is False
        conn._conn.change_mode.assert_called_once_with(RobotMode.WALKING)

    def test_transport_error_returns_false_instead_of_raising(self, conn):
        conn._conn.get_mode.side_effect = grpc.RpcError("transient")
        assert conn.standup() is False

    def test_vendor_operation_failure_returns_false(self, conn):
        # The SDK raises RuntimeError when the robot reports OPERATION_FAIL.
        conn._conn.get_mode.return_value = RobotMode.PREPARE
        conn._conn.change_mode.side_effect = RuntimeError("OPERATION_FAIL")
        assert conn.standup() is False

    def test_sit_returns_false_on_vendor_failure(self, conn):
        conn._conn.call.side_effect = RuntimeError("OPERATION_FAIL")
        assert conn.sit() is False


class TestWalkEnvelope:
    def test_clamp_limits_both_signs(self):
        assert _clamp(2.0, WALK_MAX_LINEAR_MPS) == WALK_MAX_LINEAR_MPS
        assert _clamp(-2.0, WALK_MAX_LINEAR_MPS) == -WALK_MAX_LINEAR_MPS
        assert _clamp(0.3, WALK_MAX_LINEAR_MPS) == 0.3

    def test_linear_clamp_bounds_the_vector_magnitude(self):
        x, y = _clamp_linear(0.5, 0.5, WALK_MAX_LINEAR_MPS)
        assert (x**2 + y**2) ** 0.5 == pytest.approx(WALK_MAX_LINEAR_MPS)
        assert _clamp_linear(0.1, 0.2, WALK_MAX_LINEAR_MPS) == (0.1, 0.2)


@pytest.fixture
def make_k1(request):
    """Builds a real K1Connection with the transport mocked, torn down after the test."""

    def build(send_failed=False, stop_confirmed=True):
        conn = MagicMock()
        conn.send_failed = send_failed
        conn.confirm_stop.return_value = stop_confirmed
        with patch("dimos.robot.booster.k1.connection.make_connection", return_value=conn):
            k1 = K1Connection(ip="mock")
        request.addfinalizer(k1._close_module)
        return k1, conn

    return build


class TestWalkSkill:
    pass

    def test_rejects_non_finite_input(self, make_k1):
        k1, conn = make_k1()
        assert "finite" in k1.walk(x=float("nan"), duration=1.0)
        conn.move.assert_not_called()

    def test_walks_then_confirms_the_stop(self, make_k1):
        k1, conn = make_k1()
        result = k1.walk(x=0.3, duration=0.15)
        assert "then stopped" in result
        conn.confirm_stop.assert_called_once()
        last_twist = conn.move.call_args_list[-1].args[0]
        assert (last_twist.linear.x, last_twist.angular.z) == (0.0, 0.0)

    def test_clamps_the_commanded_velocity(self, make_k1):
        k1, conn = make_k1()
        k1.walk(x=5.0, duration=0.15)
        first_twist = conn.move.call_args_list[0].args[0]
        assert first_twist.linear.x == WALK_MAX_LINEAR_MPS

    def test_caps_duration(self, make_k1, monkeypatch):
        monkeypatch.setattr("dimos.robot.booster.k1.connection.WALK_MAX_DURATION_S", 0.15)
        k1, conn = make_k1()
        start = time.perf_counter()
        result = k1.walk(x=0.2, duration=60.0)
        assert time.perf_counter() - start < 2.0
        assert "0.15s" in result

    def test_rejected_sends_still_stop_and_report(self, make_k1):
        k1, conn = make_k1(send_failed=True)
        result = k1.walk(x=0.3, duration=1.0)
        assert "rejected" in result
        last_twist = conn.move.call_args_list[-1].args[0]
        assert last_twist.linear.x == 0.0
        conn.confirm_stop.assert_called_once()

    def test_unconfirmed_stop_is_reported(self, make_k1):
        k1, conn = make_k1(stop_confirmed=False)
        result = k1.walk(x=0.3, duration=0.15)
        assert "could not confirm the stop" in result

    def test_is_armed_only_in_walking(self, conn):
        conn._conn.get_mode.return_value = RobotMode.WALKING
        assert conn.is_armed() is True
        conn._conn.get_mode.return_value = RobotMode.DAMPING
        assert conn.is_armed() is False

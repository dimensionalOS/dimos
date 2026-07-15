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

"""Unit tests for Go2CommandModule's operator-command handling.

No robot / no WebRTC: the command logic is exercised with a small harness that
initializes the command-plane fields, a mocked ``go2`` RPC ref, and only the
streams the tested methods touch. Covers the safety-relevant paths — sport
allow-list, E-STOP latch + fence, nonce dedup, and the drive guard
(stale/future/reorder).
"""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.teleop.hosted.command_executor import SerializedCommandExecutor
from dimos.teleop.hosted.go2_command import ALLOWED_SPORT_CMDS, Go2CommandModule

_live_modules: list[_CommandHarness] = []


@pytest.fixture(autouse=True)
def _reap_cmd_executors():
    yield
    while _live_modules:
        _live_modules.pop()._cmd.stop()
    deadline = time.time() + 2.0
    while time.time() < deadline and any(
        t.name.startswith("HostedCmd-") for t in threading.enumerate()
    ):
        time.sleep(0.005)


class _CommandHarness(Go2CommandModule):
    """A Go2CommandModule harness with just the command-path dependencies."""

    def __init__(self) -> None:
        self.go2 = MagicMock()
        self.config = SimpleNamespace(
            cmd_stale_after_sec=0.5,
            damp_on_operator_lost=False,
            max_nav_goal_m=100.0,
            allow_acrobatics=False,
            max_linear_mps=1.5,
            max_angular_rps=2.0,
        )
        self._estopped = False
        self._cmd = SerializedCommandExecutor(
            lambda nonce, ok: self._send_ack(nonce, ok), lambda: self._estopped
        )
        self._cmd.start()
        self._rage_active = False
        self._obstacle_avoidance = True
        self._light = 0.0
        self._posture = "StandReady"
        self._last_cmd_ts = 0.0
        self._last_cmd_nonzero = False
        self.cmd_ack = MagicMock()
        self.tele_cmd_vel = MagicMock()
        self.robot_state = MagicMock()
        self.goal_request = MagicMock()
        self.stop_movement = MagicMock()


@pytest.fixture
def m() -> Go2CommandModule:
    """A Go2CommandModule with only the fields the command paths need.

    Registered in ``_live_modules`` so ``_reap_cmd_executors`` stops its worker.
    """
    module = _CommandHarness()
    _live_modules.append(module)
    return module


def _twist(ts: float, *, vx: float = 0.3) -> Any:
    """A drive frame at time ``ts``. Defaults to moving (vx=0.3); pass vx=0 for
    an idle-joystick frame."""
    return SimpleNamespace(
        ts=ts, linear=SimpleNamespace(x=vx, y=0, z=0), angular=SimpleNamespace(x=0, y=0, z=0)
    )


def _wait_for(cond, timeout: float = 2.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cond():
            return
        time.sleep(0.005)


# ─── sport allow-list (RPC to driver) ────────────────────────────────

_NON_ACROBATIC = [n for n in ALLOWED_SPORT_CMDS if n not in ("FrontJump", "FrontPounce")]


@pytest.mark.parametrize("name", _NON_ACROBATIC)
def test_allowed_sport_cmd_calls_driver_rpc(
    m: Go2CommandModule, name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))
    m.go2.sport_command.return_value = True

    m._handle_sport_cmd({"name": name, "nonce": 7})
    _wait_for(lambda: bool(acks))

    m.go2.sport_command.assert_called_once_with(ALLOWED_SPORT_CMDS[name])
    assert acks == [(7, True)]


@pytest.mark.parametrize("name", ["Backflip", "", None, 1013])
def test_disallowed_sport_cmd_rejected(
    m: Go2CommandModule, name: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    m._handle_sport_cmd({"name": name, "nonce": 9})

    m.go2.sport_command.assert_not_called()
    assert acks == [(9, False)]


@pytest.mark.parametrize("name", ["FrontJump", "FrontPounce"])
def test_acrobatics_blocked_by_default(
    m: Go2CommandModule, name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    m._handle_sport_cmd({"name": name, "nonce": 3})

    assert acks == [(3, False)]
    m.go2.sport_command.assert_not_called()


# ─── drive guard (stream filter → tele_cmd_vel) ───────────────────────


def test_drive_drops_stale(m: Go2CommandModule) -> None:
    m._on_cmd_vel_in(_twist(time.time() - 1.0))
    m.tele_cmd_vel.publish.assert_not_called()


def test_drive_drops_future(m: Go2CommandModule) -> None:
    m._on_cmd_vel_in(_twist(time.time() + 5.0))
    m.tele_cmd_vel.publish.assert_not_called()
    assert m._last_cmd_ts == 0.0  # future stamp must not poison the guard


def test_drive_drops_in_window_future_without_poisoning_guard(m: Go2CommandModule) -> None:
    # A future stamp SMALLER than cmd_stale_after_sec (clock skew) must still be
    # rejected and must NOT advance _last_cmd_ts — otherwise every subsequent
    # in-order frame would be dropped as out-of-order until wall-clock catches
    # up, stalling drive. Regression guard for the in-window future case.
    m._on_cmd_vel_in(_twist(time.time() + 0.2))  # +0.2s < 0.5s stale window
    m.tele_cmd_vel.publish.assert_not_called()
    assert m._last_cmd_ts == 0.0
    # a normal fresh frame right after must still be forwarded
    m._on_cmd_vel_in(_twist(time.time()))
    m.tele_cmd_vel.publish.assert_called_once()


def test_drive_drops_out_of_order(m: Go2CommandModule) -> None:
    m._last_cmd_ts = time.time()
    m._on_cmd_vel_in(_twist(m._last_cmd_ts - 0.1))
    m.tele_cmd_vel.publish.assert_not_called()


def test_drive_forwards_fresh(m: Go2CommandModule) -> None:
    ts = time.time()
    m._on_cmd_vel_in(_twist(ts))
    m.tele_cmd_vel.publish.assert_called_once()
    assert m._last_cmd_ts == ts


def test_drive_suppresses_idle_zero_stream(m: Go2CommandModule) -> None:
    # Idle-joystick zeros must NOT be forwarded — MovementManager treats any
    # tele_cmd_vel as active manual drive and would cancel the nav plan.
    # Stamps in the recent past (transit delay), monotonically increasing.
    base = time.time() - 0.1
    m._on_cmd_vel_in(_twist(base, vx=0.0))
    m._on_cmd_vel_in(_twist(base + 0.01, vx=0.0))
    m.tele_cmd_vel.publish.assert_not_called()


def test_drive_forwards_release_edge_zero(m: Go2CommandModule) -> None:
    # A zero right after a moving frame IS forwarded (manual stop), then the
    # idle stream goes quiet again. Stamps in the recent past, increasing.
    base = time.time() - 0.1
    m._on_cmd_vel_in(_twist(base, vx=0.3))  # moving → forwarded
    m._on_cmd_vel_in(_twist(base + 0.01, vx=0.0))  # release edge → forwarded (stop)
    m._on_cmd_vel_in(_twist(base + 0.02, vx=0.0))  # idle → suppressed
    assert m.tele_cmd_vel.publish.call_count == 2


def test_estopped_drive_is_dropped(m: Go2CommandModule) -> None:
    m._estopped = True
    m._on_cmd_vel_in(_twist(time.time()))
    m.tele_cmd_vel.publish.assert_not_called()


def test_drive_drops_nan_timestamp(m: Go2CommandModule) -> None:
    # A NaN ts passes every comparison and would poison _last_cmd_ts (ts <= NaN
    # is False forever → reorder guard permanently disabled). Must be rejected.
    m._on_cmd_vel_in(_twist(float("nan")))
    m.tele_cmd_vel.publish.assert_not_called()
    assert m._last_cmd_ts == 0.0  # guard not poisoned


def test_drive_drops_non_finite_velocity(m: Go2CommandModule) -> None:
    t = _twist(time.time())
    t.linear.x = float("inf")
    m._on_cmd_vel_in(t)
    m.tele_cmd_vel.publish.assert_not_called()


def test_drive_clamps_excessive_velocity(m: Go2CommandModule) -> None:
    # An untrusted operator sending huge velocities is clamped to the envelope.
    t = _twist(time.time())
    t.linear.x = 99.0  # way over max_linear_mps=1.5
    t.angular.z = -50.0  # way under -max_angular_rps=2.0
    m._on_cmd_vel_in(t)
    published = m.tele_cmd_vel.publish.call_args[0][0]
    assert published.linear.x == 1.5
    assert published.angular.z == -2.0


# ─── E-STOP + fence ──────────────────────────────────────────────────


def test_estop_latches_and_damps(m: Go2CommandModule, monkeypatch: pytest.MonkeyPatch) -> None:
    m.go2.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    m._on_state_json(b'{"type": "estop", "nonce": 1}')
    assert m._estopped is True
    m.stop_movement.publish.assert_called_once()  # nav cancelled
    # robot_state published immediately on latch (not only inside the Damp task),
    # so the UI shows estopped:true even if Damp is slow/fails.
    m.robot_state.publish.assert_called()
    _wait_for(lambda: (1, True) in acks)
    m.go2.sport_command.assert_called_with(ALLOWED_SPORT_CMDS["Damp"])


def test_estop_clear_publishes_state(m: Go2CommandModule, monkeypatch: pytest.MonkeyPatch) -> None:
    # Clearing E-STOP must publish robot_state immediately so the UI drops
    # estopped:true without waiting for an unrelated update.
    m._estopped = True
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: None)
    m.robot_state.publish.reset_mock()
    m._on_state_json(b'{"type": "estop_clear", "nonce": 2}')
    assert m._estopped is False
    m.robot_state.publish.assert_called_once()


def test_repeated_estop_reissues_damp(m: Go2CommandModule, monkeypatch: pytest.MonkeyPatch) -> None:
    m.go2.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    m._on_state_json(b'{"type": "estop", "nonce": 1}')
    _wait_for(lambda: m.go2.sport_command.call_count == 1)
    m._on_state_json(b'{"type": "estop", "nonce": 1}')  # retransmit, same nonce
    _wait_for(lambda: m.go2.sport_command.call_count == 2)

    assert acks.count((1, True)) == 2


def test_estop_clear_cancels_plan_and_rearms(
    m: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    m._estopped = True
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: None)

    m._on_state_json(b'{"type": "estop_clear"}')

    assert m._estopped is False
    m.stop_movement.publish.assert_called_once()  # active plan cancelled


def test_nav_cancel_stops_planner(m: Go2CommandModule, monkeypatch: pytest.MonkeyPatch) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    m._on_state_json(b'{"type": "nav_cancel", "nonce": 22}')

    (msg,) = m.stop_movement.publish.call_args.args
    assert msg.data is True
    assert acks == [(22, True)]


def test_stand_ready_aborts_on_mid_sequence_estop(
    m: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    m._posture = "Sit"
    m.go2.standup.return_value = True
    m.go2.sport_command.return_value = True
    m.go2.balance_stand.return_value = True
    m.go2.switch_joystick.return_value = True

    def fake_sleep(_s: float) -> None:
        if not m._estopped:
            m._estopped = True
            m._cmd.bump_safety_epoch()

    monkeypatch.setattr(time, "sleep", fake_sleep)

    assert m._stand_ready_task(m._cmd.safety_epoch) is False
    m.go2.standup.assert_called_once()
    m.go2.balance_stand.assert_not_called()
    m.go2.switch_joystick.assert_not_called()
    assert m._posture == "Sit"


# ─── nonce dedup ─────────────────────────────────────────────────────


def test_duplicate_nonce_reacks_without_reexecution(
    m: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    m.go2.set_light.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    m._handle_light({"brightness": 1.0, "nonce": 5})
    _wait_for(lambda: (5, True) in acks)
    m._handle_light({"brightness": 1.0, "nonce": 5})  # duplicate
    _wait_for(lambda: acks.count((5, True)) == 2)

    m.go2.set_light.assert_called_once()  # executed once, re-acked


# ─── nav goal ────────────────────────────────────────────────────────


def test_nav_goal_publishes_and_acks(m: Go2CommandModule, monkeypatch: pytest.MonkeyPatch) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    m._handle_nav_goal({"x": 2.5, "y": -1.0, "nonce": 11})

    (pose,) = m.goal_request.publish.call_args.args
    assert pose.position.x == pytest.approx(2.5)
    assert acks == [(11, True)]


def test_nav_goal_rejected_when_estopped(
    m: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    m._estopped = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(m, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    m._handle_nav_goal({"x": 1, "y": 1, "nonce": 13})

    m.goal_request.publish.assert_not_called()
    assert acks == [(13, False)]

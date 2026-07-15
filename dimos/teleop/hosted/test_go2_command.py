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

from collections.abc import Iterator
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.core.module import Module
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.hosted.go2_command import ALLOWED_SPORT_CMDS, Go2CommandModule
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> Iterator[Go2CommandModule]:
    """A Go2CommandModule with the command-plane state initialized for real
    (only the framework Module.__init__ is skipped) and its ports / driver ref /
    config mocked. The command executor is stopped (worker joined) on teardown."""
    monkeypatch.setattr(Module, "__init__", lambda self, **kwargs: None)
    module = Go2CommandModule()
    module.go2 = MagicMock()
    module.config = SimpleNamespace(
        cmd_stale_after_sec=0.5,
        damp_on_operator_lost=False,
        max_nav_goal_m=100.0,
        allow_acrobatics=False,
        max_linear_mps=1.5,
        max_angular_rps=2.0,
    )
    for port in ("cmd_ack", "tele_cmd_vel", "robot_state", "goal_request", "stop_movement"):
        setattr(module, port, MagicMock())
    module._cmd.start()
    yield module
    module._cmd.stop()


def _twist(ts: float, *, vx: float = 0.3) -> TwistStamped:
    """A drive frame at time ``ts`` (vx=0.3 moving, vx=0 idle-joystick)."""
    t = TwistStamped(ts=ts, linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0])
    t.linear.x = vx
    return t


# ─── sport allow-list (RPC to driver) ────────────────────────────────

_NON_ACROBATIC = [n for n in ALLOWED_SPORT_CMDS if n not in ("FrontJump", "FrontPounce")]


@pytest.mark.parametrize("name", _NON_ACROBATIC)
def test_allowed_sport_cmd_calls_driver_rpc(
    module: Go2CommandModule, name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))
    module.go2.sport_command.return_value = True

    module._handle_sport_cmd({"name": name, "nonce": 7})
    wait_until(lambda: bool(acks), timeout=2.0)

    module.go2.sport_command.assert_called_once_with(ALLOWED_SPORT_CMDS[name])
    assert acks == [(7, True)]


@pytest.mark.parametrize("name", ["Backflip", "", None, 1013])
def test_disallowed_sport_cmd_rejected(
    module: Go2CommandModule, name: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_sport_cmd({"name": name, "nonce": 9})

    module.go2.sport_command.assert_not_called()
    assert acks == [(9, False)]


@pytest.mark.parametrize("name", ["FrontJump", "FrontPounce"])
def test_acrobatics_blocked_by_default(
    module: Go2CommandModule, name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_sport_cmd({"name": name, "nonce": 3})

    assert acks == [(3, False)]
    module.go2.sport_command.assert_not_called()


# ─── drive guard (stream filter → tele_cmd_vel) ───────────────────────


def test_drive_drops_stale(module: Go2CommandModule) -> None:
    module._on_cmd_vel_in(_twist(time.time() - 1.0))
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_drops_future(module: Go2CommandModule) -> None:
    module._on_cmd_vel_in(_twist(time.time() + 5.0))
    module.tele_cmd_vel.publish.assert_not_called()
    assert module._last_cmd_ts == 0.0  # future stamp must not poison the guard


def test_drive_drops_in_window_future_without_poisoning_guard(module: Go2CommandModule) -> None:
    # A future stamp SMALLER than cmd_stale_after_sec (clock skew) must still be
    # rejected and must NOT advance _last_cmd_ts — otherwise every subsequent
    # in-order frame would be dropped as out-of-order until wall-clock catches
    # up, stalling drive. Regression guard for the in-window future case.
    module._on_cmd_vel_in(_twist(time.time() + 0.2))  # +0.2s < 0.5s stale window
    module.tele_cmd_vel.publish.assert_not_called()
    assert module._last_cmd_ts == 0.0
    # a normal fresh frame right after must still be forwarded
    module._on_cmd_vel_in(_twist(time.time()))
    module.tele_cmd_vel.publish.assert_called_once()


def test_drive_drops_out_of_order(module: Go2CommandModule) -> None:
    module._last_cmd_ts = time.time()
    module._on_cmd_vel_in(_twist(module._last_cmd_ts - 0.1))
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_forwards_fresh(module: Go2CommandModule) -> None:
    ts = time.time()
    module._on_cmd_vel_in(_twist(ts))
    module.tele_cmd_vel.publish.assert_called_once()
    assert module._last_cmd_ts == ts


def test_drive_suppresses_idle_zero_stream(module: Go2CommandModule) -> None:
    # Idle-joystick zeros must NOT be forwarded — MovementManager treats any
    # tele_cmd_vel as active manual drive and would cancel the nav plan.
    # Stamps in the recent past (transit delay), monotonically increasing.
    base = time.time() - 0.1
    module._on_cmd_vel_in(_twist(base, vx=0.0))
    module._on_cmd_vel_in(_twist(base + 0.01, vx=0.0))
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_forwards_release_edge_zero(module: Go2CommandModule) -> None:
    # A zero right after a moving frame IS forwarded (manual stop), then the
    # idle stream goes quiet again. Stamps in the recent past, increasing.
    base = time.time() - 0.1
    module._on_cmd_vel_in(_twist(base, vx=0.3))  # moving → forwarded
    module._on_cmd_vel_in(_twist(base + 0.01, vx=0.0))  # release edge → forwarded (stop)
    module._on_cmd_vel_in(_twist(base + 0.02, vx=0.0))  # idle → suppressed
    assert module.tele_cmd_vel.publish.call_count == 2


def test_estopped_drive_is_dropped(module: Go2CommandModule) -> None:
    module._estopped = True
    module._on_cmd_vel_in(_twist(time.time()))
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_drops_nan_timestamp(module: Go2CommandModule) -> None:
    # A NaN ts passes every comparison and would poison _last_cmd_ts (ts <= NaN
    # is False forever → reorder guard permanently disabled). Must be rejected.
    module._on_cmd_vel_in(_twist(float("nan")))
    module.tele_cmd_vel.publish.assert_not_called()
    assert module._last_cmd_ts == 0.0  # guard not poisoned


def test_drive_drops_non_finite_velocity(module: Go2CommandModule) -> None:
    t = _twist(time.time())
    t.linear.x = float("inf")
    module._on_cmd_vel_in(t)
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_clamps_excessive_velocity(module: Go2CommandModule) -> None:
    # An untrusted operator sending huge velocities is clamped to the envelope.
    t = _twist(time.time())
    t.linear.x = 99.0  # way over max_linear_mps=1.5
    t.angular.z = -50.0  # way under -max_angular_rps=2.0
    module._on_cmd_vel_in(t)
    published = module.tele_cmd_vel.publish.call_args[0][0]
    assert published.linear.x == 1.5
    assert published.angular.z == -2.0


# ─── E-STOP + fence ──────────────────────────────────────────────────


def test_estop_latches_and_damps(module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch) -> None:
    module.go2.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    assert module._estopped is True
    module.stop_movement.publish.assert_called_once()  # nav cancelled
    # robot_state published immediately on latch (not only inside the Damp task),
    # so the UI shows estopped:true even if Damp is slow/fails.
    module.robot_state.publish.assert_called()
    wait_until(lambda: (1, True) in acks, timeout=2.0)
    module.go2.sport_command.assert_called_with(ALLOWED_SPORT_CMDS["Damp"])


def test_estop_clear_publishes_state(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Clearing E-STOP must publish robot_state immediately so the UI drops
    # estopped:true without waiting for an unrelated update.
    module._estopped = True
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: None)
    module.robot_state.publish.reset_mock()
    module._on_state_json(b'{"type": "estop_clear", "nonce": 2}')
    assert module._estopped is False
    module.robot_state.publish.assert_called_once()


def test_repeated_estop_reissues_damp(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module.go2.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    wait_until(lambda: module.go2.sport_command.call_count == 1, timeout=2.0)
    module._on_state_json(b'{"type": "estop", "nonce": 1}')  # retransmit, same nonce
    wait_until(lambda: module.go2.sport_command.call_count == 2, timeout=2.0)

    assert acks.count((1, True)) == 2


def test_estop_clear_cancels_plan_and_rearms(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._estopped = True
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: None)

    module._on_state_json(b'{"type": "estop_clear"}')

    assert module._estopped is False
    module.stop_movement.publish.assert_called_once()  # active plan cancelled


def test_nav_cancel_stops_planner(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._on_state_json(b'{"type": "nav_cancel", "nonce": 22}')

    (msg,) = module.stop_movement.publish.call_args.args
    assert msg.data is True
    assert acks == [(22, True)]


def test_stand_ready_aborts_on_mid_sequence_estop(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._posture = "Sit"
    module.go2.standup.return_value = True
    module.go2.sport_command.return_value = True
    module.go2.balance_stand.return_value = True
    module.go2.switch_joystick.return_value = True

    def fake_sleep(_s: float) -> None:
        if not module._estopped:
            module._estopped = True
            module._cmd.bump_safety_epoch()

    monkeypatch.setattr(time, "sleep", fake_sleep)

    assert module._stand_ready_task(module._cmd.safety_epoch) is False
    module.go2.standup.assert_called_once()
    module.go2.balance_stand.assert_not_called()
    module.go2.switch_joystick.assert_not_called()
    assert module._posture == "Sit"


# ─── nonce dedup ─────────────────────────────────────────────────────


def test_duplicate_nonce_reacks_without_reexecution(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module.go2.set_light.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_light({"brightness": 1.0, "nonce": 5})
    wait_until(lambda: (5, True) in acks, timeout=2.0)
    module._handle_light({"brightness": 1.0, "nonce": 5})  # duplicate
    wait_until(lambda: acks.count((5, True)) == 2, timeout=2.0)

    module.go2.set_light.assert_called_once()  # executed once, re-acked


# ─── nav goal ────────────────────────────────────────────────────────


def test_nav_goal_publishes_and_acks(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_nav_goal({"x": 2.5, "y": -1.0, "nonce": 11})

    (pose,) = module.goal_request.publish.call_args.args
    assert pose.position.x == pytest.approx(2.5)
    assert acks == [(11, True)]


def test_nav_goal_rejected_when_estopped(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._estopped = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_nav_goal({"x": 1, "y": 1, "nonce": 13})

    module.goal_request.publish.assert_not_called()
    assert acks == [(13, False)]

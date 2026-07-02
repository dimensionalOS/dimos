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

"""Unit tests for Go2HostedConnection's operator-command handling.

No robot / no WebRTC: ``Go2HostedConnection.__init__`` builds a whole Module,
so we exercise the pure command logic on a bare instance (``object.__new__``)
with a mocked ``connection`` and only the attributes the tested methods touch.
Covers the security-relevant paths — the sport-command allow-list and the
stale / out-of-order cmd_vel drop on the unreliable wire.
"""

from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.robot.unitree.go2.hosted_connection import (
    ALLOWED_SPORT_CMDS,
    Go2HostedConnection,
)


# Executors created by _bare_connection, reaped after each test so the
# repo-wide non-closed-thread check stays green.
_live_executors: list[ThreadPoolExecutor] = []


@pytest.fixture(autouse=True)
def _reap_cmd_executors():
    yield
    while _live_executors:
        _live_executors.pop().shutdown(wait=True)
    # Urgent (Damp/E-STOP) runners are plain threads — give them a beat to die.
    deadline = time.time() + 2.0
    while time.time() < deadline and any(
        t.name.startswith("Go2Cmd-") for t in threading.enumerate()
    ):
        time.sleep(0.005)


def _bare_connection() -> Go2HostedConnection:
    """A Go2HostedConnection with only the fields the command paths need."""
    conn = object.__new__(Go2HostedConnection)
    conn.connection = MagicMock()
    conn._last_cmd_ts = 0.0
    conn.config = SimpleNamespace(cmd_stale_after_sec=0.5)
    # Command execution plane (normally built in start()).
    conn._cmd_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="Go2CmdTest")
    conn._cmd_pending = 0
    conn._cmd_lock = threading.Lock()
    conn._rage_active = False
    _live_executors.append(conn._cmd_executor)
    return conn


def _twist(ts: float) -> Any:
    return SimpleNamespace(
        ts=ts, linear=SimpleNamespace(x=0, y=0, z=0), angular=SimpleNamespace(x=0, y=0, z=0)
    )


# ─── sport-command allow-list ────────────────────────────────────────


@pytest.mark.parametrize("name", list(ALLOWED_SPORT_CMDS))
def test_allowed_sport_cmd_dispatched(name: str, monkeypatch: pytest.MonkeyPatch) -> None:
    """Every allow-listed name maps to its api_id and calls sport_command."""
    conn = _bare_connection()
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))
    conn.connection.sport_command.return_value = True

    conn._handle_sport_cmd({"name": name, "nonce": 7})
    # runs on a worker thread — wait for the ack instead of sleeping blindly.
    for _ in range(200):
        if acks:
            break
        time.sleep(0.005)

    conn.connection.sport_command.assert_called_once_with(ALLOWED_SPORT_CMDS[name])
    assert acks == [(7, True)]


@pytest.mark.parametrize("name", ["Backflip", "", "sport_command", None, 1013])
def test_disallowed_sport_cmd_rejected(name: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    """Unknown / non-allow-listed names are rejected with ok=False, no call."""
    conn = _bare_connection()
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": name, "nonce": 9})

    conn.connection.sport_command.assert_not_called()
    assert acks == [(9, False)]


def test_standready_is_not_a_raw_sport_command(monkeypatch: pytest.MonkeyPatch) -> None:
    """StandReady routes to the combo task, never a single sport_command call."""
    conn = _bare_connection()
    called: list[bool] = []
    monkeypatch.setattr(conn, "_stand_ready_task", lambda: called.append(True) or True)
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": "StandReady", "nonce": 3})
    _wait_ack(acks)

    assert called == [True]
    assert acks == [(3, True)]
    conn.connection.sport_command.assert_not_called()


# ─── cmd_vel stale / out-of-order drop ───────────────────────────────


def test_move_drops_stale_cmd() -> None:
    """A twist older than cmd_stale_after_sec is dropped, robot not moved."""
    conn = _bare_connection()
    old = time.time() - 1.0  # > 0.5s stale threshold
    assert conn.move(_twist(old)) is False


def test_move_drops_out_of_order_cmd() -> None:
    """A twist with ts <= the newest seen is dropped (reorder guard)."""
    conn = _bare_connection()
    now = time.time()
    conn._last_cmd_ts = now
    assert conn.move(_twist(now)) is False  # equal → drop
    assert conn.move(_twist(now - 0.1)) is False  # older → drop


def test_move_accepts_fresh_in_order_cmd() -> None:
    """A fresh, newer twist is forwarded and advances _last_cmd_ts."""
    conn = _bare_connection()
    conn.connection.move.return_value = True
    ts = time.time()

    assert conn.move(_twist(ts)) is True
    assert conn._last_cmd_ts == ts
    conn.connection.move.assert_called_once()


# ─── speed-mode / rage toggle ────────────────────────────────────────


def _wait_ack(acks: list[Any]) -> None:
    for _ in range(200):
        if acks:
            return
        time.sleep(0.005)


def test_set_mode_unknown_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    conn._rage_active = False
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_set_mode({"mode": "ludicrous", "nonce": 1})

    assert acks == [(1, False)]
    conn.connection.set_rage_mode.assert_not_called()


@pytest.mark.parametrize("mode", ["normal", "high"])
def test_set_mode_non_rage_does_not_touch_firmware(
    mode: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    """normal/high are browser-side scale only — acked, no firmware toggle."""
    conn = _bare_connection()
    conn._rage_active = False
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_set_mode({"mode": mode, "nonce": 2})
    _wait_ack(acks)

    assert acks == [(2, True)]
    conn.connection.set_rage_mode.assert_not_called()


def test_set_mode_rage_boundary_toggles_firmware(monkeypatch: pytest.MonkeyPatch) -> None:
    """Crossing into rage calls set_rage_mode(True) and flips _rage_active."""
    conn = _bare_connection()
    conn._rage_active = False
    conn.connection.set_rage_mode.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_set_mode({"mode": "rage", "nonce": 4})
    _wait_ack(acks)

    conn.connection.set_rage_mode.assert_called_once_with(True)
    assert conn._rage_active is True
    assert acks == [(4, True)]


def test_set_mode_already_in_rage_short_circuits(monkeypatch: pytest.MonkeyPatch) -> None:
    """Re-selecting rage while already active acks without a firmware call."""
    conn = _bare_connection()
    conn._rage_active = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_set_mode({"mode": "rage", "nonce": 5})
    _wait_ack(acks)

    assert acks == [(5, True)]
    conn.connection.set_rage_mode.assert_not_called()


# ─── command execution plane: ordering, backlog bound, urgent bypass ─


def _wait_for(cond, timeout: float = 2.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cond():
            return
        time.sleep(0.005)


def test_rapid_rage_toggles_execute_in_order(monkeypatch: pytest.MonkeyPatch) -> None:
    """Serialized worker: rage on→off lands in order, no interleaved race."""
    conn = _bare_connection()
    calls: list[bool] = []
    conn.connection.set_rage_mode.side_effect = lambda v: calls.append(v) or True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_set_mode({"mode": "rage", "nonce": 1})
    conn._handle_set_mode({"mode": "normal", "nonce": 2})
    _wait_for(lambda: len(acks) == 2)

    assert calls == [True, False]  # both toggles fired, in submit order
    assert conn._rage_active is False
    assert sorted(acks) == [(1, True), (2, True)]


def test_backlog_past_max_pending_is_busy_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    """Queued commands beyond _MAX_PENDING_CMDS ack ok=False immediately."""
    conn = _bare_connection()
    release = threading.Event()
    conn.connection.sport_command.side_effect = lambda _id: release.wait(2.0) or True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    total = conn._MAX_PENDING_CMDS + 3
    for i in range(total):
        conn._handle_sport_cmd({"name": "Hello", "nonce": i})

    # Overflow rejections ack synchronously while the first command blocks.
    rejected = [a for a in acks if a[1] is False]
    assert len(rejected) == total - conn._MAX_PENDING_CMDS

    release.set()
    _wait_for(lambda: len(acks) == total)
    accepted = [a for a in acks if a[1] is True]
    assert len(accepted) == conn._MAX_PENDING_CMDS


def test_damp_bypasses_busy_queue(monkeypatch: pytest.MonkeyPatch) -> None:
    """Damp (E-STOP) runs urgently even while the worker is wedged."""
    conn = _bare_connection()
    wedge = threading.Event()
    damped = threading.Event()

    def fake_sport(api_id: int) -> bool:
        if api_id == ALLOWED_SPORT_CMDS["Damp"]:
            damped.set()
            return True
        wedge.wait(5.0)  # Hello holds the single worker
        return True

    conn.connection.sport_command.side_effect = fake_sport
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": "Hello", "nonce": 1})  # occupies the worker
    conn._handle_sport_cmd({"name": "Damp", "nonce": 2})

    assert damped.wait(1.0), "Damp must not queue behind a blocking command"
    wedge.set()
    _wait_for(lambda: len(acks) == 2)

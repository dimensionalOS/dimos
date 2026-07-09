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

from concurrent.futures import ThreadPoolExecutor
import threading
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.teleop.quest_hosted.go2_hosted_connection import (
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
    conn._hosted_init(["cam1", "cam2"])  # mux + cmd stats + E-STOP latch
    conn._last_cmd_ts = 0.0
    conn._posture = "StandReady"
    conn._obstacle_avoidance = True
    conn._light = 0.0
    conn.mux_image = MagicMock()
    conn._cmd_stats = SimpleNamespace(snapshot=lambda: None)
    conn.config = SimpleNamespace(
        cmd_stale_after_sec=0.5,
        damp_on_operator_lost=False,
        latency_stamp=False,
        video_max_width=0,
        video_max_fps=0.0,
        map_hz=2.0,
        map_min_resolution=0.1,
        odom_hz=15.0,
        nav_yield_sec=1.0,
        max_nav_goal_m=100.0,
        allow_acrobatics=False,
    )
    conn._last_map_pub = 0.0
    conn._last_drive_ts = 0.0
    conn._last_nav_ts = 0.0
    conn._last_odom_pub = 0.0
    conn.telemetry_out = MagicMock()
    conn.map_out = MagicMock()
    conn.goal_request = MagicMock()
    conn.stop_movement = MagicMock()
    conn.audio_out = MagicMock()
    conn._speaker_track = None
    # Command execution plane (normally built in start()).
    conn._cmd_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="Go2CmdTest")
    conn._cmd_pending = 0
    conn._cmd_lock = threading.Lock()
    conn._nonce_results = {}
    conn._rage_active = False
    _live_executors.append(conn._cmd_executor)
    return conn


def _twist(ts: float) -> Any:
    return SimpleNamespace(
        ts=ts, linear=SimpleNamespace(x=0, y=0, z=0), angular=SimpleNamespace(x=0, y=0, z=0)
    )


# ─── sport-command allow-list ────────────────────────────────────────


# Acrobatics are gated separately (allow_acrobatics) — tested below.
_NON_ACROBATIC = [n for n in ALLOWED_SPORT_CMDS if n not in ("FrontJump", "FrontPounce")]


@pytest.mark.parametrize("name", _NON_ACROBATIC)
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


@pytest.mark.parametrize("name", ["FrontJump", "FrontPounce"])
def test_acrobatics_blocked_by_default(name: str, monkeypatch: pytest.MonkeyPatch) -> None:
    """Acrobatic leaps are rejected unless allow_acrobatics is set — no firmware call."""
    conn = _bare_connection()  # fixture default allow_acrobatics=False
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": name, "nonce": 3})

    assert acks == [(3, False)]
    conn.connection.sport_command.assert_not_called()


@pytest.mark.parametrize("name", ["FrontJump", "FrontPounce"])
def test_acrobatics_allowed_when_enabled(name: str, monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    conn.config.allow_acrobatics = True
    conn.connection.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": name, "nonce": 4})
    for _ in range(200):
        if acks:
            break
        time.sleep(0.005)

    conn.connection.sport_command.assert_called_once_with(ALLOWED_SPORT_CMDS[name])
    assert acks == [(4, True)]


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


def test_move_drops_future_stamped_cmd() -> None:
    """A twist stamped in the future is dropped and must NOT poison
    _last_cmd_ts — otherwise every later legitimate command reads as
    out-of-order and manual driving silently dies."""
    conn = _bare_connection()
    future = time.time() + 5.0  # > cmd_stale_after_sec ahead
    assert conn.move(_twist(future)) is False
    assert conn._last_cmd_ts == 0.0  # unchanged — future stamp did not stick
    # A normal, current command still drives afterwards.
    conn.connection.move.return_value = True
    assert conn.move(_twist(time.time())) is True


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


# ─── nonce dedup (transport/UI duplicates) ───────────────────────────


def test_duplicate_nonce_reacks_without_reexecution(monkeypatch: pytest.MonkeyPatch) -> None:
    """Same nonce twice → one execution, second gets the cached ack."""
    conn = _bare_connection()
    conn.connection.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": "Hello", "nonce": 42})
    _wait_ack(acks)
    conn._handle_sport_cmd({"name": "Hello", "nonce": 42})  # replayed frame
    _wait_for(lambda: len(acks) == 2)

    conn.connection.sport_command.assert_called_once()
    assert acks == [(42, True), (42, True)]


def test_inflight_duplicate_is_dropped(monkeypatch: pytest.MonkeyPatch) -> None:
    """Duplicate while the original is still running: no second execution,
    no second ack (the original's ack covers it)."""
    conn = _bare_connection()
    release = threading.Event()
    conn.connection.sport_command.side_effect = lambda _id: release.wait(2.0) or True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": "Hello", "nonce": 9})
    conn._handle_sport_cmd({"name": "Hello", "nonce": 9})  # dupe while running
    release.set()
    _wait_ack(acks)
    time.sleep(0.05)  # would-be second ack window

    assert conn.connection.sport_command.call_count == 1
    assert acks == [(9, True)]


def test_busy_rejection_does_not_poison_nonce(monkeypatch: pytest.MonkeyPatch) -> None:
    """A busy-rejected nonce can retry once the backlog drains."""
    conn = _bare_connection()
    release = threading.Event()
    conn.connection.sport_command.side_effect = lambda _id: release.wait(2.0) or True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    # Fill worker + queue, then one more (nonce=99) gets busy-rejected.
    for i in range(conn._MAX_PENDING_CMDS):
        conn._handle_sport_cmd({"name": "Hello", "nonce": i})
    conn._handle_sport_cmd({"name": "Hello", "nonce": 99})
    assert (99, False) in acks

    release.set()
    _wait_for(lambda: len([a for a in acks if a[1]]) == conn._MAX_PENDING_CMDS)

    # Retry of the rejected nonce now executes for real.
    conn._handle_sport_cmd({"name": "Hello", "nonce": 99})
    _wait_for(lambda: (99, True) in acks)
    assert conn.connection.sport_command.call_count == conn._MAX_PENDING_CMDS + 1


# ─── Stand/Drive combo leaves the robot drive-ready ──────────────────


def test_stand_ready_ends_drive_ready(monkeypatch: pytest.MonkeyPatch) -> None:
    """The combo must end in BalanceStand + joystick listening ON — WASD is
    wireless-controller stick emulation, dead without either. Ending in
    RecoveryStand (the old order) or leaving SwitchJoystick off (rage-off
    side effect) both silently killed drive."""
    conn = _bare_connection()
    monkeypatch.setattr(time, "sleep", lambda _s: None)

    conn.connection.standup.return_value = True
    conn.connection.sport_command.return_value = True
    conn.connection.balance_stand.return_value = True
    conn.connection.switch_joystick.return_value = True

    assert conn._stand_ready_task() is True

    names = [c[0] for c in conn.connection.method_calls]
    assert names == ["standup", "sport_command", "balance_stand", "switch_joystick"]
    conn.connection.sport_command.assert_called_once_with(ALLOWED_SPORT_CMDS["RecoveryStand"])
    conn.connection.switch_joystick.assert_called_once_with(True)


def test_stand_ready_returns_false_and_stops_on_step_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A failed step aborts the sequence and reports False (no false-success ack)."""
    conn = _bare_connection()
    conn._posture = "Sit"  # not already StandReady, so we can see it stays put
    monkeypatch.setattr(time, "sleep", lambda _s: None)
    conn.connection.standup.return_value = True
    conn.connection.sport_command.return_value = True
    conn.connection.balance_stand.return_value = False  # fails here

    assert conn._stand_ready_task() is False
    # switch_joystick is never reached, and posture is not latched to StandReady.
    conn.connection.switch_joystick.assert_not_called()
    assert conn._posture == "Sit"


# ─── telemetry state snapshot (operator UI seeding) ──────────────────


def test_telemetry_payload_carries_robot_state() -> None:
    conn = _bare_connection()
    conn._rage_active = True
    conn._obstacle_avoidance = False
    conn._cam_selected = ["cam1", "cam2"]
    conn._estopped = True
    conn._posture = "Sit"

    p = conn._telemetry_payload()

    assert p["type"] == "robot_telemetry"
    assert p["state"] == {
        "posture": "Sit",
        "rage": True,
        "obstacle_avoidance": False,
        "light": 0.0,
        "cams": ["cam1", "cam2"],
        "estopped": True,
    }
    assert "robot_ts" in p


def test_light_brightness_maps_to_firmware_level(monkeypatch: pytest.MonkeyPatch) -> None:
    """brightness 0..1 → VUI level 0-10; state tracks the 0..1 value."""
    conn = _bare_connection()
    conn.connection.set_light.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._on_state_json(b'{"type": "light", "brightness": 0.4, "nonce": 11}')
    _wait_for(lambda: (11, True) in acks)

    conn.connection.set_light.assert_called_once_with(4)
    assert conn._light == 0.4
    assert conn._telemetry_payload()["state"]["light"] == 0.4


def test_light_legacy_enabled_bool_still_works(monkeypatch: pytest.MonkeyPatch) -> None:
    """The shipped on/off toggle sends enabled — maps to full/zero brightness."""
    conn = _bare_connection()
    conn.connection.set_light.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._on_state_json(b'{"type": "light", "enabled": true, "nonce": 12}')
    _wait_for(lambda: (12, True) in acks)
    conn.connection.set_light.assert_called_with(10)
    assert conn._light == 1.0

    conn._on_state_json(b'{"type": "light", "enabled": false, "nonce": 13}')
    _wait_for(lambda: (13, True) in acks)
    conn.connection.set_light.assert_called_with(0)
    assert conn._light == 0.0


def test_light_brightness_clamped_and_validated(monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    conn.connection.set_light.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._on_state_json(b'{"type": "light", "brightness": 7, "nonce": 14}')  # >1 → clamp
    _wait_for(lambda: (14, True) in acks)
    conn.connection.set_light.assert_called_with(10)

    conn._on_state_json(b'{"type": "light", "brightness": "bogus", "nonce": 15}')
    assert (15, False) in acks  # rejected synchronously, no firmware call
    assert conn.connection.set_light.call_count == 1


def test_light_failure_keeps_state(monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    conn.connection.set_light.return_value = False
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._on_state_json(b'{"type": "light", "brightness": 1.0, "nonce": 16}')
    _wait_for(lambda: (16, False) in acks)

    assert conn._light == 0.0


def test_posture_tracks_successful_posture_commands(monkeypatch: pytest.MonkeyPatch) -> None:
    """Posture cmds update _posture on success; gestures (Hello) don't."""
    conn = _bare_connection()
    conn.connection.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": "Sit", "nonce": 1})
    _wait_for(lambda: (1, True) in acks)
    assert conn._posture == "Sit"

    conn._handle_sport_cmd({"name": "Hello", "nonce": 2})
    _wait_for(lambda: (2, True) in acks)
    assert conn._posture == "Sit"  # gesture leaves posture untouched


def test_failed_posture_command_leaves_posture(monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    conn.connection.sport_command.return_value = False
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": "StandDown", "nonce": 3})
    _wait_for(lambda: (3, False) in acks)
    assert conn._posture == "StandReady"


# ─── publish-side video caps (mux fps / width) ───────────────────────


def _img(w: int, h: int):
    import numpy as np

    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

    return Image(data=np.zeros((h, w, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="t")


def test_video_max_fps_caps_mux_publish_rate() -> None:
    conn = _bare_connection()
    conn.config.video_max_fps = 5.0  # 200ms budget — two immediate frames → one publish

    conn._on_cam("cam1", _img(64, 48))
    conn._on_cam("cam1", _img(64, 48))

    assert conn.mux_image.publish.call_count == 1


def test_video_max_fps_zero_publishes_every_frame() -> None:
    conn = _bare_connection()

    conn._on_cam("cam1", _img(64, 48))
    conn._on_cam("cam1", _img(64, 48))

    assert conn.mux_image.publish.call_count == 2


def test_video_max_width_downscales_composite() -> None:
    conn = _bare_connection()
    conn.config.video_max_width = 320

    conn._on_cam("cam1", _img(640, 480))

    out = conn.mux_image.publish.call_args[0][0]
    assert out.data.shape[1] == 320
    assert out.data.shape[0] == 240  # aspect preserved


def test_video_max_width_zero_keeps_source_resolution() -> None:
    conn = _bare_connection()

    conn._on_cam("cam1", _img(640, 480))

    out = conn.mux_image.publish.call_args[0][0]
    assert out.data.shape[:2] == (480, 640)


# ─── E-STOP latch + operator-loss safety ─────────────────────────────


def test_estop_latches_and_damps(monkeypatch: pytest.MonkeyPatch) -> None:
    """estop → immediate latch (move refused) + urgent Damp + ack."""
    conn = _bare_connection()
    conn.connection.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._on_state_json(b'{"type": "estop", "nonce": 1}')
    assert conn._estopped is True
    assert conn.move(_twist(time.time())) is False  # latched before RPC even lands
    _wait_ack(acks)
    conn.connection.sport_command.assert_called_with(ALLOWED_SPORT_CMDS["Damp"])
    assert (1, True) in acks


def test_estop_rejects_commands_until_cleared(monkeypatch: pytest.MonkeyPatch) -> None:
    """While latched: sport_cmd rejected; after estop_clear it executes."""
    conn = _bare_connection()
    conn._estopped = True
    conn.connection.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": "Hello", "nonce": 2})
    assert acks == [(2, False)]
    conn.connection.sport_command.assert_not_called()

    conn._on_state_json(b'{"type": "estop_clear", "nonce": 3}')
    assert conn._estopped is False
    assert (3, True) in acks

    conn._handle_sport_cmd({"name": "Hello", "nonce": 4})
    _wait_for(lambda: (4, True) in acks)
    conn.connection.sport_command.assert_called_once_with(ALLOWED_SPORT_CMDS["Hello"])


def test_estop_clear_does_not_move_the_robot(monkeypatch: pytest.MonkeyPatch) -> None:
    """Re-arm must not trigger motion — operator must Stand/Drive explicitly."""
    conn = _bare_connection()
    conn._estopped = True
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: None)

    conn._on_state_json(b'{"type": "estop_clear"}')

    conn.connection.sport_command.assert_not_called()
    conn.connection.standup.assert_not_called()
    conn.connection.move.assert_not_called()


def test_estop_clear_cancels_nav_and_resets_yield_timers(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Clearing E-STOP cancels the planner and resets the yield timers, so a
    pre-E-STOP nav stream can't resume driving the instant we un-latch."""
    conn = _bare_connection()
    conn._estopped = True
    conn._last_nav_ts = time.time()  # planner was driving before the E-STOP
    conn._last_drive_ts = time.time()
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: None)

    conn._on_state_json(b'{"type": "estop_clear"}')

    conn.stop_movement.publish.assert_called_once()  # nav cancelled
    assert conn._last_nav_ts == 0.0
    assert conn._last_drive_ts == 0.0


def test_repeated_estop_reissues_damp(monkeypatch: pytest.MonkeyPatch) -> None:
    """A repeated E-STOP reusing the same nonce must fire Damp again, never be
    dropped as a duplicate — browsers retransmit on the unreliable channel."""
    conn = _bare_connection()
    conn.connection.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._on_state_json(b'{"type": "estop", "nonce": 1}')
    _wait_for(lambda: conn.connection.sport_command.call_count == 1)
    # Same nonce again (retransmit / repeated press) → a second Damp, not a drop.
    conn._on_state_json(b'{"type": "estop", "nonce": 1}')
    _wait_for(lambda: conn.connection.sport_command.call_count == 2)

    assert conn.connection.sport_command.call_args_list == [
        (((ALLOWED_SPORT_CMDS["Damp"],)),),
        (((ALLOWED_SPORT_CMDS["Damp"],)),),
    ]
    assert acks.count((1, True)) == 2


def test_operator_lost_stops_motion_and_clears_nonces(monkeypatch: pytest.MonkeyPatch) -> None:
    """Provider-injected operator_lost → stop_movement + nonce cache reset."""
    conn = _bare_connection()
    conn._nonce_results = {7: (True, time.monotonic())}

    conn._on_state_json(b'{"type": "operator_lost"}')

    conn.connection.stop_movement.assert_called_once()
    assert conn._nonce_results == {}
    conn.connection.sport_command.assert_not_called()  # damp off by default
    assert conn._estopped is False  # a link blip must not require re-arm


def test_operator_lost_damps_when_configured(monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    conn.config = SimpleNamespace(cmd_stale_after_sec=0.5, damp_on_operator_lost=True)
    conn.connection.sport_command.return_value = True
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: None)

    conn._on_state_json(b'{"type": "operator_lost"}')

    conn.connection.stop_movement.assert_called_once()
    _wait_for(lambda: conn.connection.sport_command.call_count == 1)
    conn.connection.sport_command.assert_called_with(ALLOWED_SPORT_CMDS["Damp"])


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


# ─── map + odom overlay (robot → operator minimap) ───────────────────


def _published_json(mock: MagicMock, msg_type: str) -> dict[str, Any] | None:
    """Return the last JSON payload of the given type published on a mock."""
    import json

    for call in reversed(mock.publish.call_args_list):
        (data,) = call.args
        try:
            msg = json.loads(data)
        except (ValueError, TypeError):
            continue
        if msg.get("type") == msg_type:
            return msg
    return None


def _occupancy(grid: Any) -> Any:
    import numpy as np

    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

    return OccupancyGrid(grid=np.asarray(grid, dtype=np.int8), resolution=0.1)


def test_costmap_encodes_and_publishes_map() -> None:
    conn = _bare_connection()
    grid = _occupancy([[-1, 0, 100], [0, 0, -1]])
    conn._on_costmap(grid)

    msg = _published_json(conn.map_out, "map")
    assert msg is not None, "no map message published"
    assert msg["fmt"] == "png" and msg["png_b64"]
    assert msg["w"] == 3 and msg["h"] == 2
    assert msg["res"] == pytest.approx(0.1)
    assert len(msg["origin"]) == 2


def test_costmap_png_round_trips_palette() -> None:
    import base64

    import cv2
    import numpy as np

    conn = _bare_connection()
    conn._on_costmap(_occupancy([[-1, 0, 100]]))
    msg = _published_json(conn.map_out, "map")
    assert msg is not None
    raw = base64.b64decode(msg["png_b64"])
    # BGRA (color + alpha) — the rerun palette baked in by the robot.
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_UNCHANGED)
    assert img.shape[2] == 4  # has alpha
    row = [tuple(int(v) for v in px) for px in img[0]]
    # unknown → transparent; free → dark cyan; occupied(100) → white-hot lethal.
    assert row[0] == (0, 0, 0, 0)  # unknown transparent
    assert row[1] == (68, 58, 30, 255)  # free #1e3a44 in BGRA
    assert row[2] == (255, 255, 255, 255)  # 100 = lethal #ffffff


def test_costmap_rate_gated() -> None:
    conn = _bare_connection()
    conn._on_costmap(_occupancy([[0, 0]]))
    first = len(conn.telemetry_out.publish.call_args_list)
    conn._on_costmap(_occupancy([[0, 0]]))  # immediately again → gated out
    assert len(conn.telemetry_out.publish.call_args_list) == first


def test_block_max_preserves_obstacle_when_coarsening() -> None:
    import base64

    import cv2
    import numpy as np

    conn = _bare_connection()
    # 0.02 m/cell → coarsen by 5× to reach 0.1. A lone obstacle must survive.
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

    cells = np.zeros((10, 10), dtype=np.int8)
    cells[3, 3] = 100
    conn._on_costmap(OccupancyGrid(grid=cells, resolution=0.02))
    msg = _published_json(conn.map_out, "map")
    assert msg is not None
    assert msg["res"] == pytest.approx(0.1)  # coarsened 5×
    raw = base64.b64decode(msg["png_b64"])
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_UNCHANGED)
    # Lethal (100) survives as an opaque white pixel (BGRA #ffffff).
    lethal = np.all(img == (255, 255, 255, 255), axis=-1)
    assert lethal.any(), "obstacle erased by coarsening"


def test_odom_publishes_planar_pose() -> None:
    import math

    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    conn = _bare_connection()
    q = Quaternion.from_euler(Vector3(0.0, 0.0, math.pi / 2))  # yaw = 90°
    pose = PoseStamped(ts=123.0, position=[1.5, -2.0, 0.3], orientation=[q.x, q.y, q.z, q.w])
    conn._on_odom(pose)

    msg = _published_json(conn.map_out, "odom")
    assert msg is not None
    assert msg["x"] == pytest.approx(1.5) and msg["y"] == pytest.approx(-2.0)
    assert msg["yaw"] == pytest.approx(math.pi / 2, abs=1e-3)
    assert msg["ts"] == pytest.approx(123.0)


def test_odom_rate_gated() -> None:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    conn = _bare_connection()
    conn._on_odom(PoseStamped(ts=1.0, position=[0, 0, 0]))
    first = len(conn.telemetry_out.publish.call_args_list)
    conn._on_odom(PoseStamped(ts=1.0, position=[0, 0, 0]))  # gated
    assert len(conn.telemetry_out.publish.call_args_list) == first


def test_empty_costmap_publishes_nothing() -> None:
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

    conn = _bare_connection()
    conn._on_costmap(OccupancyGrid())  # no-arg = empty 1D grid; must be skipped
    assert _published_json(conn.map_out, "map") is None


def test_audio_frame_published_with_header() -> None:
    import struct

    conn = _bare_connection()
    conn._on_audio_frame(b"\x01\x02\x03\x04", 48000, 2)

    (data,) = conn.audio_out.publish.call_args.args
    sr, ch, fmt = struct.unpack("<IHH", data[:8])
    assert (sr, ch, fmt) == (48000, 2, 0)
    assert data[8:] == b"\x01\x02\x03\x04"


def test_audio_frame_fans_out_to_speaker_track() -> None:
    conn = _bare_connection()
    conn._speaker_track = MagicMock()

    conn._on_audio_frame(b"\x01\x02", 48000, 1)

    conn._speaker_track.push.assert_called_once_with(b"\x01\x02", 48000, 1)
    conn.audio_out.publish.assert_called_once()


# ─── click-to-navigate ───────────────────────────────────────────────


def test_nav_goal_publishes_pose_and_acks(monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_nav_goal({"x": 2.5, "y": -1.0, "nonce": 11})

    (pose,) = conn.goal_request.publish.call_args.args
    assert pose.position.x == pytest.approx(2.5)
    assert pose.position.y == pytest.approx(-1.0)
    assert pose.frame_id == "world"
    assert acks == [(11, True)]


@pytest.mark.parametrize(
    "msg",
    [
        {},
        {"x": "a", "y": 1},
        {"x": float("nan"), "y": 0},
        {"x": float("inf"), "y": 0},
        {"x": -float("inf"), "y": 0},
        {"x": 1e309, "y": 0},  # overflows to inf
        {"x": 500.0, "y": 0},  # beyond max_nav_goal_m
    ],
)
def test_nav_goal_malformed_rejected(msg: dict, monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_nav_goal({**msg, "nonce": 12})

    conn.goal_request.publish.assert_not_called()
    assert acks == [(12, False)]


def test_nav_goal_rejected_when_estopped(monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    conn._estopped = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_nav_goal({"x": 1, "y": 1, "nonce": 13})

    conn.goal_request.publish.assert_not_called()
    assert acks == [(13, False)]


def test_nav_cmd_yields_to_recent_operator_steering() -> None:
    conn = _bare_connection()
    conn._last_drive_ts = time.time()  # operator actively steering (non-zero input)

    conn._on_nav_cmd(_twist(time.time()))

    conn.connection.move.assert_not_called()


def test_idle_zero_twists_do_not_suppress_nav() -> None:
    """The browser streams zero twists when idle — they must not count as
    steering, and while nav drives they must not zero the base either."""
    conn = _bare_connection()
    conn.connection.move.return_value = True

    # Nav is active; an idle zero twist arrives from the wire.
    conn._last_nav_ts = time.time()
    assert conn.move(_twist(time.time())) is True  # swallowed, reported ok
    conn.connection.move.assert_not_called()  # ...but never forwarded
    assert conn._last_drive_ts == 0.0  # zero input isn't steering

    # Nav twists still flow (operator not steering).
    conn._on_nav_cmd(_twist(time.time()))
    conn.connection.move.assert_called_once()


def test_nonzero_operator_twist_stamps_steering_and_forwards() -> None:
    conn = _bare_connection()
    conn.connection.move.return_value = True
    t = _twist(time.time())
    t.linear.x = 0.5

    assert conn.move(t) is True
    conn.connection.move.assert_called_once()
    assert conn._last_drive_ts > 0.0


def test_nav_cmd_drives_when_operator_idle() -> None:
    conn = _bare_connection()
    conn._last_cmd_ts = time.time() - 5.0  # operator idle

    conn._on_nav_cmd(_twist(time.time()))

    conn.connection.move.assert_called_once()


def test_estop_cancels_nav(monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    monkeypatch.setattr(conn, "_send_ack", lambda *_: None)

    conn._handle_estop({"nonce": 1}.get("nonce"))

    (msg,) = conn.stop_movement.publish.call_args.args
    assert msg.data is True

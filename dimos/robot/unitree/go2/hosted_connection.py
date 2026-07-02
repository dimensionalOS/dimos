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

"""Go2 driver + hosted-teleop control plane in ONE module.

The broker provider is a per-process singleton, and ``GO2Connection`` is
``dedicated_worker=True`` (its own process), so all hosted broker transports
(cmd, video, state, state_back) must live on this one module to share a single
CF session — a separate bridge module lands in another worker = a 2nd session
the operator can't see. Opt-in subclass; plain ``GO2Connection`` is unchanged.
"""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import json
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
from dimos.teleop.utils.stream_stats import LiveStreamStats
from dimos.teleop.utils.video_stats import VideoStats
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Operator-allowed sport commands → SPORT_CMD api_id (robot-side allow-list).
ALLOWED_SPORT_CMDS: dict[str, int] = {
    "StandDown": 1005,
    "RecoveryStand": 1006,
    "Sit": 1009,
    "Hello": 1016,
    "Stretch": 1017,
    "Damp": 1001,
    "FrontPounce": 1032,  # acrobatic — leaps
    "FrontJump": 1031,  # acrobatic — leaps
}

# Commands that latch a posture the UI should reflect (gestures like Hello
# return to the prior stance, so they don't touch it).
_POSTURE_SPORT_CMDS = frozenset({"StandDown", "RecoveryStand", "Sit", "Damp"})


class Go2HostedConnectionConfig(ConnectionConfig):
    telemetry_hz: float = 3.0  # robot → operator HUD telemetry push rate
    cmd_stale_after_sec: float = 0.5  # cmd_vel twists older than this are dropped
    latency_stamp: bool = False  # benchmark: paint capture-time into frame corner
    # Also Damp (go limp) when the operator link drops, on top of the always-on
    # zero-velocity stop. Off by default: a WiFi blip shouldn't drop the robot
    # mid-patrol; the 0.2s cmd_vel deadman + stop_movement cover the base.
    damp_on_operator_lost: bool = False
    # Publish-side video caps (0 = source rate/resolution). Applied at the mux,
    # so they hold on both transports: capping here beats letting congestion
    # show up as encoder drops/freezes on a constrained uplink.
    video_max_width: int = 0
    video_max_fps: float = 0.0


# Frame-embedded capture time for glass-to-glass latency, read back by the
# operator (webrtc.js readLatencyStamp). Encoded as B/W cells in a strip
# APPENDED below the frame (video content untouched; operator reads then crops).
# MSB-first: SYNC then time. Constants MUST match webrtc.js readLatencyStamp.
_STAMP_CELL_PX = 16  # cell width — big enough to survive H.264 compression
_STAMP_STRIP_PX = 16  # height of the appended timestamp band, in rows
_STAMP_SYNC = (1, 0, 1, 0)  # both sides must agree
_STAMP_TIME_BITS = 44  # ms since epoch (~41 bits) + headroom
_STAMP_CELLS = len(_STAMP_SYNC) + _STAMP_TIME_BITS


class Go2HostedConnection(GO2Connection):
    """GO2Connection + the hosted-teleop state plane, colocated (one session)."""

    config: Go2HostedConnectionConfig

    state_json: In[bytes]
    cmd_raw: In[bytes]
    video_stats: Out[VideoStats]
    telemetry_out: Out[bytes]
    cam2_in: In[Image]
    mux_image: Out[Image]
    cmd_vel_stamped: Out[TwistStamped]

    # Queued (non-urgent) commands beyond this are busy-rejected — bounds the
    # backlog a spamming/laggy operator can build behind a slow command.
    _MAX_PENDING_CMDS = 4
    # Nonce dedup: transport/UI duplicates within this window re-ack the prior
    # result instead of re-executing. Short on purpose — browser nonces restart
    # at 1 per session, so entries must age out before a quick reconnect reuses
    # them. Applies to nonce'd JSON commands only; cmd_vel twists carry no
    # nonce and are guarded by the monotonic-ts drop in move().
    _NONCE_TTL_SEC = 10.0
    _NONCE_CACHE_MAX = 64

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._cmd_stats = LiveStreamStats()
        self._telemetry_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._rage_active = False
        self._last_cmd_ts = 0.0
        self._cam_lock = threading.Lock()
        self._cam_frames: dict[str, Image] = {}
        self._cam_selected = ["cam1"]
        # Single worker (repo pattern, cf. utils/threadpool + drake_world):
        # commands execute strictly in order, so state like _rage_active can't
        # race between overlapping runners. Bounded by _MAX_PENDING_CMDS.
        self._cmd_executor: ThreadPoolExecutor | None = None
        self._cmd_pending = 0
        self._cmd_lock = threading.Lock()
        # nonce → (result | None while in flight, monotonic stamp)
        self._nonce_results: dict[Any, tuple[bool | None, float]] = {}
        # E-STOP latch: set by {"type":"estop"}; while latched, move() refuses
        # twists and non-urgent commands are rejected until {"type":"estop_clear"}.
        self._estopped = False
        # Robot-authoritative UI state, pushed in telemetry so a reconnecting
        # operator's cockpit reflects reality instead of optimistic defaults.
        # GO2Connection.start() stands the robot up, hence the initial posture.
        self._posture = "StandReady"
        self._obstacle_avoidance = True  # corrected from config.g in start()
        self._light = 0.0  # head-LED brightness 0..1, assumed off until set
        self._last_mux_pub = 0.0  # monotonic stamp for the video_max_fps cap

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._cmd_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="Go2Cmd")
        # Firmware OA was just set from config.g by GO2Connection.start().
        try:
            self._obstacle_avoidance = bool(self.config.g.obstacle_avoidance)
        except AttributeError:
            pass
        # Force firmware out of Rage so _rage_active=False matches reality —
        # a prior session may have left it on, and the set_mode short-circuit
        # then locks the user in Rage.
        try:
            self.connection.set_rage_mode(False)
        except Exception:
            logger.exception("startup set_rage_mode(False) failed")
        # Sync subscribes (not async handle_*): keep-latest would drop bursts.
        for stream, cb in (
            (self.state_json, self._on_state_json),
            (self.cmd_raw, self._on_cmd_raw),
        ):
            self.register_disposable(Disposable(stream.subscribe(cb)))
        # Mux: tap the base's color_image as cam1, RealSense as cam2 → mux_image.
        self.register_disposable(
            Disposable(self.color_image.subscribe(lambda i: self._on_cam("cam1", i)))
        )
        self.register_disposable(
            Disposable(self.cam2_in.subscribe(lambda i: self._on_cam("cam2", i)))
        )
        self._start_telemetry()

    # ─── Camera mux ──────────────────────────────────────────────────
    def _on_cam(self, cam: str, img: Image) -> None:
        with self._cam_lock:
            self._cam_frames[cam] = img
            shown = cam in self._cam_selected
        if not shown:
            return
        # FPS cap before any mux/encode work — skipping here is nearly free.
        max_fps = self.config.video_max_fps
        if max_fps > 0:
            now = time.monotonic()
            if now - self._last_mux_pub < 1.0 / max_fps:
                return
            self._last_mux_pub = now
        out = self._composite()
        if out is not None:
            self.mux_image.publish(out)

    def _composite(self) -> Image | None:
        with self._cam_lock:
            order = [c for c in ("cam1", "cam2") if c in self._cam_selected]
            imgs = [self._cam_frames[c] for c in order if c in self._cam_frames]
        if not imgs:
            return None
        if len(imgs) == 1:
            return self._stamp(self._downscale(imgs[0]))
        import cv2

        target_h = min(im.data.shape[0] for im in imgs)
        tiles = []
        for im in imgs:
            h, w = im.data.shape[:2]
            tiles.append(
                cv2.resize(im.data, (int(w * target_h / h), target_h)) if h != target_h else im.data
            )
        return self._stamp(
            self._downscale(
                Image(data=np.hstack(tiles), format=imgs[0].format, frame_id="camera_mux")
            )
        )

    def _downscale(self, img: Image) -> Image:
        """Cap publish width at config.video_max_width (0 = off). Runs before
        _stamp so the strip's 16px cells stay decodable at the sent size."""
        max_w = self.config.video_max_width
        if max_w <= 0 or img.data.ndim < 2:
            return img
        h, w = img.data.shape[:2]
        if w <= max_w:
            return img
        import cv2

        out = cv2.resize(img.data, (max_w, max(1, int(h * max_w / w))))
        return Image(data=out, format=img.format, frame_id=img.frame_id)

    def _stamp(self, img: Image) -> Image:
        """Append a bottom strip encoding capture time as B/W cells (benchmark).

        Rows are ADDED below the frame (height grows by ``_STAMP_STRIP_PX``), so
        the video content is never overwritten — the operator reads the strip,
        then crops it on display. No-op unless ``config.latency_stamp``.
        """
        if not self.config.latency_stamp:
            return img

        ms = int(time.time() * 1000)
        bits = list(_STAMP_SYNC) + [
            (ms >> (_STAMP_TIME_BITS - 1 - i)) & 1 for i in range(_STAMP_TIME_BITS)
        ]

        s = _STAMP_CELL_PX
        data = img.data
        if data.ndim < 2 or data.shape[1] < _STAMP_CELLS * s:
            return img

        # Build the strip (black), paint cells across it, then stack below.
        strip_shape = (_STAMP_STRIP_PX, data.shape[1], *data.shape[2:])
        strip = np.zeros(strip_shape, dtype=data.dtype)
        for i, bit in enumerate(bits):
            if bit:
                strip[:, i * s : (i + 1) * s] = 255
        out = np.vstack([data, strip])
        return Image(data=out, format=img.format, frame_id=img.frame_id)

    def _set_cam_selection(self, cams: list[str]) -> None:
        sel = [c for c in cams if c in ("cam1", "cam2")] or ["cam1"]
        with self._cam_lock:
            self._cam_selected = sel
        logger.info("camera selection → %s", sel)
        out = self._composite()
        if out is not None:
            self.mux_image.publish(out)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._cmd_executor is not None:
            self._cmd_executor.shutdown(wait=False, cancel_futures=True)
            self._cmd_executor = None
        if self._telemetry_thread is not None:
            self._telemetry_thread.join(timeout=2.0)
            self._telemetry_thread = None
        super().stop()

    # ─── Inbound state plane (operator → robot) ──────────────────────

    def _on_state_json(self, data: Any) -> None:
        if isinstance(data, str):
            data = data.encode()
        if not data.startswith(b"{"):
            return  # not JSON
        try:
            msg = json.loads(data)
        except ValueError:
            logger.warning("state_reliable: malformed JSON: %r", data[:80])
            return

        kind = msg.get("type")
        if kind == "estop":
            self._handle_estop(msg.get("nonce"))
        elif kind == "estop_clear":
            self._handle_estop_clear(msg.get("nonce"))
        elif kind == "operator_lost":  # synthetic, injected by the provider
            self._on_operator_lost()
        elif kind == "sport_cmd":
            self._handle_sport_cmd(msg)
        elif kind == "set_mode":
            self._handle_set_mode(msg)
        elif kind == "camera_select":
            self._set_cam_selection(msg.get("cams", []))
        elif kind == "obstacle_avoidance":
            self._handle_obstacle_avoidance(msg)
        elif kind == "light":
            self._handle_light(msg)
        elif kind == "video_stats":
            self.video_stats.publish(VideoStats.from_dict(msg))
        elif kind == "clock_report":
            logger.info(
                "clock-sync: operator rtt=%s offset=%s",
                msg.get("rtt_ms"),
                msg.get("offset_ms"),
            )
        # ping answered by BrokerProvider; unknown types ignored.

    # ─── E-STOP latch + operator-loss safety ─────────────────────────

    def _handle_estop(self, nonce: Any) -> None:
        """Latch FIRST (gates move() immediately, before the RPC lands), then
        Damp urgently — never queued behind slower commands."""
        self._estopped = True
        logger.warning("E-STOP latched by operator")

        def task() -> bool:
            ok = bool(self.connection.sport_command(ALLOWED_SPORT_CMDS["Damp"]))
            if ok:
                self._posture = "Damp"
            return ok

        self._submit_cmd("estop", nonce, task, urgent=True)

    def _handle_estop_clear(self, nonce: Any) -> None:
        """Re-arm. Deliberately does NOT move the robot — the operator must
        explicitly Stand/Drive afterwards."""
        self._estopped = False
        logger.warning("E-STOP cleared by operator")
        self._send_ack(nonce, True)

    def _on_operator_lost(self) -> None:
        """Provider-injected when the operator's command plane goes away.

        Always zero the base (belt to the 0.2s cmd_vel deadman's braces — an
        in-flight duration move keeps publishing without it) and drop the
        nonce cache: browser nonces restart at 1 on the next session, so a
        stale entry would re-ack instead of executing. Damp only if configured."""
        logger.warning("operator link lost — stopping motion")
        with self._cmd_lock:
            self._nonce_results.clear()
        try:
            self.connection.stop_movement()
        except Exception:
            logger.exception("stop_movement on operator loss failed")
        if self.config.damp_on_operator_lost:
            self._submit_cmd(
                "damp_on_operator_lost",
                None,
                lambda: bool(self.connection.sport_command(ALLOWED_SPORT_CMDS["Damp"])),
                urgent=True,
            )

    def _submit_cmd(
        self, label: str, nonce: Any, task: Callable[[], bool], *, urgent: bool = False
    ) -> None:
        """Run a blocking command off the WebRTC/video loop and ack the result.

        Non-urgent commands go through a single worker — strict ordering, so
        stateful toggles (rage) can't interleave — with a bounded backlog:
        past _MAX_PENDING_CMDS they're busy-rejected (ack ok=False) instead of
        piling up threads. urgent=True (Damp / E-STOP) bypasses the queue on a
        dedicated thread: a stop must never wait behind a 3s StandReady.
        """

        # E-STOP latch: only urgent work (Damp itself) may run while latched;
        # estop_clear is handled upstream and never reaches here.
        if self._estopped and not urgent:
            logger.warning("%s rejected: E-STOP latched", label)
            self._send_ack(nonce, False)
            return

        # Nonce dedup (B2): a duplicate of a finished command re-acks its
        # result; a duplicate of an in-flight command is dropped (the original
        # will ack). Transient rejections below UNWIND the reservation so a
        # genuine retry can still execute.
        if nonce is not None:
            now = time.monotonic()
            with self._cmd_lock:
                self._nonce_results = {
                    n: (r, t)
                    for n, (r, t) in self._nonce_results.items()
                    if now - t < self._NONCE_TTL_SEC
                }
                if nonce in self._nonce_results:
                    prior, _ = self._nonce_results[nonce]
                    logger.info(
                        "%s: duplicate nonce %r — %s",
                        label,
                        nonce,
                        "re-acking" if prior is not None else "in flight",
                    )
                    if prior is not None:
                        self._send_ack(nonce, prior)
                    return
                if len(self._nonce_results) >= self._NONCE_CACHE_MAX:
                    oldest = min(self._nonce_results, key=lambda n: self._nonce_results[n][1])
                    del self._nonce_results[oldest]
                self._nonce_results[nonce] = (None, now)

        def _unwind_nonce() -> None:
            if nonce is not None:
                with self._cmd_lock:
                    self._nonce_results.pop(nonce, None)

        def runner() -> None:
            ok = False
            try:
                ok = bool(task())
            except Exception:
                logger.exception("%s failed", label)
            finally:
                if not urgent:
                    with self._cmd_lock:
                        self._cmd_pending -= 1
            if nonce is not None:
                with self._cmd_lock:
                    self._nonce_results[nonce] = (ok, time.monotonic())
            self._send_ack(nonce, ok)

        if urgent:
            threading.Thread(target=runner, daemon=True, name=f"Go2Cmd-{label}").start()
            return

        executor = self._cmd_executor
        if executor is None:  # not started / already stopped
            _unwind_nonce()
            self._send_ack(nonce, False)
            return
        with self._cmd_lock:
            busy = self._cmd_pending >= self._MAX_PENDING_CMDS
            if busy:
                self._nonce_results.pop(nonce, None)
            else:
                self._cmd_pending += 1
        if busy:
            logger.warning("%s rejected: command backlog full", label)
            self._send_ack(nonce, False)
            return
        try:
            executor.submit(runner)
        except RuntimeError:  # shutdown raced us
            with self._cmd_lock:
                self._cmd_pending -= 1
            _unwind_nonce()
            self._send_ack(nonce, False)

    def _handle_sport_cmd(self, msg: dict[str, Any]) -> None:
        """Operator button → allow-listed SPORT_MOD request, ack on cmd_ack."""
        name = msg.get("name")
        nonce = msg.get("nonce")

        # StandReady is the standup+balance combo, never the two separately.
        if name == "StandReady":
            self._submit_cmd("StandReady", nonce, self._stand_ready_task)
            return

        api_id = ALLOWED_SPORT_CMDS.get(name) if isinstance(name, str) else None
        if api_id is None:
            logger.warning("sport_cmd: disallowed/unknown name %r", name)
            self._send_ack(nonce, False)
            return

        def task() -> bool:
            ok = bool(self.connection.sport_command(api_id))
            if ok and name in _POSTURE_SPORT_CMDS:
                self._posture = name
            return ok

        # Damp is the E-STOP: it must jump the queue, not wait behind slower
        # queued commands (StandReady holds the worker for ~3.3s).
        self._submit_cmd(f"sport_cmd {name}", nonce, task, urgent=(name == "Damp"))

    def _stand_ready_task(self) -> bool:
        """Standup → RecoveryStand → BalanceStand → joystick ON (drive-ready).

        WASD drives via wireless-controller stick emulation, which needs BOTH
        the BalanceStand FSM (so ending in RecoveryStand left drive dead) and
        firmware joystick listening enabled — SwitchJoystick(False) is left
        behind by rage-off transitions on older set_rage_mode. RecoveryStand
        runs mid-sequence to recover from Sit / Damp / Rage weirdness.
        """
        self.connection.standup()
        time.sleep(3.0)  # standup must finish before the FSM transitions
        self.connection.sport_command(ALLOWED_SPORT_CMDS["RecoveryStand"])
        time.sleep(0.3)
        self.connection.balance_stand()
        time.sleep(0.3)
        self.connection.switch_joystick(True)
        self._posture = "StandReady"
        return True

    def _handle_set_mode(self, msg: dict[str, Any]) -> None:
        """Speed-mode select. normal/high differ only by browser-side scale;
        only the rage on/off boundary toggles the firmware (set_rage_mode)."""
        mode = msg.get("mode")
        nonce = msg.get("nonce")
        if mode not in ("normal", "high", "rage"):
            logger.warning("set_mode: unknown mode %r", mode)
            self._send_ack(nonce, False)
            return
        want_rage = mode == "rage"

        # The rage check runs INSIDE the serialized task: checking on the
        # callback thread raced the previous toggle's write of _rage_active.
        def task() -> bool:
            if want_rage == self._rage_active:
                return True  # already in the right FSM
            ok = bool(self.connection.set_rage_mode(want_rage))
            if ok:
                self._rage_active = want_rage
            logger.info("set_mode: rage=%s ok=%s", want_rage, ok)
            return ok

        self._submit_cmd(f"set_mode {mode}", nonce, task)

    def _handle_obstacle_avoidance(self, msg: dict[str, Any]) -> None:
        """Toggle the Go2's onboard obstacle avoidance on/off."""
        enabled = bool(msg.get("enabled"))
        nonce = msg.get("nonce")

        def task() -> bool:
            self.connection.set_obstacle_avoidance(enabled)
            self._obstacle_avoidance = enabled
            logger.info("obstacle_avoidance: enabled=%s", enabled)
            return True

        self._submit_cmd(f"obstacle_avoidance {enabled}", nonce, task)

    def _handle_light(self, msg: dict[str, Any]) -> None:
        """Head-LED brightness. The slider sends brightness 0..1; the original
        toggle sent an ``enabled`` bool — map it so deployed frontends keep
        working. 0..1 → firmware level 0-10 (0 = off)."""
        nonce = msg.get("nonce")
        raw = msg.get("brightness")
        if raw is None:
            raw = 1.0 if msg.get("enabled") else 0.0
        try:
            brightness = float(raw)
        except (TypeError, ValueError):
            logger.warning("light: malformed brightness %r", raw)
            self._send_ack(nonce, False)
            return
        if brightness != brightness:  # NaN
            self._send_ack(nonce, False)
            return
        brightness = max(0.0, min(1.0, brightness))
        level = round(brightness * 10)

        def task() -> bool:
            ok = bool(self.connection.set_light(level))
            if ok:
                self._light = brightness
            logger.info("light: brightness=%.1f (level %d) ok=%s", brightness, level, ok)
            return ok

        self._submit_cmd(f"light {brightness:.1f}", nonce, task)

    def _send_ack(self, nonce: Any, ok: bool) -> None:
        # Best-effort: the ack rides state_reliable_back, which doesn't exist
        # while no operator is connected — a dropped ack there is expected, but
        # a failure once connected means the operator's button spins, so warn.
        try:
            self.telemetry_out.publish(
                json.dumps({"type": "cmd_ack", "nonce": nonce, "ok": ok}).encode()
            )
        except Exception:
            logger.warning("cmd_ack publish failed", exc_info=True)

    # ─── Command-plane health (robot → operator) ─────────────────────

    def move(self, twist: Any, duration: float = 0.0) -> bool:
        """Drop stale + out-of-order cmd_vel from the unreliable wire."""
        if self._estopped:
            return False  # latched: no motion until estop_clear
        ts = float(twist.ts)
        age = time.time() - ts
        if age > self.config.cmd_stale_after_sec:
            logger.debug("dropping stale cmd_vel: age=%.3fs", age)
            return False
        if ts <= self._last_cmd_ts:
            logger.debug("dropping out-of-order cmd_vel: ts=%.3f last=%.3f", ts, self._last_cmd_ts)
            return False
        self._last_cmd_ts = ts
        return super().move(twist, duration)

    def _on_cmd_raw(self, data: Any) -> None:
        """Decode the operator cmd: record its send-stamp for latency stats and
        re-publish it as ``TwistStamped`` so the recorder can tap it over LCM
        (avoids a 2nd CF session — see quest_hosted/blueprints.py)."""
        if isinstance(data, str):
            data = data.encode()
        try:
            cmd = TwistStamped.lcm_decode(data)
        except Exception:
            return  # foreign / undecodable frame — skip
        self._cmd_stats.record(cmd.ts, nbytes=len(data))
        self.cmd_vel_stamped.publish(cmd)

    def _battery_soc(self) -> int | None:
        """Battery SOC from the cached lowstate, without invoking the logged
        ``get_battery_soc`` skill (which the 3 Hz telemetry loop would spam)."""
        try:
            return int(self._latest_lowstate["data"]["bms_state"]["soc"])  # type: ignore[index]
        except (KeyError, TypeError, ValueError):
            return None

    def _telemetry_payload(self) -> dict[str, Any]:
        """One telemetry frame. `state` is robot-authoritative UI state so a
        (re)connecting operator seeds its cockpit from reality — posture, rage,
        obstacle avoidance, camera selection, E-STOP latch."""
        return {
            "type": "robot_telemetry",
            "cmd": self._cmd_stats.snapshot(),
            "soc": self._battery_soc(),
            "state": {
                "posture": self._posture,
                "rage": self._rage_active,
                "obstacle_avoidance": self._obstacle_avoidance,
                "light": self._light,  # brightness 0..1
                "cams": list(self._cam_selected),
                "estopped": self._estopped,
            },
            "robot_ts": time.time(),
        }

    def _start_telemetry(self) -> None:
        def runner() -> None:
            interval = 1.0 / max(self.config.telemetry_hz, 0.1)
            while not self._stop_event.is_set():
                payload = json.dumps(self._telemetry_payload())
                # debug (not warning): this fires at telemetry_hz with no
                # operator connected, so a failed publish here is the norm
                # and would flood the log at a higher level.
                try:
                    self.telemetry_out.publish(payload.encode())
                except Exception:
                    logger.debug("telemetry publish failed", exc_info=True)
                self._stop_event.wait(interval)

        self._telemetry_thread = threading.Thread(
            target=runner, daemon=True, name="Go2HostedTelemetry"
        )
        self._telemetry_thread.start()


__all__ = ["Go2HostedConnection", "Go2HostedConnectionConfig"]

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

All hosted broker transports must live on this one module to share a single
CF session — GO2Connection is dedicated_worker, and the broker provider is a
per-process singleton. Opt-in subclass; plain GO2Connection is unchanged.
"""

from __future__ import annotations

import base64
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import json
import math
import struct
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
    set_audio_sink,
    shutdown_all_providers,
)
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
from dimos.robot.unitree.go2.speaker import PCMAudioTrack
from dimos.teleop.quest_hosted.hosted_base import HostedConnectionMixin
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

# High-risk acrobatic leaps — gated off the default allow-list; enable only
# with config.allow_acrobatics (a hosted web operator shouldn't trigger these
# by default).
_ACROBATIC_SPORT_CMDS = frozenset({"FrontPounce", "FrontJump"})


class Go2HostedConnectionConfig(ConnectionConfig):
    telemetry_hz: float = 3.0
    cmd_stale_after_sec: float = 0.5
    latency_stamp: bool = False
    damp_on_operator_lost: bool = False
    video_max_width: int = 0
    video_max_fps: float = 0.0
    map_hz: float = 2.0
    map_min_resolution: float = 0.1
    odom_hz: float = 15.0
    speaker: bool = True
    nav_yield_sec: float = 1.0
    # Reject click-to-nav goals beyond this (axis-aligned bound, not radius).
    max_nav_goal_m: float = 100.0
    allow_acrobatics: bool = False  # let the operator trigger FrontJump/Pounce


class Go2HostedConnection(GO2Connection, HostedConnectionMixin):
    """GO2Connection + hosted state plane. Shared control plane lives in
    HostedConnectionMixin; this adds the Go2 commands + serialized executor."""

    config: Go2HostedConnectionConfig

    # Cloudflare / LiveKit over WebRTC
    state_json: In[bytes]  # state_reliable
    cmd_raw: In[bytes]  # cmd_unreliable
    mux_image: Out[Image]  # video track
    telemetry_out: Out[bytes]  # state_reliable_back
    map_out: Out[bytes]  # map_unreliable

    # Other modules
    cam2_in: In[Image]
    global_costmap: In[OccupancyGrid]
    nav_cmd_vel: In[Twist]
    cmd_vel_stamped: Out[TwistStamped]
    video_stats: Out[VideoStats]
    audio_out: Out[bytes]
    goal_request: Out[PoseStamped]
    stop_movement: Out[Bool]

    _MAX_PENDING_CMDS = 4
    _NONCE_TTL_SEC = 10.0
    _NONCE_CACHE_MAX = 64
    _MAX_MAP_BYTES = 16 * 1024  # CF datachannel per-message ceiling

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._hosted_init(["cam1", "cam2"])
        self._stop_event = threading.Event()
        self._rage_active = False
        self._last_cmd_ts = 0.0
        self._last_drive_ts = 0.0
        self._last_nav_ts = 0.0
        self._cmd_executor: ThreadPoolExecutor | None = None
        self._cmd_pending = 0
        self._cmd_lock = threading.Lock()
        self._nonce_results: dict[Any, tuple[bool | None, float]] = {}
        self._speaker_track: PCMAudioTrack | None = None
        self._posture = "StandReady"
        self._obstacle_avoidance = True
        self._light = 0.0
        self._last_map_pub = 0.0
        self._last_odom_pub = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._cmd_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="Go2Cmd")
        try:
            self._obstacle_avoidance = bool(self.config.g.obstacle_avoidance)
        except AttributeError:
            pass
        try:
            self.connection.set_rage_mode(False)
        except Exception:
            logger.exception("startup set_rage_mode(False) failed")
        for stream, cb in (
            (self.state_json, self._on_state_json),
            (self.cmd_raw, self._on_cmd_raw),
        ):
            self.register_disposable(Disposable(stream.subscribe(cb)))
        self.register_disposable(
            Disposable(self.color_image.subscribe(lambda i: self._on_cam("cam1", i)))
        )
        self.register_disposable(
            Disposable(self.cam2_in.subscribe(lambda i: self._on_cam("cam2", i)))
        )
        if self.config.map_hz > 0:
            self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_costmap)))
        self.register_disposable(Disposable(self.nav_cmd_vel.subscribe(self._on_nav_cmd)))
        if self.config.odom_hz > 0:
            self.register_disposable(self.connection.odom_stream().subscribe(self._on_odom))
        set_audio_sink(self._on_audio_frame)
        if self.config.speaker:
            self._attach_speaker()
        self._start_telemetry()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._cmd_executor is not None:
            self._cmd_executor.shutdown(wait=False, cancel_futures=True)
            self._cmd_executor = None
        self._stop_telemetry()
        set_audio_sink(None)  # drop the provider's ref to this module's sink
        # Graceful broker disconnect so the worker exits promptly instead of
        # being force-killed and reaped ~30s later. See shutdown_all_providers.
        shutdown_all_providers()
        super().stop()

    # ─── Go2-specific state-plane types (rest handled by the mixin) ──

    def _handle_robot_msg(self, kind: Any, msg: dict[str, Any]) -> None:
        if kind == "sport_cmd":
            self._handle_sport_cmd(msg)
        elif kind == "set_mode":
            self._handle_set_mode(msg)
        elif kind == "obstacle_avoidance":
            self._handle_obstacle_avoidance(msg)
        elif kind == "light":
            self._handle_light(msg)
        elif kind == "nav_goal":
            self._handle_nav_goal(msg)

    # ─── Click-to-navigate ────────────────────────────────────────────

    def _handle_nav_goal(self, msg: dict[str, Any]) -> None:
        """Operator map click → PoseStamped goal for the planner."""
        nonce = msg.get("nonce")
        if self._estopped:
            logger.warning("nav_goal rejected: E-STOP latched")
            self._send_ack(nonce, False)
            return
        try:
            x, y = float(msg["x"]), float(msg["y"])
        except (KeyError, TypeError, ValueError):
            logger.warning("nav_goal: malformed %r", msg)
            self._send_ack(nonce, False)
            return
        # Reject NaN/inf and absurd distances — a malformed operator click must
        # not inject an unreachable goal into the planner.
        limit = self.config.max_nav_goal_m
        if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > limit or abs(y) > limit:
            logger.warning("nav_goal: out-of-range (%r, %r)", x, y)
            self._send_ack(nonce, False)
            return
        pose = PoseStamped(
            ts=time.time(), frame_id="world", position=[x, y, 0.0], orientation=[0, 0, 0, 1]
        )
        try:
            self.goal_request.publish(pose)
        except Exception:
            logger.warning("nav_goal publish failed", exc_info=True)
            self._send_ack(nonce, False)
            return
        logger.info("nav_goal: (%.2f, %.2f)", x, y)
        self._send_ack(nonce, True)

    def _on_nav_cmd(self, twist: Twist) -> None:
        """Planner → base, unless the operator steered within nav_yield_sec."""
        if self._estopped:
            return
        now = time.time()
        if now - self._last_drive_ts < self.config.nav_yield_sec:
            return  # operator is actively steering — manual wins
        self._last_nav_ts = now
        GO2Connection.move(self, twist)  # base move — the wire guards don't apply

    def _cancel_nav(self) -> None:
        try:
            msg = Bool()
            msg.data = True
            self.stop_movement.publish(msg)
        except Exception:
            logger.debug("nav cancel publish failed", exc_info=True)

    # ─── E-STOP latch + operator-loss safety ─────────────────────────

    def _handle_estop(self, nonce: Any) -> None:
        # Latch first so move() is gated before the Damp RPC even lands.
        self._estopped = True
        logger.warning("E-STOP latched by operator")
        self._cancel_nav()

        def task() -> bool:
            ok = bool(self.connection.sport_command(ALLOWED_SPORT_CMDS["Damp"]))
            if ok:
                self._posture = "Damp"
            return ok

        self._submit_cmd("estop", nonce, task, urgent=True)

    def _handle_estop_clear(self, nonce: Any) -> None:
        # Re-arm only; does NOT move — operator must Stand/Drive explicitly.
        self._cancel_nav()
        self._last_nav_ts = 0.0
        self._last_drive_ts = 0.0
        self._estopped = False
        logger.warning("E-STOP cleared by operator")
        self._send_ack(nonce, True)

    def _on_operator_lost(self) -> None:
        # Zero the base and clear the nonce cache (browser nonces restart at 1
        # per session, so a stale entry would re-ack instead of executing).
        logger.warning("operator link lost — stopping motion")
        self._cancel_nav()
        with self._cmd_lock:
            self._nonce_results.clear()
        try:
            # Not on the connection protocol — the webrtc deadman stop; sim
            # connections don't have it and the 0.2s cmd_vel timeout covers them.
            stop = getattr(self.connection, "stop_movement", None)
            if stop is not None:
                stop()
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
        """Run a blocking command off the loop and ack it. Non-urgent commands
        serialize on one worker (bounded backlog, busy-rejected past
        _MAX_PENDING_CMDS); urgent (Damp/E-STOP) bypasses the queue."""

        # E-STOP latch: only urgent work (Damp itself) may run while latched.
        if self._estopped and not urgent:
            logger.warning("%s rejected: E-STOP latched", label)
            self._send_ack(nonce, False)
            return

        # Nonce dedup: a duplicate of a finished command re-acks its result;
        # a duplicate of an in-flight one is dropped (the original will ack).
        # Transient rejections below unwind the reservation so a genuine
        # retry can still execute. Urgent (E-STOP/Damp) skips dedup entirely.
        if nonce is not None and not urgent:
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
            if nonce is not None and not urgent:
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
        """Operator button → allow-listed SPORT_MOD request."""
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
        if name in _ACROBATIC_SPORT_CMDS and not self.config.allow_acrobatics:
            logger.warning("sport_cmd: %s blocked (allow_acrobatics=False)", name)
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
        # Standup → RecoveryStand → BalanceStand → joystick ON. WASD drives via
        # stick emulation, which needs BOTH the BalanceStand FSM and firmware
        # joystick listening on; the sequence + sleeps are load-bearing. Each
        # step is checked so the operator's ack reflects reality, not optimism.
        def _step(label: str, ok: object) -> bool:
            if not ok:
                logger.warning("StandReady: %s failed", label)
            return bool(ok)

        if not _step("standup", self.connection.standup()):
            return False
        time.sleep(3.0)  # standup must finish before the FSM transitions
        if not _step(
            "RecoveryStand", self.connection.sport_command(ALLOWED_SPORT_CMDS["RecoveryStand"])
        ):
            return False
        time.sleep(0.3)
        if not _step("balance_stand", self.connection.balance_stand()):
            return False
        time.sleep(0.3)
        if not _step("switch_joystick", self.connection.switch_joystick(True)):
            return False
        self._posture = "StandReady"
        return True

    def _handle_set_mode(self, msg: dict[str, Any]) -> None:
        """Speed mode. normal/high are browser-side scale only; only the rage
        boundary toggles firmware."""
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
        """Head-LED brightness 0..1 → firmware level 0-10."""
        nonce = msg.get("nonce")
        raw = msg.get("brightness")
        if raw is None:
            raw = 1.0 if msg.get("enabled") else 0.0  # legacy on/off toggle
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
        if age < -self.config.cmd_stale_after_sec:  # future-stamped: skew/corrupt ts
            logger.debug("dropping future-stamped cmd_vel: age=%.3fs", age)
            return False
        if ts <= self._last_cmd_ts:
            logger.debug("dropping out-of-order cmd_vel: ts=%.3f last=%.3f", ts, self._last_cmd_ts)
            return False
        self._last_cmd_ts = ts
        now = time.time()
        steering = (
            abs(twist.linear.x) > 1e-3 or abs(twist.linear.y) > 1e-3 or abs(twist.angular.z) > 1e-3
        )
        if steering:
            self._last_drive_ts = now
        elif now - self._last_nav_ts < self.config.nav_yield_sec:
            # Idle zero-twist while the planner is driving: swallow it, or the
            # operator's idle stream zeroes the base between nav commands.
            return True
        return super().move(twist, duration)

    def _on_cmd_raw(self, data: Any) -> None:
        # Record the send-stamp for latency stats and re-publish as TwistStamped
        # so the recorder can tap it over LCM (no 2nd CF session).
        if isinstance(data, str):
            data = data.encode()
        try:
            cmd = TwistStamped.lcm_decode(data)
        except Exception:
            return  # foreign / undecodable frame — skip
        self._cmd_stats.record(cmd.ts, nbytes=len(data))
        self.cmd_vel_stamped.publish(cmd)

    # ─── Map overlay (robot → operator minimap, on map_unreliable) ──

    def _on_costmap(self, grid: OccupancyGrid) -> None:
        """Throttle, coarsen, colorize, PNG-encode and push the map to the
        operator. Coarsen + PNG keeps it under the 16 KB CF message ceiling."""
        now = time.monotonic()
        if now - self._last_map_pub < 1.0 / self.config.map_hz:
            return

        cells = grid.grid
        if cells is None or cells.size == 0:
            return

        # Coarsen to >= map_min_resolution (block-max preserves obstacles).
        res = grid.resolution
        img_cells = cells
        if 0 < res < self.config.map_min_resolution:
            factor = max(1, round(self.config.map_min_resolution / res))
            if factor > 1:
                img_cells = self._block_max(cells, factor)
                res = res * factor

        # Colorize + PNG-encode; colors are baked in so the browser just blits it.
        png_bgra = self._occupancy_to_bgra(img_cells)
        try:
            import cv2

            ok, buf = cv2.imencode(".png", png_bgra)
        except Exception:
            # Unlike a dropped publish, a broken encode (e.g. missing cv2)
            # kills the minimap permanently — say so audibly.
            logger.warning("map encode failed", exc_info=True)
            return
        if not ok:
            return
        png_b64 = base64.b64encode(buf.tobytes()).decode("ascii")

        # origin lets the browser place map + robot: cell = (world_xy - origin)/res.
        h, w = img_cells.shape[:2]
        origin = grid.origin.position
        payload = {
            "type": "map",
            "fmt": "png",
            "w": int(w),
            "h": int(h),
            "res": float(res),
            "origin": [float(origin.x), float(origin.y)],
            "stamp": float(grid.ts),
            "png_b64": png_b64,
        }
        data = json.dumps(payload, separators=(",", ":")).encode()
        # Coarsening usually keeps this under the ceiling; a high-entropy grid
        # can still exceed it, and an oversized send destabilizes the channel —
        # drop it rather than blow the datachannel.
        if len(data) > self._MAX_MAP_BYTES:
            logger.warning("map payload too large (%d bytes), dropping frame", len(data))
            self._last_map_pub = now  # don't retry the same oversized frame immediately
            return
        try:
            self.map_out.publish(data)
        except Exception:
            logger.debug("map publish failed", exc_info=True)
            return
        self._last_map_pub = now

    def _on_odom(self, pose: PoseStamped) -> None:
        """Throttle and push a compact 2D pose (x/y/yaw) on map_unreliable so
        the marker moves at odom rate between the slower map frames."""
        now = time.monotonic()
        if now - self._last_odom_pub < 1.0 / self.config.odom_hz:
            return
        yaw = float(pose.orientation.to_euler().yaw)
        payload = {
            "type": "odom",
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": yaw,
            "ts": float(pose.ts),
        }
        try:
            self.map_out.publish(json.dumps(payload).encode())
        except Exception:
            logger.debug("odom publish failed", exc_info=True)
            return
        self._last_odom_pub = now

    def _on_audio_frame(self, pcm: bytes, sample_rate: int, channels: int) -> None:
        """Operator mic frame → speaker track + audio_out (8-byte header:
        sample_rate u32, channels u16, format u16=0 for s16, then raw PCM)."""
        track = self._speaker_track
        if track is not None:
            track.push(pcm, sample_rate, channels)
        try:
            self.audio_out.publish(struct.pack("<IHH", sample_rate, channels, 0) + pcm)
        except Exception:
            # No consumer/transport bound is the norm until something subscribes.
            logger.debug("audio publish failed", exc_info=True)

    def _attach_speaker(self) -> None:
        """Feed the dog PC's negotiated audio m-line: replaceTrack + enable the
        audio channel. Best-effort — sim/replay have no PC and just skip."""
        # Driver internals, not on the connection protocol — narrow via getattr.
        drv = getattr(self.connection, "conn", None)  # unitree_webrtc_connect driver
        loop = getattr(self.connection, "loop", None)
        pc = getattr(drv, "pc", None)
        if drv is None or pc is None or loop is None:
            logger.debug("speaker: connection has no WebRTC PC (sim/replay) — skipped")
            return
        try:
            sender = next((t.sender for t in pc.getTransceivers() if t.kind == "audio"), None)
            if sender is None:
                logger.warning("speaker: dog PC has no audio transceiver")
                return
            self._speaker_track = PCMAudioTrack()
            loop.call_soon_threadsafe(sender.replaceTrack, self._speaker_track)
            loop.call_soon_threadsafe(drv.datachannel.switchAudioChannel, True)
            logger.debug("speaker: operator audio track attached")
        except Exception:
            self._speaker_track = None
            logger.exception("speaker attach failed — operator audio won't play on the dog")

    @staticmethod
    def _block_max(cells: Any, factor: int) -> Any:
        """Downsample an int8 occupancy grid by block maximum (max not mean, so
        coarsening never erases an obstacle; unknown -1 is lowest priority)."""
        import numpy as np

        h, w = cells.shape[:2]
        new_h, new_w = h // factor, w // factor
        if new_h == 0 or new_w == 0:
            return cells
        trimmed = cells[: new_h * factor, : new_w * factor]
        blocks = trimmed.reshape(new_h, factor, new_w, factor)
        # Sink unknown below every known value for the max, then map it back.
        as_int = blocks.astype(np.int16)
        as_int[as_int < 0] = -1
        known = np.where(as_int < 0, -1000, as_int)
        reduced = known.max(axis=(1, 3))
        reduced[reduced == -1000] = -1
        return reduced.astype(np.int8)

    @staticmethod
    def _occupancy_to_bgra(cells: Any) -> Any:
        """Colorize occupancy int8 {-1,0,1..100} → BGRA (cv2 order) for a PNG:
        free/obstacle/lethal in the cockpit cyan, unknown transparent."""
        import numpy as np

        # (B, G, R, A) — RGB reversed for OpenCV.
        c_unknown = (0, 0, 0, 0)  # transparent
        c_free = (68, 58, 30, 255)  # #1e3a44 dark cyan
        c_occupied = (239, 220, 143, 255)  # #8fdcef bright cyan
        c_lethal = (255, 255, 255, 255)  # #ffffff white-hot

        out = np.empty((*cells.shape, 4), dtype=np.uint8)
        out[...] = c_unknown  # default; -1 stays transparent
        out[cells == 0] = c_free
        out[cells >= 1] = c_occupied
        out[cells >= 100] = c_lethal
        return out

    def _battery_soc(self) -> int | None:
        """Battery SOC from the cached lowstate (not the logged skill, which the
        3 Hz telemetry loop would spam)."""
        try:
            return int(self._latest_lowstate["data"]["bms_state"]["soc"])  # type: ignore[index]
        except (KeyError, TypeError, ValueError):
            return None

    def _telemetry_extra(self) -> dict[str, Any]:
        return {"soc": self._battery_soc()}

    def _telemetry_state(self) -> dict[str, Any]:
        """Robot UI state (cams + estopped merged in by the mixin)."""
        return {
            "posture": self._posture,
            "rage": self._rage_active,
            "obstacle_avoidance": self._obstacle_avoidance,
            "light": self._light,  # brightness 0..1
        }

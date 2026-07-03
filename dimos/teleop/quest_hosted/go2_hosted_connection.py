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

import base64
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import json
import struct
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
    set_audio_sink,
    shutdown_all_providers,
)
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
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
    # Map/odom overlay on the map_unreliable channel. Map is slow + few-KB; odom
    # is fast + tiny so the marker moves smoothly. Coarsen keeps the PNG < 16 KB.
    map_hz: float = 2.0  # occupancy-grid push rate (0 = off)
    map_min_resolution: float = 0.1  # coarsen finer grids to this m/cell before encode
    odom_hz: float = 15.0  # robot-pose push rate (0 = off)


class Go2HostedConnection(GO2Connection, HostedConnectionMixin):
    """GO2Connection + the hosted-teleop state plane, colocated (one session).

    The shared control plane (state_json dispatch, camera mux + latency
    stamp, cmd_ack, telemetry loop) lives in ``HostedConnectionMixin``; this
    class adds the Go2 parts: sport commands, rage mode, obstacle avoidance,
    the head LED, and the serialized command executor they run on.
    """

    config: Go2HostedConnectionConfig

    state_json: In[bytes]
    cmd_raw: In[bytes]
    video_stats: Out[VideoStats]
    telemetry_out: Out[bytes]
    cam2_in: In[Image]
    mux_image: Out[Image]
    cmd_vel_stamped: Out[TwistStamped]
    # Map overlay input (occupancy grid) + output (compressed map/odom on the
    # dedicated map_unreliable channel). Odom is not a stream — we tap
    # connection.odom_stream() in start() (the source the base uses for TF).
    global_costmap: In[OccupancyGrid]
    map_out: Out[bytes]
    # Operator mic audio, re-published for local consumers (Go2 has no speaker
    # yet). 8-byte header (sample_rate u32, channels u16, 0=s16 u16) + PCM.
    # Frames only flow when the provider runs with transports.broker.audio_in=true.
    audio_out: Out[bytes]

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
        # E-STOP latch from _hosted_init: while set, move() refuses twists and
        # non-urgent commands are rejected until estop_clear.
        self._hosted_init(["cam1", "cam2"])
        self._stop_event = threading.Event()
        self._rage_active = False
        self._last_cmd_ts = 0.0
        # Single worker (repo pattern, cf. utils/threadpool + drake_world):
        # commands execute strictly in order, so state like _rage_active can't
        # race between overlapping runners. Bounded by _MAX_PENDING_CMDS.
        self._cmd_executor: ThreadPoolExecutor | None = None
        self._cmd_pending = 0
        self._cmd_lock = threading.Lock()
        # nonce → (result | None while in flight, monotonic stamp)
        self._nonce_results: dict[Any, tuple[bool | None, float]] = {}
        # Robot-authoritative UI state, pushed in telemetry so a reconnecting
        # operator's cockpit reflects reality instead of optimistic defaults.
        # GO2Connection.start() stands the robot up, hence the initial posture.
        self._posture = "StandReady"
        self._obstacle_avoidance = True  # corrected from config.g in start()
        self._light = 0.0  # head-LED brightness 0..1, assumed off until set
        # Map/odom throttle gates (monotonic stamps, cf. the mux's _last_mux_pub).
        self._last_map_pub = 0.0
        self._last_odom_pub = 0.0

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
        # Map overlay → operator minimap; throttled + compressed in the handlers.
        if self.config.map_hz > 0:
            self.register_disposable(
                Disposable(self.global_costmap.subscribe(self._on_costmap))
            )
        # Odom: tap the raw stream the base consumes for TF (no stream port).
        if self.config.odom_hz > 0:
            self.register_disposable(
                self.connection.odom_stream().subscribe(self._on_odom)
            )
        # Operator mic → audio_out. The subscribes above already forced the
        # broker provider into existence, so the registry sweep finds it; frames
        # only arrive when it runs with audio_in=true.
        if set_audio_sink(self._on_audio_frame):
            logger.info("operator audio sink wired")
        self._start_telemetry()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._cmd_executor is not None:
            self._cmd_executor.shutdown(wait=False, cancel_futures=True)
            self._cmd_executor = None
        self._stop_telemetry()
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

        # E-STOP latch: only urgent work (Damp itself) may run while latched.
        if self._estopped and not urgent:
            logger.warning("%s rejected: E-STOP latched", label)
            self._send_ack(nonce, False)
            return

        # Nonce dedup: a duplicate of a finished command re-acks its result;
        # a duplicate of an in-flight one is dropped (the original will ack).
        # Transient rejections below unwind the reservation so a genuine
        # retry can still execute.
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

    # ─── Map overlay (robot → operator minimap, on map_unreliable) ──

    def _on_costmap(self, grid: OccupancyGrid) -> None:
        """Throttle, coarsen, colorize, and push an occupancy grid to the operator.

        Rides map_out (map_unreliable channel). Coarsen + PNG keeps the payload
        under the 16 KB CF datachannel ceiling. Best-effort — dropped downstream
        while no operator is connected.
        """
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
            logger.debug("map encode failed", exc_info=True)
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
        try:
            self.map_out.publish(json.dumps(payload).encode())
        except Exception:
            logger.debug("map publish failed", exc_info=True)
            return
        self._last_map_pub = now

    def _on_odom(self, pose: PoseStamped) -> None:
        """Throttle the Go2 pose and push a compact 2D pose to the operator.

        Rides map_unreliable alongside the map (same channel, distinct "type") so
        the marker moves at odom rate between the slower map frames. Only x/y/yaw
        — planar yaw is derived here so the browser needs no quaternion math.
        """
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
        """Operator mic frame (from the broker provider's audio track) →
        audio_out. Self-describing: 8-byte header (sample_rate u32, channels
        u16, format u16 = 0 for s16 interleaved) then the raw PCM."""
        try:
            self.audio_out.publish(struct.pack("<IHH", sample_rate, channels, 0) + pcm)
        except Exception:
            # No consumer/transport bound is the norm until something subscribes.
            logger.debug("audio publish failed", exc_info=True)

    @staticmethod
    def _block_max(cells: Any, factor: int) -> Any:
        """Downsample an int8 occupancy grid by `factor` via block maximum.

        Max (not mean) so coarsening never erases an obstacle. Unknown (-1) is
        lowest priority — a block with any known cell reports the known state.
        """
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
        """Colorize occupancy int8 {-1,0,1..100} → BGRA for a color PNG.

        Single cyan hue matching the cockpit accent: dark cyan free, bright cyan
        obstacles, white-hot lethal. Unknown is transparent so the map floats
        over the canvas. BGRA because cv2.imencode uses OpenCV order.
        """
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
        """Battery SOC from the cached lowstate, without invoking the logged
        ``get_battery_soc`` skill (which the 3 Hz telemetry loop would spam)."""
        try:
            return int(self._latest_lowstate["data"]["bms_state"]["soc"])  # type: ignore[index]
        except (KeyError, TypeError, ValueError):
            return None

    def _telemetry_extra(self) -> dict[str, Any]:
        return {"soc": self._battery_soc()}

    def _telemetry_state(self) -> dict[str, Any]:
        """Robot-authoritative UI state — posture, rage, obstacle avoidance,
        head LED (cams + estopped are merged in by the mixin)."""
        return {
            "posture": self._posture,
            "rage": self._rage_active,
            "obstacle_avoidance": self._obstacle_avoidance,
            "light": self._light,  # brightness 0..1
        }


__all__ = ["Go2HostedConnection", "Go2HostedConnectionConfig"]

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

"""Hosted stats module: state-plane stats dispatch + telemetry push.

Handles the stats kinds on ``state_json`` (video_stats, clock_report), taps
``cmd_raw`` for command-link stats, and pushes the periodic telemetry frame to
the operator (``telemetry_out``) and the recorder (``robot_telemetry``, local
LCM — the broker channel is outbound-only and can't be tapped). UI state comes
robot-authoritative from the command module on ``robot_state``.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.utils.stream_stats import LiveStreamStats
from dimos.teleop.utils.video_stats import VideoStats
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class HostedStatsConfig(ModuleConfig):
    telemetry_hz: float = 3.0


class HostedStatsModule(Module):
    """State-plane stats dispatch, cmd-link stats, and the robot_telemetry push."""

    config: HostedStatsConfig

    # RPC ref to the driver, for battery SOC pulled in the telemetry loop.
    # Optional: the xarm blueprints have no GO2Connection — soc stays None there.
    go2: GO2Connection | None

    state_json: In[bytes]
    cmd_raw: In[bytes]
    robot_state: In[bytes]
    telemetry_out: Out[bytes]
    robot_telemetry: Out[bytes]
    video_stats: Out[VideoStats]
    cmd_vel_stamped: Out[TwistStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._cmd_stats = LiveStreamStats()
        self._telemetry_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._latest_state: dict[str, Any] = {}

    @rpc
    def start(self) -> None:
        """Subscribe state_json/cmd_raw/robot_state; start the telemetry loop."""
        super().start()
        self._stop_event.clear()
        self.register_disposable(Disposable(self.state_json.subscribe(self._on_state_json)))
        self.register_disposable(Disposable(self.cmd_raw.subscribe(self._on_cmd_raw)))
        self.register_disposable(Disposable(self.robot_state.subscribe(self._on_robot_state)))
        self._start_telemetry()

    @rpc
    def stop(self) -> None:
        """Stop the telemetry loop."""
        self._stop_event.set()
        if self._telemetry_thread is not None:
            self._telemetry_thread.join(timeout=2.0)
            if self._telemetry_thread.is_alive():
                logger.warning("telemetry thread did not stop within 2s")
            self._telemetry_thread = None
        super().stop()

    # ─── inbound state plane (stats kinds only) ───────────────────────

    def _on_state_json(self, data: Any) -> None:
        """Handle the stats kinds; other kinds belong to the command/camera modules."""
        if isinstance(data, str):
            data = data.encode()
        if not data.startswith(b"{"):
            return
        try:
            msg = json.loads(data)
        except ValueError:
            logger.warning("state_reliable: malformed JSON: %r", data[:80])
            return

        kind = msg.get("type")
        if kind == "video_stats":
            try:
                self.video_stats.publish(VideoStats.from_dict(msg))
            except (TypeError, ValueError):
                logger.warning("state_reliable: malformed video_stats, dropping")
        elif kind == "clock_report":
            logger.info(
                "clock-sync: operator rtt=%s offset=%s",
                msg.get("rtt_ms"),
                msg.get("offset_ms"),
            )

    def _on_cmd_raw(self, data: Any) -> None:
        """Tap raw cmd_vel for stats and re-publish for the recorder — the full
        unguarded operator stream, before the command module's filtering."""
        if isinstance(data, str):
            data = data.encode()
        try:
            cmd = TwistStamped.lcm_decode(data)
        except Exception:
            return
        self._cmd_stats.record(cmd.ts, nbytes=len(data))
        self.cmd_vel_stamped.publish(cmd)

    def _on_robot_state(self, data: Any) -> None:
        """Cache the robot-authoritative UI state pushed by the command module."""
        if isinstance(data, str):
            data = data.encode()
        try:
            self._latest_state = json.loads(data)
        except (ValueError, TypeError):
            logger.warning("robot_state: malformed, keeping previous")

    # ─── telemetry (robot → operator) ─────────────────────────────────

    def _telemetry_payload(self) -> dict[str, Any]:
        """One robot_telemetry frame: cmd stats + latest robot_state + battery."""
        soc = None
        if self.go2 is not None:
            try:
                soc = self.go2.battery_soc()
            except Exception:
                pass
        return {
            "type": "robot_telemetry",
            "cmd": self._cmd_stats.snapshot(),
            "soc": soc,
            "state": self._latest_state,
            "robot_ts": time.time(),
        }

    def _start_telemetry(self) -> None:
        def runner() -> None:
            interval = 1.0 / max(self.config.telemetry_hz, 0.1)
            warned = False  # log the first failure of a streak, not every tick
            while not self._stop_event.is_set():
                data = json.dumps(self._telemetry_payload()).encode()
                try:
                    self.robot_telemetry.publish(data)  # → recorder (local LCM)
                    self.telemetry_out.publish(data)  # → operator (broker)
                    warned = False
                except Exception:
                    if not warned:
                        warned = True
                        logger.warning("telemetry publish failing", exc_info=True)
                self._stop_event.wait(interval)

        self._telemetry_thread = threading.Thread(
            target=runner, daemon=True, name="HostedStatsTelemetry"
        )
        self._telemetry_thread.start()

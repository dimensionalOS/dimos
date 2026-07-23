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

Owns the operator↔robot state plane that is NOT robot-command-specific:
- parses inbound ``state_json`` and handles the stats kinds (video_stats,
  clock_report). Command/estop/camera_select kinds are owned by other modules,
  which share the same inbound channel (the provider fans one inbound channel
  to every subscriber),
- taps ``cmd_raw`` for command-link latency/rate stats,
- pushes the periodic telemetry frame (cmd stats + soc + robot state) to the
  operator on ``telemetry_out`` (state_reliable_back), and the same payload on
  ``robot_telemetry`` (local LCM) so the recorder can capture it — the broker
  channel is outbound-only and can't be tapped locally.

Robot-authoritative UI state (posture/rage/battery) arrives on ``robot_state``
from the command module, so telemetry reflects reality.
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
    go2: GO2Connection

    state_json: In[bytes]  # broker state_reliable (fanned; also read by command mod)
    cmd_raw: In[bytes]  # cmd_unreliable stats tap
    robot_state: In[bytes]  # robot-authoritative UI state from the command module
    telemetry_out: Out[bytes]  # → CloudflareTransport("state_reliable_back")
    robot_telemetry: Out[bytes]  # same payload on a local stream → recorder (LCM)
    video_stats: Out[VideoStats]
    cmd_vel_stamped: Out[TwistStamped]  # decoded operator cmd → recorder (LCM)

    def __init__(self, **kwargs: Any) -> None:
        """Init cmd-stats accumulator, telemetry thread handle, latest state."""
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
            self._telemetry_thread = None
        super().stop()

    # ─── inbound state plane (stats kinds only) ───────────────────────

    def _on_state_json(self, data: Any) -> None:
        """Handle stats kinds (video_stats/clock_report); ignore the rest — the
        command / camera modules own their kinds on this shared channel."""
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
        """Tap raw cmd_vel for latency/rate stats and re-publish it as
        TwistStamped over LCM for the recorder (no 2nd CF session). This is the
        full unguarded operator stream — the complete drive trace, unlike the
        E-STOP/stale-filtered subset Go2CommandModule forwards to the driver."""
        if isinstance(data, str):
            data = data.encode()
        try:
            cmd = TwistStamped.lcm_decode(data)
        except Exception:
            return  # foreign / undecodable frame — skip
        self._cmd_stats.record(cmd.ts, nbytes=len(data))
        self.cmd_vel_stamped.publish(cmd)

    def _on_robot_state(self, data: Any) -> None:
        """Cache the robot-authoritative UI state pushed by the command module."""
        if isinstance(data, str):
            data = data.encode()
        try:
            self._latest_state = json.loads(data)
        except (ValueError, TypeError):
            logger.debug("robot_state: malformed, keeping previous")

    # ─── telemetry (robot → operator) ─────────────────────────────────

    def _telemetry_payload(self) -> dict[str, Any]:
        """One robot_telemetry frame: cmd stats + latest robot_state + battery."""
        try:
            soc = self.go2.battery_soc()
        except Exception:
            soc = None
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
                    self.telemetry_out.publish(data)  # → operator (broker)
                    self.robot_telemetry.publish(data)  # → recorder (local LCM)
                    warned = False
                except Exception:
                    if not warned:
                        warned = True
                        logger.debug("telemetry publish failing", exc_info=True)
                self._stop_event.wait(interval)

        self._telemetry_thread = threading.Thread(
            target=runner, daemon=True, name="HostedStatsTelemetry"
        )
        self._telemetry_thread.start()

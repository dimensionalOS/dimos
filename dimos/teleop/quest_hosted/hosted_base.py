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

"""Shared hosted-teleop control plane (state_json dispatch, cmd_ack, E-STOP
latch, telemetry loop, camera mux) for robot connection modules. See
``Go2HostedConnection`` for the reference shape.

Host contract: declare the ``state_json``/``telemetry_out``/``mux_image``/
``video_stats`` streams (all on ONE module = one broker session) + an
In[Image] per camera; provide ``self._stop_event`` and the ``telemetry_hz`` /
mux config; call ``_hosted_init(cameras)`` in ``__init__``, wire subscriptions
+ ``_start_telemetry()`` in ``start()``, ``_stop_telemetry()`` in ``stop()``.

Required hooks: ``_handle_estop``, ``_handle_estop_clear``,
``_on_operator_lost``, ``_telemetry_state``. Optional: ``_handle_robot_msg``,
``_telemetry_extra``. Clock-sync pings are answered by the provider, not here.
"""

from __future__ import annotations

# Runtime import (not TYPE_CHECKING): blueprint.config() runs get_type_hints()
# on this class, evaluating the class-level hook annotations below at runtime.
from collections.abc import Callable
import json
import threading
import time
from typing import Any

from dimos.teleop.utils.camera_mux import CameraMuxMixin
from dimos.teleop.utils.stream_stats import LiveStreamStats
from dimos.teleop.utils.video_stats import VideoStats
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class HostedConnectionMixin(CameraMuxMixin):
    """Hosted control plane: state_json dispatch + acks + telemetry + mux."""

    def _hosted_init(self, cameras: list[str]) -> None:
        """Set up hosted state; call from the host module's ``__init__``."""
        self._mux_init(cameras)
        self._cmd_stats = LiveStreamStats()
        self._telemetry_thread: threading.Thread | None = None
        self._estopped = False

    # ─── Inbound state plane (operator → robot) ───────────────────────

    def _on_state_json(self, data: Any) -> None:
        """Dispatch a JSON control-plane message from state_reliable."""
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
        if kind == "estop":
            self._handle_estop(msg.get("nonce"))
        elif kind == "estop_clear":
            self._handle_estop_clear(msg.get("nonce"))
        elif kind == "operator_lost":  # synthetic, injected by the provider
            self._on_operator_lost()
        elif kind == "camera_select":
            self._set_cam_selection(msg.get("cams", []))
        elif kind == "video_stats":
            try:
                self.video_stats.publish(VideoStats.from_dict(msg))  # type: ignore[attr-defined]
            except (TypeError, ValueError):
                logger.warning("state_reliable: malformed video_stats, dropping")
        elif kind == "clock_report":
            logger.info(
                "clock-sync: operator rtt=%s offset=%s",
                msg.get("rtt_ms"),
                msg.get("offset_ms"),
            )
        else:
            self._handle_robot_msg(kind, msg)

    # Required hooks — no default on purpose: a missing E-STOP path is a bug,
    # not a no-op.
    _handle_estop: Callable[[Any], None]
    _handle_estop_clear: Callable[[Any], None]
    _on_operator_lost: Callable[[], None]

    def _handle_robot_msg(self, kind: Any, msg: dict[str, Any]) -> None:
        """Optional hook for robot-specific state_json types; unknown kinds are
        ignored by default."""

    def _send_ack(self, nonce: Any, ok: bool) -> None:
        # Best-effort ack on state_reliable_back; warn on failure (a live
        # operator's button spins without it).
        try:
            self.telemetry_out.publish(  # type: ignore[attr-defined]
                json.dumps({"type": "cmd_ack", "nonce": nonce, "ok": ok}).encode()
            )
        except Exception:
            logger.warning("cmd_ack publish failed", exc_info=True)

    # ─── Telemetry (robot → operator) ─────────────────────────────────

    def _telemetry_state(self) -> dict[str, Any]:
        """Robot-authoritative UI state; implement in the host module."""
        raise NotImplementedError

    def _telemetry_extra(self) -> dict[str, Any]:
        """Extra top-level telemetry keys (e.g. battery soc)."""
        return {}

    def _telemetry_payload(self) -> dict[str, Any]:
        """One telemetry frame. `state` seeds a (re)connecting operator's
        cockpit from reality instead of optimistic defaults."""
        return {
            "type": "robot_telemetry",
            "cmd": self._cmd_stats.snapshot(),
            **self._telemetry_extra(),
            "state": {
                **self._telemetry_state(),
                "cams": self._mux_state(),
                "estopped": self._estopped,
            },
            "robot_ts": time.time(),
        }

    def _start_telemetry(self) -> None:
        def runner() -> None:
            interval = 1.0 / max(self.config.telemetry_hz, 0.1)  # type: ignore[attr-defined]
            while not self._stop_event.is_set():  # type: ignore[attr-defined]
                payload = json.dumps(self._telemetry_payload())
                try:
                    self.telemetry_out.publish(payload.encode())  # type: ignore[attr-defined]
                except Exception:
                    logger.debug("telemetry publish failed", exc_info=True)
                self._stop_event.wait(interval)  # type: ignore[attr-defined]

        self._telemetry_thread = threading.Thread(
            target=runner, daemon=True, name=f"{type(self).__name__}Telemetry"
        )
        self._telemetry_thread.start()

    def _stop_telemetry(self) -> None:
        """Join the telemetry thread; the host must set _stop_event first."""
        if self._telemetry_thread is not None:
            self._telemetry_thread.join(timeout=2.0)
            self._telemetry_thread = None

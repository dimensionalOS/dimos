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

"""Hosted operator plane for R1 Lite quest teleoperation.

The R1 Lite analog of the hosted ArmCommandModule: the same dual-arm,
gripper, and chassis teleop as R1LiteQuestTeleopModule, but operator
frames arrive from the broker over the ``cmd_raw`` plane instead of a
robot-local WebSocket server, with the WAN protections remote operation
needs — stale/future/out-of-order pose drops, an operator E-STOP latch
that also freezes the chassis, and command acks. Local teleop behavior
and configuration are inherited unchanged, so the remote feel matches
the validated local feel.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.control.coordinator import ControlCoordinator
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.galaxea.r1lite.quest_module import (
    R1LiteQuestTeleopConfig,
    R1LiteQuestTeleopModule,
)
from dimos.teleop.hosted.command_executor import SerializedCommandExecutor
from dimos.teleop.quest.quest_types import Hand, QuestControllerState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class R1LiteHostedTeleopConfig(R1LiteQuestTeleopConfig):
    cmd_stale_after_sec: float = 0.5


class R1LiteHostedTeleopModule(R1LiteQuestTeleopModule):
    """Broker-driven quest teleop for the R1 Lite: arms, grippers, chassis."""

    config: R1LiteHostedTeleopConfig

    coordinator: ControlCoordinator

    cmd_raw: In[bytes]
    state_json: In[bytes]
    cmd_ack: Out[bytes]
    robot_state: Out[bytes]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._estopped = False
        self._cmd = SerializedCommandExecutor(
            lambda nonce, ok: self._send_ack(nonce, ok), lambda: self._estopped
        )
        self._last_pose_ts = {Hand.LEFT: 0.0, Hand.RIGHT: 0.0}
        self._last_stale_warn = 0.0
        self._last_future_warn = 0.0

    # No local WebSocket server — the operator connects through the broker.
    def _start_server(self) -> None:
        pass

    def _stop_server(self) -> None:
        pass

    @rpc
    def start(self) -> None:
        super().start()
        self._cmd.start()
        for stream, cb in (
            (self.cmd_raw, self._on_cmd_raw),
            (self.state_json, self._on_state_json),
        ):
            self.register_disposable(Disposable(stream.subscribe(cb)))
        self._publish_robot_state()

    @rpc
    def stop(self) -> None:
        self._cmd.stop()
        super().stop()

    # ─── Inbound command plane (operator → robot) ─────────────────────

    def _on_cmd_raw(self, data: Any) -> None:
        """Fingerprint-dispatch LCM bytes into the inherited handlers."""
        if isinstance(data, str):
            data = data.encode()
        decoder = self._decoders.get(data[:8])
        if decoder is None:
            return
        try:
            decoder(data)
        except Exception:
            logger.warning("cmd_raw decode failed", exc_info=True)

    def _on_pose_bytes(self, data: bytes) -> None:
        """WAN guards in front of the inherited validation + conversion.

        Stale, future-stamped, and out-of-order poses are dropped before
        they can move an arm; everything that passes flows through the
        exact local-teleop path so remote feel matches local feel.
        """
        msg = PoseStamped.lcm_decode(data)
        if msg.frame_id == "left":
            hand = Hand.LEFT
        elif msg.frame_id == "right":
            hand = Hand.RIGHT
        else:
            return
        ts = float(msg.ts)
        if not math.isfinite(ts):
            return
        age = time.time() - ts
        if age > self.config.cmd_stale_after_sec:
            now = time.monotonic()
            if now - self._last_stale_warn >= 1.0:
                self._last_stale_warn = now
                logger.warning("dropping stale pose: age=%.2fs — operator link lagging", age)
            return
        if age < 0:  # future-stamped: advancing the watermark would freeze the hand
            now = time.monotonic()
            if now - self._last_future_warn >= 1.0:
                self._last_future_warn = now
                logger.warning("dropping future-stamped pose — operator clock sync likely off")
            return
        if ts <= self._last_pose_ts[hand]:
            return
        self._last_pose_ts[hand] = ts
        super()._on_pose_bytes(data)

    def _on_state_json(self, data: Any) -> None:
        """Operator E-STOP plane; gripper stays on the analog triggers."""
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

    def _send_ack(self, nonce: Any, ok: bool) -> None:
        try:
            self.cmd_ack.publish(json.dumps({"type": "cmd_ack", "nonce": nonce, "ok": ok}).encode())
        except Exception:
            logger.warning("cmd_ack publish failed", exc_info=True)

    # ─── E-STOP gating: arms via engagement, chassis via zero twist ───

    def _handle_engage(self) -> None:
        if self._estopped:
            for hand in Hand:
                if self._is_engaged[hand]:
                    self._disengage(hand)
            return
        super()._handle_engage()

    def _should_publish(self, hand: Hand) -> bool:
        return not self._estopped and super()._should_publish(hand)

    def _chassis_twist(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
        now: float,
    ) -> Twist:
        if self._estopped:
            return Twist.zero()
        return super()._chassis_twist(left, right, now)

    # ─── E-STOP / operator-loss hooks ─────────────────────────────────

    def _handle_estop(self, nonce: Any) -> None:
        """Latch first (gates input and chassis), disengage, then latch the
        coordinator off-thread; the ack carries the coordinator result."""
        self._estopped = True
        logger.warning("E-STOP latched by operator")
        with self._lock:
            self._disengage()
        try:
            self.cmd_vel.publish(Twist.zero())
        except Exception:
            logger.exception("E-STOP chassis zero publish failed")
        self._publish_robot_state()
        self._cmd.submit("estop", nonce, self._apply_coordinator_latch, urgent=True)

    def _handle_estop_clear(self, nonce: Any) -> None:
        """Re-arm; a still-held engage re-engages next tick and rebaselines."""
        self._estopped = False
        logger.warning("E-STOP cleared by operator")
        self._publish_robot_state()
        self._cmd.submit("estop_clear", nonce, self._apply_coordinator_latch, urgent=True)

    def _apply_coordinator_latch(self, _epoch: int) -> bool:
        estopped = self._estopped
        try:
            self.coordinator.set_estop(estopped)
            return True
        except Exception:
            logger.exception("coordinator.set_estop(%s) failed", estopped)
            return False

    def _on_operator_lost(self) -> None:
        """Disengage and halt the base so a dead link cannot keep it moving."""
        logger.warning("operator link lost — disengaging")
        with self._lock:
            self._disengage()
        try:
            self.cmd_vel.publish(Twist.zero())
        except Exception:
            logger.exception("operator-lost chassis zero publish failed")
        self._publish_robot_state()

    # ─── Robot-authoritative state → stats telemetry ──────────────────

    def _publish_robot_state(self) -> None:
        with self._lock:
            state = {
                "estopped": self._estopped,
                "engaged": {
                    "left": self._is_engaged[Hand.LEFT],
                    "right": self._is_engaged[Hand.RIGHT],
                },
            }
        try:
            self.robot_state.publish(json.dumps(state).encode())
        except Exception:
            logger.warning("robot_state publish failed", exc_info=True)

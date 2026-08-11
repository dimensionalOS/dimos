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

import dataclasses
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
    # Hand-gesture chassis drive: the hosted arm pipeline transmits full
    # Quest Joy frames but zero-fills the thumbstick axes, so the sticks
    # cannot drive remotely. Holding a secondary button (Y/B) turns that
    # hand into a virtual joystick instead: displacement from the press
    # anchor maps to velocity — left hand translates, right hand yaws —
    # with release-to-zero as the dead-man. Real stick input, if it ever
    # arrives, takes priority automatically.
    drive_gesture_full_scale_m: float = 0.15
    drive_gesture_deadband_m: float = 0.03


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
        # Per-hand drive-gesture anchor (robot-frame x, y at press time)
        self._drive_anchor: dict[Hand, tuple[float, float] | None] = {
            Hand.LEFT: None,
            Hand.RIGHT: None,
        }

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
        # A driving hand cannot hold (or gain) an arm engagement: its
        # primary is masked so the parent logic sees a released button,
        # disengages, and requires a fresh press — and with it a fresh
        # baseline — after the steering ends. No arm jump from the
        # distance the hand traveled while driving.
        masked: dict[Hand, QuestControllerState] = {}
        for hand in Hand:
            controller = self._controllers.get(hand)
            if controller is not None and controller.secondary and controller.primary:
                masked[hand] = controller
                self._controllers[hand] = dataclasses.replace(controller, primary=False)
        try:
            super()._handle_engage()
        finally:
            for hand, controller in masked.items():
                self._controllers[hand] = controller

    def _should_publish(self, hand: Hand) -> bool:
        # A hand that is driving the chassis must not stream arm targets.
        if self._drive_held(hand):
            return False
        return not self._estopped and super()._should_publish(hand)

    # ─── Hand-gesture chassis drive ───────────────────────────────────

    def _drive_held(self, hand: Hand) -> bool:
        controller = self._controllers.get(hand)
        return controller is not None and controller.secondary

    def _gesture_axis(self, delta: float) -> float:
        deadband = self.config.drive_gesture_deadband_m
        span = max(1e-6, self.config.drive_gesture_full_scale_m - deadband)
        magnitude = abs(delta)
        if magnitude <= deadband:
            return 0.0
        scaled = (magnitude - deadband) / span
        return math.copysign(min(1.0, scaled), delta)

    def _gesture_deltas(self, hand: Hand, now: float) -> tuple[float, float] | None:
        """Anchor-relative robot-frame hand displacement while driving."""
        pose = self._current_poses.get(hand)
        if pose is None or not self._drive_held(hand) or not self._fresh(hand, now):
            self._drive_anchor[hand] = None
            return None
        anchor = self._drive_anchor[hand]
        if anchor is None:
            self._drive_anchor[hand] = (pose.position.x, pose.position.y)
            return (0.0, 0.0)
        return (pose.position.x - anchor[0], pose.position.y - anchor[1])

    def _chassis_twist(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
        now: float,
    ) -> Twist:
        if self._estopped:
            self._drive_anchor[Hand.LEFT] = None
            self._drive_anchor[Hand.RIGHT] = None
            return Twist.zero()
        # Real stick input wins whenever the client provides it.
        for hand, controller in ((Hand.LEFT, left), (Hand.RIGHT, right)):
            if (
                controller is not None
                and self._fresh(hand, now)
                and (
                    abs(controller.thumbstick.x) > self.config.deadzone
                    or abs(controller.thumbstick.y) > self.config.deadzone
                )
            ):
                return super()._chassis_twist(left, right, now)
        twist = super()._chassis_twist(left, right, now)
        left_drive = self._gesture_deltas(Hand.LEFT, now)
        if left_drive is not None:
            dx, dy = left_drive
            twist.linear.x = self._gesture_axis(dx) * self.config.linear_speed
            twist.linear.y = self._gesture_axis(dy) * self.config.linear_speed
        right_drive = self._gesture_deltas(Hand.RIGHT, now)
        if right_drive is not None:
            _, dy = right_drive
            twist.angular.z = self._gesture_axis(dy) * self.config.angular_speed
        return twist

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

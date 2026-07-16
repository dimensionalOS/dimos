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

"""Arm hosted-teleop command module (module-based hosted pattern).

The arm analog of ``Go2CommandModule`` — the operator command/E-STOP plane for a
coordinator-driven arm. Unlike the Go2 (a stateless per-command RPC driver), the
arm streams a continuous engage/delta-pose at ``control_loop_hz``, so this module
SUBCLASSES ``ArmTeleopModule`` to inherit that control loop, engage handling, and
the LCM fingerprint decoder table. There is no robot driver to hold an RPC ref
to: actuation runs through the ControlCoordinator, fed over LCM.

    operator (WebXR) ──cmd_unreliable──▶ cmd_raw ─┐  (PoseStamped/Joy/TwistStamped
                                                  │   fingerprint-dispatched)
                                                  ├─ engage/delta-pose loop
    coordinator ◀── /coordinator_cartesian_command┘   (frame_id = task name)
                ◀── /teleop_buttons                     (analog triggers → gripper)
    coordinator ◀── /coordinator_ee_twist_command       (browser keyboard EE-twist)
                ◀── /gripper_command                     (gripper toggle, Bool)

The state plane (E-STOP / gripper JSON) arrives on ``state_json`` — the SAME
broker ``state_reliable`` channel the stats + camera-mux modules also read (the
provider fans one inbound channel to every subscriber); each filters for its own
``type``. Camera selection is owned by ``CameraMuxModule``; video_stats /
clock_report by ``HostedStatsModule``. This module owns estop / gripper.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.std_msgs.Bool import Bool
from dimos.protocol.pubsub.impl.webrtc.providers.spec import shutdown_all_providers
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.teleop.quest.quest_extensions import ArmTeleopConfig, ArmTeleopModule
from dimos.teleop.quest.quest_types import Hand
from dimos.teleop.utils.teleop_transforms import webxr_to_robot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ArmCommandConfig(ArmTeleopConfig):
    # task_names ("left"/"right" → coordinator task name) and control_loop_hz
    # come from ArmTeleopConfig; server_port is unused (no local web server).
    cmd_stale_after_sec: float = 0.5


class ArmCommandModule(ArmTeleopModule):
    """Operator command/E-STOP plane for a coordinator-driven arm."""

    config: ArmCommandConfig

    # Broker-bound (bind Cloudflare* transports to these in the blueprint).
    cmd_raw: In[bytes]  # cmd_unreliable: LCM PoseStamped/Joy/TwistStamped, dispatched
    state_json: In[bytes]  # state_reliable JSON (fanned; estop/gripper here)
    cmd_ack: Out[bytes]  # → state_reliable_back (command acks)
    robot_state: Out[bytes]  # robot-authoritative UI state → stats module telemetry

    # Robot-internal, over LCM. left/right_controller_output + teleop_buttons are
    # inherited from ArmTeleopModule (bound to the coordinator in the blueprint).
    coordinator_ee_twist_command: Out[TwistStamped]  # browser keyboard EE-twist
    gripper_command: Out[Bool]  # gripper toggle

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._estopped = False
        self._last_twist_ts = 0.0
        self._last_stale_warn = 0.0
        # Browser keyboard EE-twist rides cmd_unreliable alongside the VR pose:
        # register its LCM fingerprint on the inherited dispatch table.
        from dimos_lcm.geometry_msgs import TwistStamped as LCMTwistStamped

        self._decoders[LCMTwistStamped._get_packed_fingerprint()] = self._on_twist_bytes
        # While the E-STOP latch is set, hands are held disengaged and no poses
        # are published — the coordinator's TeleopIKTask keeps its last target,
        # so the arm freezes in place.

    # No local WebSocket server — the operator connects through the broker.
    def _start_server(self) -> None:
        pass

    def _stop_server(self) -> None:
        pass

    @rpc
    def start(self) -> None:
        super().start()  # Module start + control loop (web server no-op'd)
        # Sync subscribes (not async handle_*): keep-latest would drop bursts.
        for stream, cb in (
            (self.cmd_raw, self._on_cmd_raw),
            (self.state_json, self._on_state_json),
        ):
            self.register_disposable(Disposable(stream.subscribe(cb)))
        self._publish_robot_state()

    @rpc
    def stop(self) -> None:
        super().stop()  # stops the control loop + Module teardown
        # Graceful broker disconnect so the worker exits promptly instead of
        # being force-killed and reaped ~30s later.
        shutdown_all_providers()

    # ─── Inbound command plane (operator → robot) ─────────────────────

    def _on_cmd_raw(self, data: Any) -> None:
        """Fingerprint-dispatch LCM bytes from cmd_unreliable via the decoder
        table inherited from QuestTeleopModule (PoseStamped, Joy) plus the
        TwistStamped decoder registered in __init__."""
        if isinstance(data, str):
            data = data.encode()
        decoder = self._decoders.get(data[:8])
        if decoder is None:
            return  # foreign / undecodable frame — skip
        try:
            decoder(data)
        except Exception:
            logger.debug("cmd_raw decode failed", exc_info=True)

    def _on_pose_bytes(self, data: bytes) -> None:
        """Controller pose → robot frame. Overrides the quest version to drop
        (not raise on) unexpected frame_ids from the wire."""
        msg = PoseStamped.lcm_decode(data)
        try:
            hand = self._resolve_hand(msg.frame_id)
        except ValueError:
            return
        robot_pose = webxr_to_robot(msg, is_left_controller=(hand == Hand.LEFT))
        with self._lock:
            self._current_poses[hand] = robot_pose

    def _on_twist_bytes(self, data: bytes) -> None:
        """Browser keyboard EE-twist → coordinator's eef_twist task. Re-stamp
        frame_id with the task name so the coordinator routes it, and drop it
        while E-STOP is latched (mirrors the pose gate). Stale / future-stamped /
        out-of-order twists are dropped like the Go2 drive guard — the eef_twist
        task integrates each twist until the next one, so a late frame would
        jog the arm on an input the operator released long ago."""
        if self._estopped:
            return
        msg = TwistStamped.lcm_decode(data)
        ts = float(msg.ts)
        if not math.isfinite(ts):
            return
        age = time.time() - ts
        if age > self.config.cmd_stale_after_sec:
            now = time.monotonic()
            if now - self._last_stale_warn >= 1.0:
                self._last_stale_warn = now
                logger.warning("dropping stale ee_twist: age=%.2fs — operator link lagging", age)
            return
        if age < 0:  # future-stamped: don't advance _last_twist_ts (would stall the jog)
            return
        if ts <= self._last_twist_ts:  # out-of-order
            return
        self._last_twist_ts = ts
        self.coordinator_ee_twist_command.publish(
            TwistStamped(
                frame_id=EEF_TWIST_TASK_NAME,
                linear=[msg.linear.x, msg.linear.y, msg.linear.z],
                angular=[msg.angular.x, msg.angular.y, msg.angular.z],
                ts=msg.ts,
            )
        )

    def _on_state_json(self, data: Any) -> None:
        """Dispatch the state kinds this module owns (estop / gripper); ignore
        the rest — the stats / camera modules own their kinds on this shared
        channel."""
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
        elif kind == "gripper":
            self.gripper_command.publish(Bool(data=bool(msg.get("closed", False))))

    def _send_ack(self, nonce: Any, ok: bool) -> None:
        """Publish a cmd_ack on the broker-back channel."""
        try:
            self.cmd_ack.publish(json.dumps({"type": "cmd_ack", "nonce": nonce, "ok": ok}).encode())
        except Exception:
            logger.warning("cmd_ack publish failed", exc_info=True)

    # ─── E-STOP gating over the inherited control loop ────────────────

    def _handle_engage(self) -> None:
        """While E-STOP is latched, refuse engagement (and drop any lingering
        engage) so clearing the latch never resumes motion by itself — the
        operator must release and re-press the engage button."""
        if self._estopped:
            for hand in Hand:
                if self._is_engaged[hand]:
                    self._disengage(hand)
            return
        super()._handle_engage()

    def _should_publish(self, hand: Hand) -> bool:
        # Belt to _handle_engage's braces: the latch can flip mid-iteration
        # (subscriber thread), so gate the publish path too.
        return not self._estopped and super()._should_publish(hand)

    # ─── E-STOP / operator-loss hooks ─────────────────────────────────

    def _handle_estop(self, nonce: Any) -> None:
        """Latch FIRST (gates the control loop immediately), then disengage."""
        self._estopped = True
        logger.warning("E-STOP latched by operator")
        with self._lock:
            self._disengage()
        self._publish_robot_state()  # UI must show estopped:true immediately
        self._send_ack(nonce, True)

    def _handle_estop_clear(self, nonce: Any) -> None:
        """Re-arm. Deliberately does NOT resume motion — hands stay disengaged
        until the operator releases and re-presses the engage button (which
        recaptures the initial pose, so the delta restarts from zero)."""
        self._estopped = False
        logger.warning("E-STOP cleared by operator")
        self._publish_robot_state()
        self._send_ack(nonce, True)

    def _on_operator_lost(self) -> None:
        """Command plane gone: disengage so a stale engage can't keep streaming
        the last delta into the coordinator when the operator reconnects."""
        logger.warning("operator link lost — disengaging")
        with self._lock:
            self._disengage()
        self._publish_robot_state()

    # ─── Robot-authoritative state → stats module telemetry ───────────

    def _publish_robot_state(self) -> None:
        """Push per-hand engage state + estopped on robot_state (LCM) so the
        stats module's telemetry frame reflects reality."""
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
            logger.debug("robot_state publish failed", exc_info=True)


__all__ = ["ArmCommandConfig", "ArmCommandModule"]

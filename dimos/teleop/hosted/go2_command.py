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

"""Go2 command plane: operator command / E-STOP / drive dispatch.

Driver calls go over RPC (``go2: GO2Connection`` ref); operator-facing planes go
over transport — commands in on ``state_json``, acks out on ``cmd_ack``. Drive is
a guarded stream filter: operator cmd_vel → E-STOP/stale/reorder/nav-yield guard
→ ``tele_cmd_vel``.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

from dimos_lcm.std_msgs import Bool
from reactivex.disposable import Disposable
from unitree_webrtc_connect.constants import SPORT_CMD

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.hosted.command_executor import SerializedCommandExecutor
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_ALLOWED_SPORT_NAMES = frozenset(
    {"StandDown", "RecoveryStand", "Sit", "Hello", "Stretch", "Damp", "FrontPounce", "FrontJump"}
)
# Sorted so dict order is hash-seed independent: tests parametrize over this
# dict, and unstable order breaks pytest-xdist collection across workers.
ALLOWED_SPORT_CMDS: dict[str, int] = {n: SPORT_CMD[n] for n in sorted(_ALLOWED_SPORT_NAMES)}
_POSTURE_SPORT_CMDS = frozenset({"StandDown", "RecoveryStand", "Sit", "Damp"})
_ACROBATIC_SPORT_CMDS = frozenset({"FrontPounce", "FrontJump"})


def _all_finite(t: Twist) -> bool:
    """True when every linear/angular component is finite (no NaN/inf)."""
    return all(
        math.isfinite(v)
        for v in (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
    )


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class Go2CommandConfig(ModuleConfig):
    cmd_stale_after_sec: float = 0.5
    damp_on_operator_lost: bool = False
    max_nav_goal_m: float = 100.0
    allow_acrobatics: bool = False
    # Robot-side drive clamps (untrusted operator; browser scaling isn't a boundary).
    max_linear_mps: float = 1.5
    max_angular_rps: float = 2.0


class Go2CommandModule(Module):
    """Operator command/E-STOP/drive plane, driving GO2Connection over RPC."""

    config: Go2CommandConfig

    # RPC ref to the driver (framework injects an RPCClient) for discrete commands.
    go2: GO2Connection

    state_json: In[bytes]  # broker state_reliable (also read by stats mod)
    cmd_ack: Out[bytes]  # → state_reliable_back (command acks)

    # Manual drive: raw operator cmd_vel IN, guarded, republished as tele_cmd_vel
    # to MovementManager (which arbitrates manual vs nav and owns cmd_vel).
    cmd_vel_in: In[Twist]  # raw operator drive (broker cmd_unreliable)
    tele_cmd_vel: Out[Twist]  # guarded manual drive → MovementManager

    goal_request: Out[PoseStamped]  # click-to-nav goal → planner
    robot_state: Out[bytes]  # posture/rage/battery → stats module telemetry
    stop_movement: Out[Bool]  # cancel the planner (on E-STOP / operator-lost)

    def __init__(self, **kwargs: Any) -> None:
        """Init command state (executor, safety epoch, posture, drive timers)."""
        super().__init__(**kwargs)
        self._estopped = False
        self._cmd = SerializedCommandExecutor(
            lambda nonce, ok: self._send_ack(nonce, ok), lambda: self._estopped
        )
        self._rage_active = False
        self._obstacle_avoidance = True
        self._light = 0.0
        self._posture = "StandReady"
        self._last_cmd_ts = 0.0
        self._last_cmd_nonzero = False  # was the last forwarded drive frame moving?

    @rpc
    def start(self) -> None:
        """Wire state_json/drive/nav subscriptions; start the command executor."""
        super().start()
        self._cmd.start()
        self.register_disposable(Disposable(self.state_json.subscribe(self._on_state_json)))
        self.register_disposable(Disposable(self.cmd_vel_in.subscribe(self._on_cmd_vel_in)))
        self._publish_robot_state()

    @rpc
    def stop(self) -> None:
        """Shut the executor."""
        self._cmd.stop()
        super().stop()

    def _send_ack(self, nonce: Any, ok: bool) -> None:
        """Publish a cmd_ack on the broker-back channel."""
        try:
            self.cmd_ack.publish(json.dumps({"type": "cmd_ack", "nonce": nonce, "ok": ok}).encode())
        except Exception:
            logger.warning("cmd_ack publish failed", exc_info=True)

    # ─── inbound command dispatch (state_json over transport) ─────────

    def _on_state_json(self, data: Any) -> None:
        """Dispatch command/estop kinds; ignore stats kinds (stats module owns)."""
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
        elif kind == "sport_cmd":
            self._handle_sport_cmd(msg)
        elif kind == "set_mode":
            self._handle_set_mode(msg)
        elif kind == "obstacle_avoidance":
            self._handle_obstacle_avoidance(msg)
        elif kind == "light":
            self._handle_light(msg)
        elif kind == "nav_goal":
            self._handle_nav_goal(msg)
        elif kind == "nav_cancel":
            self._handle_nav_cancel(msg.get("nonce"))

    # ─── E-STOP + operator-loss ───────────────────────────────────────

    def _handle_estop(self, nonce: Any) -> None:
        """Latch E-STOP (drive filter reads the latch → stops tele_cmd_vel
        instantly), bump safety epoch, urgently Damp via go2 RPC."""
        self._estopped = True
        self._cmd.bump_safety_epoch()
        logger.warning("E-STOP latched by operator")
        self._cancel_nav()
        # Publish the latch NOW, not inside the Damp task — the UI must show
        # estopped:true even if Damp is slow or fails.
        self._publish_robot_state()

        def task(_ep: int) -> bool:
            ok = bool(self.go2.sport_command(ALLOWED_SPORT_CMDS["Damp"]))
            if ok:
                self._posture = "Damp"
                self._publish_robot_state()
            return ok

        self._cmd.submit("estop", nonce, task, urgent=True)

    def _handle_estop_clear(self, nonce: Any) -> None:
        """Un-latch; cancel any active plan; does not move the robot."""
        self._cancel_nav()
        self._estopped = False
        logger.warning("E-STOP cleared by operator")
        self._publish_robot_state()  # clear estopped:true in the UI immediately
        self._send_ack(nonce, True)

    def _on_operator_lost(self) -> None:
        """Stop motion, bump epoch, clear nonces, optionally Damp."""
        logger.warning("operator link lost — stopping motion")
        self._cmd.bump_safety_epoch()
        self._cancel_nav()
        self._cmd.clear_nonces()
        try:
            self.go2.stop_movement()
        except Exception:
            logger.exception("stop_movement on operator loss failed")
        if self.config.damp_on_operator_lost:
            self._cmd.submit(
                "damp_on_operator_lost",
                None,
                lambda _ep: bool(self.go2.sport_command(ALLOWED_SPORT_CMDS["Damp"])),
                urgent=True,
            )

    # ─── discrete commands (RPC to the driver) ────────────────────────

    def _handle_sport_cmd(self, msg: dict[str, Any]) -> None:
        """Allow-listed sport cmd → go2.sport_command(api_id) RPC → ack."""
        name = msg.get("name")
        nonce = msg.get("nonce")

        if name == "StandReady":
            self._cmd.submit("StandReady", nonce, self._stand_ready_task)
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

        def task(_ep: int) -> bool:
            ok = bool(self.go2.sport_command(api_id))
            if ok and name in _POSTURE_SPORT_CMDS:
                self._posture = name
                self._publish_robot_state()
            return ok

        self._cmd.submit(f"sport_cmd {name}", nonce, task, urgent=(name == "Damp"))

    def _stand_ready_task(self, epoch: int) -> bool:
        """Standup → RecoveryStand → BalanceStand → joystick via go2 RPC, aborts
        if E-STOP / operator-lost fires after submission (epoch-fenced)."""

        def _step(label: str, ok: object) -> bool:
            if not ok:
                logger.warning("StandReady: %s failed", label)
            return bool(ok)

        def _fenced_sleep(sec: float) -> bool:
            time.sleep(sec)
            if not self._cmd.safety_ok(epoch):
                logger.warning("StandReady aborted: E-STOP / operator-lost mid-sequence")
                return False
            return True

        if not _step("standup", self.go2.standup()):
            return False
        if not _fenced_sleep(3.0):
            return False
        if not _step("RecoveryStand", self.go2.sport_command(ALLOWED_SPORT_CMDS["RecoveryStand"])):
            return False
        if not _fenced_sleep(0.3):
            return False
        if not _step("balance_stand", self.go2.balance_stand()):
            return False
        if not _fenced_sleep(0.3):
            return False
        if not _step("switch_joystick", self.go2.switch_joystick(True)):
            return False
        self._posture = "StandReady"
        self._publish_robot_state()
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

        def task(epoch: int) -> bool:
            if want_rage == self._rage_active:
                return True
            # set_rage_mode is a ~2.3s blocking driver sequence we can't fence over
            # RPC; if E-STOP fired during it, re-Damp (its trailing BalanceStand/
            # SwitchJoystick may have re-enabled motion past the E-STOP's Damp).
            ok = bool(self.go2.set_rage_mode(want_rage))
            if not self._cmd.safety_ok(epoch):
                logger.warning("set_mode aborted: E-STOP / operator-lost mid-toggle — re-Damping")
                try:
                    self.go2.sport_command(ALLOWED_SPORT_CMDS["Damp"])
                except Exception:
                    logger.exception("re-Damp after aborted rage toggle failed")
                return False
            if ok:
                self._rage_active = want_rage
                self._publish_robot_state()
            logger.info("set_mode: rage=%s ok=%s", want_rage, ok)
            return ok

        self._cmd.submit(f"set_mode {mode}", nonce, task)

    def _handle_obstacle_avoidance(self, msg: dict[str, Any]) -> None:
        """Toggle the Go2's onboard obstacle avoidance on/off."""
        enabled = bool(msg.get("enabled"))
        nonce = msg.get("nonce")

        def task(_ep: int) -> bool:
            ok = bool(self.go2.set_obstacle_avoidance(enabled))
            if ok:
                self._obstacle_avoidance = enabled
                self._publish_robot_state()
            logger.info("obstacle_avoidance: enabled=%s ok=%s", enabled, ok)
            return ok

        self._cmd.submit(f"obstacle_avoidance {enabled}", nonce, task)

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
        if math.isnan(brightness):
            self._send_ack(nonce, False)
            return
        brightness = max(0.0, min(1.0, brightness))
        level = round(brightness * 10)

        def task(_ep: int) -> bool:
            ok = bool(self.go2.set_light(level))
            if ok:
                self._light = brightness
                self._publish_robot_state()
            logger.info("light: brightness=%.1f (level %d) ok=%s", brightness, level, ok)
            return ok

        self._cmd.submit(f"light {brightness:.1f}", nonce, task)

    # ─── click-to-navigate ────────────────────────────────────────────

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

    def _handle_nav_cancel(self, nonce: Any) -> None:
        """Operator cancel-plan button → stop the planner."""
        self._cancel_nav()
        logger.info("nav_cancel: plan cancelled by operator")
        self._send_ack(nonce, True)

    def _cancel_nav(self) -> None:
        """Tell the planner to stop (publish stop_movement)."""
        try:
            msg = Bool()
            msg.data = True
            self.stop_movement.publish(msg)
        except Exception:
            logger.warning("nav cancel publish failed", exc_info=True)

    # ─── manual drive guard (stream filter → tele_cmd_vel, NOT RPC) ────

    def _on_cmd_vel_in(self, twist: Twist) -> None:
        """Guard raw operator drive — E-STOP gate, stale/future/out-of-order
        drop — then republish on tele_cmd_vel to MovementManager, which
        arbitrates it against the planner and owns the driver's cmd_vel."""
        if self._estopped:
            return  # latched: no motion until estop_clear
        ts = float(twist.ts)
        if not math.isfinite(ts):
            # NaN ts passes every comparison below and would poison _last_cmd_ts.
            return
        age = time.time() - ts
        if age > self.config.cmd_stale_after_sec:
            return
        if age < 0:  # future-stamped: don't advance _last_cmd_ts (would stall drive)
            return
        if ts <= self._last_cmd_ts:  # out-of-order
            return
        self._last_cmd_ts = ts

        # Untrusted operator: reject non-finite and clamp to the Go2 envelope
        # (driver only reads linear.x/linear.y/angular.z).
        if not _all_finite(twist):
            logger.warning("dropping non-finite cmd_vel")
            return
        lin_max = self.config.max_linear_mps
        ang_max = self.config.max_angular_rps
        twist.linear.x = _clamp(twist.linear.x, -lin_max, lin_max)
        twist.linear.y = _clamp(twist.linear.y, -lin_max, lin_max)
        twist.angular.z = _clamp(twist.angular.z, -ang_max, ang_max)

        # Idle zeros would make MovementManager cancel the nav plan, so forward a
        # zero only as the release edge (prev frame moving), then stay silent.
        moving = not twist.is_zero()
        if not moving and not self._last_cmd_nonzero:
            return  # idle joystick — don't preempt nav
        self._last_cmd_nonzero = moving

        # Strip the header (MovementManager wants a plain Twist).
        self.tele_cmd_vel.publish(Twist(linear=twist.linear, angular=twist.angular))

    # ─── robot-authoritative state → stats module ─────────────────────

    def _robot_state(self) -> dict[str, Any]:
        return {
            "posture": self._posture,
            "rage": self._rage_active,
            "obstacle_avoidance": self._obstacle_avoidance,
            "light": self._light,
            "estopped": self._estopped,
        }

    def _publish_robot_state(self) -> None:
        """Push posture/rage/obstacle/light/estopped on robot_state."""
        try:
            self.robot_state.publish(json.dumps(self._robot_state()).encode())
        except Exception:
            logger.warning("robot_state publish failed", exc_info=True)

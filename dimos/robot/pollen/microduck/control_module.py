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

"""DuckControlModule: teleop/nav velocity mux + cockpit command router for
the Microduck.

Replaces MovementManager in the cockpit blueprint. Three things happen here:

* ``tele_cmd_vel`` (keyboard pad) and ``nav_cmd_vel`` (planner) are merged
  into one ``cmd_vel`` with MovementManager semantics (teleop wins, nav
  resumes after a cooldown) plus a mode switch: in ``agent`` mode the pad is
  ignored so the LLM owns the body. A locked policy (oneshot running,
  seated, fallen...) zeroes everything.
* ``ui_command`` JSON from the cockpit is routed: ``set_mode`` flips the
  mode, ``policy`` is re-published as ``policy_request`` for the sim,
  ``cancel_nav`` cancels the planner goal over RPC.
* A small state thread publishes ``nav_state`` / ``mode`` JSON at
  ``state_hz`` so the cockpit (and late viewers) always know what the duck
  is doing.

Navigation is cancelled through the planner's ``cancel_goal()`` RPC via the
``_navigation`` module ref; nobody publishes ``stop_movement`` here.

Wire formats (all JSON ``str`` streams carry ``"t": time.time()``):

* ``mode``:          ``{"mode": "teleop"|"agent", "t": t}``
* ``nav_state``:     ``{"state": <NAV_STATES>, "goal": {"x","y","yaw"}|null, "since": t, "t": t}``
  (``goal`` is non-null exactly while a goal is in progress)
* ``policy_request``: ``{"policy": "kick_left", "action": "start"|"stop"|"toggle", "t": t}``
  (``policy`` omitted for a bare ``{"action": "stop"}``)
* ``ui_command`` (in): ``{"name": "set_mode", "args": {"mode": ...}}`` |
  ``{"name": "policy", "args": {"policy": ..., "action": ...}}`` | ``{"name": "cancel_nav"}``
"""

from __future__ import annotations

from collections.abc import Callable
import json
import threading
import time
from typing import Any, Literal

from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Mode = Literal["teleop", "agent"]
MODES: tuple[str, ...] = ("teleop", "agent")
UI_COMMANDS: tuple[str, ...] = ("set_mode", "policy", "cancel_nav")
POLICY_ACTIONS: tuple[str, ...] = ("start", "stop", "toggle")

# nav_state "state" values. The first three mirror NavigationState; the
# rest are terminal outcomes DuckControl derives itself.
NAV_STATES: tuple[str, ...] = (
    "idle",
    "following_path",
    "recovery",
    "reached",
    "cancelled",
    "no_path",
)

# A goal whose planner state never leaves IDLE for this long is reported as
# no_path (the planner logs "no path" but exposes nothing over RPC).
NO_PATH_TIMEOUT_SEC = 15.0
# The planner drops out of FOLLOWING_PATH briefly on every replan; only an
# idle spell longer than this (after following started) counts as the goal
# having been stopped from elsewhere.
STOP_SETTLE_SEC = 2.0
_LOG_THROTTLE_SEC = 1.0


def _dumps(obj: dict[str, Any]) -> str:
    return json.dumps({**obj, "t": time.time()}, separators=(",", ":"))


def _is_zero(msg: Twist) -> bool:
    return (
        msg.linear.x == 0
        and msg.linear.y == 0
        and msg.linear.z == 0
        and msg.angular.x == 0
        and msg.angular.y == 0
        and msg.angular.z == 0
    )


class DuckControlConfig(ModuleConfig):
    tele_cooldown_sec: float = 1.0
    state_hz: float = 5.0
    default_mode: Mode = "teleop"
    # (linear.x, linear.y, angular.z) multipliers applied to teleop twists.
    tele_cmd_vel_scaling: tuple[float, float, float] = (1.0, 1.0, 1.0)


class DuckControlModule(Module):
    """Mux tele_cmd_vel/nav_cmd_vel into cmd_vel, route cockpit ui_command,
    publish mode + nav_state JSON."""

    config: DuckControlConfig

    _navigation: NavigationInterfaceSpec

    nav_cmd_vel: In[Twist]
    tele_cmd_vel: In[Twist]
    ui_command: In[str]
    policy_state: In[str]
    goal_request: In[PoseStamped]

    cmd_vel: Out[Twist]
    mode: Out[str]
    nav_state: Out[str]
    policy_request: Out[str]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        # Monotonic clock for durations; tests swap it for a fake.
        self._clock: Callable[[], float] = time.monotonic
        self._mode: str = self.config.default_mode
        # A teleop episode is in progress: a nonzero pad twist arrived and no
        # zero (release / e-stop) has ended it yet.
        self._teleop_active = False
        # _clock() of the last pad message (nonzero or e-stop zero); nav
        # twists are dropped within tele_cooldown_sec of it.
        self._last_teleop_time = -float("inf")
        self._locked = False
        self._zero_sent_while_locked = False
        # Navigation bookkeeping (guarded by _lock).
        self._goal: dict[str, float] | None = None
        self._goal_set_at = 0.0  # _clock() when the current goal arrived
        self._nav_active = False  # a goal is in progress (not yet terminal)
        self._nav_started = False  # planner seen non-idle for this goal
        # is_goal_reached() may still be latched from a previous goal when a
        # new one arrives; only a True seen after a False (or after
        # following) counts. _last_reached is the previous poll's answer.
        self._nav_armed = False
        self._last_reached = False
        self._idle_since: float | None = None  # _clock() when idle-after-following began
        self._state = "idle"
        self._since = time.time()
        self._stop_event = threading.Event()
        self._state_thread: threading.Thread | None = None
        self._last_log: dict[str, float] = {}

    # ------------------------------------------------------------------ lifecycle

    @rpc
    def start(self) -> None:
        super().start()
        self._subscribe(self.nav_cmd_vel, self._on_nav)
        self._subscribe(self.tele_cmd_vel, self._on_teleop)
        self._subscribe(self.ui_command, self._on_ui_command)
        self._subscribe(self.policy_state, self._on_policy_state)
        self._subscribe(self.goal_request, self._on_goal_request)
        self._publish_mode()
        self._stop_event.clear()
        self._state_thread = threading.Thread(
            target=self._state_loop, name="DuckControl-state", daemon=True
        )
        self._state_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        thread = self._state_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            if thread.is_alive():
                logger.error("DuckControl state thread did not stop in time.")
        self._state_thread = None
        with self._lock:
            self._teleop_active = False
        super().stop()

    def _subscribe(self, stream: In[Any], callback: Callable[[Any], None]) -> None:
        if stream.transport is None:
            logger.warning(
                "DuckControl input has no transport; not subscribing", stream=stream.name
            )
            return
        self.register_disposable(Disposable(stream.subscribe(callback)))

    # ------------------------------------------------------------------ helpers

    def _log_throttled(self, key: str, message: str, **kwargs: Any) -> None:
        now = self._clock()
        if now - self._last_log.get(key, -float("inf")) < _LOG_THROTTLE_SEC:
            return
        self._last_log[key] = now
        logger.warning(message, **kwargs)

    def _nav_rpc(self, name: str) -> Any:
        """Call a NavigationInterfaceSpec method; None when the planner is
        unavailable (not wired yet, RPC timeout...). Never raises."""
        navigation = getattr(self, "_navigation", None)
        if navigation is None:
            self._log_throttled("nav_missing", "DuckControl has no _navigation ref", call=name)
            return None
        try:
            return getattr(navigation, name)()
        except Exception as e:
            self._log_throttled(f"nav_{name}", "navigation RPC failed", call=name, error=str(e))
            return None

    def _publish_zero(self) -> None:
        self.cmd_vel.publish(Twist())

    def _publish_mode(self) -> None:
        self.mode.publish(_dumps({"mode": self._mode}))

    def _publish_nav_state(self, snapshot: dict[str, Any]) -> None:
        self.nav_state.publish(_dumps(snapshot))

    def _snapshot_locked(self) -> dict[str, Any]:
        return {
            "state": self._state,
            "goal": dict(self._goal) if self._nav_active and self._goal is not None else None,
            "since": self._since,
        }

    def _set_state_locked(self, state: str) -> None:
        if state != self._state:
            self._state = state
            self._since = time.time()

    def _set_terminal_locked(self, state: str) -> None:
        self._set_state_locked(state)
        self._nav_active = False
        self._nav_started = False
        self._nav_armed = False
        self._idle_since = None

    def _cancel_nav(self) -> None:
        """Cancel the planner goal over RPC, zero cmd_vel and report
        `cancelled` (when a goal was being tracked)."""
        self._nav_rpc("cancel_goal")
        self._publish_zero()
        with self._lock:
            if not self._nav_active:
                return
            self._set_terminal_locked("cancelled")
            snapshot = self._snapshot_locked()
        self._publish_nav_state(snapshot)

    # ------------------------------------------------------------------ inputs

    def _on_ui_command(self, raw: str) -> None:
        try:
            command = json.loads(raw)
        except (TypeError, ValueError):
            self._log_throttled("ui_bad_json", "ui_command is not JSON", raw=str(raw)[:120])
            return
        if not isinstance(command, dict) or command.get("name") not in UI_COMMANDS:
            self._log_throttled("ui_unknown", "unknown ui_command", raw=str(raw)[:120])
            return
        name = command["name"]
        args = command.get("args") or {}
        if not isinstance(args, dict):
            self._log_throttled("ui_bad_args", "ui_command args must be an object", name=name)
            return
        if name == "set_mode":
            self._set_mode(args.get("mode"))
        elif name == "policy":
            self._forward_policy(args.get("policy"), args.get("action"))
        else:
            self._cancel_nav()

    def _set_mode(self, mode: Any) -> None:
        if mode not in MODES:
            self._log_throttled("ui_bad_mode", "set_mode with unknown mode", mode=str(mode))
            return
        with self._lock:
            changed = mode != self._mode
            self._mode = mode
            zero = changed and self._teleop_active
            if changed:
                self._teleop_active = False
        if changed:
            logger.info("DuckControl mode", mode=mode)
        if zero:
            # The pad's release zero will be ignored in agent mode; do not
            # leave the last held twist running in the sim.
            self._publish_zero()
        self._publish_mode()

    def _forward_policy(self, policy: Any, action: Any) -> None:
        if action not in POLICY_ACTIONS:
            self._log_throttled(
                "ui_bad_action", "policy command with bad action", action=str(action)
            )
            return
        if policy is not None and (not isinstance(policy, str) or not policy):
            self._log_throttled(
                "ui_bad_policy", "policy command with bad policy", policy=str(policy)
            )
            return
        if policy is None and action != "stop":
            self._log_throttled("ui_no_policy", "policy command without a policy", action=action)
            return
        request: dict[str, Any] = {"action": action}
        if policy is not None:
            request["policy"] = policy
        self.policy_request.publish(_dumps(request))

    def _on_teleop(self, msg: Twist) -> None:
        with self._lock:
            locked = self._locked
            send_zero = locked and not self._zero_sent_while_locked
            if locked:
                self._zero_sent_while_locked = True
            mode = self._mode
        if locked:
            if send_zero:
                self._publish_zero()
            return
        if mode == "agent":
            self._log_throttled("tele_agent", "tele_cmd_vel ignored in agent mode")
            return

        zero = _is_zero(msg)
        with self._lock:
            goal_active = self._nav_active
            if zero:
                # e-stop / key release: meaningful only if something was moving.
                stop = self._teleop_active or goal_active
                self._teleop_active = False
            else:
                stop = False
                self._teleop_active = True
            if stop or not zero:
                self._last_teleop_time = self._clock()
        if zero:
            if stop:
                self._cancel_nav()  # publishes the zero
            return
        if goal_active:
            self._cancel_nav()
        sx, sy, sw = self.config.tele_cmd_vel_scaling
        self.cmd_vel.publish(
            Twist(
                linear=Vector3(msg.linear.x * sx, msg.linear.y * sy, msg.linear.z),
                angular=Vector3(msg.angular.x, msg.angular.y, msg.angular.z * sw),
            )
        )

    def _on_nav(self, msg: Twist) -> None:
        with self._lock:
            if self._locked:
                return
            if self._clock() - self._last_teleop_time < self.config.tele_cooldown_sec:
                return
            self._teleop_active = False
        self.cmd_vel.publish(msg)

    def _on_policy_state(self, raw: str) -> None:
        try:
            state = json.loads(raw)
        except (TypeError, ValueError):
            self._log_throttled("policy_bad_json", "policy_state is not JSON")
            return
        if not isinstance(state, dict):
            return
        locked = bool(state.get("locked", False))
        with self._lock:
            rising = locked and not self._locked
            self._locked = locked
            if rising:
                self._zero_sent_while_locked = True  # _cancel_nav below sends it
                self._teleop_active = False
            elif not locked:
                self._zero_sent_while_locked = False
        if rising:
            logger.info("policy locked; cancelling navigation")
            self._cancel_nav()

    def _on_goal_request(self, goal: PoseStamped) -> None:
        with self._lock:
            self._goal = {
                "x": float(goal.position.x),
                "y": float(goal.position.y),
                "yaw": float(goal.yaw),
            }
            self._goal_set_at = self._clock()
            self._nav_active = True
            self._nav_started = False
            self._nav_armed = not self._last_reached
            self._idle_since = None
            self._set_state_locked("idle")
            snapshot = self._snapshot_locked()
        self._publish_nav_state(snapshot)

    # ------------------------------------------------------------------ state thread

    def _state_loop(self) -> None:
        period = 1.0 / max(self.config.state_hz, 0.1)
        while not self._stop_event.is_set():
            try:
                self._tick()
            except Exception:
                logger.exception("DuckControl state tick failed")
            self._stop_event.wait(period)

    def _tick(self) -> None:
        """One state-thread iteration: poll the planner, publish nav_state + mode."""
        planner_state = self._nav_rpc("get_state")
        reached = self._nav_rpc("is_goal_reached")
        snapshot = self._update_nav_state(planner_state, reached)
        self._publish_nav_state(snapshot)
        self._publish_mode()

    def _update_nav_state(self, planner_state: Any, reached: Any) -> dict[str, Any]:
        """Fold one planner poll into the nav bookkeeping; returns the
        nav_state snapshot (without "t"). A poll that failed (None) leaves
        the bookkeeping untouched so a flaky RPC cannot fake an outcome."""
        now = self._clock()
        with self._lock:
            if not isinstance(planner_state, NavigationState):
                return self._snapshot_locked()
            if isinstance(reached, bool):
                self._last_reached = reached
                if not reached:
                    self._nav_armed = True

            if planner_state in (NavigationState.FOLLOWING_PATH, NavigationState.RECOVERY):
                # A goal set behind our back (planner RPC set_goal) still
                # shows up as navigation in progress, goal unknown.
                if not self._nav_active:
                    self._goal = None
                    self._goal_set_at = now
                    self._nav_active = True
                self._nav_started = True
                self._nav_armed = True
                self._idle_since = None
                self._set_state_locked(planner_state.value)
                return self._snapshot_locked()

            if not self._nav_active:
                return self._snapshot_locked()  # idle, or a sticky terminal state

            # Goal tracked, planner idle: waiting for a plan, done, or stopped.
            if reached is True and self._nav_armed:
                self._set_terminal_locked("reached")
            elif self._nav_started:
                if self._idle_since is None:
                    self._idle_since = now
                if now - self._idle_since >= STOP_SETTLE_SEC:
                    self._set_terminal_locked("cancelled")
                else:
                    self._set_state_locked(planner_state.value)
            elif now - self._goal_set_at >= NO_PATH_TIMEOUT_SEC:
                self._set_terminal_locked("no_path")
            else:
                self._set_state_locked(planner_state.value)
            return self._snapshot_locked()

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

"""Guarded high-level velocity connection for the Deep Robotics Lynx M20."""

from __future__ import annotations

from collections.abc import Callable
from datetime import datetime
import json
import math
import socket
import struct
from threading import Condition, Event, RLock, Thread
import time
from typing import Any, Literal, TypeVar

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.robot.deeprobotics.m20.constants import WORLD_FRAME
from dimos.utils.logging_config import setup_logger
from dimos.utils.sequential_ids import SequentialIds

logger = setup_logger()

T = TypeVar("T")

MOTION_STAND = 1
MOTION_SIT = 4
MOTION_RL_CONTROL = 17

GAIT_BASIC = 0x1001
GAIT_FLAT_AGILE = 0x3002
GAIT_STAIR_AGILE = 0x3003
NavigationTerrain = Literal["flat", "stairs"]
_NAVIGATION_GAITS: dict[NavigationTerrain, int] = {
    "flat": GAIT_FLAT_AGILE,
    "stairs": GAIT_STAIR_AGILE,
}

USAGE_MODE_NAVIGATION = 1

_BASIC_SERVER_MAGIC = bytes.fromhex("eb91eb90")
_BASIC_SERVER_JSON = 1
_BASIC_SERVER_HEADER = struct.Struct("<4sHHB7s")
_BASIC_SERVER_MODE_TYPE = 1101
_BASIC_SERVER_MODE_COMMAND = 5
_BASIC_SERVER_HEARTBEAT_TYPE = 100
_BASIC_SERVER_HEARTBEAT_COMMAND = 100
_BASIC_SERVER_HEARTBEAT_PERIOD_S = 0.5
_BASIC_SERVER_REPORT_TYPE = 1002
_BASIC_SERVER_ERROR_LIST_COMMAND = 3
_BASIC_SERVER_BASIC_STATUS_COMMAND = 6
_BASIC_SERVER_CHARGE_TYPE = 2
_BASIC_SERVER_CHARGE_COMMAND = 24
_BASIC_SERVER_POLL_S = 0.25
# basic_server's own wait for a gait change (its "waitForGaitChange timeout" log line).
_VENDOR_GAIT_WAIT_S = 3.0

CHARGE_COMMAND_STOP = 0
CHARGE_COMMAND_START = 1

CHARGE_IDLE = 0
CHARGE_ENTERING = 1
CHARGE_CHARGING = 2
CHARGE_EXITING = 3
CHARGE_ERROR = 4
CHARGE_ON_DOCK = 5
_CHARGE_LABELS = {
    CHARGE_IDLE: "idle",
    CHARGE_ENTERING: "entering the dock",
    CHARGE_CHARGING: "charging",
    CHARGE_EXITING: "exiting the dock",
    CHARGE_ERROR: "error",
    CHARGE_ON_DOCK: "on the dock, not charging",
}
_CHARGE_BUSY_STATES = frozenset({CHARGE_ENTERING, CHARGE_CHARGING, CHARGE_EXITING, CHARGE_ON_DOCK})
_CHARGE_ERROR_NAMES = {
    0x8501: "location timeout",
    0x8502: "reflector localization failed to start",
    0x8503: "no location, reflectors not found",
    0x8506: "no current from the dock",
    0x8509: "dock entry timeout",
    0x8510: "dock exit timeout",
}
_CHARGE_STATUS_MAX_AGE_S = 3.0
_CHARGE_STATUS_WAIT_S = 5.0
# Reports arrive at 2 Hz, so a status read this soon after a command can predate it.
_CHARGE_COMMAND_GRACE_S = 1.0

DOCK_CHARGING = "charging"

ChargeTask = Literal["dock", "undock"]


class _StoppingError(Exception):
    """The module started stopping during a standup or gait switch."""


class M20ConnectionConfig(ModuleConfig):
    """State-transition and basic-server settings for M20 control."""

    stand_timeout_s: float = Field(default=12.0, gt=0.0)
    rl_control_timeout_s: float = Field(default=5.0, gt=0.0)
    gait_timeout_s: float = Field(default=5.0, gt=0.0)
    # After a stand basic_server took two of its gait waits to settle.
    navigation_mode_attempts: int = Field(default=3, ge=1)
    control_ready_timeout_s: float = Field(default=15.0, gt=0.0)
    basic_server_host: str = "10.21.31.103"
    basic_server_tcp_port: int = Field(default=30001, ge=1, le=65535)
    basic_server_udp_port: int = Field(default=30000, ge=1, le=65535)
    basic_server_timeout_s: float = Field(default=3.0, gt=0.0)
    sit_timeout_s: float = Field(default=3.0, gt=0.0)
    auto_stand: bool = False
    dock_settle_s: float = Field(default=1.0, ge=0.0)
    dock_start_timeout_s: float = Field(default=5.0, gt=0.0)
    dock_timeout_s: float = Field(default=150.0, gt=0.0)
    undock_timeout_s: float = Field(default=45.0, gt=0.0)


def dock_verdict(
    status: int,
    entered: bool,
    elapsed_s: float,
    start_timeout_s: float,
    dock_timeout_s: float,
) -> str | None:
    """Return None while docking continues, DOCK_CHARGING on success, else why it failed."""
    if status == CHARGE_CHARGING:
        return DOCK_CHARGING
    if status == CHARGE_ERROR:
        return "the robot reported a charging error"
    if status == CHARGE_EXITING:
        return "the robot gave up and is backing off the dock"
    if status == CHARGE_IDLE:
        if entered:
            return "the robot stopped docking and returned to idle"
        if elapsed_s >= start_timeout_s:
            return "the robot did not accept the dock command"
        return None
    if elapsed_s >= dock_timeout_s:
        if status == CHARGE_ON_DOCK:
            return "the robot is on the dock but not charging"
        return f"timed out after {dock_timeout_s:g} s"
    return None


class M20Connection(Module):
    """Expose the planner-facing M20 command and terrain surface.

    The hardware bridge owns ROS 2/DrDDS and the command watchdog. This module
    publishes manual movement RPCs on the standard DimOS ``cmd_vel`` stream. The
    robot-local bridge consumes that stream and is the single command-validation
    and velocity-clamping boundary.

    ``standup()`` is the normal one-call operator entry point: it completes the
    vendor state and gait transitions and waits for the command path.
    ``set_navigation_terrain()`` is the only exposed vendor-specific control and
    selects one of the documented agile navigation gaits without exposing raw
    gait values.
    """

    config: M20ConnectionConfig

    cmd_vel: Out[Twist]
    command_ready: In[Bool]
    motion_state: In[Int32]
    gait_state: In[UInt32]
    odometry: In[Odometry]
    motion_state_cmd: Out[Int32]
    gait_cmd: Out[UInt32]
    odom: Out[PoseStamped]
    goal: Out[PointStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._state_condition = Condition(self._lock)
        self._command_ready = False
        self._motion_state: int | None = None
        self._gait_state: int | None = None
        self._navigation_terrain: NavigationTerrain = "stairs"
        self._basic_server_message_ids = SequentialIds()
        self._link_stop = Event()
        self._stopping = Event()
        self._auto_stand_thread: Thread | None = None
        self._udp: socket.socket | None = None
        self._link_threads: list[Thread] = []
        self._charge_status: int | None = None
        self._charge_status_time = 0.0
        self._charge_error: int | None = None
        self._usage_mode: int | None = None
        self._charge_task: ChargeTask | None = None
        self._charge_thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.command_ready.subscribe(self._on_command_ready)))
        self.register_disposable(Disposable(self.motion_state.subscribe(self._on_motion_state)))
        self.register_disposable(Disposable(self.gait_state.subscribe(self._on_gait_state)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self._stopping.clear()
        self._start_basic_server_link()
        if self.config.auto_stand:
            self._auto_stand_thread = Thread(
                target=self._auto_stand, name="m20-auto-stand", daemon=True
            )
            self._auto_stand_thread.start()

    def _auto_stand(self) -> None:
        if not self._wait_for_charge_status(_CHARGE_STATUS_WAIT_S):
            if self._stopping.is_set():
                return
            logger.warning(
                "M20 reported no charge status, so it stands up without knowing it is off the dock"
            )
        if self._charge_busy():
            logger.info("M20 is on the dock, so it will leave before standing up")
            self.undock()
        elif not self.standup() and not self._stopping.is_set():
            logger.error("M20 did not stand up on start")

    @rpc
    def stop(self) -> None:
        with self._state_condition:
            self._stopping.set()
            self._state_condition.notify_all()
        if self._charge_busy():
            logger.warning("M20 is on the dock, so it will stay as it is")
        elif self.config.auto_stand:
            self.liedown()
            with self._state_condition:
                sat = self._state_condition.wait_for(
                    lambda: self._motion_state == MOTION_SIT, timeout=self.config.sit_timeout_s
                )
            if not sat:
                logger.warning("M20 did not confirm Sit before stopping")
        else:
            self.stop_movement()
        self._stop_basic_server_link()
        if self._auto_stand_thread is not None:
            self._auto_stand_thread.join(timeout=self.config.basic_server_timeout_s)
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Publish velocity to the native validation boundary.

        ``duration`` is accepted for connection compatibility. Command lifetime
        is enforced by the native bridge's monotonic watchdog.
        """
        del duration
        if self._refuse_while_charging("move"):
            return False
        with self._lock:
            ready = self._command_ready
        self.cmd_vel.publish(twist)
        return ready

    @rpc
    def stop_movement(self) -> None:
        """Publish an immediate zero velocity without disabling future commands.

        Velocity would fight the vendor's own docking, so a dock in progress is
        cancelled instead, and the robot backs off the dock and stands.
        """
        with self._lock:
            docking = self._charge_task == "dock" or self._fresh_charge_status() == CHARGE_ENTERING
        if docking:
            logger.info("M20 stop_movement cancels docking")
            self.undock()
        elif self._charge_busy():
            logger.info("M20 stop_movement left the robot to the vendor's dock control")
        else:
            self.cmd_vel.publish(Twist.zero())

    @rpc
    def standup(self) -> bool:
        """Bring the M20 to a command-enabled, navigation-ready standing state."""
        if self._refuse_while_charging("standup"):
            return False
        try:
            return self._stand()
        except _StoppingError:
            logger.info("M20 standup cancelled because the module is stopping")
            return False

    def _stand(self) -> bool:
        with self._lock:
            was_walking = self._motion_state == MOTION_RL_CONTROL
        if not self._ensure_rl_control():
            return False
        attempts = self.config.navigation_mode_attempts
        with self._lock:
            terrain = self._navigation_terrain
        gait = _NAVIGATION_GAITS[terrain]

        def navigating() -> bool:
            return self._usage_mode in (None, USAGE_MODE_NAVIGATION)

        if not was_walking:
            # Standing up makes basic_server reset the usage mode and gait itself, and a mode sent before that lands is lost.
            self._wait_for_gait_state(GAIT_BASIC, _VENDOR_GAIT_WAIT_S * attempts)

        # Right after a stand basic_server is still finishing its own gait change; a usage mode sent then is
        # answered OK but never applied, so the mode and gait are watched for one such wait and sent again if they fell back.
        for attempt in range(1, attempts + 1):
            if not self._enter_navigation_mode():
                logger.error("M20 basic_server rejected the navigation usage mode")
                return False
            # A report from before the switch can still show the old mode.
            self._wait_unless_stopping(navigating, _VENDOR_GAIT_WAIT_S)
            # Switching usage mode resets the gait to Basic. Confirm that reset,
            # then select the documented autonomous-navigation gait.
            if not self._set_gait_and_wait(GAIT_BASIC):
                logger.error("M20 did not confirm the Basic gait")
                return False
            if not self._set_gait_and_wait(gait):
                logger.error("M20 did not confirm the agile %s navigation gait", terrain)
                return False
            if self._holds(lambda: self._gait_state == gait and navigating(), _VENDOR_GAIT_WAIT_S):
                break
            if attempt == attempts:
                logger.error(
                    "M20 dropped navigation mode %d times right after accepting it", attempts
                )
                return False
            logger.warning("M20 dropped navigation mode, sending it again")
        if not self._wait_for_control_readiness(self.config.control_ready_timeout_s):
            logger.error("M20 robot control path did not become ready")
            return False
        return True

    @rpc
    def liedown(self) -> bool:
        """Disable velocity output and command the M20 to its Sit/prone state."""
        if self._refuse_while_charging("liedown"):
            return False
        self.stop_movement()
        self.motion_state_cmd.publish(Int32(MOTION_SIT))
        return True

    @rpc
    def set_navigation_terrain(self, terrain: NavigationTerrain) -> bool:
        """Select the agile navigation gait for flat ground or stairs.

        Call this while the robot is stationary. In RL Control the switch is
        applied immediately and confirmed through ``/MOTION_INFO``. Otherwise
        the selection is retained and applied by the next ``standup()``.
        """
        if self._refuse_while_charging("set_navigation_terrain"):
            return False
        gait = _NAVIGATION_GAITS.get(terrain)
        if gait is None:
            logger.error("Unsupported M20 navigation terrain: %s", terrain)
            return False

        with self._lock:
            if self._motion_state != MOTION_RL_CONTROL:
                self._navigation_terrain = terrain
                logger.info("M20 will select the agile %s gait on next standup", terrain)
                return True
            if self._gait_state == gait:
                self._navigation_terrain = terrain
                return True

        self.stop_movement()
        try:
            switched = self._set_gait_and_wait(gait)
        except _StoppingError:
            return False
        if switched:
            with self._lock:
                self._navigation_terrain = terrain
            logger.info("M20 selected the agile %s navigation gait", terrain)
        else:
            logger.error("M20 did not confirm the agile %s navigation gait", terrain)

        return switched

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def dock(self) -> str:
        """Start docking and charging, and return immediately.

        The robot must already stand still, facing the dock with both reflectors in view,
        about 2 m in front of it. It can't walk to the dock from elsewhere. A tool update
        reports each stage and the result.
        """
        self.start_tool("dock")
        with self._state_condition:
            if self._charge_task == "dock":
                return "Already docking. A tool update reports the result."
            refusal = self._dock_refusal()
            if refusal is None:
                self._charge_task = "dock"
                self._charge_error = None
        if refusal is not None:
            self.stop_tool("dock")
            return f"Not docking: {refusal}."

        self.goal.publish(PointStamped(math.nan, math.nan, math.nan, frame_id=WORLD_FRAME))
        self._charge_thread = Thread(target=self._run_dock, name="m20-dock", daemon=True)
        self._charge_thread.start()
        return "Docking. A tool update reports each stage and the result."

    @skill(lifecycle="background")
    def undock(self) -> str:
        """Stop docking or charging, leave the dock and stand ready to navigate.

        Returns immediately. A tool update reports the result.
        """
        self.start_tool("undock")
        with self._state_condition:
            if self._charge_task == "undock":
                return "Already undocking. A tool update reports the result."
            status = self._fresh_charge_status()
            if self._charge_task is None and status == CHARGE_IDLE:
                self.stop_tool("undock")
                return "Not undocking: the robot is not on the dock."
            previous = self._charge_thread
            self._charge_task = "undock"
            self._charge_error = None
            self._state_condition.notify_all()

        if status != CHARGE_EXITING and not self._send_charge_command(CHARGE_COMMAND_STOP):
            with self._state_condition:
                self._charge_task = None
            self.stop_tool("undock")
            return "Not undocking: could not reach the robot."
        self._charge_thread = Thread(
            target=self._run_undock, args=(previous,), name="m20-undock", daemon=True
        )
        self._charge_thread.start()
        return "Undocking. A tool update reports the result."

    @skill
    def charge_status(self) -> str:
        """Report the robot's charging state and the charging error it reports now, if any."""
        with self._lock:
            status = self._fresh_charge_status()
            error = self._charge_error
        label = "unknown" if status is None else _CHARGE_LABELS.get(status, "unknown")
        return f"{label}, error {_charge_error_text(error)}" if error is not None else label

    def _dock_refusal(self) -> str | None:
        status = self._fresh_charge_status()
        if status is None:
            return "the robot is not reporting its charge status"
        if self._charge_task == "undock":
            return "an undock is still running"
        if status != CHARGE_IDLE:
            return f"its charge status is {_CHARGE_LABELS.get(status, status)}"
        if self._motion_state != MOTION_RL_CONTROL:
            return "the robot is not standing ready to walk"
        return None

    def _run_dock(self) -> None:
        try:
            outcome = self._watch_dock()
        except _StoppingError:
            outcome = "Stopped docking, module stopping."
        except Exception:
            logger.exception("M20 dock failed")
            outcome = "Docking failed with an internal error."
        logger.info("M20 dock: %s", outcome)
        self.tool_update("dock", outcome)
        self.stop_tool("dock")
        with self._state_condition:
            if self._charge_task == "dock":
                self._charge_task = None
            self._state_condition.notify_all()

    def _watch_dock(self) -> str:
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._charge_task != "dock" or self._link_stop.is_set(),
                timeout=self.config.dock_settle_s,
            )
            if self._charge_task != "dock":
                return "Docking cancelled before it started."
            if self._link_stop.is_set():
                return "Docking cancelled, module stopping."
        if not self._send_charge_command(CHARGE_COMMAND_START):
            return "Docking failed: could not reach the robot."

        started = time.monotonic()
        entered = False
        reported: int | None = None
        # The robot can clear its error as it gives up, so keep the last one seen for the verdict.
        error: int | None = None
        while True:
            with self._state_condition:
                self._state_condition.wait(timeout=_BASIC_SERVER_POLL_S)
                if self._charge_task != "dock":
                    return "Docking cancelled by undock."
                if self._link_stop.is_set():
                    return "Stopped watching docking, module stopping."
                status = self._fresh_charge_status()
                error = self._charge_error or error

            if status is None:
                self._send_charge_command(CHARGE_COMMAND_STOP)
                return "Docking failed: the robot stopped reporting its charge status."
            entered = entered or status != CHARGE_IDLE
            if status != reported and status != CHARGE_IDLE:
                reported = status
                label = _CHARGE_LABELS.get(status, str(status))
                logger.info("M20 charge status: %s", label)
                self.tool_update("dock", f"Charge status: {label}.")

            verdict = dock_verdict(
                status,
                entered,
                time.monotonic() - started,
                self.config.dock_start_timeout_s,
                self.config.dock_timeout_s,
            )
            if verdict == DOCK_CHARGING:
                return "Docked and charging."
            if verdict is None:
                continue
            if status in (CHARGE_ENTERING, CHARGE_ON_DOCK):
                self._send_charge_command(CHARGE_COMMAND_STOP)
            detail = "" if error is None else f" ({_charge_error_text(error)})"
            return f"Docking failed: {verdict}{detail}. {self._resume_navigation()}"

    def _resume_navigation(self) -> str:
        """Restore navigation mode once the robot is idle, because idle switches it to Basic."""
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._charge_task != "dock"
                or self._link_stop.is_set()
                or self._fresh_charge_status() == CHARGE_IDLE,
                timeout=self.config.undock_timeout_s,
            )
            if self._charge_task != "dock":
                return "Undocking took over."
            if self._link_stop.is_set():
                return "The module is stopping."
            if self._fresh_charge_status() != CHARGE_IDLE:
                return "The robot did not return to idle, so navigation mode was not restored."
            if self._motion_state != MOTION_RL_CONTROL:
                return "The robot is not standing. Call standup() before navigating."
        if not self._stand():
            return "The robot did not return to navigation mode. Call standup() before navigating."
        return "The robot is standing ready to navigate."

    def _run_undock(self, previous: Thread | None) -> None:
        try:
            if previous is not None:
                previous.join()
            outcome = self._watch_undock()
            if outcome is None:
                stood = self._stand()
                outcome = "Off the dock and ready to navigate."
                if not stood:
                    outcome = "Off the dock, but the robot did not stand ready to navigate."
        except _StoppingError:
            outcome = "Stopped undocking, module stopping."
        except Exception:
            logger.exception("M20 undock failed")
            outcome = "Undocking failed with an internal error."
        logger.info("M20 undock: %s", outcome)
        self.tool_update("undock", outcome)
        self.stop_tool("undock")
        with self._state_condition:
            self._charge_task = None
            self._state_condition.notify_all()

    def _watch_undock(self) -> str | None:
        """Wait for the robot to leave the dock and return why it did not, or None."""
        started = time.monotonic()
        with self._state_condition:
            while True:
                self._state_condition.wait(timeout=_BASIC_SERVER_POLL_S)
                if self._link_stop.is_set():
                    return "Stopped watching undocking, module stopping."
                elapsed = time.monotonic() - started
                if elapsed >= _CHARGE_COMMAND_GRACE_S:
                    error = self._charge_error
                    if error is not None:
                        return f"Undocking failed: {_charge_error_text(error)}."
                    if self._fresh_charge_status() == CHARGE_IDLE:
                        return None
                if elapsed >= self.config.undock_timeout_s:
                    return f"Undocking failed: not off the dock after {elapsed:.0f} s."

    def _refuse_while_charging(self, action: str) -> bool:
        if not self._charge_busy():
            return False
        logger.warning("M20 refused %s while docking or on the dock", action)
        return True

    def _charge_busy(self) -> bool:
        with self._lock:
            return (
                self._charge_task is not None or self._fresh_charge_status() in _CHARGE_BUSY_STATES
            )

    def _fresh_charge_status(self) -> int | None:
        with self._lock:
            age = time.monotonic() - self._charge_status_time
            return self._charge_status if age <= _CHARGE_STATUS_MAX_AGE_S else None

    def _wait_for_charge_status(self, timeout_s: float) -> bool:
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._stopping.is_set() or self._fresh_charge_status() is not None,
                timeout=timeout_s,
            )
            return self._fresh_charge_status() is not None

    def _send_charge_command(self, command: int) -> bool:
        _message_id, apdu = self._encode_basic_server_apdu(
            _BASIC_SERVER_CHARGE_TYPE, _BASIC_SERVER_CHARGE_COMMAND, {"Charge": command}
        )
        return self._send_udp(apdu)

    def _on_command_ready(self, msg: Bool) -> None:
        with self._state_condition:
            self._command_ready = bool(msg.data)
            self._state_condition.notify_all()

    def _on_motion_state(self, msg: Int32) -> None:
        with self._state_condition:
            self._motion_state = int(msg.data)
            self._state_condition.notify_all()

    def _on_gait_state(self, msg: UInt32) -> None:
        with self._state_condition:
            self._gait_state = int(msg.data)
            self._state_condition.notify_all()

    def _on_odometry(self, odometry: Odometry) -> None:
        self.odom.publish(odometry.to_pose_stamped())

    def _ensure_rl_control(self) -> bool:
        with self._lock:
            motion_state = self._motion_state
        if motion_state == MOTION_RL_CONTROL:
            return True
        if motion_state != MOTION_STAND:
            self._command(self.motion_state_cmd, Int32(MOTION_STAND))
            if not self._wait_for_motion_state(MOTION_STAND, self.config.stand_timeout_s):
                logger.error("M20 did not confirm Stand before the transition timeout")
                return False
        self._command(self.motion_state_cmd, Int32(MOTION_RL_CONTROL))
        if not self._wait_for_motion_state(MOTION_RL_CONTROL, self.config.rl_control_timeout_s):
            logger.error("M20 did not confirm RL Control after standing")
            return False
        return True

    def _set_gait_and_wait(self, gait: int) -> bool:
        # The robot drops a gait command that arrives with its own reset to Basic.
        for _attempt in range(2):
            self._command(self.gait_cmd, UInt32(gait))
            if self._wait_for_gait_state(gait, self.config.gait_timeout_s):
                return True
        return False

    def _command(self, stream: Out[T], msg: T) -> None:
        with self._lock:
            if self._stopping.is_set():
                raise _StoppingError
            stream.publish(msg)

    def _enter_navigation_mode(self) -> bool:
        response = self._basic_server_request(
            message_type=_BASIC_SERVER_MODE_TYPE,
            command=_BASIC_SERVER_MODE_COMMAND,
            items={"Mode": USAGE_MODE_NAVIGATION},
        )
        try:
            error_code = int(response["PatrolDevice"]["Items"]["ErrorCode"])
        except (KeyError, TypeError, ValueError):
            logger.error("M20 basic_server returned an invalid usage-mode response")
            return False
        if error_code != 0:
            logger.error("M20 basic_server usage-mode switch failed: error %s", error_code)
            return False
        return True

    def _basic_server_request(
        self,
        *,
        message_type: int,
        command: int,
        items: dict[str, Any],
    ) -> dict[str, Any]:
        message_id, apdu = self._encode_basic_server_apdu(message_type, command, items)
        try:
            with socket.create_connection(
                (self.config.basic_server_host, self.config.basic_server_tcp_port),
                timeout=self.config.basic_server_timeout_s,
            ) as connection:
                connection.settimeout(self.config.basic_server_timeout_s)
                connection.sendall(apdu)
                response_header = _recv_exact(connection, _BASIC_SERVER_HEADER.size)
                magic, length, response_id, encoding, _reserved = _BASIC_SERVER_HEADER.unpack(
                    response_header
                )
                if magic != _BASIC_SERVER_MAGIC:
                    raise ValueError("invalid APDU magic")
                if response_id != message_id:
                    raise ValueError("response APDU message ID does not match request")
                if encoding != _BASIC_SERVER_JSON:
                    raise ValueError("basic_server response is not JSON")
                response_payload = _recv_exact(connection, length)
        except (OSError, ValueError) as exc:
            logger.error("M20 basic_server request failed: %s", exc)
            return {}

        try:
            decoded = json.loads(response_payload)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            logger.error("M20 basic_server returned invalid JSON: %s", exc)
            return {}
        return decoded if isinstance(decoded, dict) else {}

    def _start_basic_server_link(self) -> None:
        self._link_stop.clear()
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.bind(("", 0))
        udp.settimeout(_BASIC_SERVER_POLL_S)
        self._udp = udp
        self._link_threads = [
            Thread(target=self._heartbeat, name="m20-heartbeat", daemon=True),
            Thread(target=self._read_reports, name="m20-reports", daemon=True),
        ]
        for thread in self._link_threads:
            thread.start()

    def _stop_basic_server_link(self) -> None:
        self._link_stop.set()
        with self._state_condition:
            self._state_condition.notify_all()
        threads = list(self._link_threads)
        if self._charge_thread is not None:
            threads.append(self._charge_thread)
        for thread in threads:
            if thread.is_alive():
                thread.join(timeout=2.0)
        if self._udp is not None:
            self._udp.close()
            self._udp = None

    def _send_udp(self, apdu: bytes) -> bool:
        try:
            return self._sendto(apdu)
        except OSError as exc:
            logger.warning("M20 basic_server send failed: %s", exc)
            return False

    def _sendto(self, apdu: bytes) -> bool:
        udp = self._udp
        if udp is None:
            return False
        udp.sendto(apdu, (self.config.basic_server_host, self.config.basic_server_udp_port))
        return True

    def _heartbeat(self) -> None:
        failing = False
        while not self._link_stop.is_set():
            _message_id, apdu = self._encode_basic_server_apdu(
                _BASIC_SERVER_HEARTBEAT_TYPE, _BASIC_SERVER_HEARTBEAT_COMMAND, {}
            )
            try:
                self._sendto(apdu)
            except OSError as exc:
                if not failing:
                    logger.warning("M20 basic_server heartbeat failed: %s", exc)
                failing = True
            else:
                if failing:
                    logger.info("M20 basic_server heartbeat recovered")
                failing = False
            self._link_stop.wait(_BASIC_SERVER_HEARTBEAT_PERIOD_S)

    def _read_reports(self) -> None:
        udp = self._udp
        if udp is None:
            return
        while not self._link_stop.is_set():
            try:
                frame = udp.recv(0xFFFF)
            except TimeoutError:
                continue
            except OSError as exc:
                logger.warning("M20 basic_server read failed: %s", exc)
                self._link_stop.wait(_BASIC_SERVER_POLL_S)
                continue
            self._on_report(frame)

    def _on_report(self, frame: bytes) -> None:
        device = _decode_report(frame)
        if device is None or device.get("Type") != _BASIC_SERVER_REPORT_TYPE:
            return
        items = device.get("Items")
        if not isinstance(items, dict):
            return
        command = device.get("Command")
        if command == _BASIC_SERVER_BASIC_STATUS_COMMAND:
            basic_status = items.get("BasicStatus")
            if not isinstance(basic_status, dict):
                return
            charge = basic_status.get("Charge")
            usage_mode = basic_status.get("ControlUsageMode")
            with self._state_condition:
                if isinstance(charge, int):
                    self._charge_status = charge
                    self._charge_status_time = time.monotonic()
                if isinstance(usage_mode, int):
                    self._usage_mode = usage_mode
                self._state_condition.notify_all()
        elif command == _BASIC_SERVER_ERROR_LIST_COMMAND:
            error_list = items.get("ErrorList")
            if not isinstance(error_list, list):
                return
            codes = [
                error.get("errorCode") if isinstance(error, dict) else None for error in error_list
            ]
            charge_errors = [
                code for code in codes if isinstance(code, int) and code in _CHARGE_ERROR_NAMES
            ]
            with self._state_condition:
                # The list is every active error, so a charge error the robot no longer reports has cleared.
                self._charge_error = charge_errors[-1] if charge_errors else None
                self._state_condition.notify_all()

    def _encode_basic_server_apdu(
        self, message_type: int, command: int, items: dict[str, Any]
    ) -> tuple[int, bytes]:
        payload = json.dumps(
            {
                "PatrolDevice": {
                    "Type": message_type,
                    "Command": command,
                    "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Items": items,
                }
            },
            separators=(",", ":"),
        ).encode()
        if len(payload) > 0xFFFF:
            raise ValueError("M20 basic_server payload exceeds the APDU limit")

        message_id = self._basic_server_message_ids.next() & 0xFFFF
        header = _BASIC_SERVER_HEADER.pack(
            _BASIC_SERVER_MAGIC,
            len(payload),
            message_id,
            _BASIC_SERVER_JSON,
            b"\0" * 7,
        )
        return message_id, header + payload

    def _wait_for_motion_state(self, expected: int, timeout_s: float) -> bool:
        return self._wait_unless_stopping(lambda: self._motion_state == expected, timeout_s)

    def _wait_for_gait_state(self, expected: int, timeout_s: float) -> bool:
        return self._wait_unless_stopping(lambda: self._gait_state == expected, timeout_s)

    def _wait_for_control_readiness(self, timeout_s: float) -> bool:
        return self._wait_unless_stopping(lambda: self._command_ready, timeout_s)

    def _holds(self, ready: Callable[[], bool], seconds: float) -> bool:
        """Watch for `seconds` and return False as soon as `ready` stops being true."""
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._stopping.is_set() or not ready(), timeout=seconds
            )
            if self._stopping.is_set():
                raise _StoppingError
            return ready()

    def _wait_unless_stopping(self, ready: Callable[[], bool], timeout_s: float) -> bool:
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._stopping.is_set() or ready(), timeout=timeout_s
            )
            if self._stopping.is_set():
                raise _StoppingError
            return ready()


def _charge_error_text(code: int) -> str:
    return f"{code:#x} {_CHARGE_ERROR_NAMES.get(code, 'unknown')}"


def _decode_report(frame: bytes) -> dict[str, Any] | None:
    if len(frame) < _BASIC_SERVER_HEADER.size or not frame.startswith(_BASIC_SERVER_MAGIC):
        return None
    try:
        decoded = json.loads(frame[_BASIC_SERVER_HEADER.size :])
    except ValueError:
        return None
    device = decoded.get("PatrolDevice") if isinstance(decoded, dict) else None
    return device if isinstance(device, dict) else None


def _recv_exact(connection: socket.socket, size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        chunk = connection.recv(size - len(result))
        if not chunk:
            raise OSError("basic_server closed the connection before completing its response")
        result.extend(chunk)
    return bytes(result)

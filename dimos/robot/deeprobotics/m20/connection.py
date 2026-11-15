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
from dataclasses import dataclass, field
import math
from threading import Condition, Event, RLock, Thread
import time
from typing import Any, Literal, TypeVar

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.robot.deeprobotics.m20.basic_server import (
    CHARGE_CHARGING,
    CHARGE_COMMAND_START,
    CHARGE_COMMAND_STOP,
    CHARGE_ENTERING,
    CHARGE_ERROR,
    CHARGE_ERROR_NAMES,
    CHARGE_EXITING,
    CHARGE_IDLE,
    CHARGE_ON_DOCK,
    USAGE_MODE_NAVIGATION,
    BasicServerLink,
    BatteryReport,
)
from dimos.robot.deeprobotics.m20.constants import WORLD_FRAME
from dimos.utils.logging_config import setup_logger

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
_GAIT_COMMAND_ATTEMPTS = 2
# basic_server's own gait-change wait, logged as waitForGaitChange timeout.
_VENDOR_GAIT_WAIT_S = 3.0

_CHARGE_LABELS = {
    CHARGE_IDLE: "idle",
    CHARGE_ENTERING: "entering the dock",
    CHARGE_CHARGING: "charging",
    CHARGE_EXITING: "exiting the dock",
    CHARGE_ERROR: "error",
    CHARGE_ON_DOCK: "on the dock, not charging",
}
_CHARGE_BUSY_STATES = frozenset({CHARGE_ENTERING, CHARGE_CHARGING, CHARGE_EXITING, CHARGE_ON_DOCK})
_REPORT_MAX_AGE_S = 3.0
_CHARGE_STATUS_WAIT_S = 5.0
_CHARGE_POLL_S = 0.25
# Reports arrive at 2 Hz, so a status read this soon after a command can predate it.
_CHARGE_COMMAND_GRACE_S = 1.0
_BUSY_GOAL_CANCEL_PERIOD_S = 1.0

ChargeTask = Literal["dock", "undock"]


@dataclass
class _ChargeJob:
    task: ChargeTask
    started: bool = False
    failure: str | None = None
    done: Event = field(default_factory=Event)


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


def dock_failure(
    status: int,
    entered: bool,
    elapsed_s: float,
    *,
    start_timeout_s: float,
    dock_timeout_s: float,
) -> str | None:
    """Return why docking failed, or None while it has not."""
    if status == CHARGE_CHARGING:
        return None
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


def _battery_state(report: BatteryReport) -> BatteryState | None:
    """Combine the two packs into one BatteryState. A pack at 0 V is absent, not empty."""
    packs = [pack for pack in (report.left, report.right) if pack.voltage > 0.0]
    if not packs:
        return None
    return BatteryState(
        voltage=sum(pack.voltage for pack in packs) / len(packs),
        percentage=sum(pack.level for pack in packs) / len(packs) / 100.0,
        temperature=max(pack.temperature for pack in packs),
    )


class M20Connection(Module):
    """Expose the planner-facing M20 command and terrain surface.

    The hardware bridge owns ROS 2/DrDDS and the command watchdog. This module
    forwards ``requested_cmd_vel`` and its own movement RPCs on the standard DimOS
    ``cmd_vel`` stream, and drops them while docking or on the dock. The
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
    requested_cmd_vel: In[Twist]
    command_ready: In[Bool]
    motion_state: In[Int32]
    gait_state: In[UInt32]
    odometry: In[Odometry]
    motion_state_cmd: Out[Int32]
    gait_cmd: Out[UInt32]
    odom: Out[PoseStamped]
    goal: Out[PointStamped]
    battery: Out[BatteryState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._state_condition = Condition(self._lock)
        self._command_ready = False
        self._motion_state: int | None = None
        self._gait_state: int | None = None
        self._navigation_terrain: NavigationTerrain = "stairs"
        self._stopping = Event()
        self._auto_stand_thread: Thread | None = None
        self._basic_server = BasicServerLink(
            host=self.config.basic_server_host,
            tcp_port=self.config.basic_server_tcp_port,
            udp_port=self.config.basic_server_udp_port,
            timeout_s=self.config.basic_server_timeout_s,
            on_report=self._on_basic_server_report,
            on_battery=self._on_battery_report,
        )
        self._charge_job: _ChargeJob | None = None
        self._charge_thread: Thread | None = None
        self._last_busy_goal_cancel = -math.inf

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.requested_cmd_vel.subscribe(self._on_requested_cmd_vel))
        )
        self.register_disposable(Disposable(self.command_ready.subscribe(self._on_command_ready)))
        self.register_disposable(Disposable(self.motion_state.subscribe(self._on_motion_state)))
        self.register_disposable(Disposable(self.gait_state.subscribe(self._on_gait_state)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self._stopping.clear()
        self._basic_server.start()
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
            try:
                self.undock()
            except RuntimeError as error:
                if not self._stopping.is_set():
                    logger.error("M20 did not leave the dock on start: %s", error)
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
        for thread in (self._charge_thread, self._auto_stand_thread):
            if thread is not None:
                thread.join(timeout=self.config.basic_server_timeout_s)
        self._basic_server.stop()
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
        cancelled instead.
        """
        with self._lock:
            job = self._charge_job
            docking = (job is not None and job.task == "dock") or (
                self._fresh_charge_status() == CHARGE_ENTERING
            )
        if docking:
            logger.info("M20 stop_movement cancels docking")
            try:
                self._start_undock()
            except RuntimeError as error:
                logger.warning("M20 stop_movement did not cancel docking: %s", error)
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
            return self._basic_server.usage_mode in (None, USAGE_MODE_NAVIGATION)

        if not was_walking:
            # A stand makes basic_server reset the mode and gait, losing any mode sent before.
            self._wait_for_gait_state(GAIT_BASIC, _VENDOR_GAIT_WAIT_S * attempts)

        # Right after a stand basic_server answers OK to a usage mode but drops it.
        # Resend if the mode or gait falls back.
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

    @rpc
    def dock(self) -> None:
        """Dock and charge. Returns once charging. Raises RuntimeError with the reason otherwise.

        The robot must stand still about 2 m in front of the dock, facing both reflectors.
        """
        self._finish(self._start_dock())

    @rpc
    def undock(self) -> None:
        """Leave the dock and stand ready to navigate. Raises RuntimeError on failure.

        Returns at once when the robot is off the dock.
        """
        job = self._start_undock()
        if job is not None:
            self._finish(job)

    @rpc
    def charge_status(self) -> str:
        """Report the robot's charging state, its battery level and any charging error."""
        status = self._fresh_charge_status()
        error = self._basic_server.charge_error
        parts = ["unknown" if status is None else _CHARGE_LABELS.get(status, "unknown")]
        battery = self._fresh_battery_state()
        if battery is not None:
            parts.append(f"battery {battery.percentage:.0%}")
        if error is not None:
            parts.append(f"error {_charge_error_text(error)}")
        return ", ".join(parts)

    @staticmethod
    def _finish(job: _ChargeJob) -> None:
        job.done.wait()
        if job.failure is not None:
            raise RuntimeError(job.failure)

    def _start_dock(self) -> _ChargeJob:
        with self._state_condition:
            if self._charge_job is not None and self._charge_job.task == "dock":
                raise RuntimeError("Not docking: a dock is already under way.")
            refusal = self._dock_refusal()
            if refusal is not None:
                raise RuntimeError(f"Not docking: {refusal}.")
            job = _ChargeJob("dock")
            self._charge_job = job
            self._basic_server.charge_error = None

        self._cancel_nav_goal()
        self._charge_thread = Thread(
            target=self._run_dock, args=(job,), name="m20-dock", daemon=True
        )
        self._charge_thread.start()
        return job

    def _start_undock(self) -> _ChargeJob | None:
        """Start leaving the dock. Returns None when there is nothing to wait for."""
        with self._state_condition:
            current = self._charge_job
            if current is not None and current.task == "undock":
                raise RuntimeError("Not undocking: an undock is already under way.")
            if current is not None and not current.started:
                self._charge_job = None
                self._state_condition.notify_all()
                return None
            status = self._fresh_charge_status()
            if current is None and status == CHARGE_IDLE:
                return None
            previous = self._charge_thread
            job = _ChargeJob("undock")
            self._charge_job = job
            self._basic_server.charge_error = None
            self._state_condition.notify_all()

        if status != CHARGE_EXITING and not self._basic_server.send_charge(CHARGE_COMMAND_STOP):
            with self._state_condition:
                self._charge_job = None
            raise RuntimeError("Not undocking: could not reach the robot.")
        self._charge_thread = Thread(
            target=self._run_undock, args=(job, previous), name="m20-undock", daemon=True
        )
        self._charge_thread.start()
        return job

    def _dock_refusal(self) -> str | None:
        status = self._fresh_charge_status()
        if status is None:
            return "the robot is not reporting its charge status"
        if self._charge_job is not None:
            return "an undock is still running"
        if self._charge_thread is not None and self._charge_thread.is_alive():
            return "the last dock or undock is still finishing"
        if status != CHARGE_IDLE:
            return f"its charge status is {_CHARGE_LABELS.get(status, status)}"
        if self._motion_state != MOTION_RL_CONTROL:
            return "the robot is not standing ready to walk"
        return None

    def _run_dock(self, job: _ChargeJob) -> None:
        try:
            job.failure = self._watch_dock(job)
        except _StoppingError:
            job.failure = "Stopped docking, module stopping."
        except Exception:
            logger.exception("M20 dock failed")
            job.failure = "Docking failed with an internal error."
        logger.info("M20 dock: %s", job.failure or "Docked and charging.")
        with self._state_condition:
            if self._charge_job is job:
                self._charge_job = None
            self._state_condition.notify_all()
        job.done.set()

    def _watch_dock(self, job: _ChargeJob) -> str | None:
        """Wait for the robot to charge and return why it did not, or None."""
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._charge_job is not job or self._stopping.is_set(),
                timeout=self.config.dock_settle_s,
            )
            if self._charge_job is not job:
                return "Docking cancelled before it started."
            if self._stopping.is_set():
                return "Docking cancelled, module stopping."
            job.started = True
            if not self._basic_server.send_charge(CHARGE_COMMAND_START):
                return "Docking failed: could not reach the robot."

        started_at = time.monotonic()
        entered = False
        reported: int | None = None
        # The robot can clear its error as it gives up, so keep the last one seen for the verdict.
        error: int | None = None
        while True:
            with self._state_condition:
                self._state_condition.wait(timeout=_CHARGE_POLL_S)
                if self._charge_job is not job:
                    return "Docking cancelled by undock."
                if self._stopping.is_set():
                    return "Stopped watching docking, module stopping."
                status = self._fresh_charge_status()
                error = self._basic_server.charge_error or error

            if status is None:
                self._basic_server.send_charge(CHARGE_COMMAND_STOP)
                return "Docking failed: the robot stopped reporting its charge status."
            entered = entered or status != CHARGE_IDLE
            if status != reported and status != CHARGE_IDLE:
                reported = status
                label = _CHARGE_LABELS.get(status, str(status))
                logger.info("M20 charge status: %s", label)

            if status == CHARGE_CHARGING:
                return None
            failure = dock_failure(
                status,
                entered,
                time.monotonic() - started_at,
                start_timeout_s=self.config.dock_start_timeout_s,
                dock_timeout_s=self.config.dock_timeout_s,
            )
            if failure is None:
                continue
            if status in (CHARGE_ENTERING, CHARGE_ON_DOCK):
                self._basic_server.send_charge(CHARGE_COMMAND_STOP)
            detail = "" if error is None else f" ({_charge_error_text(error)})"
            return f"Docking failed: {failure}{detail}. {self._resume_navigation(job)}"

    def _resume_navigation(self, job: _ChargeJob) -> str:
        """Restore navigation mode once the robot is idle, because idle switches it to Basic."""
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._charge_job is not job
                or self._stopping.is_set()
                or self._fresh_charge_status() == CHARGE_IDLE,
                timeout=self.config.undock_timeout_s,
            )
            if self._charge_job is not job:
                return "Undocking took over."
            if self._stopping.is_set():
                return "The module is stopping."
            if self._fresh_charge_status() != CHARGE_IDLE:
                return "The robot did not return to idle, so navigation mode was not restored."
            if self._motion_state != MOTION_RL_CONTROL:
                return "The robot is not standing. Call standup() before navigating."
        if not self._stand():
            return "The robot did not return to navigation mode. Call standup() before navigating."
        return "The robot is standing ready to navigate."

    def _run_undock(self, job: _ChargeJob, previous: Thread | None) -> None:
        try:
            if previous is not None:
                previous.join()
            job.failure = self._watch_undock()
            if job.failure is None and not self._stand():
                job.failure = "Off the dock, but the robot did not stand ready to navigate."
        except _StoppingError:
            job.failure = "Stopped undocking, module stopping."
        except Exception:
            logger.exception("M20 undock failed")
            job.failure = "Undocking failed with an internal error."
        logger.info("M20 undock: %s", job.failure or "Off the dock and ready to navigate.")
        with self._state_condition:
            self._charge_job = None
            self._state_condition.notify_all()
        job.done.set()

    def _watch_undock(self) -> str | None:
        """Wait for the robot to leave the dock and return why it did not, or None."""
        started_at = time.monotonic()
        with self._state_condition:
            while True:
                self._state_condition.wait(timeout=_CHARGE_POLL_S)
                if self._stopping.is_set():
                    return "Stopped watching undocking, module stopping."
                elapsed = time.monotonic() - started_at
                if elapsed >= _CHARGE_COMMAND_GRACE_S:
                    error = self._basic_server.charge_error
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
        report = self._basic_server.charge
        on_dock = report is not None and report.status in _CHARGE_BUSY_STATES
        with self._lock:
            return on_dock or self._charge_job is not None

    def _fresh_charge_status(self) -> int | None:
        report = self._basic_server.charge
        if report is None or time.monotonic() - report.received_at > _REPORT_MAX_AGE_S:
            return None
        return report.status

    def _fresh_battery_state(self) -> BatteryState | None:
        report = self._basic_server.battery
        if report is None or time.monotonic() - report.received_at > _REPORT_MAX_AGE_S:
            return None
        return _battery_state(report)

    def _wait_for_charge_status(self, timeout_s: float) -> bool:
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._stopping.is_set() or self._fresh_charge_status() is not None,
                timeout=timeout_s,
            )
            return self._fresh_charge_status() is not None

    def _cancel_nav_goal(self) -> None:
        self.goal.publish(PointStamped(math.nan, math.nan, math.nan, frame_id=WORLD_FRAME))

    def _on_basic_server_report(self) -> None:
        with self._state_condition:
            self._state_condition.notify_all()

    def _on_battery_report(self, report: BatteryReport) -> None:
        state = _battery_state(report)
        if state is not None:
            self.battery.publish(state)

    def _on_requested_cmd_vel(self, twist: Twist) -> None:
        if not self._charge_busy():
            self.cmd_vel.publish(twist)
            return
        now = time.monotonic()
        with self._lock:
            cancel = now - self._last_busy_goal_cancel >= _BUSY_GOAL_CANCEL_PERIOD_S
            if cancel:
                self._last_busy_goal_cancel = now
        if cancel:
            logger.warning("M20 dropped velocity while docking or on the dock, cancelling the goal")
            self._cancel_nav_goal()

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
        for _attempt in range(_GAIT_COMMAND_ATTEMPTS):
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
        return self._basic_server.set_usage_mode(USAGE_MODE_NAVIGATION)

    def _wait_for_motion_state(self, expected: int, timeout_s: float) -> bool:
        return self._wait_unless_stopping(lambda: self._motion_state == expected, timeout_s)

    def _wait_for_gait_state(self, expected: int, timeout_s: float) -> bool:
        return self._wait_unless_stopping(lambda: self._gait_state == expected, timeout_s)

    def _wait_for_control_readiness(self, timeout_s: float) -> bool:
        return self._wait_unless_stopping(lambda: self._command_ready, timeout_s)

    def _holds(self, condition: Callable[[], bool], duration_s: float) -> bool:
        """Watch for duration_s and return False as soon as condition stops holding."""
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._stopping.is_set() or not condition(), timeout=duration_s
            )
            if self._stopping.is_set():
                raise _StoppingError
            return condition()

    def _wait_unless_stopping(self, ready: Callable[[], bool], timeout_s: float) -> bool:
        with self._state_condition:
            self._state_condition.wait_for(
                lambda: self._stopping.is_set() or ready(), timeout=timeout_s
            )
            if self._stopping.is_set():
                raise _StoppingError
            return ready()


def _charge_error_text(code: int) -> str:
    return f"{code:#x} {CHARGE_ERROR_NAMES.get(code, 'unknown')}"

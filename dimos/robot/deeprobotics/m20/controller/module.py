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

"""The M20 controller: one native module for the motor bus, the batteries and the walking policy that replaces rl_deploy.

It runs on AOS, where the motor bus (eth1) and the ecan shim's socket are. Joint names and the motor-to-URDF
calibration match the vendor URDF/MJCF (``fl_hipx_joint`` ...).

- ``mode="tap"`` (default): read the frames the ``ecan_tap.so`` shim streams out of rl_deploy (installed by
  ``m20tool setup``) and publish joint state, joint commands and both battery packs. Sends nothing.
- ``mode="owner"`` (rl_deploy stopped): own the motor bus with m20-ecan's owner (rl_deploy's startup handshake, then
  1 kHz) and send the BMS charge-enable request from m20-battery's rule while docked and lying.
  ``policy=False`` (default) sends only rl_deploy's zero-gain sitting command, which keeps the board reporting
  without holding or moving anything. ``policy=True`` runs rl_deploy's state machine and ONNX runners
  (``rust/m20_controller.toml``) on the bus, driven by ``cmd_vel`` (m/s, rad/s), ``motion_cmd`` (passive, stand,
  rl_control, lie_down, joint_damping) and ``gait_cmd`` (vendor gait code), on the body IMU from ``imu``. Gains reach
  the motors only with ``allow_gains=True``; off (the default) every command that holds a joint goes out as the
  sitting command, so the state machine runs but nothing moves. Never turn it on without an operator's go.
- Docking (owner + policy): ``dock()`` / ``undock()`` run the vendor charge_manager's sequence inside the controller
  (``rust/src/dock``, with the vendor's own ``charge_manager.toml``) on the dock pose and the obstacle check from
  ``M20DockDetector`` (``dock_pose``, ``dock_obstacle``) and the packs' charge MOS; ``charge_status()`` is its
  latest status.
- The vendor phone app (owner + policy, ``[app_server]`` in ``rust/m20_controller.toml``): basic_server's UDP 30000 /
  TCP 30001 server, its commands on the same path as ``cmd_vel``, ``motion_cmd`` and ``gait_cmd``.
"""

from __future__ import annotations

from collections.abc import Callable
import functools
import json
import threading
import time
from typing import TYPE_CHECKING, Any, Literal

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.resource import CompositeResource
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.String import String
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.navigation.spec import cancel_goal
from dimos.robot.deeprobotics.m20.basic_server import (
    CHARGE_CHARGING,
    CHARGE_ENTERING,
    CHARGE_EXITING,
    CHARGE_IDLE,
    CHARGE_ON_DOCK,
    USAGE_MODE_NAVIGATION,
)
from dimos.robot.deeprobotics.m20.connection import (
    _CHARGE_BUSY_STATES as CHARGE_BUSY_STATES,
    _CHARGE_COMMAND_GRACE_S as CHARGE_COMMAND_GRACE_S,
    _CHARGE_LABELS as CHARGE_LABELS,
    _CHARGE_POLL_S as CHARGE_POLL_S,
    dock_failure,
)
from dimos.robot.deeprobotics.m20.constants import WORLD_FRAME
from dimos.utils.logging_config import setup_logger

# rl_deploy's basic flat gait, the entry gait rl_control is started on (as rl_deploy accepts it)
BASIC_FLAT_GAIT = 0x1001

NavigationTerrain = Literal["flat", "stairs"]
# rl_deploy's agile navigation gaits, the ones vendor-mode M20Connection walks on (flat 0x3002 NaviFlat, stairs 0x3003)
NAVIGATION_GAITS: dict[str, int] = {"flat": 0x3002, "stairs": 0x3003}


def _gait(status: dict[str, Any]) -> int | None:
    gait = status.get("gait")
    return int(gait, 16) if isinstance(gait, str) else None


logger = setup_logger()


_PYTHON_ONLY = frozenset(
    {
        "navigation_terrain",
        "stand_timeout_s",
        "rl_control_timeout_s",
        "gait_timeout_s",
        "navigation_mode_attempts",
        "control_ready_timeout_s",
        "vendor_gait_wait_s",
        "gait_command_attempts",
        "dock_settle_s",
        "dock_start_timeout_s",
        "dock_timeout_s",
        "undock_timeout_s",
        "report_max_age_s",
        "busy_goal_cancel_period_s",
    }
)


class M20ControllerConfig(NativeModuleConfig):
    # SIGTERM -> the native lies a standing robot down (safety.shutdown_lie_down_max_s 11 s) before it exits
    shutdown_timeout: float = 15.0
    source_dir: str | None = "dimos/robot/deeprobotics/m20/controller/rust"
    executable: str = "result/bin/m20_controller_module"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    # frame_id is a NativeModuleConfig field the native config needs too
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # "tap": read rl_deploy's frames through its shim; "owner" (rl_deploy stopped): own the motor bus
    mode: Literal["tap", "owner"] = "tap"
    # tap: the unix datagram socket the shim inside rl_deploy streams to (this module binds it)
    socket_path: str = "/var/opt/robot/data/ecan_capture/live.sock"
    # owner: the motor-bus interface, and whether to send the BMS charge-enable request (docked, lying)
    interface: str = "eth1"
    charge_enable: bool = True
    frame_id: str = "base_link"
    # seconds without a frame before warning that rl_deploy or its shim is not running
    silence_warning_s: float = Field(default=5.0, gt=0.0, le=3600.0)
    # owner: board replies between progress log lines
    progress_log_replies: int = Field(default=5000, ge=1)
    # owner: run the walking policy on the bus (off: the zero-gain sitting command only)
    policy: bool = False
    # owner + policy: let commands carrying kp / kd reach the motors; never on without an operator's go
    allow_gains: bool = False
    # policy: an m20_controller.toml path on the board ("" = the one built into the binary)
    controller_config: str = ""
    # policy: the extracted rl_deploy ONNX policies on the board ("" = ~/.cache/dimos/m20_policies)
    policy_dir: str = ""
    # policy: seconds without a new command before the velocity is zeroed, over the TOML's
    # safety.command_timeout_s (0 keeps the TOML's; the native config takes no None)
    command_timeout_s: float = Field(default=0.0, ge=0.0)
    # policy: the gait code rl_control is entered on (rl_deploy's accepted entry gaits; 0x1001 basic flat)
    gait: int = BASIC_FLAT_GAIT
    # standup(): the agile gait switched to once rl_control's stand-in is over (M20Connection's default is stairs)
    navigation_terrain: NavigationTerrain = "stairs"
    # policy: the vendor's charge_manager.toml (docking reads it at startup); a board without it takes the copy
    # M20DockDetector publishes from NOS
    charge_manager_toml: str = "/opt/robot/share/charge_manager/config/charge_manager.toml"
    # policy: log the status when the commanded body height moves this far (m); other changes always log
    status_log_height_step_m: float = Field(default=0.05, gt=0.0)
    # policy: the vendor bridge's cmd_vel clamp (m/s, m/s, rad/s) and command_ready rate
    cmd_vel_limits: tuple[float, float, float] = (2.0, 1.0, 2.0)
    command_ready_hz: float = Field(default=10.0, gt=0.0)
    # standup(): M20Connection's timeouts and tries
    stand_timeout_s: float = Field(default=12.0, gt=0.0)
    rl_control_timeout_s: float = Field(default=5.0, gt=0.0)
    gait_timeout_s: float = Field(default=5.0, gt=0.0)
    navigation_mode_attempts: int = Field(default=3, ge=1)
    control_ready_timeout_s: float = Field(default=15.0, gt=0.0)
    vendor_gait_wait_s: float = Field(default=3.0, gt=0.0)
    gait_command_attempts: int = Field(default=2, ge=1)
    # dock() / undock(): M20Connection's waits
    dock_settle_s: float = Field(default=1.0, ge=0.0)
    dock_start_timeout_s: float = Field(default=5.0, gt=0.0)
    dock_timeout_s: float = Field(default=150.0, gt=0.0)
    undock_timeout_s: float = Field(default=45.0, gt=0.0)
    report_max_age_s: float = Field(default=3.0, gt=0.0)
    busy_goal_cancel_period_s: float = Field(default=1.0, gt=0.0)

    def to_config_dict(self) -> dict[str, Any]:
        # the Python RPCs' fields; the native struct rejects unknown fields
        return {k: v for k, v in super().to_config_dict().items() if k not in _PYTHON_ONLY}


class M20ControllerApi(CompositeResource):
    """The ports and RPCs of the controller, over `_send` / `_watch` (the native process's topics, or the service's)."""

    config: Any

    def _send(self, port: str, msg: Any) -> None:
        raise NotImplementedError

    def _watch(self, port: str, callback: Callable[[Any], None]) -> Callable[[], None]:
        raise NotImplementedError

    cmd_vel: In[Twist]
    imu: In[Imu]
    # rt/IMU_DATA's roll, pitch, yaw (deg) from the body IMU module: rl_deploy's gravity and tilt
    imu_rpy: In[Vector3]
    motion_cmd: In[String]
    gait_cmd: In[UInt32]
    # basic_server's usage mode: 0 regular, 1 navigation (cmd_vel is obeyed in 1 and 2), 2 assist
    usage_mode: In[UInt32]
    dock_pose: In[PoseStamped]
    dock_cmd: In[String]
    dock_obstacle: In[Float32]
    charge_manager_config: In[String]
    joint_state: Out[JointState]
    joint_command: Out[JointState]
    battery: Out[BatteryState]
    battery_left: Out[BatteryState]
    battery_right: Out[BatteryState]
    command_ready: Out[Bool]
    controller_state: Out[String]
    dock_state: Out[String]
    body_height: Out[Float32]
    controller_report: Out[String]
    # /HES_STATUS as rl_deploy writes it: the ECAN board's hard-stop byte (0 clear, 1 pressed)
    hes_status: Out[Int32]
    # true while the ECAN board's hard-stop byte is non-zero (hes_status as a Bool)
    hardware_estop: Out[Bool]
    # nav goal cancels around docking (M20Connection's)
    goal: Out[PoseStamped]

    @rpc
    def start(self) -> None:
        self._state = ""
        self._dock_state: dict[str, Any] = {}
        self._status: dict[str, Any] = {}
        self._status_at = 0.0
        self._charge_job: str | None = None
        self._last_goal_cancel = 0.0
        self._battery: dict[str, dict[str, Any]] = {}
        self._state_lock = threading.Lock()
        self._status_changed = threading.Condition(self._state_lock)
        # before the native process starts: it publishes the state only when it changes, so its first one would be missed
        self.register_disposable(
            Disposable(self._watch("controller_state", self._on_controller_state))
        )
        self.register_disposable(Disposable(self._watch("dock_state", self._on_dock_state)))
        self.register_disposable(Disposable(self._watch("cmd_vel", self._on_planner_cmd_vel)))
        self.register_disposable(
            Disposable(self._watch("controller_report", self._on_controller_report))
        )
        for name in ("battery", "battery_left", "battery_right"):
            self.register_disposable(
                Disposable(self._watch(name, functools.partial(self._on_battery, name)))
            )
        super().start()

    def _on_dock_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            logger.warning(f"M20 dock_state is not JSON: {msg.data!r}")
            return
        with self._state_lock:
            self._dock_state = state

    def _on_controller_report(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            logger.warning(f"M20 controller_report is not JSON: {msg.data!r}")
            return
        with self._state_lock:
            self._status = status
            self._status_at = time.monotonic()
            self._status_changed.notify_all()

    def _on_battery(self, name: str, msg: BatteryState) -> None:
        reading = {
            "percentage": msg.percentage,
            "voltage": msg.voltage,
            "current": msg.current,
            "temperature": msg.temperature,
            "power_supply_status": msg.power_supply_status,
        }
        with self._state_lock:
            self._battery[name] = reading

    def _on_controller_state(self, msg: String) -> None:
        with self._state_lock:
            self._state = msg.data

    def _wait_status(self, ready: Callable[[dict[str, Any]], bool], timeout_s: float) -> bool:
        with self._status_changed:
            return self._status_changed.wait_for(lambda: ready(self._status), timeout=timeout_s)

    def _holds(self, condition: Callable[[dict[str, Any]], bool], duration_s: float) -> bool:
        with self._status_changed:
            self._status_changed.wait_for(lambda: not condition(self._status), timeout=duration_s)
            return condition(self._status)

    def _ensure_rl_control(self) -> bool:
        with self._state_lock:
            state = self._status.get("state")
        if state == "RlControl":
            return True
        if state != "StandUp":
            self._send("motion_cmd", String("stand"))
            if not self._wait_status(
                lambda s: s.get("state") == "StandUp", self.config.stand_timeout_s
            ):
                logger.error("M20 did not confirm Stand before the transition timeout")
                return False
        self._send("motion_cmd", String("rl_control"))
        if not self._wait_status(
            lambda s: s.get("state") == "RlControl", self.config.rl_control_timeout_s
        ):
            logger.error("M20 did not confirm RL Control after standing")
            return False
        return True

    def _set_gait_and_wait(self, gait: int) -> bool:
        for _ in range(self.config.gait_command_attempts):
            self._send("gait_cmd", UInt32(gait))
            if self._wait_status(lambda s: _gait(s) == gait, self.config.gait_timeout_s):
                return True
        return False

    @rpc
    def standup(self) -> bool:
        """M20Connection.standup() on this controller: stand, RL control, usage mode navigation, confirm 0x1001, then the navigation terrain's agile gait."""
        if self._refuse_while_charging("standup"):
            return False
        with self._state_lock:
            was_walking = self._status.get("state") == "RlControl"
        if not self._ensure_rl_control():
            return False
        gait = NAVIGATION_GAITS[self.config.navigation_terrain]
        attempts = self.config.navigation_mode_attempts

        def navigating(s: dict[str, Any]) -> bool:
            return s.get("usage_mode") == USAGE_MODE_NAVIGATION

        if not was_walking:
            # the stand resets the mode and gait (basic_server's SetMotionState)
            self._wait_status(
                lambda s: _gait(s) == BASIC_FLAT_GAIT, self.config.vendor_gait_wait_s * attempts
            )
        for attempt in range(1, attempts + 1):
            self._send("usage_mode", UInt32(USAGE_MODE_NAVIGATION))
            self._wait_status(navigating, self.config.vendor_gait_wait_s)
            if not self._set_gait_and_wait(BASIC_FLAT_GAIT):
                logger.error("M20 did not confirm the Basic gait")
                return False
            if not self._set_gait_and_wait(gait):
                logger.error(
                    f"M20 did not confirm the agile {self.config.navigation_terrain} navigation gait"
                )
                return False
            if self._holds(
                lambda s: _gait(s) == gait and navigating(s), self.config.vendor_gait_wait_s
            ):
                break
            if attempt == attempts:
                logger.error(
                    f"M20 dropped navigation mode {attempts} times right after accepting it"
                )
                return False
            logger.warning("M20 dropped navigation mode, sending it again")
        if not self._wait_status(
            lambda s: bool(s.get("ready")), self.config.control_ready_timeout_s
        ):
            logger.error("M20 robot control path did not become ready")
            return False
        return True

    @rpc
    def set_usage_mode(self, mode: int) -> bool:
        """basic_server's control usage mode: 0 regular (cmd_vel ignored, gait 0x1001), 1 navigation, 2 assist (gait 0x1001)."""
        if mode not in (0, 1, 2):
            logger.error(f"unsupported M20 usage mode: {mode}")
            return False
        self._send("usage_mode", UInt32(mode))
        return True

    @rpc
    def liedown(self) -> bool:
        """Zero the velocity and lie down."""
        if self._refuse_while_charging("liedown"):
            return False
        self.stop_movement()
        self._send("motion_cmd", String("lie_down"))
        return True

    @rpc
    def stand(self) -> bool:
        """Same as standup()."""
        return self.standup()

    @rpc
    def lie_down(self) -> bool:
        """Same as liedown()."""
        return self.liedown()

    @rpc
    def align_body_posture(self) -> bool:
        """basic_server's fall recovery (aligningBodyPosture): from Idle, SixAxisStand on 0xf003 gets it up, then 0x1001."""
        self._send("motion_cmd", String("align_body_posture"))
        return True

    @rpc
    def passive(self) -> bool:
        """Motion state 0, as rl_deploy takes it: only joint damping and a lie-down end in Idle; standing ignores it."""
        self.stop_movement()
        self._send("motion_cmd", String("passive"))
        return True

    @rpc
    def damp(self) -> bool:
        """Joint damping: the legs sink slowly instead of holding (rl_deploy's fault response)."""
        self.stop_movement()
        self._send("motion_cmd", String("joint_damping"))
        return True

    @rpc
    def set_navigation_terrain(self, terrain: NavigationTerrain) -> bool:
        """Select the agile gait for flat ground or stairs; applied now while walking, else by the next standup()."""
        gait = NAVIGATION_GAITS.get(terrain)
        if gait is None:
            logger.error(f"unsupported M20 navigation terrain: {terrain}")
            return False
        if self._refuse_while_charging("set_navigation_terrain"):
            return False
        self.config.navigation_terrain = terrain
        with self._state_lock:
            walking = self._status.get("state") == "RlControl"
        if not walking:
            return True
        self.stop_movement()
        return self._set_gait_and_wait(gait)

    def _charge_busy(self) -> bool:
        with self._state_lock:
            job = self._charge_job
        return job is not None or self._charger() in CHARGE_BUSY_STATES

    def _refuse_while_charging(self, action: str) -> bool:
        if not self._charge_busy():
            return False
        logger.warning(f"M20 refused {action} while docking or on the dock")
        return True

    @rpc
    def stop_movement(self) -> None:
        """M20Connection.stop_movement(): a zero velocity, or a dock under way cancelled (charge command 0)."""
        with self._state_lock:
            docking = self._charge_job == "dock" or self._charger() == CHARGE_ENTERING
        if docking:
            logger.info("M20 stop_movement cancels docking")
            with self._status_changed:
                self._charge_job = None
                self._status_changed.notify_all()
            self._send("dock_cmd", String("undock"))
        elif self._charge_busy():
            logger.info("M20 stop_movement left the robot to the docking sequence")
        else:
            self._send("cmd_vel", Twist.zero())

    @rpc
    def controller_status(self) -> str:
        """The state machine's latest state (e.g. "RlControl", "StandUp", "JointDamping (fault: ...)")."""
        with self._state_lock:
            return self._state

    @rpc
    def status(self) -> dict[str, Any]:
        """controller_report (state, fault, gait, runner, heights) plus dock_state and the batteries; {} fields before the first."""
        with self._state_lock:
            out = dict(self._status)
            out["dock"] = dict(self._dock_state)
            out["batteries"] = {k: dict(v) for k, v in self._battery.items()}
        return out

    def _can_sequence(self, what: str) -> bool:
        if self.config.mode != "owner" or not self.config.policy:
            logger.error(
                f"M20 {what}: needs mode='owner' with policy=True (the controller drives the robot)"
            )
            return False
        return True

    def _charger(self) -> int | None:
        """charge_manager's state from the latest controller report; None when older than M20Connection allows."""
        # no lock: wait predicates call it under the status condition (both are replaced whole)
        status, at = self._status, self._status_at
        if not status or time.monotonic() - at > self.config.report_max_age_s:
            return None
        state = status.get("charger_state")
        return state if isinstance(state, int) else None

    def _charge_error(self) -> str | None:
        with self._state_lock:
            dock = dict(self._dock_state)
        return str(dock.get("result")) if dock.get("error") else None

    def _wait_charger(self, ready: Callable[[int | None], bool], timeout_s: float) -> bool:
        return self._wait_status(lambda _: ready(self._charger()), timeout_s)

    def _cancel_nav_goal(self) -> None:
        self.goal.publish(cancel_goal(frame_id=WORLD_FRAME))

    def _on_planner_cmd_vel(self, _twist: Twist) -> None:
        # M20Connection drops velocity while docking or on the dock and cancels the goal (the controller drops it)
        if not self._charge_busy():
            return
        now = time.monotonic()
        with self._state_lock:
            cancel = now - self._last_goal_cancel >= self.config.busy_goal_cancel_period_s
            if cancel:
                self._last_goal_cancel = now
        if cancel:
            logger.warning("M20 dropped velocity while docking or on the dock, cancelling the goal")
            self._cancel_nav_goal()

    def _dock_refusal(self) -> str | None:
        status = self._charger()
        if status is None:
            return "the robot is not reporting its charge status"
        with self._state_lock:
            job, state = self._charge_job, self._status.get("state")
        if job is not None:
            return f"a {job} is still running"
        if status != CHARGE_IDLE:
            return f"its charge status is {CHARGE_LABELS.get(status, status)}"
        if state != "RlControl":
            return "the robot is not standing ready to walk"
        return None

    @rpc
    def dock(self) -> None:
        """M20Connection.dock(): returns once charging, else raises RuntimeError with the reason."""
        if not self._can_sequence("dock"):
            raise RuntimeError("Not docking: the controller does not drive the robot.")
        with self._state_lock:
            if self._charge_job == "dock":
                raise RuntimeError("Not docking: a dock is already under way.")
        refusal = self._dock_refusal()
        if refusal is not None:
            raise RuntimeError(f"Not docking: {refusal}.")
        with self._state_lock:
            self._charge_job = "dock"
        self._cancel_nav_goal()
        try:
            failure = self._watch_dock()
        finally:
            with self._status_changed:
                if self._charge_job == "dock":
                    self._charge_job = None
                self._status_changed.notify_all()
        logger.info(f"M20 dock: {failure or 'Docked and charging.'}")
        if failure is not None:
            raise RuntimeError(failure)

    def _watch_dock(self) -> str | None:
        # the settle lets an undock cancel the dock before it starts
        if self._wait_status(lambda _: self._charge_job != "dock", self.config.dock_settle_s):
            return "Docking cancelled before it started."
        self._send("dock_cmd", String("dock"))
        started_at = time.monotonic()
        entered = False
        error: str | None = None
        while True:
            self._wait_status(lambda _: self._charge_job != "dock", CHARGE_POLL_S)
            if self._charge_job != "dock":
                return "Docking cancelled by undock."
            status = self._charger()
            error = self._charge_error() or error
            if status is None:
                self._send("dock_cmd", String("cancel"))
                return "Docking failed: the robot stopped reporting its charge status."
            entered = entered or status != CHARGE_IDLE
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
                self._send("dock_cmd", String("cancel"))
            detail = "" if error is None else f" ({error})"
            return f"Docking failed: {failure}{detail}. {self._resume_navigation()}"

    def _resume_navigation(self) -> str:
        """M20Connection's: navigation mode back once the robot is idle (idle switches it to Basic)."""
        if not self._wait_charger(lambda s: s == CHARGE_IDLE, self.config.undock_timeout_s):
            return "The robot did not return to idle, so navigation mode was not restored."
        with self._state_lock:
            state = self._status.get("state")
        if state != "RlControl":
            return "The robot is not standing. Call standup() before navigating."
        if not self.standup():
            return "The robot did not return to navigation mode. Call standup() before navigating."
        return "The robot is standing ready to navigate."

    @rpc
    def undock(self) -> None:
        """M20Connection.undock(): leave the dock, stand ready to navigate; raises RuntimeError on failure."""
        if not self._can_sequence("undock"):
            raise RuntimeError("Not undocking: the controller does not drive the robot.")
        charger = self._charger()
        with self._status_changed:
            if self._charge_job == "undock":
                raise RuntimeError("Not undocking: an undock is already under way.")
            if self._charge_job == "dock":
                # a dock under way (or still settling) is cancelled by the undock
                self._charge_job = None
                self._status_changed.notify_all()
            elif charger == CHARGE_IDLE:
                return
            self._charge_job = "undock"
        try:
            if self._charger() != CHARGE_EXITING:
                self._send("dock_cmd", String("undock"))
            failure = self._watch_undock()
            if failure is None and not self.standup():
                failure = "Off the dock, but the robot did not stand ready to navigate."
        finally:
            with self._status_changed:
                self._charge_job = None
                self._status_changed.notify_all()
        logger.info(f"M20 undock: {failure or 'Off the dock and ready to navigate.'}")
        if failure is not None:
            raise RuntimeError(failure)

    def _watch_undock(self) -> str | None:
        started_at = time.monotonic()
        while True:
            self._wait_status(lambda _: False, CHARGE_POLL_S)
            elapsed = time.monotonic() - started_at
            if elapsed >= CHARGE_COMMAND_GRACE_S:
                error = self._charge_error()
                if error is not None:
                    return f"Undocking failed: {error}."
                if self._charger() == CHARGE_IDLE:
                    return None
            if elapsed >= self.config.undock_timeout_s:
                return f"Undocking failed: not off the dock after {elapsed:.0f} s."

    @rpc
    def cancel_dock(self) -> bool:
        """Stop a dock / undock where it is (zero velocity; lying stays lying)."""
        self._send("dock_cmd", String("cancel"))
        return True

    @rpc
    def charge_status(self) -> dict[str, Any]:
        """The docking sequence's latest status: phase, active, result, error, charging, packs_present,
        packs_charge_mos_on, retries, fine_point ({} before the controller has published one)."""
        with self._state_lock:
            return dict(self._dock_state)


class M20Controller(M20ControllerApi, NativeModule):
    """Joint state (1 kHz, URDF names), joint commands, both battery packs and, with the policy on, walking on cmd_vel."""

    config: M20ControllerConfig

    def _send(self, port: str, msg: Any) -> None:
        getattr(self, port).transport.publish(msg)

    def _watch(self, port: str, callback: Callable[[Any], None]) -> Callable[[], None]:
        # an Out's own subscribe() sees only this process's publishes; the native publishes on the transport
        transport = getattr(getattr(self, port), "_transport", None)
        if transport is None:
            return lambda: None
        return transport.subscribe(callback)  # type: ignore[no-any-return]


if TYPE_CHECKING:
    M20Controller()

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
"""

from __future__ import annotations

import json
import threading
from typing import TYPE_CHECKING, Any, Literal

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.String import String
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.utils.logging_config import setup_logger

# rl_deploy's basic flat gait, the entry gait rl_control is started on (as rl_deploy accepts it)
BASIC_FLAT_GAIT = 0x1001

NavigationTerrain = Literal["flat", "stairs"]
# rl_deploy's agile navigation gaits, the ones vendor-mode M20Connection walks on (flat 0x3002 NaviFlat, stairs 0x3003)
NAVIGATION_GAITS: dict[str, int] = {"flat": 0x3002, "stairs": 0x3003}

logger = setup_logger()


class M20ControllerConfig(NativeModuleConfig):
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

    def to_config_dict(self) -> dict[str, Any]:
        # navigation_terrain is the Python RPCs' alone; the native struct rejects unknown fields
        return {k: v for k, v in super().to_config_dict().items() if k != "navigation_terrain"}


class M20Controller(NativeModule):
    """Joint state (1 kHz, URDF names), joint commands, both battery packs and, with the policy on, walking on cmd_vel."""

    config: M20ControllerConfig

    cmd_vel: In[Twist]
    imu: In[Imu]
    motion_cmd: In[String]
    gait_cmd: In[UInt32]
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

    @rpc
    def start(self) -> None:
        self._state = ""
        self._dock_state: dict[str, Any] = {}
        self._navigate_once_walking: int | None = None
        self._state_lock = threading.Lock()
        # before the native process starts: it publishes the state only when it changes, so its first one would be missed
        self.register_disposable(
            Disposable(self.controller_state.subscribe(self._on_controller_state))
        )
        self.register_disposable(Disposable(self.dock_state.subscribe(self._on_dock_state)))
        super().start()

    def _on_dock_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            logger.warning(f"M20 dock_state is not JSON: {msg.data!r}")
            return
        with self._state_lock:
            self._dock_state = state

    def _on_controller_state(self, msg: String) -> None:
        with self._state_lock:
            self._state = msg.data
            gait, walking = self._navigate_once_walking, msg.data == "RlControl"
            if walking:
                self._navigate_once_walking = None
        # the state machine holds a gait change until rl_control's stand-in is over, as rl_deploy does
        if walking and gait is not None:
            self.gait_cmd.transport.publish(UInt32(gait))

    @rpc
    def standup(self) -> bool:
        """Stand up, enter rl_control on the entry gait, then switch to the navigation terrain's agile gait (as vendor-mode M20Connection does)."""
        with self._state_lock:
            self._navigate_once_walking = NAVIGATION_GAITS[self.config.navigation_terrain]
        self.gait_cmd.transport.publish(UInt32(self.config.gait))
        self.motion_cmd.transport.publish(String("rl_control"))
        return True

    @rpc
    def liedown(self) -> bool:
        """Zero the velocity and lie down."""
        with self._state_lock:
            self._navigate_once_walking = None
        self.stop_movement()
        self.motion_cmd.transport.publish(String("lie_down"))
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
    def passive(self) -> bool:
        """Go limp (no gains), from any state."""
        self.stop_movement()
        self.motion_cmd.transport.publish(String("passive"))
        return True

    @rpc
    def damp(self) -> bool:
        """Joint damping: the legs sink slowly instead of holding (rl_deploy's fault response)."""
        self.stop_movement()
        self.motion_cmd.transport.publish(String("joint_damping"))
        return True

    @rpc
    def set_navigation_terrain(self, terrain: NavigationTerrain) -> bool:
        """Select the agile gait for flat ground or stairs; applied now while walking, else by the next standup()."""
        gait = NAVIGATION_GAITS.get(terrain)
        if gait is None:
            logger.error(f"unsupported M20 navigation terrain: {terrain}")
            return False
        self.config.navigation_terrain = terrain
        with self._state_lock:
            walking = self._state == "RlControl"
        if walking:
            self.stop_movement()
            self.gait_cmd.transport.publish(UInt32(gait))
        return True

    @rpc
    def stop_movement(self) -> None:
        """Publish a zero velocity now (later commands still move it)."""
        self.cmd_vel.transport.publish(Twist.zero())

    @rpc
    def controller_status(self) -> str:
        """The state machine's latest state (e.g. "RlControl", "StandUp", "JointDamping (fault: ...)")."""
        with self._state_lock:
            return self._state

    def _can_sequence(self, what: str) -> bool:
        if self.config.mode != "owner" or not self.config.policy:
            logger.error(
                f"M20 {what}: needs mode='owner' with policy=True (the controller drives the robot)"
            )
            return False
        return True

    @rpc
    def dock(self) -> bool:
        """Dock and charge: from standing in rl_control with the dock (its two reflector posts) in sight ahead.

        The controller runs charge_manager's sequence: NaviCharge to the staging point and the fine point (stopping
        for obstacles ahead), FinetuneStand, lie down, confirm every pack's charge MOS, retry from the staging point.
        Returns whether the request was sent; the controller may still refuse it (charge_status()["result"] says
        why)."""
        if not self._can_sequence("dock"):
            return False
        with self._state_lock:
            state, self._navigate_once_walking = self._state, None
        if state != "RlControl":
            logger.error(
                f"M20 dock: needs the robot standing in rl_control (controller state {state!r})"
            )
            return False
        self.dock_cmd.transport.publish(String("dock"))
        return True

    @rpc
    def undock(self) -> bool:
        """While charging (or after a failed dock): stand on NaviCharge, back off to where docking started (1.3 m out
        when no dock this run recorded one), switch back to the gait docking started from. Returns whether the request
        was sent."""
        if not self._can_sequence("undock"):
            return False
        with self._state_lock:
            self._navigate_once_walking = None
        self.dock_cmd.transport.publish(String("undock"))
        return True

    @rpc
    def cancel_dock(self) -> bool:
        """Stop a dock / undock where it is (zero velocity; lying stays lying)."""
        self.dock_cmd.transport.publish(String("cancel"))
        return True

    @rpc
    def charge_status(self) -> dict[str, Any]:
        """The docking sequence's latest status: phase, active, result, error, charging, packs_present,
        packs_charge_mos_on, retries, fine_point ({} before the controller has published one)."""
        with self._state_lock:
            return dict(self._dock_state)


if TYPE_CHECKING:
    M20Controller()

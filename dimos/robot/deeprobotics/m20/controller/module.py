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
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String
from dimos.msgs.std_msgs.UInt32 import UInt32


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
    # owner: stop on a reply showing torque (N m) / joint speed (rad/s) above these, or a status word other than 1;
    # with allow_gains they must cover what the policy does
    guard_max_torque: float = Field(default=1.0, gt=0.0)
    guard_max_velocity: float = Field(default=0.5, gt=0.0)
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
    # policy: seconds an IMU sample stays usable (older, the state machine sees stale state and faults)
    imu_timeout_s: float = Field(default=0.05, gt=0.0)
    # policy: seconds without a new command before the velocity is zeroed, over the TOML's
    # safety.command_timeout_s (0 keeps the TOML's; the native config takes no None)
    command_timeout_s: float = Field(default=0.0, ge=0.0)


class M20Controller(NativeModule):
    """Joint state (1 kHz, URDF names), joint commands, both battery packs and, with the policy on, walking on cmd_vel."""

    config: M20ControllerConfig

    cmd_vel: In[Twist]
    imu: In[Imu]
    motion_cmd: In[String]
    gait_cmd: In[UInt32]
    joint_state: Out[JointState]
    joint_command: Out[JointState]
    battery: Out[BatteryState]
    battery_left: Out[BatteryState]
    battery_right: Out[BatteryState]
    command_ready: Out[Bool]
    controller_state: Out[String]

    @rpc
    def stand(self) -> None:
        """Stand up (policy on; the legs only move with allow_gains)."""
        self.motion_cmd.transport.publish(String("stand"))

    @rpc
    def lie_down(self) -> None:
        """Lie down."""
        self.motion_cmd.transport.publish(String("lie_down"))


if TYPE_CHECKING:
    M20Controller()

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

"""The M20's measured joint state, rl_deploy's joint commands and both battery packs, off the motor bus.

The native ``m20_ecan`` module reads the frames the ``ecan_tap.so`` shim streams out of the vendor walking controller
(rl_deploy) on AOS and sends nothing to the bus. The motor board only answers rl_deploy's 1 kHz commands, so it needs
rl_deploy running with the shim installed (``tap/record.sh install`` on AOS) and runs on AOS, where the socket is.
Joint names and the motor-to-URDF calibration match the vendor URDF/MJCF (``fl_hipx_joint`` ...).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.JointState import JointState


class M20EcanConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/deeprobotics/m20/ecan/rust"
    executable: str = "result/bin/m20_ecan"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    # the unix datagram socket the shim inside rl_deploy streams to (this module binds it)
    socket_path: str = "/var/opt/robot/data/ecan_capture/live.sock"
    frame_id: str = "base_link"
    # seconds without a frame before warning that rl_deploy or its shim is not running
    silence_warning_s: float = Field(default=5.0, gt=0.0, le=3600.0)


class M20Ecan(NativeModule):
    """Joint state (1 kHz, URDF names), commanded joint targets, and the two battery packs (1 Hz) of the M20."""

    config: M20EcanConfig

    joint_state: Out[JointState]
    joint_command: Out[JointState]
    battery: Out[BatteryState]
    battery_left: Out[BatteryState]
    battery_right: Out[BatteryState]


if TYPE_CHECKING:
    M20Ecan()

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

"""The M20's body control MCU on the AOS board's UART, through the native ``m20_ctrlmcu`` module (the m20-ctrlmcu crate), in place of the vendor ``ctrlmcu_node``.

Each ``Int32`` input is one vendor DDS service with the vendor's meaning (front/back pairs are ``back | front << 16``,
switches are ``value != 0``); ``led_status`` takes 6 bytes per LED entry (type, dir, num, cycle, color0, color1), up
to two. Every request waits for the MCU's reply and is reported on ``status`` as JSON; the two queries also publish
their reading. As with the vendor node nothing is sent at startup (``startup_commands`` is empty), nothing is polled
and there is no heartbeat: the MCU keeps its lidar, camera and fan state on its own.

Power inputs (lidar, video, ext_load, fp, gps) are refused unless ``allow_power``. The port has one owner: the module
will not open it while ``ctrlmcu_node`` runs.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos_lcm.std_msgs import UInt8MultiArray, UInt32MultiArray
from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.String import String
from dimos.msgs.std_msgs.UInt32 import UInt32

# The vendor's reply waits (ms) per input.
VENDOR_REPLY_TIMEOUT_MS = {
    "led_status": 1000,
    "fill_light": 2000,
    "fan_speed": 2000,
    "fan_speed_query": 2000,
    "ext_load_current_query": 1000,
    "ext_load_power": 1000,
    "host_led": 1000,
    "ext_led": 1000,
    "fp_power": 1000,
    "lidar_power": 1000,
    "gps_power": 1000,
    "video_power": 1000,
}


class M20CtrlMcuConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/deeprobotics/m20/ctrlmcu/rust"
    executable: str = "result/bin/m20_ctrlmcu"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    device: str = "/dev/ttyS3"
    baud: int = Field(default=115200, ge=1)
    read_chunk_bytes: int = Field(default=128, ge=1)
    # A declared payload longer than this is a false header (the longest known is 16).
    max_payload_bytes: int = Field(default=256, ge=0, le=65535)
    max_buffered_bytes: int = Field(default=4096, ge=10)
    reply_timeout_ms: dict[str, int] = Field(default_factory=lambda: dict(VENDOR_REPLY_TIMEOUT_MS))
    allow_lights: bool = True
    allow_fans: bool = True
    # Lidar, camera, external-load, FP radio and GPS power: off unless asked for.
    allow_power: bool = False
    # (input name, vendor int32) sent once the port opens; the vendor sends nothing.
    startup_commands: list[tuple[str, int]] = Field(default_factory=list)
    exclusive_with: list[str] = Field(default_factory=lambda: ["ctrlmcu_node"])


class M20CtrlMcu(NativeModule):
    """The M20's LEDs, fill lights, fans and power switches; replies and readings out."""

    config: M20CtrlMcuConfig

    led_status: In[UInt8MultiArray]
    fill_light: In[Int32]
    fan_speed: In[Int32]
    fan_speed_query: In[Int32]
    ext_load_current_query: In[Int32]
    ext_load_power: In[Int32]
    host_led: In[Int32]
    ext_led: In[Int32]
    fp_power: In[Int32]
    lidar_power: In[Int32]
    gps_power: In[Int32]
    video_power: In[Int32]

    status: Out[String]
    fan_speeds: Out[UInt32MultiArray]
    ext_load_current: Out[UInt32]


if TYPE_CHECKING:
    M20CtrlMcu()

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

"""Python NativeModule wrapper for the virtual_mid360 Rust binary.

virtual_mid360 replays a recorded Livox Mid-360 pcap onto a virtual network
interface, rewriting packet timestamps to current-time and synthesizing the
Livox SDK2 control protocol so an unmodified consumer (e.g. PointLio) connects
to it as if it were a real sensor. It carries no dimos streams; it speaks the
Livox wire protocol over UDP, so consumers reach it by host_ip/lidar_ip, not by
stream wiring.

Usage::

    from dimos.hardware.sensors.lidar.livox.virtual_mid360.module import VirtualMid360
    from dimos.hardware.sensors.lidar.pointlio.module import PointLio
    from dimos.core.coordination.blueprints import autoconnect

    autoconnect(
        VirtualMid360.blueprint(pcap="/path/to/ruwik2.pcap"),
        PointLio.blueprint(),
    )
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig


class VirtualMid360Config(NativeModuleConfig):
    cwd: str | None = "."
    executable: str = "result/bin/virtual_mid360"
    build_command: str | None = "nix build .#default"

    # pcap/lidar_ip/host_ip/lidar_netns default from DIMOS_MID360_* env vars so
    # blueprints needn't restate them. pcap/lidar_ip/host_ip are required — empty
    # makes the binary error.
    pcap: str = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_PCAP", ""))
    # Replay speed; 1.0 = original timing.
    rate: float = 1.0
    # Seconds to wait before streaming begins.
    delay: float = 0.0
    # IP the fake lidar sends from (on this netns's veth).
    lidar_ip: str = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_LIDAR_IP", ""))
    # Host IP the data is delivered to (where the SDK listens).
    host_ip: str = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_HOST_IP", ""))
    # Network namespace the fake lidar runs in.
    lidar_netns: str = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_NETNS", "lidar"))
    # Multicast group for point/IMU. 224.1.1.5 is the Livox default the SDK joins.
    mcast_data: str = "224.1.1.5"


class VirtualMid360(NativeModule):
    config: VirtualMid360Config


# Verify the module constructs (mirrors the pointlio wrapper's check).
if TYPE_CHECKING:
    VirtualMid360()

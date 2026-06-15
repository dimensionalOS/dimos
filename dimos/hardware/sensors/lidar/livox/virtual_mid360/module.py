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

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig


class VirtualMid360Config(NativeModuleConfig):
    cwd: str | None = "."
    executable: str = "result/bin/virtual_mid360"
    build_command: str | None = "nix build .#default"

    # Recorded Mid-360 pcap (point/IMU/status UDP). Read fully into RAM.
    pcap: str = ""
    # Replay-speed multiplier; 1.0 = original inter-packet timing.
    rate: float = 1.0
    # Seconds to wait after start before streaming begins.
    delay: float = 0.0
    # IP the fake lidar sends from (must be on this netns's veth).
    lidar_ip: str = "192.168.1.155"
    # Host IP the recorded data is delivered to (where the SDK listens).
    host_ip: str = "192.168.1.5"
    # Network namespace the fake lidar runs inside.
    lidar_netns: str = "lidar"
    # Multicast group the point/IMU streams are sent to. 224.1.1.5 is the Livox
    # default that the SDK joins; override only to match a consumer configured
    # with a different multicast_ip.
    mcast_data: str = "224.1.1.5"


class VirtualMid360(NativeModule):
    config: VirtualMid360Config


# Verify the module constructs (mirrors the pointlio wrapper's check).
if TYPE_CHECKING:
    VirtualMid360()

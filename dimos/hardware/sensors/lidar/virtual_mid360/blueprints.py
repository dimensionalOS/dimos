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

"""Demos: a SLAM consumer fed by a VirtualMid360 replaying a pcap (live SDK path).

Each module reads its own config from env vars (DIMOS_MID360_* for the sensor,
DIMOS_FASTLIO_* / DIMOS_POINTLIO_* for the consumer); set the lidar/host IPs so
the two ends agree.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.virtual_mid360.module import VirtualMid360
from dimos.visualization.vis_module import vis_module

demo_virtual_mid360_fastlio = autoconnect(
    VirtualMid360.blueprint(),
    FastLio2.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="virtual_mid360_fastlio")

demo_virtual_mid360_pointlio = autoconnect(
    VirtualMid360.blueprint(),
    PointLio.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="virtual_mid360_pointlio")

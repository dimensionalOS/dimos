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

"""Blueprint: PointLio fed by a VirtualMid360 replaying a recorded pcap.

VirtualMid360 stands up a fake Mid-360 on a virtual NIC and replays the pcap
over the Livox wire protocol; PointLio connects to it exactly as it would to
real hardware (no replay_pcap — it runs in live SDK mode and never knows the
sensor is synthetic). Use this to re-run a recorded session through the live
SLAM path, e.g. to confirm a clip does not diverge.

The two talk over UDP on lidar_ip/host_ip, so they need a network where those
IPs are reachable (the e2e harness runs VirtualMid360 in a `lidar` netns and
PointLio in a `drv` netns joined by a veth carrying lidar_ip).
"""

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.livox.virtual_mid360.module import VirtualMid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.visualization.vis_module import vis_module

# Point this at a recorded Mid-360 capture via the env var, e.g. the ruwik2_part3
# LFS sample:
#   VIRTUAL_MID360_PCAP="$(python -c 'from dimos.utils.data import get_data; \
#       print(get_data("ruwik2_part3/ruwik2_part3.pcap"))')" dimos run ...
# Read here (not get_data at import) so registering the blueprint never triggers
# an LFS pull.
_PCAP = os.environ.get("VIRTUAL_MID360_PCAP", "")

demo_virtual_mid360_pointlio = autoconnect(
    VirtualMid360.blueprint(pcap=_PCAP),
    PointLio.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="virtual_mid360_pointlio")

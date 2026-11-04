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

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.robosense.module import RoboSenseLidar
from dimos.visualization.vis_module import vis_module

# A RoboSense lidar on its factory-default ports; set model/lidar_ip/multicast_ip to match yours.
robosense_lidar = autoconnect(
    RoboSenseLidar.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=2, robot_model="robosense_lidar")

# Replays the capture named by DIMOS_ROBOSENSE_PCAP (required) through the
# same driver pipeline as the live sensor.
demo_robosense_pcap_replay = autoconnect(
    RoboSenseLidar.blueprint(pcap=os.environ.get("DIMOS_ROBOSENSE_PCAP", "")),
    vis_module("rerun"),
).global_config(n_workers=2, robot_model="robosense_pcap_replay")

#!/usr/bin/env python3

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

"""unitree-go2-multi plus one keyboard teleop window per robot.

Each robot's namespace holds its own KeyboardTeleop, so its cmd_vel connects
namespace-locally: the teleop window you focus is the robot you drive.

Usage:
    ROBOT_IPS=10.0.0.102,10.0.0.209 dimos run unitree-go2-multi-teleop

    # Simulation: one MuJoCo sim per robot; the IPs are ignored
    ROBOT_IPS=10.0.0.102,10.0.0.209 dimos --simulation run unitree-go2-multi-teleop
"""

from dimos.core.coordination.blueprints import autoconnect, namespace
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_ips = global_config.processed_robot_ips

unitree_go2_multi_teleop = autoconnect(
    *[
        namespace(
            f"robot{i}",
            GO2Connection.blueprint(ip=ip),
            KeyboardTeleop.blueprint(),
        )
        for i, ip in enumerate(_ips)
    ],
).global_config(n_workers=max(2, 2 * len(_ips)), robot_model="unitree_go2")

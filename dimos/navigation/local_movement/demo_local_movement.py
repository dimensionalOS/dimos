#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

from dimos.core.blueprints import autoconnect
from dimos.core.transport import ROSTransport
from dimos.msgs.geometry_msgs import Pose, TwistStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.navigation.local_movement import local_movement

demo_local_movement = autoconnect(
    local_movement(),
).transports(
    {
        ("odom", Odometry): ROSTransport("/state_estimation", Odometry),
        ("move_command", Pose): ROSTransport("/local_movement", Pose),
        ("terrain", PointCloud2): ROSTransport("/terrain_map", PointCloud2),
        ("cmd_vel", TwistStamped): ROSTransport("/cmd_vel", TwistStamped),
        ("status", Int8): ROSTransport("/local_movement/status", Int8),
    }
)

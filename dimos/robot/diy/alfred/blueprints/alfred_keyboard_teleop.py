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

"""Alfred teleop with the Mid-360 (Point-LIO) and the mast D455, no navigation.

Anything publishing tele_cmd_vel (web_ctrl's keyboard, dimos topic send) drives
the base through MovementManager's teleop/nav mux.

    dimos run alfred-keyboard-teleop
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.blueprints.alfred_hardware import _alfred_hardware
from dimos.robot.diy.alfred.config import ALFRED

alfred_keyboard_teleop = autoconnect(
    _alfred_hardware,
    # No raw Mid360 module: the lidar streams to a single host endpoint, and the
    # Point-LIO native owns it, so a second Livox SDK connection gets no data.
    PointLio.blueprint(lidar_ip=ALFRED.mid360_ip).remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    MovementManager.blueprint(),
).global_config(n_workers=7, robot_model="alfred")

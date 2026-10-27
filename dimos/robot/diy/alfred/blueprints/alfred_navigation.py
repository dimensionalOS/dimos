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

"""Alfred on the move: Mid-360 navigation and teleop on the coordinator, the lift parked, no arms.

    dimos run alfred-navigation

The other half of the Orin NX's pair (``alfred-manipulation`` is the bench half): the lidar,
Point-LIO, the voxel map, both planners and the holonomic follower, with the Controller's
"Go here" and teleop. The pillar is connected so it can be homed and parked, but there is no
planner and the arms' CAN is left alone, so the box has its cores for navigation.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.diy.alfred.blueprints.alfred_full import (
    alfred_base_transports,
    alfred_coordinator,
    alfred_nav_modules,
    alfred_urdf_manager,
)
from dimos.robot.diy.alfred.blueprints.alfred_hardware import _alfred_lidar_hardware
from dimos.robot.diy.alfred.pillar_connection import PillarConnection

alfred_navigation = (
    autoconnect(
        _alfred_lidar_hardware,
        *alfred_nav_modules(),
        # No Rerun bridge (see alfred_manipulation): the Desktop reads zenoh itself.
        PillarConnection.blueprint(),
        alfred_urdf_manager(),
        alfred_coordinator(arms=False, follower=True),
    )
    .transports(alfred_base_transports())
    .global_config(n_workers=10, robot_model="alfred")
)

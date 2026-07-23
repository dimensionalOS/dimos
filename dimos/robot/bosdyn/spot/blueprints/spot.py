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

"""Boston Dynamics Spot: click/teleop driving plus full sensor streaming + Rerun.

The Rerun web UI is the driver: `RerunWebSocketServer` turns clicks into
`clicked_point` and browser keys into `tele_cmd_vel`. `MovementManager` muxes
those (and any `nav_cmd_vel`) into a single `cmd_vel`, which `SpotHighLevel`
executes. The same module streams the five fisheye + five depth cameras and body
odometry. Because the browser is the input surface there is no on-main-thread
pygame window, so this runs on macOS.

The ip auto-detects: with no `-o spothighlevel.ip=` given, it probes Spot's WiFi
AP address (192.168.80.3) then the Ethernet address (10.0.0.3) and uses
whichever answers.

Usage:
    dimos run spot \
        -o spothighlevel.username=admin -o spothighlevel.password=<password>
    # or force an address:
    dimos run spot ... -o spothighlevel.ip=10.0.0.3
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.bosdyn.spot.effectors.high_level import SpotHighLevel
from dimos.robot.bosdyn.spot.rerun import spot_camera_layout
from dimos.visualization.vis_module import vis_module

spot = autoconnect(
    SpotHighLevel.blueprint(),
    MovementManager.blueprint(),
    vis_module(global_config.viewer, rerun_config={"blueprint": spot_camera_layout}),
).remappings(
    [
        # No nav stack here, so MovementManager's goal/way_point/stop_movement
        # outputs have no consumer — park them so autoconnect stays quiet.
        (MovementManager, "goal", "_spot_goal_unused"),
        (MovementManager, "way_point", "_spot_way_point_unused"),
        (MovementManager, "stop_movement", "_spot_stop_movement_unused"),
    ]
)

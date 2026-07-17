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

"""Spot: drive it from the Rerun web UI while recording every data stream.

The same click/teleop + camera stack as the default `spot` blueprint —
`RerunWebSocketServer` turns browser clicks/keys into `clicked_point` /
`tele_cmd_vel`, `MovementManager` muxes them into `cmd_vel`, and `SpotHighLevel`
executes it while streaming the five fisheye + five depth cameras and odometry —
with `SpotRecorder` added so every one of those streams (plus the live tf tree)
is written to a memory2 SQLite db as you drive. `autoconnect` wires the
recorder's In ports to `SpotHighLevel`'s outputs by name.

Usage:
    dimos run spot-record \
        -o spothighlevel.username=admin -o spothighlevel.password=<password>
    # choose where the recording lands:
    dimos run spot-record ... -o spotrecorder.db_path=/path/to/spot.db
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.bosdyn.spot.blueprints.spot_cameras import spot_camera_layout
from dimos.robot.bosdyn.spot.effectors.high_level import SpotHighLevel
from dimos.robot.bosdyn.spot.recorder import SpotRecorder
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

spot_record = autoconnect(
    SpotHighLevel.blueprint(),
    MovementManager.blueprint(),
    SpotRecorder.blueprint(),
    RerunBridgeModule.blueprint(blueprint=spot_camera_layout),
    RerunWebSocketServer.blueprint(),
).remappings(
    [
        # No nav stack here, so MovementManager's goal/way_point/stop_movement
        # outputs have no consumer — park them so autoconnect stays quiet.
        (MovementManager, "goal", "_spot_goal_unused"),
        (MovementManager, "way_point", "_spot_way_point_unused"),
        (MovementManager, "stop_movement", "_spot_stop_movement_unused"),
    ]
)

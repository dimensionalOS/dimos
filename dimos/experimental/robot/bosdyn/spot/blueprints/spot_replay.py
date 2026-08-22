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

"""Replay the latest Spot recording to Rerun — no robot, runs anywhere.

`SpotReplay` plays a memory2 recording (from `spot-record`) back onto the same
stream names `SpotHighLevel` uses, so the Spot camera layout and per-camera
frustums light up in the Rerun 3D view. The `visual_override` routes the two
shared CameraInfo streams onto each camera's image entity so every frustum
anchors to its optical frame.

Usage:
    # the shared recording, pulled from LFS on first run:
    dimos run spot-replay --db-path=spot_small_loop.db
    # newest *.db under ~/datasets/spot:
    dimos run spot-replay
    # a specific recording:
    dimos run spot-replay --db-path=/path/to/spot.db
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.experimental.robot.bosdyn.spot.replay import SpotReplay
from dimos.experimental.robot.bosdyn.spot.rerun import (
    spot_body_static_overrides,
    spot_camera_layout,
    spot_camera_visual_overrides,
)
from dimos.mapping.odometry_path import OdometryPath
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

# Compose only the Rerun bridge (+ its websocket server) directly instead of the
# shared `vis_module`: replay just needs the 3D viewer, and `vis_module` also
# bundles the WebsocketVisModule, which auto-opens the 7779 Command Center tab.
spot_replay = autoconnect(
    SpotReplay.blueprint(),
    OdometryPath.blueprint(),
    RerunBridgeModule.blueprint(
        pubsubs=[LCM()],
        rerun_open=global_config.rerun_open,
        rerun_web=global_config.rerun_web,
        blueprint=spot_camera_layout,
        visual_override=spot_camera_visual_overrides(),
        static=spot_body_static_overrides(),
    ),
    RerunWebSocketServer.blueprint(),
).remappings([(OdometryPath, "path", "odom_path")])

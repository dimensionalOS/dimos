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

"""Manual-map → on-demand YOLO seat-find demo (no LLM).

Operator flow:
  1. Launch this blueprint. Rerun opens with the live camera + global map.
  2. Drive the Go2 manually — click-to-goal on the map in Rerun, or use
     keyboard teleop — to explore and build the voxel map.
  3. From another terminal, trigger detection on demand:
         dimos mcp call find_empty_seat_now
     SeatPlanner picks an empty seat in the current view, projects it to 3D,
     and publishes goal_request. A* draws the path and (if MovementManager is
     enabled) the robot walks there.

McpServer exposes the @skill over HTTP; McpClient (LLM agent) is not included.
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.seat_planner import SeatPlanner
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.connection import GO2Connection

unitree_go2_seat_demo = (
    autoconnect(
        unitree_go2,
        McpServer.blueprint(),
        WebInput.blueprint(),
        SeatPlanner.blueprint(camera_info=GO2Connection.camera_info_static),
    )
    .remappings(
        [
            (SeatPlanner, "pointcloud", "global_map"),
        ]
    )
    .transports(
        {
            ("detections_image", SeatPlanner): LCMTransport(
                "/seatplanner/detections", Image
            ),
        }
    )
    .global_config(n_workers=6, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_seat_demo"]

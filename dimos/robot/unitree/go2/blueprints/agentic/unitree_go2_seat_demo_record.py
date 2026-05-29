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

"""Seat-demo + recording: makes a reusable map while you map manually.

Same skills as `unitree-go2-seat-demo` but adds Go2Memory recording on top
of the plain smart stack — *without* MarkerTfModule so we don't get TF spam
from missing AprilTags. LiDAR/odom/color get written to `recording_go2.db`
for later premap export.

Operator flow (paired with `unitree-go2-seat-demo-reuse`):
  1. dimos run unitree-go2-seat-demo-record
     → manually click-to-goal in Rerun and walk the robot over the area.
  2. Ctrl+C to stop (recording is flushed to recording_go2.db in the cwd).
  3. dimos export-premap recording_go2
     → produces data/recording_go2_twopass_map.pc2.lcm
  4. Next session: use `unitree-go2-seat-demo-reuse` with that premap.
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.seat_planner import SeatPlanner
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import Go2Memory, unitree_go2
from dimos.robot.unitree.go2.connection import GO2Connection

unitree_go2_seat_demo_record = (
    autoconnect(
        unitree_go2,
        Go2Memory.blueprint(),
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
    .global_config(n_workers=8, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_seat_demo_record"]

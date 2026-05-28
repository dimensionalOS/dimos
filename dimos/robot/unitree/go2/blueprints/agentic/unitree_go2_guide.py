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

"""Slim "guide dog" blueprint: navigate a leashed person to an empty seat.

Trimmed from unitree-go2-agentic to keep GPU/compute light for guiding. Drops
the heavy modules that are unused here: SecurityModule (EdgeTAM, eager CUDA
load), SpatialMemory (CLIP), PerceiveLoopSkill and PersonFollowSkill (we lead,
not follow). Keeps the base nav stack, the agent, voice I/O and SeatFinder.

SeatFinder runs its own continuous, sharpness-filtered YOLO stream and publishes
an annotated frame on ``/seatfinder/detections`` so the viewer can show it.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.seat_finder import SeatFinderSkill
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer

unitree_go2_guide = (
    autoconnect(
        unitree_go2,
        McpServer.blueprint(),
        McpClient.blueprint(),
        SeatFinderSkill.blueprint(camera_info=GO2Connection.camera_info_static),
        UnitreeSkillContainer.blueprint(),
        WebInput.blueprint(),
        SpeakSkill.blueprint(),
    )
    .remappings(
        [
            # 3D projection needs a world-frame cloud; use the VoxelGrid map
            # (the raw GO2 /pointcloud is not populated here), like Detection3D.
            (SeatFinderSkill, "pointcloud", "global_map"),
        ]
    )
    .transports(
        {
            ("detections_image", SeatFinderSkill): LCMTransport(
                "/seatfinder/detections", Image
            ),
        }
    )
    .global_config(n_workers=8, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_guide"]

#!/usr/bin/env python3
"""Seat-demo on top of a previously-recorded premap.

Same skills as `unitree-go2-seat-demo` but layered on `unitree_go2_relocalization`
so a saved premap is loaded and the live scan gets ICP-aligned to it. Once
relocalize succeeds you can navigate by map-frame coordinates without
re-walking the room.

Operator flow:
  1. dimos run unitree-go2-seat-demo-reuse \\
        -o relocalizationmodule.map_file=<basename_without_suffix> \\
        -o relocalizationmodule.publish_loaded_map=true
  2. Wait for `relocalize: fitness=... TF 'world' -> 'map' published` in the log.
  3. Click-to-goal in Rerun / dimos mcp call navigate_to_point / voice
     "椅子まで行って" — same as the no-map demo.
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.seat_planner import SeatPlanner
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2_relocalization
from dimos.robot.unitree.go2.connection import GO2Connection

unitree_go2_seat_demo_reuse = (
    autoconnect(
        unitree_go2_relocalization,
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

__all__ = ["unitree_go2_seat_demo_reuse"]

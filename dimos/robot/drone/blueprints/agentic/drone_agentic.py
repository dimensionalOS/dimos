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

"""Agentic drone blueprint with full autonomous capabilities."""

from dimos.agents.agent import agent
from dimos.agents.skills.google_maps_skill_container import GoogleMapsSkillContainer
from dimos.agents.skills.osm import OsmSkill
from dimos.agents.web_human_input import web_input
from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.service.system_configurator import ClockSyncConfigurator
from dimos.robot.drone.camera_module import DroneCameraModule
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.robot.drone.drone_tracking_module import DroneTrackingModule
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

DRONE_SYSTEM_PROMPT = """\
You are controlling a DJI drone with MAVLink interface.
You have access to drone control skills you are already flying so only run move_twist, set_mode, and fly_to.
When the user gives commands, use the appropriate skills to control the drone.
Always confirm actions and report results. Send fly_to commands only at above 200 meters altitude to be safe.
Here are some GPS locations to remember
6th and Natoma intersection: 37.78019978319006, -122.40770815020853,
454 Natoma (Office): 37.780967465525244, -122.40688342010769
5th and mission intersection: 37.782598539339695, -122.40649441875473
6th and mission intersection: 37.781007204789354, -122.40868447123661"""


def _static_drone_body(rr):  # type: ignore
    """Static visualization of drone body."""
    return [
        rr.Boxes3D(
            half_sizes=[0.25, 0.25, 0.1],
            colors=[(255, 100, 0)],
            fill_mode="wireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


rerun_config = {
    "pubsubs": [LCM(autoconf=True)],
    "static": {
        "world/tf/base_link": _static_drone_body,
    },
}

# Build blueprint with conditional visualization
_transports_base = autoconnect()

if global_config.viewer == "foxglove":
    from dimos.robot.foxglove_bridge import foxglove_bridge

    with_vis = autoconnect(
        _transports_base,
        foxglove_bridge(),
    )
elif global_config.viewer.startswith("rerun"):
    from dimos.visualization.rerun.bridge import _resolve_viewer_mode, rerun_bridge

    with_vis = autoconnect(
        _transports_base,
        rerun_bridge(viewer_mode=_resolve_viewer_mode(), **rerun_config),
    )
else:
    with_vis = _transports_base


# Determine connection string and outdoor mode based on config
connection_string = "udp:0.0.0.0:14550"
video_port = 5600
outdoor = False

if global_config.replay:
    connection_string = "replay"

drone_agentic = (
    autoconnect(
        with_vis,
        DroneConnectionModule.blueprint(
            connection_string=connection_string,
            video_port=video_port,
            outdoor=outdoor,
        ),
        DroneCameraModule.blueprint(camera_intrinsics=[1000.0, 1000.0, 960.0, 540.0]),
        DroneTrackingModule.blueprint(outdoor=outdoor),
        websocket_vis(),
        GoogleMapsSkillContainer.blueprint(),
        OsmSkill.blueprint(),
        agent(system_prompt=DRONE_SYSTEM_PROMPT, model="gpt-4o"),
        web_input(),
    )
    .remappings(
        [
            (DroneTrackingModule, "video_input", "video"),
            (DroneTrackingModule, "cmd_vel", "movecmd_twist"),
        ]
    )
    .global_config(n_workers=4, robot_model="drone")
    .configurators(ClockSyncConfigurator())
)


__all__ = [
    "drone_agentic",
]

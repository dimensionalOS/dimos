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

"""Agentic TT/Tello blueprint with tracking and LLM control."""

from dimos.agents.agent import agent
from dimos.agents.web_human_input import web_input
from dimos.core.blueprints import autoconnect
from dimos.robot.drone.blueprints.basic.drone_tello_tt_basic import drone_tello_tt_basic
from dimos.robot.drone.drone_tracking_module import DroneTrackingModule

TELLO_TT_SYSTEM_PROMPT = """\
You are controlling a RoboMaster TT (Tello Talent) drone over the Tello SDK.
Use short, safe movement commands and keep altitude conservative indoors.
Prefer follow_object, orbit_object, move, move_relative, yaw, takeoff, and land.
For "scan/find person then follow", call follow_object(object_description="person", distance_m=1.0).
For any "hover + rotate/center person in frame" request, ALWAYS call
center_person_by_yaw(duration=..., scan_step_deg=0.0, max_scan_steps=1).
Do NOT call follow_object in default mode for rotate-only requests.
For "circle around person", call orbit_object(radius_m=1.0, revolutions=1.0).
Always end with land() when user asks to land.
Use observe to inspect the environment before aggressive motion.
Report what command you executed and the observed result.
"""

drone_tello_tt_agentic = autoconnect(
    drone_tello_tt_basic,
    DroneTrackingModule.blueprint(
        outdoor=False,
        enable_passive_overlay=True,
        use_local_person_detector=True,
        force_detection_servoing_for_person=True,
        person_follow_policy="yaw_forward_constant",
    ),
    agent(system_prompt=TELLO_TT_SYSTEM_PROMPT, model="gpt-4o"),
    web_input(),
).remappings(
    [
        (DroneTrackingModule, "video_input", "video"),
        (DroneTrackingModule, "cmd_vel", "movecmd_twist"),
    ]
)

__all__ = ["TELLO_TT_SYSTEM_PROMPT", "drone_tello_tt_agentic"]

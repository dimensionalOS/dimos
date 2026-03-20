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

"""Agentic G1 onboard stack: ROSNav + perception + LLM agent with full skill set.

G1HighLevelDdsSdk exposes @skill methods (move_velocity, execute_arm_command,
execute_mode_command) directly, so the agent discovers them automatically
without a separate skill container.

Requires ``unitree_sdk2py`` to be installed on the robot for the DDS SDK.
"""

from dimos.agents.agent import agent
from dimos.agents.skills.navigation import navigation_skill
from dimos.agents.skills.person_follow import person_follow_skill
from dimos.agents.skills.speak_skill import speak_skill
from dimos.agents.web_human_input import web_input
from dimos.core.blueprints import autoconnect
from dimos.perception.object_tracker import object_tracking
from dimos.perception.perceive_loop_skill import PerceiveLoopSkill
from dimos.perception.spatial_perception import spatial_memory
from dimos.robot.unitree.g1.blueprints.perceptive.unitree_g1_rosnav_onboard import (
    unitree_g1_rosnav_onboard,
)
from dimos.robot.unitree.go2.connection import _camera_info_static

unitree_g1_agentic_onboard = autoconnect(
    unitree_g1_rosnav_onboard,
    agent(),
    navigation_skill(),
    person_follow_skill(camera_info=_camera_info_static()),
    spatial_memory(),
    object_tracking(frame_id="camera_link"),
    PerceiveLoopSkill.blueprint(),
    web_input(),
    speak_skill(),
).global_config(n_workers=8)

__all__ = ["unitree_g1_agentic_onboard"]

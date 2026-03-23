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

"""Agentic G1 ROSNav Unity sim stack: perception + LLM agent with full skill set.

Builds on the ROSNav simulation base and adds spatial memory, object tracking,
perceive-loop, LLM agent, navigation skills, person following, voice output,
and a web UI for human input.
"""

from dimos.agents.agent import Agent
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.blueprints import autoconnect
from dimos.perception.object_tracker import ObjectTracking
from dimos.perception.perceive_loop_skill import PerceiveLoopSkill
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.g1.blueprints.perceptive.unitree_g1_rosnav_sim import (
    unitree_g1_rosnav_sim,
)
from dimos.robot.unitree.go2.connection import _camera_info_static

unitree_g1_agentic_sim = autoconnect(
    unitree_g1_rosnav_sim,
    Agent.blueprint(),
    NavigationSkillContainer.blueprint(),
    PersonFollowSkillContainer.blueprint(camera_info=_camera_info_static()),
    SpatialMemory.blueprint(),
    ObjectTracking.blueprint(frame_id="camera_link"),
    PerceiveLoopSkill.blueprint(),
    WebInput.blueprint(),
    SpeakSkill.blueprint(),
).global_config(n_workers=8)

__all__ = ["unitree_g1_agentic_sim"]

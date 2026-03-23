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

"""Agentic G1 MuJoCo stack: MuJoCo simulation + perception + LLM agent with skills.

This is the new-architecture equivalent of the legacy ``unitree_g1_agentic_sim``
blueprint, using the MuJoCo simulator instead of ROSNav/Unity for simulation.
"""

from dimos.agents.agent import Agent
from dimos.core.blueprints import autoconnect
from dimos.perception.object_tracker import ObjectTracking
from dimos.perception.perceive_loop_skill import PerceiveLoopSkill
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.g1.blueprints.agentic._mujoco_skills import _mujoco_agentic_skills
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_mujoco import unitree_g1_mujoco

unitree_g1_agentic_mujoco = autoconnect(
    unitree_g1_mujoco,
    SpatialMemory.blueprint(),
    ObjectTracking.blueprint(frame_id="camera_link"),
    PerceiveLoopSkill.blueprint(),
    Agent.blueprint(),
    _mujoco_agentic_skills,
).global_config(n_workers=8)

__all__ = ["unitree_g1_agentic_mujoco"]

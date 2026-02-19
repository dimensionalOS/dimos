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

"""Agentic blueprint for Deep Robotics M20: smart + LLM agent + skills.

Full autonomous M20 with natural language control:
    Smart (connection + SLAM + nav) + M20SkillContainer + Agent
"""

from dimos.agents.agent import agent
from dimos.core.blueprints import autoconnect
from dimos.robot.deeprobotics.m20.blueprints.smart.m20_smart import m20_smart
from dimos.robot.deeprobotics.m20.skill_container import m20_skills

m20_agentic = autoconnect(
    m20_smart,
    m20_skills(),
    agent(),
).global_config(
    n_dask_workers=8,
    robot_model="deeprobotics_m20",
)

__all__ = ["m20_agentic"]

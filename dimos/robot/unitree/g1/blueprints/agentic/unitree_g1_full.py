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

"""Full featured G1 stack with agentic skills and teleop."""

from dimos.core.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.agentic._agentic_skills import _agentic_skills
from dimos.robot.unitree.g1.blueprints.perceptive.unitree_g1_shm import unitree_g1_shm
from dimos.robot.unitree.keyboard_teleop import keyboard_teleop

unitree_g1_full = autoconnect(
    unitree_g1_shm,
    _agentic_skills,
    keyboard_teleop(),
)

__all__ = ["unitree_g1_full"]

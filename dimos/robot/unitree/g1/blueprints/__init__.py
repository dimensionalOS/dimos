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

"""Cascaded G1 blueprints split into focused modules."""

import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "agentic._agentic_skills": ["_agentic_skills"],
        "agentic.unitree_g1_agentic": ["unitree_g1_agentic"],
        "agentic.unitree_g1_agentic_sim": ["unitree_g1_agentic_sim"],
        "agentic.unitree_g1_full": ["unitree_g1_full"],
        "basic.unitree_g1_basic": ["unitree_g1_basic"],
        "basic.unitree_g1_basic_sim": ["unitree_g1_basic_sim"],
        "basic.unitree_g1_joystick": ["unitree_g1_joystick"],
        "perceptive._perception_and_memory": ["_perception_and_memory"],
        "perceptive.unitree_g1": ["unitree_g1"],
        "perceptive.unitree_g1_detection": ["unitree_g1_detection"],
        "perceptive.unitree_g1_shm": ["unitree_g1_shm"],
        "perceptive.unitree_g1_sim": ["unitree_g1_sim"],
        "primitive.uintree_g1_primitive_no_nav": ["uintree_g1_primitive_no_nav", "basic_no_nav"],
    },
)

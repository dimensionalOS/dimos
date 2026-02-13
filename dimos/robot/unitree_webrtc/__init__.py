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

"""Compatibility package for legacy dimos.robot.unitree_webrtc imports."""

from importlib import import_module
import sys

_ALIAS_MODULES = {
    "demo_error_on_name_conflicts": "dimos.robot.unitree.demo_error_on_name_conflicts",
    "depth_module": "dimos.robot.unitree.depth_module",
    "keyboard_teleop": "dimos.robot.unitree.keyboard_teleop",
    "mujoco_connection": "dimos.robot.unitree.mujoco_connection",
    "type": "dimos.robot.unitree.type",
    "unitree_g1_skill_container": "dimos.robot.unitree.g1.skill_container",
    "unitree_skill_container": "dimos.robot.unitree.unitree_skill_container",
}

for alias, target in _ALIAS_MODULES.items():
    sys.modules[f"{__name__}.{alias}"] = import_module(target)

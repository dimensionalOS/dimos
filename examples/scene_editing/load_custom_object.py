#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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

"""Create a custom object from code (no GLB needed)."""

from dimos.robot.sim.scene_client import SceneClient

with SceneClient() as scene:
    scene.add_object(
        "box",
        size=(1, 0.5, 1),
        color=0x8B4513,
        position=(3, 0.25, 2),
        name="crate",
    )
    scene.add_object(
        "sphere",
        size=(0.3,),
        color=0xFF0000,
        position=(2, 2, 2),
        name="ball",
        dynamic=True,
        mass=0.5,
        restitution=0.8,
    )
    print("Custom objects added")

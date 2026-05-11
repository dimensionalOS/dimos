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

"""Author a scene from scratch and save it.

Workflow:
    1. Start the sim with the empty scene::
           DIMSIM_SCENE=empty dimos --simulation run sim-basic

    2. In another terminal::
           python examples/scene_editing/author_warehouse.py

    3. Refresh the browser tab — the warehouse scene is now sticky.

    4. Restart the sim with --scene warehouse and the bridge will boot
       directly into the saved scene::
           dimos --simulation run sim-basic --scene warehouse
"""

from dimos.robot.sim.scene_client import SceneClient

with SceneClient() as scene:
    # Idempotent reset — wipes any previous authoring from this session.
    scene.reset()

    # Empty scene ships with no embodiment — pick one for this scene.
    scene.set_embodiment("quadruped")

    # Spawn the dog just above where the floor will be.
    scene.set_spawn_point(0, 0.5, 0)

    # Floor pad to walk on.
    scene.add_object(
        "box",
        size=(20, 0.1, 20),
        color=0x808080,
        position=(0, -0.05, 0),
        name="floor",
    )

    # A row of crates.
    for i in range(4):
        scene.add_object(
            "box",
            size=(1, 1, 1),
            color=0x8B4513,
            position=(3 + i * 1.5, 0.5, 2),
            name=f"crate-{i}",
        )

    # A bouncy ball off to the side.
    scene.add_object(
        "sphere",
        size=(0.3,),
        color=0xff4444,
        position=(-2, 2, -2),
        name="ball",
        dynamic=True,
        mass=0.5,
        restitution=0.8,
    )

    path = scene.save("warehouse")
    print(f"Saved: {path}")

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

"""Scenes as MuJoCo geometry."""

from __future__ import annotations

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.simulation.scenes.procedural import Box, Scene

SCENE_GROUP = 0
# Seen by the lidar only. The viewer switches this group off so the room stays visible.
CEILING_GROUP = 1
SCENE_GROUPS: NDArray[np.uint8] = np.isin(np.arange(6), (SCENE_GROUP, CEILING_GROUP)).astype(
    np.uint8
)


def geom_name(index: int, box: Box) -> str:
    return f"{box.kind}_{index}"


def add_boxes(spec: mujoco.MjSpec, scene: Scene) -> None:
    """Every box of the scene as a static box geom, named by geom_name."""
    for i, box in enumerate(scene.boxes):
        geom = spec.worldbody.add_geom()
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.name = geom_name(i, box)
        geom.pos = box.center
        geom.size = box.half
        if box.kind == "ceiling":
            geom.group = CEILING_GROUP


def scene_model(scene: Scene) -> mujoco.MjModel:
    """The scene alone, with nothing in it."""
    spec = mujoco.MjSpec()
    add_boxes(spec, scene)
    return spec.compile()

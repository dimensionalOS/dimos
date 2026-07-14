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

"""Go2 scene understanding + spatial memory: the most complete catalog.

Runs the ObjectSceneRegistrationModule (lidar-localized object detection ->
ObjectDB) alongside SpatialMemory (CLIP frame embeddings + named/tagged
locations) in one graph. With both present, the scene module's optional
SceneMemorySpec dependency auto-wires to SpatialMemory, so `catalog_scene`
fuses detected objects with the robot's known places and tags each object
with the nearest place.

Composes `unitree-go2-scene` directly (rather than re-declaring the scene
module) so the lidar-localization tuning lives in exactly one place — see
that blueprint for the rationale behind each parameter.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_scene import unitree_go2_scene

unitree_go2_scene_memory = autoconnect(
    unitree_go2_scene,
    SpatialMemory.blueprint(),
).global_config(n_workers=9)

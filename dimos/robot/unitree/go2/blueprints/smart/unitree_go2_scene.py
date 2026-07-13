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

"""Go2 scene understanding: identify, list, and locate all observed items.

Adds the ObjectSceneRegistrationModule in lidar-localization mode. The Go2 has
an RGB camera and a world-frame lidar but no depth camera, so objects are
located by projecting the world-frame lidar cloud into each 2D detection
(see Object.from_2d_to_list_lidar). Exposes the prompt-free
`list_observed_items` skill returning each item's world position + confidence.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

unitree_go2_scene = autoconnect(
    unitree_go2,
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        localization="lidar",
        camera_optical_frame="camera_optical",
        # Lidar localization jitters more than depth (sparse mid360 returns per
        # detection), so dedup over a wider radius than the 0.2m depth default.
        distance_threshold=0.35,
    ),
).global_config(n_workers=8)

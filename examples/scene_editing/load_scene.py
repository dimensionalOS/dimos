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

"""Load a GLB scene with physics colliders.

Usage:
    python load_scene.py /path/to/scene.glb

Try it:
    Sketchfab: https://sketchfab.com/3d-models/lowpoly-fps-tdm-game-map-by-resoforge-d41a19f699ea421a9aa32b407cb7537b
"""

import sys

from dimos.robot.sim.scene_client import SceneClient

if len(sys.argv) < 2:
    print("Usage: python load_scene.py /path/to/scene.glb")
    sys.exit(1)

glb_path = sys.argv[1]

with SceneClient() as scene:
    url = scene.upload_asset(glb_path)
    result = scene.load_map(
        url=url,
        name="environment",
        collider="trimesh",
    )
    print(f"Scene loaded: {result['name']} (scale: {result.get('scaleFactor', 1.0)})")

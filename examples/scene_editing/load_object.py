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

"""Load a GLB/GLTF object into the scene.

Usage:
    python load_object.py /path/to/model.glb

Try it:
    Avocado: https://github.com/KhronosGroup/glTF-Sample-Models/blob/main/2.0/Avocado/glTF-Binary/Avocado.glb
"""

import sys

from dimos.robot.sim.scene_client import SceneClient

if len(sys.argv) < 2:
    print("Usage: python load_object.py /path/to/model.glb")
    sys.exit(1)

glb_path = sys.argv[1]

with SceneClient() as scene:
    url = scene.upload_asset(glb_path)
    result = scene.exec(f"""
        const gltf = await loadGLTF("{url}");
        const model = gltf.scene;
        model.name = "loaded-object";
        model.position.set(2, 0.5, 2);
        model.traverse(c => {{ if (c.isMesh) {{ c.castShadow = true; c.receiveShadow = true; }} }});
        autoScale(model);
        scene.add(model);
        addCollider(model, "trimesh");
        return {{ name: model.name, uuid: model.uuid }};
    """)
    print(f"Object loaded: {result}")

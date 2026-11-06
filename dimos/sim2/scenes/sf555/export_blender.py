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

"""Offline export of the exact objects used to build the saved Unreal map.

Run with Blender --background SOURCE.blend --python THIS.py -- MANIFEST OUTPUT.
The Blender source stays read-only. Arrays use the source's right-handed metric
world frame; no glTF/Unreal axis conversion is needed here.
"""

import hashlib
import json
from pathlib import Path
import sys

import bpy  # type: ignore[import-not-found]  # Provided by Blender, not the DimOS venv.
import numpy as np


def main() -> None:
    manifest_path, destination = map(Path, sys.argv[sys.argv.index("--") + 1 :])
    manifest = json.loads(manifest_path.read_text())
    source = Path(bpy.data.filepath)
    if hashlib.sha256(source.read_bytes()).hexdigest() != manifest["source_sha256"]:
        raise ValueError("Blender source differs from the one exported into the Unreal map")
    names = sorted({name for row in manifest["objects_manifest"] for name in row["source_objects"]})
    destination.mkdir(parents=True, exist_ok=True)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    rows = []
    for index, name in enumerate(names):
        obj = bpy.data.objects[name].evaluated_get(depsgraph)
        mesh = obj.to_mesh()
        try:
            mesh.calc_loop_triangles()
            flat_vertices = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
            mesh.vertices.foreach_get("co", flat_vertices)
            vertices = flat_vertices.reshape(-1, 3)
            transform = np.asarray(obj.matrix_world)
            vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
            faces = np.empty(len(mesh.loop_triangles) * 3, dtype=np.int32)
            mesh.loop_triangles.foreach_get("vertices", faces)
            path = destination / f"{index:05d}.npz"
            np.savez_compressed(path, vertices=vertices, faces=faces.reshape(-1, 3))
            rows.append({"name": name, "file": path.name})
        finally:
            obj.to_mesh_clear()
    (destination / "objects.json").write_text(
        json.dumps({"source_sha256": manifest["source_sha256"], "objects": rows})
    )
    print(f"Exported {len(rows)} source objects in metres")


if __name__ == "__main__":
    main()

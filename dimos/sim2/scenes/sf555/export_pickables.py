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

"""Blender-only export of two bottles and the affected office mesh chunks.

Usage: Blender --background --python THIS.py -- SOURCE_OFFICE OUTPUT_DIRECTORY.
Reuses the V24 source package's material/UV exporter without editing it or the
authored blend. Only chunks containing selected parts are rebuilt. The exported
manifest records the helper and source hashes used for reproducibility.
"""

import hashlib
import json
from pathlib import Path
import runpy
import sys

import bpy  # type: ignore[import-not-found]
import numpy as np


def main() -> None:
    source, output = map(Path, sys.argv[sys.argv.index("--") + 1 :])
    source_manifest = json.loads((source / "exports/unreal-v24/scene.json").read_text())
    pickables = json.loads((output / "pickables.json").read_text())
    if source_manifest["source_sha256"] != pickables["source_sha256"]:
        raise ValueError("Physical and visual scene sources differ")
    blend = source / "models/sf555_office_v24_timber_reference.blend"
    if hashlib.sha256(blend.read_bytes()).hexdigest() != pickables["source_sha256"]:
        raise ValueError("Authored Blender source changed")
    helper_path = source / "scripts/export_unreal_v24.py"
    helper = runpy.run_path(str(helper_path))
    source_to_bottle = {p["name"]: r["id"] for r in pickables["bottles"] for p in r["parts"]}
    affected = {
        r["name"]: r
        for r in source_manifest["objects_manifest"]
        if set(r["source_objects"]) & set(source_to_bottle)
    }
    names = {n for r in affected.values() for n in r["source_objects"]}
    bpy.ops.wm.open_mainfile(filepath=str(blend), load_ui=False, use_scripts=False)
    originals = list(bpy.context.scene.objects)
    for row in json.loads((source / "exports/unreal-v24/materials/materials.json").read_text()):
        helper["ROWS"][row["name"]] = row
        helper["portable"](row)
    graph = bpy.context.evaluated_depsgraph_get()
    for name in sorted(names):
        obj = bpy.data.objects[name]
        evaluated = obj.evaluated_get(graph)
        mesh = evaluated.to_mesh()
        try:
            v, f, mi, uv, detail, normals = helper["arrays"](mesh, obj.matrix_world)
        finally:
            evaluated.to_mesh_clear()
        if name in source_to_bottle:
            helper["add_bucket"](
                "Joylo_" + source_to_bottle[name], name, v, f, mi, uv, detail, normals
            )
            continue
        category = helper["safe"](next((c.name for c in obj.users_collection), "Other"))
        centers = v[f].mean(axis=1)
        gx = np.floor((centers[:, 0] + 1) / 2).astype(int)
        gy = np.floor((centers[:, 1] + 16) / 2).astype(int)
        gz = np.floor((centers[:, 2] + 0.3) / 2).astype(int)
        rows = [helper["ROWS"][helper["MATERIALS"][i]] for i in mi]
        transparency = np.array(
            [
                1
                if r["transmission"] > 0.1 or r["alpha"] < 0.5
                else 2
                if r.get("image_alpha")
                else 0
                for r in rows
            ]
        )
        codes = gx * 10000 + gy * 100 + gz * 3 + transparency
        for code in np.unique(codes):
            key = category + "_" + str(code)
            if key not in affected:
                continue
            take = codes == code
            helper["add_bucket"](
                "Joylo_static_" + key,
                name,
                v,
                f[take],
                mi[take],
                uv[take],
                detail[take],
                normals[take],
            )
    for obj in originals:
        obj.hide_render = True
        obj.hide_set(True)
    targets = [helper["make_bucket"](key, parts) for key, parts in helper["BUCKETS"].items()]
    bpy.ops.object.select_all(action="DESELECT")
    for obj in targets:
        obj.hide_render = False
        obj.hide_set(False)
        obj.select_set(True)
    bpy.context.view_layer.objects.active = targets[0]
    bpy.ops.export_scene.gltf(
        filepath=str(output / "pickables.glb"),
        export_format="GLB",
        use_selection=True,
        export_yup=True,
        export_extras=True,
        export_lights=False,
        export_cameras=False,
        export_normals=True,
        export_texcoords=True,
        export_materials="EXPORT",
        export_animations=False,
    )
    manifest = {
        "source_sha256": pickables["source_sha256"],
        "exporter_sha256": hashlib.sha256(helper_path.read_bytes()).hexdigest(),
        "remove_actors": sorted(affected),
        "exported": [
            {
                "name": o.name,
                "position": list(o.location),
                "parts": sorted(helper["SOURCES"][o.name]),
            }
            for o in targets
        ],
    }
    (output / "visual-split.json").write_text(json.dumps(manifest, indent=2))
    print(
        json.dumps({"removed_chunks": len(affected), "exported_chunks": len(targets)}), flush=True
    )


if __name__ == "__main__":
    main()

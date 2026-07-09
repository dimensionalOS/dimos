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

"""Blender-backed visual compiler for authored scene plans."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import tempfile
from typing import Any

from dimos.experimental.scene_cooking.command import (
    blender_command_env,
    blender_output_line_is_interesting,
    run_logged_command,
)
from dimos.experimental.scene_cooking.planning import SceneCookPlan

_VISUAL_PLAN_VERSION = 2

_PLAN_VISUAL_SCRIPT = r"""
import fnmatch
import json
import math
import pathlib
import sys

import bpy
from mathutils import Matrix, Vector

source = pathlib.Path(sys.argv[-2])
plan_path = pathlib.Path(sys.argv[-1])
plan = json.loads(plan_path.read_text())
static_visual_path = pathlib.Path(plan["static_visual_path"])
suffix = source.suffix.lower()


def fail(message):
    raise RuntimeError(message)


def log(message):
    print(f"DIMOS_VISUAL_PLAN {message}", flush=True)


def import_source():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    log(f"import start source={source} suffix={suffix}")
    if suffix in {".usd", ".usda", ".usdc", ".usdz"}:
        bpy.ops.wm.usd_import(filepath=str(source))
    elif suffix in {".gltf", ".glb"}:
        bpy.ops.import_scene.gltf(filepath=str(source))
    elif suffix == ".obj":
        bpy.ops.wm.obj_import(filepath=str(source))
    elif suffix == ".stl":
        bpy.ops.wm.stl_import(filepath=str(source))
    elif suffix == ".ply":
        bpy.ops.wm.ply_import(filepath=str(source))
    else:
        fail(f"unsupported visual source suffix: {suffix}")
    log(
        "import done "
        f"objects={len(bpy.context.scene.objects)} "
        f"meshes={len(bpy.data.meshes)} images={len(bpy.data.images)}"
    )


def alignment_matrix():
    align = plan["alignment"]
    yaw, pitch, roll = [math.radians(float(v)) for v in align["rotation_zyx_deg"]]
    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)
    rz = Matrix(((cz, -sz, 0.0), (sz, cz, 0.0), (0.0, 0.0, 1.0)))
    ry = Matrix(((cy, 0.0, sy), (0.0, 1.0, 0.0), (-sy, 0.0, cy)))
    rx = Matrix(((1.0, 0.0, 0.0), (0.0, cx, -sx), (0.0, sx, cx)))
    r = rz @ ry @ rx
    if bool(align["y_up"]):
        y_to_z = Matrix(((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)))
        r = r @ y_to_z
    return r, float(align["scale"]), Vector(tuple(float(v) for v in align["translation"]))


ALIGN_R, ALIGN_SCALE, ALIGN_T = alignment_matrix()


def object_candidates(obj):
    candidates = {obj.name, obj.name_full}
    if obj.parent is not None:
        candidates.add(obj.parent.name)
        candidates.add(obj.parent.name_full)
    return candidates


def matches(obj, pattern):
    return any(fnmatch.fnmatchcase(candidate, pattern) for candidate in object_candidates(obj))


def source_point_from_blender_world(point):
    if suffix in {".gltf", ".glb"}:
        # Blender imports glTF Y-up coordinates into its Z-up world as
        # (x, -z, y).  The cook plan was resolved in the source glTF frame,
        # so convert back before applying SceneMeshAlignment.
        return Vector((point.x, point.z, -point.y))
    return point


def resolve_objects(entity):
    objects = []
    missing = []
    for pattern in entity["visual_node_patterns"]:
        matched = [obj for obj in bpy.context.scene.objects if obj.type == "MESH" and matches(obj, pattern)]
        if not matched:
            missing.append(pattern)
            continue
        for obj in matched:
            if obj not in objects:
                objects.append(obj)
    if missing:
        fail(f"entity {entity['id']} visual nodes not found in Blender import: {missing}")
    return objects


def aligned_local_point(source_world, center):
    source_point = source_point_from_blender_world(source_world)
    return (ALIGN_R @ (ALIGN_SCALE * source_point)) + ALIGN_T - center


def duplicate_for_entity(obj, center, suffix):
    dup = obj.copy()
    dup.data = obj.data.copy()
    dup.animation_data_clear()
    dup.name = f"{obj.name}__{suffix}"
    bpy.context.collection.objects.link(dup)
    for vertex in dup.data.vertices:
        source_world = obj.matrix_world @ vertex.co
        vertex.co = aligned_local_point(source_world, center)
    dup.matrix_world = Matrix.Identity(4)
    return dup


def export_entity(entity, objects):
    visual_path = pathlib.Path(entity["visual_path"])
    log(
        f"entity export start id={entity['id']} "
        f"objects={len(objects)} visual_path={visual_path}"
    )
    visual_path.parent.mkdir(parents=True, exist_ok=True)
    center = Vector(tuple(float(v) for v in entity["center"]))
    duplicates = [duplicate_for_entity(obj, center, entity["safe_id"]) for obj in objects]
    try:
        bpy.ops.object.select_all(action="DESELECT")
        for dup in duplicates:
            dup.select_set(True)
        bpy.context.view_layer.objects.active = duplicates[0]
        bpy.ops.export_scene.gltf(
            filepath=str(visual_path),
            export_format="GLB",
            use_selection=True,
            export_yup=False,
            export_apply=True,
        )
    finally:
        for dup in duplicates:
            bpy.data.objects.remove(dup, do_unlink=True)
    log(f"entity export done id={entity['id']}")


def export_static_visual(objects_to_remove):
    log(f"static visual remove start objects={len(objects_to_remove)}")
    for obj in sorted(objects_to_remove, key=lambda item: item.name):
        if obj.name in bpy.data.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
    remaining = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not remaining:
        fail("static visual would contain no mesh objects after entity removal")
    static_visual_path.parent.mkdir(parents=True, exist_ok=True)
    log(f"static visual export start remaining_meshes={len(remaining)} path={static_visual_path}")
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.export_scene.gltf(
        filepath=str(static_visual_path),
        export_format="GLB",
        export_yup=True,
        export_apply=True,
    )
    log("static visual export done")


import_source()
remove_from_static = set()
report = []
entities = plan["entities"]
log(f"entity plan start count={len(entities)}")
for index, entity in enumerate(entities, start=1):
    log(f"entity plan progress index={index}/{len(entities)} id={entity['id']}")
    objects = resolve_objects(entity)
    export_entity(entity, objects)
    if entity["remove_from_static"]:
        remove_from_static.update(objects)
    report.append(
        {
            "id": entity["id"],
            "visual_nodes": [obj.name for obj in objects],
            "removed_from_static": entity["remove_from_static"],
        }
    )
export_static_visual(remove_from_static)
print("DIMOS_VISUAL_PLAN_REPORT=" + json.dumps(report, sort_keys=True))
"""


def cook_plan_visual_assets(
    source_path: str | Path,
    output_dir: str | Path,
    *,
    plan: SceneCookPlan,
    rebake: bool = False,
) -> Path:
    """Extract entity visuals and write a filtered static visual source GLB."""
    source = Path(source_path).expanduser().resolve()
    package_dir = Path(output_dir).expanduser().resolve()
    static_visual_source = package_dir / "browser" / "static_visual_source.glb"
    plan_manifest = package_dir / "browser" / "visual_plan.json"
    plan_json = _blender_plan_json(plan, static_visual_source)
    expected_paths = [
        static_visual_source,
        *(entity.visual_path for entity in plan.entities if entity.visual_path is not None),
    ]
    if (
        expected_paths
        and all(path.exists() for path in expected_paths)
        and _manifest_matches(plan_manifest, plan_json)
        and not rebake
    ):
        return static_visual_source

    blender = shutil.which("blender")
    if blender is None:
        raise RuntimeError("authored visual entity extraction requires Blender on PATH")

    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as plan_file:
        json.dump(plan_json, plan_file)
        plan_path = Path(plan_file.name)
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as script_file:
        script_file.write(_PLAN_VISUAL_SCRIPT)
        script_path = Path(script_file.name)
    try:
        run_logged_command(
            [
                blender,
                "--background",
                "--factory-startup",
                "--python",
                str(script_path),
                "--",
                str(source),
                str(plan_path),
            ],
            "blender visual plan cook",
            line_log_filter=blender_output_line_is_interesting,
            env=blender_command_env(),
        )
    finally:
        plan_path.unlink(missing_ok=True)
        script_path.unlink(missing_ok=True)

    plan_manifest.parent.mkdir(parents=True, exist_ok=True)
    plan_manifest.write_text(json.dumps(plan_json, indent=2, sort_keys=True) + "\n")
    return static_visual_source


def _blender_plan_json(plan: SceneCookPlan, static_visual_source: Path) -> dict[str, Any]:
    return {
        "visual_plan_version": _VISUAL_PLAN_VERSION,
        "alignment": {
            "scale": plan.alignment.scale,
            "rotation_zyx_deg": list(plan.alignment.rotation_zyx_deg),
            "translation": list(plan.alignment.translation),
            "y_up": plan.alignment.y_up,
        },
        "static_visual_path": str(static_visual_source),
        "entities": [
            {
                "id": entity.spec.id,
                "safe_id": entity.safe_id,
                "visual_node_patterns": list(entity.visual_node_patterns),
                "center": list(entity.center),
                "visual_path": str(entity.visual_path),
                "remove_from_static": entity.spec.remove_from_static,
            }
            # Synthetic entities have no source mesh to extract; skip them
            # entirely so Blender doesn't try to match an empty pattern set.
            for entity in plan.entities
            if entity.visual_path is not None
        ],
    }


def _manifest_matches(path: Path, expected: dict[str, Any]) -> bool:
    if not path.exists():
        return False
    try:
        return bool(json.loads(path.read_text()) == expected)
    except json.JSONDecodeError:
        return False

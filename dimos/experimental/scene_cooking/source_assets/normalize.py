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

"""Normalize authored scene sources into concrete mesh assets for cooking."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import hashlib
from pathlib import Path
import shutil
import tempfile
from typing import Any

from dimos.constants import CACHE_DIR
from dimos.experimental.scene_cooking.command import (
    blender_command_env,
    blender_output_line_is_interesting,
    run_logged_command,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DIRECT_SCENE_SUFFIXES = {
    ".glb",
    ".gltf",
    ".obj",
    ".ply",
    ".stl",
    ".usd",
    ".usda",
    ".usdc",
    ".usdz",
}

SOURCE_CACHE_DIR = CACHE_DIR / "scene_sources"
_BLENDER_NORMALIZER_VERSION = "blend-evaluated-depsgraph-v1"


@dataclass(frozen=True)
class PreparedSceneSource:
    """A source asset in a format downstream cookers can consume."""

    original_path: Path
    cook_path: Path
    normalized: bool = False
    normalizer: str | None = None
    stats: dict[str, Any] = field(default_factory=dict)

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "original_path": str(self.original_path),
            "cook_path": str(self.cook_path),
            "normalized": self.normalized,
            "normalizer": self.normalizer,
            "stats": self.stats,
        }


Normalizer = Callable[[Path, Path, bool], PreparedSceneSource]


def prepare_scene_source(
    source_path: str | Path,
    *,
    cache_root: str | Path | None = None,
    rebake: bool = False,
) -> PreparedSceneSource:
    """Return a concrete mesh source for the scene cooking pipeline.

    Most supported source formats already are mesh assets, so they pass through
    unchanged. Authored project formats such as ``.blend`` are normalized into
    GLB first, using the authoring tool to evaluate procedural data and
    instances into concrete mesh nodes.
    """
    source = Path(source_path).expanduser().resolve()
    if not source.exists():
        raise FileNotFoundError(f"scene source not found: {source}")
    if not source.is_file():
        raise ValueError(f"scene source must be a file: {source}")

    suffix = source.suffix.lower()
    if suffix in DIRECT_SCENE_SUFFIXES:
        return PreparedSceneSource(original_path=source, cook_path=source)

    normalizer = _NORMALIZERS.get(suffix)
    if normalizer is None:
        supported = ", ".join(sorted((*DIRECT_SCENE_SUFFIXES, *_NORMALIZERS)))
        raise RuntimeError(f"unsupported scene source suffix {suffix!r}; supported: {supported}")

    cache_dir = Path(cache_root).expanduser().resolve() if cache_root else SOURCE_CACHE_DIR
    cache_dir.mkdir(parents=True, exist_ok=True)
    return normalizer(source, cache_dir, rebake)


def _normalize_blend_source(
    source: Path,
    cache_dir: Path,
    rebake: bool,
) -> PreparedSceneSource:
    cache_key = _source_cache_key(source, _BLENDER_NORMALIZER_VERSION)
    target = cache_dir / f"{source.stem}-{cache_key}.glb"
    if target.exists() and not rebake:
        return PreparedSceneSource(
            original_path=source,
            cook_path=target,
            normalized=True,
            normalizer=_BLENDER_NORMALIZER_VERSION,
            stats={"cache_hit": True},
        )

    blender = shutil.which("blender")
    if blender is None:
        raise RuntimeError(".blend scene cooking requires Blender on PATH")

    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as script:
        script.write(_BLENDER_SOURCE_NORMALIZER_SCRIPT)
        script_path = Path(script.name)
    try:
        output = _run_command(
            [
                blender,
                "--background",
                str(source),
                "--python",
                str(script_path),
                "--",
                str(target),
            ],
            "blender source normalization",
        )
    finally:
        script_path.unlink(missing_ok=True)

    if not target.exists():
        raise RuntimeError(f"Blender source normalization did not write {target}")
    logger.info("normalized Blender scene source", source=source, target=target)
    return PreparedSceneSource(
        original_path=source,
        cook_path=target,
        normalized=True,
        normalizer=_BLENDER_NORMALIZER_VERSION,
        stats=_parse_normalizer_stats(output),
    )


def _source_cache_key(source: Path, version: str) -> str:
    h = hashlib.sha256()
    h.update(version.encode())
    with source.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


def _run_command(args: list[str], label: str) -> str:
    return run_logged_command(
        args,
        label,
        tail_lines=40,
        line_log_filter=blender_output_line_is_interesting,
        env=blender_command_env(),
    )


def _parse_normalizer_stats(output: str) -> dict[str, Any]:
    stats: dict[str, Any] = {"cache_hit": False}
    for line in output.splitlines():
        if not line.startswith("DIMOS_BLEND_NORMALIZER "):
            continue
        for item in line.removeprefix("DIMOS_BLEND_NORMALIZER ").split():
            key, sep, value = item.partition("=")
            if not sep:
                continue
            try:
                stats[key] = int(value)
            except ValueError:
                stats[key] = value
    return stats


_BLENDER_SOURCE_NORMALIZER_SCRIPT = r"""
import pathlib
import re
import sys

import bpy

target = pathlib.Path(sys.argv[-1])
target.parent.mkdir(parents=True, exist_ok=True)


def log(message):
    print(f"DIMOS_BLEND_SOURCE {message}", flush=True)


log(f"start target={target}")
depsgraph = bpy.context.evaluated_depsgraph_get()
log("depsgraph ready")
collection = bpy.data.collections.new("DIMOS_Normalized_Source")
bpy.context.scene.collection.children.link(collection)

name_counts = {}
mesh_cache = {}
realized = []
skipped_empty = 0
skipped_non_mesh = 0
instances = 0
base_objects = 0


def safe_name(raw):
    cleaned = re.sub(r"[^0-9A-Za-z_.-]+", "_", raw).strip("_")
    return cleaned or "mesh"


def unique_name(raw):
    base = safe_name(raw)
    count = name_counts.get(base, 0)
    name_counts[base] = count + 1
    if count == 0:
        return base
    return f"{base}.{count:04d}"


def object_key(obj):
    original = getattr(obj, "original", None)
    if original is not None:
        obj = original
    return str(obj.as_pointer())


def mesh_for_source(obj):
    key = object_key(obj)
    cached = mesh_cache.get(key)
    if cached is not None:
        return cached

    evaluated = obj.evaluated_get(depsgraph)
    try:
        temp = evaluated.to_mesh(preserve_all_data_layers=True, depsgraph=depsgraph)
    except TypeError:
        temp = evaluated.to_mesh()

    if temp is None or len(temp.vertices) == 0 or len(temp.polygons) == 0:
        if temp is not None:
            evaluated.to_mesh_clear()
        mesh_cache[key] = None
        return None

    mesh = temp.copy()
    mesh.name = f"{safe_name(obj.name)}_Mesh"
    evaluated.to_mesh_clear()
    mesh_cache[key] = mesh
    return mesh


for index, inst in enumerate(depsgraph.object_instances):
    source_obj = inst.instance_object if inst.is_instance and inst.instance_object else inst.object
    if source_obj is None or source_obj.type != "MESH":
        skipped_non_mesh += 1
        continue

    mesh = mesh_for_source(source_obj)
    if mesh is None:
        skipped_empty += 1
        continue

    parent_name = inst.parent.name if inst.parent is not None else None
    if inst.is_instance:
        instances += 1
        raw_name = f"{parent_name or inst.object.name}__{source_obj.name}"
    else:
        base_objects += 1
        raw_name = source_obj.name

    obj = bpy.data.objects.new(unique_name(raw_name), mesh)
    obj.matrix_world = inst.matrix_world.copy()
    collection.objects.link(obj)
    realized.append(obj)
    if len(realized) % 100 == 0:
        log(f"realized progress objects={len(realized)} depsgraph_index={index}")

if not realized:
    raise RuntimeError("Blender scene normalization produced no mesh objects")

bpy.ops.object.select_all(action="DESELECT")
for obj in realized:
    obj.select_set(True)
bpy.context.view_layer.objects.active = realized[0]

log(f"export start realized_objects={len(realized)} unique_meshes={sum(1 for mesh in mesh_cache.values() if mesh is not None)}")
bpy.ops.export_scene.gltf(
    filepath=str(target),
    export_format="GLB",
    export_yup=True,
    use_selection=True,
    export_cameras=False,
    export_lights=False,
    export_apply=False,
)
log("export done")

print(
    "DIMOS_BLEND_NORMALIZER "
    f"base_objects={base_objects} "
    f"instances={instances} "
    f"realized_objects={len(realized)} "
    f"unique_meshes={sum(1 for mesh in mesh_cache.values() if mesh is not None)} "
    f"skipped_empty={skipped_empty} "
    f"skipped_non_mesh={skipped_non_mesh}"
)
"""


_NORMALIZERS: dict[str, Normalizer] = {
    ".blend": _normalize_blend_source,
}

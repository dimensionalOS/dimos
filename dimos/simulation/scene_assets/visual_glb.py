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

"""Cook browser visual assets for real-time browser rendering."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any

from dimos.simulation.scene_assets.inspect import inspect_scene_asset
from dimos.simulation.scene_assets.spec import BrowserVisualSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_BLENDER_INPUT_SUFFIXES = {
    ".usd",
    ".usda",
    ".usdc",
    ".usdz",
    ".gltf",
    ".glb",
    ".obj",
    ".stl",
    ".ply",
}
_GLTFPACK_INPUT_SUFFIXES = {".gltf", ".glb", ".obj"}
_COMMAND_TAIL_LINES = 30

_BLENDER_SCRIPT = r"""
import pathlib
import sys

import bpy

source = pathlib.Path(sys.argv[-4])
target = pathlib.Path(sys.argv[-3])
simplify_ratio = float(sys.argv[-2])
max_texture_size = int(sys.argv[-1])
suffix = source.suffix.lower()

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete()

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
    raise RuntimeError(f"unsupported visual source suffix: {suffix}")

for obj in list(bpy.context.scene.objects):
    if obj.type != "MESH":
        bpy.data.objects.remove(obj, do_unlink=True)

if max_texture_size > 0:
    for image in bpy.data.images:
        width, height = image.size
        largest = max(width, height)
        if largest <= max_texture_size:
            continue
        scale = max_texture_size / largest
        try:
            image.scale(max(1, int(width * scale)), max(1, int(height * scale)))
        except RuntimeError:
            # Blender cannot scale some generated or missing images; keep those
            # untouched instead of aborting the entire scene cook.
            pass

if 0.0 < simplify_ratio < 0.999:
    for obj in list(bpy.context.scene.objects):
        if obj.type != "MESH":
            continue
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        modifier = obj.modifiers.new("dimos_decimate", "DECIMATE")
        modifier.ratio = simplify_ratio
        try:
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        except RuntimeError:
            obj.modifiers.remove(modifier)

mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
if len(mesh_objects) > 1:
    bpy.ops.object.select_all(action="DESELECT")
    for obj in mesh_objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_objects[0]
    bpy.ops.object.join()

bpy.ops.export_scene.gltf(
    filepath=str(target),
    export_format="GLB",
    export_yup=True,
    export_apply=True,
)
"""


@dataclass(frozen=True)
class BrowserVisualCookResult:
    path: Path
    stats: dict[str, Any]
    tool: str


def cook_browser_visual(
    source_path: str | Path,
    output_dir: str | Path,
    *,
    spec: BrowserVisualSpec | None = None,
    rebake: bool = False,
) -> BrowserVisualCookResult | None:
    """Write the browser visual GLB for a scene package.

    ``gltfpack`` is the default path because browser performance is dominated
    by draw calls, scene nodes, decoded texture memory, and shader/material
    switches.  Blender is kept as a conversion fallback for formats gltfpack
    does not read directly.
    """
    visual_spec = spec or BrowserVisualSpec()
    if not visual_spec.enabled:
        return None

    source = Path(source_path).expanduser().resolve()
    out_dir = Path(output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / visual_spec.output_name
    if out_path.exists() and not rebake:
        return BrowserVisualCookResult(
            path=out_path,
            stats=inspect_scene_asset(out_path).to_json_dict(),
            tool="cache",
        )

    source_stats = inspect_scene_asset(source).to_json_dict()
    with tempfile.TemporaryDirectory(prefix="dimos-visual-cook-") as temp_dir_raw:
        temp_dir = Path(temp_dir_raw)
        temp_out = temp_dir / out_path.name
        tool, report = _cook_visual(source, temp_out, visual_spec)
        stats = inspect_scene_asset(temp_out).to_json_dict()
        _validate_output(source_stats, stats, visual_spec)
        if report is not None:
            stats["optimizer_report"] = report
        shutil.move(str(temp_out), out_path)
        stats["path"] = str(out_path)

    warnings = _budget_warnings(stats, visual_spec)
    if warnings:
        stats["warnings"] = warnings
        for warning in warnings:
            logger.warning("browser visual budget: %s", warning)
    return BrowserVisualCookResult(path=out_path, stats=stats, tool=tool)


def _cook_visual(
    source: Path, target: Path, spec: BrowserVisualSpec
) -> tuple[str, dict[str, Any] | None]:
    optimizer = spec.optimizer.lower()
    if optimizer == "copy":
        if source.suffix.lower() != ".glb":
            raise RuntimeError("copy visual optimizer requires a GLB source")
        shutil.copy2(source, target)
        return ("copy", None)
    if optimizer == "blender":
        _export_with_blender(
            source,
            target,
            simplify_ratio=spec.simplify_ratio,
            max_texture_size=spec.max_texture_size,
        )
        return ("blender", None)
    if optimizer == "gltfpack":
        return _export_with_gltfpack(source, target, spec)
    raise ValueError(f"unknown browser visual optimizer: {spec.optimizer}")


def _export_with_blender(
    source: Path,
    target: Path,
    *,
    simplify_ratio: float = 1.0,
    max_texture_size: int | None = None,
) -> None:
    blender = shutil.which("blender")
    if blender is None:
        raise RuntimeError(
            f"{source.suffix} visual export requires Blender on PATH. Install Blender."
        )

    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as script:
        script.write(_BLENDER_SCRIPT)
        script_path = Path(script.name)
    try:
        _run_command(
            [
                blender,
                "--background",
                "--factory-startup",
                "--python",
                str(script_path),
                "--",
                str(source),
                str(target),
                str(simplify_ratio),
                str(max_texture_size or 0),
            ],
            "blender",
        )
    finally:
        script_path.unlink(missing_ok=True)


def _export_with_gltfpack(
    source: Path,
    target: Path,
    spec: BrowserVisualSpec,
) -> tuple[str, dict[str, Any] | None]:
    command = _gltfpack_command()
    source_for_gltfpack = source
    with tempfile.TemporaryDirectory(prefix="dimos-gltfpack-source-") as temp_dir_raw:
        if source.suffix.lower() not in _GLTFPACK_INPUT_SUFFIXES:
            if source.suffix.lower() not in _BLENDER_INPUT_SUFFIXES:
                raise RuntimeError(f"unsupported visual source suffix: {source.suffix}")
            source_for_gltfpack = Path(temp_dir_raw) / "source.glb"
            _export_with_blender(source, source_for_gltfpack)

        report_path = target.with_suffix(".gltfpack.json")
        args = [
            *command,
            "-i",
            str(source_for_gltfpack),
            "-o",
            str(target),
            "-mm",
            "-si",
            str(spec.simplify_ratio),
            "-se",
            str(spec.simplify_error),
            "-r",
            str(report_path),
        ]
        if spec.texture_format == "webp":
            args.append("-tw")
        elif spec.texture_format == "ktx2":
            args.append("-tc")
        elif spec.texture_format is not None:
            raise ValueError(f"unknown browser texture format: {spec.texture_format}")
        if spec.max_texture_size is not None:
            if spec.texture_format is None:
                raise ValueError("max_texture_size requires texture_format='webp' or 'ktx2'")
            args.extend(["-tl", str(spec.max_texture_size)])

        output = _run_command(args, "gltfpack")
        if output and "Warning:" in output:
            logger.warning("gltfpack output:\n%s", _tail(output))
        report = _read_json(report_path)
    return ("gltfpack", report)


def _gltfpack_command() -> list[str]:
    gltfpack = shutil.which("gltfpack")
    if gltfpack is not None:
        return [gltfpack]
    npx = shutil.which("npx")
    if npx is not None:
        return [npx, "-y", "gltfpack"]
    raise RuntimeError(
        "browser visual optimization requires gltfpack. Install it with "
        "`npm install -g gltfpack` or use --visual-optimizer blender/copy."
    )


def _run_command(args: list[str], label: str) -> str:
    result = subprocess.run(
        args,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    output = result.stdout or ""
    if result.returncode != 0:
        raise RuntimeError(f"{label} failed with exit code {result.returncode}:\n{_tail(output)}")
    return output


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        logger.warning("failed to parse optimizer report: %s", path)
        return None


def _validate_output(
    source_stats: dict[str, Any],
    output_stats: dict[str, Any],
    spec: BrowserVisualSpec,
) -> None:
    source_vertices = int(source_stats.get("vertex_count") or 0)
    output_vertices = int(output_stats.get("vertex_count") or 0)
    if source_vertices <= 0 or output_vertices <= 0:
        return
    max_vertices = int(source_vertices * spec.max_vertex_growth_ratio)
    if output_vertices > max_vertices:
        raise RuntimeError(
            "browser visual cook increased vertex count from "
            f"{source_vertices} to {output_vertices}; refusing to write worse asset"
        )


def _tail(output: str) -> str:
    return "\n".join(output.splitlines()[-_COMMAND_TAIL_LINES:])


def _budget_warnings(stats: dict[str, Any], spec: BrowserVisualSpec) -> list[str]:
    warnings: list[str] = []
    mesh_count = int(stats.get("node_count") or stats.get("mesh_count") or 0)
    material_count = int(stats.get("material_count") or 0)
    texture_count = int(stats.get("texture_count") or 0)
    vertex_count = int(stats.get("vertex_count") or 0)
    if mesh_count > spec.max_meshes:
        warnings.append(f"{mesh_count} render nodes exceeds target {spec.max_meshes}")
    if material_count > spec.max_materials:
        warnings.append(f"{material_count} materials exceeds target {spec.max_materials}")
    if texture_count > spec.max_textures:
        warnings.append(f"{texture_count} textures exceeds target {spec.max_textures}")
    if vertex_count > spec.max_vertices:
        warnings.append(f"{vertex_count} vertices exceeds target {spec.max_vertices}")
    return warnings


__all__ = ["BrowserVisualCookResult", "cook_browser_visual"]

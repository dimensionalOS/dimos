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
import tempfile
from typing import Any

from dimos.experimental.scene_cooking.command import (
    blender_command_env,
    blender_output_line_is_interesting,
    run_logged_command,
)
from dimos.experimental.scene_cooking.package_config import BrowserVisualSpec
from dimos.experimental.scene_cooking.source_assets.glb import (
    demote_required_extensions,
    normalize_embedded_textures,
)
from dimos.experimental.scene_cooking.source_assets.inspect import inspect_scene_asset
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
_GLTFPACK_WARNING_TAIL_LINES = 30

_BLENDER_SCRIPT = r"""
import pathlib
import sys

import bpy

source = pathlib.Path(sys.argv[-4])
target = pathlib.Path(sys.argv[-3])
simplify_ratio = float(sys.argv[-2])
max_texture_size = int(sys.argv[-1])
suffix = source.suffix.lower()


def log(message):
    print(f"DIMOS_VISUAL_COOK {message}", flush=True)


log(
    f"start source={source} target={target} "
    f"simplify_ratio={simplify_ratio} max_texture_size={max_texture_size}"
)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete()

log(f"import start suffix={suffix}")
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
log(
    "import done "
    f"objects={len(bpy.context.scene.objects)} "
    f"meshes={len(bpy.data.meshes)} images={len(bpy.data.images)}"
)

removed_non_mesh = 0
for obj in list(bpy.context.scene.objects):
    if obj.type != "MESH":
        bpy.data.objects.remove(obj, do_unlink=True)
        removed_non_mesh += 1
log(f"removed non_mesh_objects={removed_non_mesh}")

if max_texture_size > 0:
    resized = 0
    skipped = 0
    for image in bpy.data.images:
        width, height = image.size
        largest = max(width, height)
        if largest <= max_texture_size:
            continue
        scale = max_texture_size / largest
        try:
            image.scale(max(1, int(width * scale)), max(1, int(height * scale)))
            resized += 1
        except RuntimeError:
            # Blender cannot scale some generated or missing images; keep those
            # untouched instead of aborting the entire scene cook.
            skipped += 1
    log(f"texture resize done resized={resized} skipped={skipped}")

if 0.0 < simplify_ratio < 0.999:
    mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    log(f"decimate start mesh_objects={len(mesh_objects)}")
    decimated = 0
    skipped = 0
    for index, obj in enumerate(mesh_objects, start=1):
        if obj.type != "MESH":
            continue
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        modifier = obj.modifiers.new("dimos_decimate", "DECIMATE")
        modifier.ratio = simplify_ratio
        try:
            bpy.ops.object.modifier_apply(modifier=modifier.name)
            decimated += 1
        except RuntimeError:
            obj.modifiers.remove(modifier)
            skipped += 1
        if index % 25 == 0:
            log(f"decimate progress processed={index}/{len(mesh_objects)}")
    log(f"decimate done decimated={decimated} skipped={skipped}")

mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
if len(mesh_objects) > 1:
    log(f"join start mesh_objects={len(mesh_objects)}")
    bpy.ops.object.select_all(action="DESELECT")
    for obj in mesh_objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_objects[0]
    bpy.ops.object.join()
    log(f"join done objects={len(bpy.context.scene.objects)}")

log("export start")
bpy.ops.export_scene.gltf(
    filepath=str(target),
    export_format="GLB",
    export_yup=True,
    export_apply=True,
)
log("export done")
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
    out_path = out_dir / visual_spec.artifact_name
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
        _sanitize_browser_visual_output(temp_out, visual_spec)
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
    source: Path,
    target: Path,
    spec: BrowserVisualSpec,
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
        run_logged_command(
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
            line_log_filter=blender_output_line_is_interesting,
            env=blender_command_env(),
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
            "-v",
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
        if not spec.quantize:
            args.append("-noq")
        if spec.use_gpu_instancing:
            args.append("-mi")
        if spec.texture_format == "webp":
            _require_native_gltfpack_for_texture_compression(command, spec.texture_format)
            args.append("-tw")
        elif spec.texture_format == "ktx2":
            _require_native_gltfpack_for_texture_compression(command, spec.texture_format)
            args.append("-tc")
        elif spec.texture_format is not None:
            raise ValueError(f"unknown browser texture format: {spec.texture_format}")
        if spec.max_texture_size is not None:
            if spec.texture_format is None:
                raise ValueError("max_texture_size requires texture_format='webp' or 'ktx2'")
            args.extend(["-tl", str(spec.max_texture_size)])

        try:
            output = run_logged_command(args, "gltfpack")
        except RuntimeError as exc:
            if "unreachable" in str(exc):
                raise RuntimeError(
                    "gltfpack crashed internally while optimizing the browser visual "
                    f"for {source_for_gltfpack}. This is a tool failure, not a scene "
                    "sidecar validation error. Try a native gltfpack build first; if "
                    "that still fails, partition the visual source or use "
                    "--visual-optimizer blender/copy for diagnosis."
                ) from exc
            raise
        if output and "Warning:" in output:
            logger.warning("gltfpack output:\n%s", _tail(output, _GLTFPACK_WARNING_TAIL_LINES))
        report = _read_json(report_path)
    return ("gltfpack", report)


def _sanitize_browser_visual_output(path: Path, spec: BrowserVisualSpec) -> None:
    if path.suffix.lower() != ".glb":
        return

    demoted_extensions = demote_required_extensions(path, set(spec.demote_required_extensions))
    if demoted_extensions:
        logger.info(
            "demoted browser visual GLB extensions target=%s path=%s extensions=%s",
            spec.target_key,
            path,
            sorted(demoted_extensions),
        )

    if spec.normalize_textures and spec.texture_format is None:
        normalized_textures = normalize_embedded_textures(path)
        if normalized_textures:
            logger.info(
                "normalized embedded browser visual textures target=%s path=%s count=%d",
                spec.target_key,
                path,
                normalized_textures,
            )


def _gltfpack_command() -> list[str]:
    gltfpack = shutil.which("gltfpack")
    if gltfpack is not None:
        return [gltfpack]
    npx = shutil.which("npx")
    if npx is not None:
        return [npx, "-y", "gltfpack"]
    raise RuntimeError(
        "browser visual optimization requires gltfpack. Install it with "
        "a native meshoptimizer gltfpack binary on PATH, or use "
        "--visual-optimizer blender/copy."
    )


def _require_native_gltfpack_for_texture_compression(
    command: list[str],
    texture_format: str,
) -> None:
    executable = Path(command[0]).name
    if executable != "npx":
        return
    raise RuntimeError(
        f"gltfpack texture compression requested ({texture_format}), but the "
        "available gltfpack is the Node/npx build. That build does not support "
        "WebP/KTX texture compression. Install a native gltfpack binary from "
        "meshoptimizer releases on PATH, or set --visual-texture-format none."
    )


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError:
        logger.warning("failed to parse optimizer report: %s", path)
        return None
    return data if isinstance(data, dict) else None


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


def _tail(output: str, tail_lines: int) -> str:
    return "\n".join(output.splitlines()[-tail_lines:])


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

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

"""Offline scene package cooker.

This is intentionally not a DimOS runtime module. It prepares cooked scene
packages that runtime modules consume through normal config.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
from typing import Any

from dimos.experimental.scene_cooking.browser.collision import cook_browser_collision
from dimos.experimental.scene_cooking.browser.visuals import cook_browser_visual
from dimos.experimental.scene_cooking.entities.collision import (
    COLLISION_DIR_NAME,
    cook_entity_collision_hulls,
)
from dimos.experimental.scene_cooking.entities.visuals import cook_plan_visual_assets
from dimos.experimental.scene_cooking.mujoco.collision_export import load_or_bake
from dimos.experimental.scene_cooking.mujoco.collision_policy import CollisionSpec
from dimos.experimental.scene_cooking.package_config import (
    BROWSER_VISUAL_TARGETS,
    BrowserCollisionSpec,
    BrowserVisualSpec,
    MujocoSceneSpec,
    SceneCookSpec,
    browser_visual_spec_for_target,
)
from dimos.experimental.scene_cooking.planning import EntityPrototypePlan, build_scene_cook_plan
from dimos.experimental.scene_cooking.sidecar import SceneCookSidecar
from dimos.experimental.scene_cooking.source_assets.inspect import inspect_scene_asset
from dimos.experimental.scene_cooking.source_assets.normalize import prepare_scene_source
from dimos.simulation.scene_assets.spec import (
    SceneMeshAlignment,
    ScenePackage,
)
from dimos.utils.data import get_data_dir
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SCENE_PACKAGE_DIR = get_data_dir("scene_packages")
_PACKAGE_KEY_LEN = 12
_COOK_VERSION = 4
#: Cap on entity id samples recorded in cook stats -- diagnostics only, not
#: the full entity list (that lives in ``scene.meta.json``).
_ENTITY_ID_SAMPLE_CAP = 100


def cook_scene_package(
    source_path: str | Path,
    *,
    output_dir: str | Path | None = None,
    alignment: SceneMeshAlignment | None = None,
    collision_spec: CollisionSpec | None = None,
    cook_sidecar: SceneCookSidecar | None = None,
    visual_spec: BrowserVisualSpec | None = None,
    browser_collision_spec: BrowserCollisionSpec | None = None,
    mujoco_spec: MujocoSceneSpec | None = None,
    rebake: bool = False,
) -> ScenePackage:
    """Cook one source scene into a robot-agnostic package.

    The package contains browser artifacts (visual + collision GLBs,
    semantic ``objects.json``), per-entity GLBs, and a scene-only MuJoCo
    wrapper. Robots are attached at runtime via ``MjSpec.attach()`` inside
    ``MujocoSimModule.start``; the cooker never touches robot MJCFs.
    """
    source = Path(source_path).expanduser().resolve()
    if not source.exists():
        raise FileNotFoundError(f"scene source not found: {source}")

    align = alignment or SceneMeshAlignment()
    visual = visual_spec or BrowserVisualSpec()
    browser_collision = browser_collision_spec or BrowserCollisionSpec()
    mujoco = mujoco_spec or MujocoSceneSpec()
    cook_spec = SceneCookSpec(
        source_path=source,
        alignment=align,
        browser_visual=visual,
        browser_collision=browser_collision,
        mujoco=mujoco,
    )
    sidecar = cook_sidecar or SceneCookSidecar.auto_discover(source)

    package_dir = (
        Path(output_dir).expanduser().resolve()
        if output_dir is not None
        else SCENE_PACKAGE_DIR / _package_key(cook_spec, sidecar)
    )
    browser_dir = package_dir / "browser"
    mujoco_dir = package_dir / "mujoco"
    package_dir.mkdir(parents=True, exist_ok=True)

    prepared_source = prepare_scene_source(source, rebake=rebake)
    cook_source = prepared_source.cook_path

    stats: dict[str, Any] = {
        "source": inspect_scene_asset(cook_source).to_json_dict(),
        "cook_spec": _cook_spec_json(cook_spec),
        "cook_version": _COOK_VERSION,
    }
    if prepared_source.normalized:
        stats["source_normalization"] = prepared_source.to_json_dict()
    if sidecar.path is not None or sidecar.interactables or sidecar.entity_groups:
        stats["authored_sidecar"] = sidecar.to_json_dict()

    plan = build_scene_cook_plan(
        cook_source,
        sidecar=sidecar,
        alignment=align,
        output_dir=package_dir,
        collision_spec=collision_spec,
    )
    stats["cook_plan"] = plan.to_json_dict()

    entities = plan.entities_metadata()
    if entities:
        stats["interactables"] = {
            "count": len(entities),
            "id_samples": [entity["id"] for entity in entities[:_ENTITY_ID_SAMPLE_CAP]],
            "static_visual_filter": "plan/blender",
        }

    visual_source = cook_source
    # Only invoke Blender when at least one entity actually extracts from
    # the source mesh; pure-synthetic sidecars (manip rigs) don't need it.
    needs_blender = visual.enabled and any(
        entity.visual_path is not None for entity in plan.entities
    )
    if needs_blender:
        visual_source = cook_plan_visual_assets(
            cook_source,
            package_dir,
            plan=plan,
            rebake=rebake,
        )

    if mujoco.enabled:
        prototype_hull_counts = _cook_entity_prototype_collision(
            plan.prototypes,
            entities,
            rebake=rebake,
        )
        hull_counts = _cook_entity_collision(entities, rebake=rebake)
        if prototype_hull_counts or hull_counts:
            stats["entity_collision"] = {}
            if prototype_hull_counts:
                stats["entity_collision"]["hulls_per_prototype"] = prototype_hull_counts
            if hull_counts:
                stats["entity_collision"]["hulls_per_entity"] = hull_counts

    visual_result = cook_browser_visual(
        visual_source,
        browser_dir,
        spec=visual,
        rebake=rebake,
    )
    if visual_result is not None:
        visual_stats = {
            "target": visual.target_key,
            "tool": visual_result.tool,
            **visual_result.stats,
        }
        stats["browser_visual"] = {
            **visual_stats,
        }
        stats["browser_visuals"] = {
            visual.target_key: visual_stats,
        }

    browser_collision_result = cook_browser_collision(
        cook_source,
        browser_dir,
        alignment=SceneMeshAlignment(y_up=False),
        spec=browser_collision,
        collision_spec=plan.collision_spec,
        rebake=rebake,
    )
    if browser_collision_result is not None:
        stats["browser_collision"] = browser_collision_result.stats

    mujoco_scene_path: Path | None = None
    mujoco_binary_path: Path | None = None
    if mujoco.enabled:
        mujoco_scene_path = load_or_bake(
            scene_mesh_path=cook_source,
            alignment=align,
            cache_root=mujoco_dir,
            collision_spec=plan.collision_spec,
            include_visual_mesh=mujoco.include_visual_mesh,
            rebake=rebake,
        )
        stats["mujoco"] = {"scene_path": str(mujoco_scene_path)}
        if mujoco.compile_binary:
            mujoco_binary_path, binary_stats = _compile_mujoco_binary(
                mujoco_scene_path,
                rebake=rebake,
            )
            stats["mujoco"]["binary_path"] = str(mujoco_binary_path)
            stats["mujoco"]["binary"] = binary_stats

    package = ScenePackage(
        package_dir=package_dir,
        source_path=source,
        alignment=align,
        visual_path=visual_result.path if visual_result else None,
        browser_visuals={visual.target_key: visual_result.path} if visual_result else {},
        browser_collision_path=browser_collision_result.path if browser_collision_result else None,
        objects_path=browser_collision_result.objects_path if browser_collision_result else None,
        mujoco_scene_path=mujoco_scene_path,
        mujoco_binary_path=mujoco_binary_path,
        metadata_path=package_dir / "scene.meta.json",
        entities=entities,
        stats=stats,
    )
    package.write_metadata()
    logger.info("scene package cooked", metadata_path=package.metadata_path)
    return package


def _cook_entity_collision(
    entities: list[dict[str, Any]],
    *,
    rebake: bool,
) -> dict[str, int]:
    """Decompose every mesh entity's GLB into package collision hulls.

    Mutates the entity metadata in place, recording the hull files as
    ``collision_paths`` so the runtime composer loads them from the
    package instead of decomposing at boot. Returns hull counts by
    entity id.
    """
    hull_counts: dict[str, int] = {}
    for entity in entities:
        if entity.get("descriptor", {}).get("shape_hint") != "mesh":
            continue
        if entity.get("collision_paths"):
            continue
        visual_path = entity.get("visual_path")
        if not visual_path or not Path(visual_path).exists():
            logger.warning(
                "mesh entity has no cooked visual GLB; "
                "no collision hulls (runtime falls back to AABB box)",
                entity_id=entity.get("id"),
            )
            continue
        hull_paths = cook_entity_collision_hulls(
            visual_path,
            Path(visual_path).parent / COLLISION_DIR_NAME,
            rebake=rebake,
        )
        if hull_paths:
            entity["collision_paths"] = [str(path) for path in hull_paths]
            hull_counts[str(entity.get("id"))] = len(hull_paths)
    return hull_counts


def _cook_entity_prototype_collision(
    prototypes: tuple[EntityPrototypePlan, ...],
    entities: list[dict[str, Any]],
    *,
    rebake: bool,
) -> dict[str, int]:
    """Cook shared mesh prototypes once and attach hull paths to instances."""
    if not prototypes:
        return {}

    hulls_by_prototype: dict[str, list[Path]] = {}
    counts: dict[str, int] = {}
    for prototype in prototypes:
        source_obj = prototype.collision_dir.parent / "source.obj"
        if rebake or not source_obj.exists():
            _write_obj(source_obj, prototype.vertices, prototype.triangles)
        hull_paths = cook_entity_collision_hulls(
            source_obj,
            prototype.collision_dir,
            rebake=rebake,
        )
        if hull_paths:
            hulls_by_prototype[prototype.id] = hull_paths
            counts[prototype.id] = len(hull_paths)

    for entity in entities:
        prototype_id = entity.get("prototype_id")
        if not isinstance(prototype_id, str):
            continue
        prototype_hull_paths = hulls_by_prototype.get(prototype_id)
        if prototype_hull_paths:
            entity["collision_paths"] = [str(path) for path in prototype_hull_paths]
    return counts


def _write_obj(path: Path, vertices: Any, triangles: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        for vertex in vertices:
            f.write(f"v {float(vertex[0])} {float(vertex[1])} {float(vertex[2])}\n")
        for tri in triangles:
            f.write(f"f {int(tri[0]) + 1} {int(tri[1]) + 1} {int(tri[2]) + 1}\n")


def _compile_mujoco_binary(scene_xml_path: Path, *, rebake: bool) -> tuple[Path, dict[str, Any]]:
    """Compile a scene-only MuJoCo XML wrapper to ``.mjb``.

    This binary is fast to load but not editable as ``MjSpec``. It is a
    cache/debug artifact for fixed scene models; robot attachment still
    requires the XML wrapper unless a robot-specific composed binary is
    produced separately.
    """
    # Lazy: mujoco is a `sim` extra, not a `scene` one -- browser/rerun-only
    # cooks (mujoco.compile_binary=False) shouldn't require it installed.
    import mujoco

    binary_path = scene_xml_path.with_suffix(".mjb")
    if binary_path.exists() and not rebake:
        return binary_path, {
            "cached": True,
            "size_bytes": binary_path.stat().st_size,
        }

    start = time.perf_counter()
    model = mujoco.MjModel.from_xml_path(str(scene_xml_path))
    compile_s = time.perf_counter() - start

    save_start = time.perf_counter()
    mujoco.mj_saveModel(model, str(binary_path))
    save_s = time.perf_counter() - save_start

    return binary_path, {
        "cached": False,
        "compile_seconds": compile_s,
        "save_seconds": save_s,
        "size_bytes": binary_path.stat().st_size,
        "nbody": int(model.nbody),
        "ngeom": int(model.ngeom),
        "nmesh": int(model.nmesh),
    }


def _package_key(
    cook_spec: SceneCookSpec,
    sidecar: SceneCookSidecar,
) -> str:
    h = hashlib.sha256()
    h.update(cook_spec.source_path.read_bytes())
    h.update(str(_COOK_VERSION).encode())
    h.update(json.dumps(_cook_spec_json(cook_spec), sort_keys=True).encode())
    h.update(json.dumps(sidecar.to_json_dict(), sort_keys=True).encode())
    return h.hexdigest()[:_PACKAGE_KEY_LEN]


def _cook_spec_json(cook_spec: SceneCookSpec) -> dict[str, Any]:
    raw = asdict(cook_spec)
    raw["source_path"] = str(cook_spec.source_path)
    return raw


def _parse_xyz(value: str) -> tuple[float, float, float]:
    parts = [float(part.strip()) for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected comma-separated x,y,z")
    return (parts[0], parts[1], parts[2])


def cli_main() -> None:
    parser = argparse.ArgumentParser(
        description="Cook a scene asset into a robot-agnostic DimOS scene package.",
    )
    parser.add_argument("source", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--cook-spec", type=Path)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--translation", type=_parse_xyz, default=(0.0, 0.0, 0.0))
    parser.add_argument("--rotation-zyx-deg", type=_parse_xyz, default=(0.0, 0.0, 0.0))
    parser.add_argument("--no-y-up", action="store_true")
    parser.add_argument("--no-visual", action="store_true")
    parser.add_argument(
        "--visual-optimizer",
        choices=("gltfpack", "blender", "copy"),
    )
    parser.add_argument(
        "--visual-target",
        choices=BROWSER_VISUAL_TARGETS,
        default="rerun",
        help=(
            "browser visual target to cook. Rerun uses conservative GLBs; "
            "Babylon can use web-oriented glTF extensions."
        ),
    )
    parser.add_argument("--visual-output-name")
    parser.add_argument("--visual-simplify-ratio", type=float)
    parser.add_argument("--visual-simplify-error", type=float)
    parser.add_argument("--visual-max-texture-size", type=int)
    parser.add_argument(
        "--visual-texture-format",
        choices=("none", "webp", "ktx2"),
    )
    parser.add_argument(
        "--visual-texture-normalization",
        dest="visual_texture_normalization",
        action="store_true",
        default=None,
        help="rewrite embedded visual textures to plain 8-bit PNGs",
    )
    parser.add_argument(
        "--no-visual-texture-normalization",
        dest="visual_texture_normalization",
        action="store_false",
        default=None,
        help="keep the optimizer's embedded texture encoding",
    )
    parser.add_argument(
        "--visual-quantize",
        dest="visual_quantize",
        action="store_true",
        default=None,
        help="allow mesh quantization when the target viewer supports it",
    )
    parser.add_argument(
        "--no-visual-quantize",
        dest="visual_quantize",
        action="store_false",
        default=None,
        help="disable mesh quantization",
    )
    parser.add_argument(
        "--visual-gpu-instancing",
        dest="visual_gpu_instancing",
        action="store_true",
        default=None,
        help="allow EXT_mesh_gpu_instancing when the target viewer supports it",
    )
    parser.add_argument(
        "--no-visual-gpu-instancing",
        dest="visual_gpu_instancing",
        action="store_false",
        default=None,
        help="disable EXT_mesh_gpu_instancing",
    )
    parser.add_argument("--no-browser-collision", action="store_true")
    parser.add_argument("--browser-collision-target-faces", type=int, default=100_000)
    parser.add_argument("--no-mujoco", action="store_true")
    parser.add_argument("--include-mujoco-visual", action="store_true")
    parser.add_argument(
        "--compile-mujoco-binary",
        action="store_true",
        help=(
            "also compile the scene-only MuJoCo wrapper.xml to wrapper.mjb. "
            "Fast to load, but not usable for runtime robot attachment by itself."
        ),
    )
    parser.add_argument("--rebake", action="store_true")
    args = parser.parse_args()

    visual_overrides: dict[str, Any] = {"enabled": not args.no_visual}
    for key, value in (
        ("output_name", args.visual_output_name),
        ("optimizer", args.visual_optimizer),
        ("simplify_ratio", args.visual_simplify_ratio),
        ("simplify_error", args.visual_simplify_error),
        ("max_texture_size", args.visual_max_texture_size),
        ("normalize_textures", args.visual_texture_normalization),
        ("quantize", args.visual_quantize),
        ("use_gpu_instancing", args.visual_gpu_instancing),
    ):
        if value is not None:
            visual_overrides[key] = value
    if args.visual_texture_format is not None:
        visual_overrides["texture_format"] = (
            None if args.visual_texture_format == "none" else args.visual_texture_format
        )

    package = cook_scene_package(
        args.source,
        output_dir=args.output_dir,
        alignment=SceneMeshAlignment(
            scale=args.scale,
            translation=args.translation,
            rotation_zyx_deg=args.rotation_zyx_deg,
            y_up=not args.no_y_up,
        ),
        cook_sidecar=SceneCookSidecar.from_json(args.cook_spec) if args.cook_spec else None,
        visual_spec=browser_visual_spec_for_target(
            args.visual_target,
            **visual_overrides,
        ),
        browser_collision_spec=BrowserCollisionSpec(
            enabled=not args.no_browser_collision,
            target_faces=args.browser_collision_target_faces,
        ),
        mujoco_spec=MujocoSceneSpec(
            enabled=not args.no_mujoco,
            include_visual_mesh=args.include_mujoco_visual,
            compile_binary=args.compile_mujoco_binary,
        ),
        rebake=args.rebake,
    )
    print(package.metadata_path)


if __name__ == "__main__":
    cli_main()

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
from typing import Any

from dimos.experimental.pimsim.scene.browser_collision import cook_browser_collision
from dimos.experimental.pimsim.scene.entity_collision import (
    COLLISION_DIR_NAME,
    cook_entity_collision_hulls,
)
from dimos.experimental.pimsim.scene.inspect import inspect_scene_asset
from dimos.experimental.pimsim.scene.plan import build_scene_cook_plan
from dimos.experimental.pimsim.scene.sidecar import SceneCookSidecar
from dimos.experimental.pimsim.scene.visual_blender import cook_plan_visual_assets
from dimos.experimental.pimsim.scene.visual_glb import cook_browser_visual
from dimos.simulation.mujoco.collision_spec import CollisionSpec
from dimos.simulation.mujoco.scene_mesh_to_mjcf import load_or_bake
from dimos.simulation.scene_assets.mesh_scene import SceneMeshAlignment
from dimos.simulation.scene_assets.spec import (
    BrowserCollisionSpec,
    BrowserVisualSpec,
    MujocoSceneSpec,
    SceneCookSpec,
    ScenePackage,
)
from dimos.utils.data import get_data_dir
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SCENE_PACKAGE_DIR = get_data_dir("scene_packages")
_PACKAGE_KEY_LEN = 12
_COOK_VERSION = 4


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

    stats: dict[str, Any] = {
        "source": inspect_scene_asset(source).to_json_dict(),
        "cook_spec": _cook_spec_json(cook_spec),
        "cook_version": _COOK_VERSION,
    }
    if sidecar.path is not None or sidecar.interactables:
        stats["authored_sidecar"] = sidecar.to_json_dict()

    plan = build_scene_cook_plan(
        source,
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
            "ids": [entity["id"] for entity in entities],
            "static_visual_filter": "plan/blender",
        }

    visual_source = source
    # Only invoke Blender when at least one entity actually extracts from
    # the source mesh; pure-synthetic sidecars (manip rigs) don't need it.
    needs_blender = visual.enabled and any(
        entity.visual_path is not None for entity in plan.entities
    )
    if needs_blender:
        visual_source = cook_plan_visual_assets(
            source,
            package_dir,
            plan=plan,
            rebake=rebake,
        )

    if mujoco.enabled:
        hull_counts = _cook_entity_collision(entities, rebake=rebake)
        if hull_counts:
            stats["entity_collision"] = {"hulls_per_entity": hull_counts}

    visual_result = cook_browser_visual(
        visual_source,
        browser_dir,
        spec=visual,
        rebake=rebake,
    )
    if visual_result is not None:
        stats["browser_visual"] = {
            "tool": visual_result.tool,
            **visual_result.stats,
        }

    browser_collision_result = cook_browser_collision(
        source,
        browser_dir,
        alignment=SceneMeshAlignment(y_up=False),
        spec=browser_collision,
        collision_spec=plan.collision_spec,
        rebake=rebake,
    )
    if browser_collision_result is not None:
        stats["browser_collision"] = browser_collision_result.stats

    mujoco_scene_path: Path | None = None
    if mujoco.enabled:
        mujoco_scene_path = load_or_bake(
            scene_mesh_path=source,
            alignment=align,
            cache_root=mujoco_dir,
            collision_spec=plan.collision_spec,
            include_visual_mesh=mujoco.include_visual_mesh,
            rebake=rebake,
        )
        stats["mujoco"] = {"scene_path": str(mujoco_scene_path)}

    package = ScenePackage(
        package_dir=package_dir,
        source_path=source,
        alignment=align,
        visual_path=visual_result.path if visual_result else None,
        browser_collision_path=browser_collision_result.path if browser_collision_result else None,
        objects_path=browser_collision_result.objects_path if browser_collision_result else None,
        mujoco_scene_path=mujoco_scene_path,
        metadata_path=package_dir / "scene.meta.json",
        entities=entities,
        stats=stats,
    )
    package.write_metadata()
    logger.info("scene package cooked: %s", package.metadata_path)
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
        visual_path = entity.get("visual_path")
        if not visual_path or not Path(visual_path).exists():
            logger.warning(
                "entity %s: mesh entity has no cooked visual GLB; "
                "no collision hulls (runtime falls back to AABB box)",
                entity.get("id"),
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
        default="gltfpack",
    )
    parser.add_argument("--visual-simplify-ratio", type=float, default=0.3)
    parser.add_argument("--visual-simplify-error", type=float, default=0.02)
    parser.add_argument("--visual-max-texture-size", type=int)
    parser.add_argument(
        "--visual-texture-format",
        choices=("none", "webp", "ktx2"),
        default="none",
    )
    parser.add_argument("--no-browser-collision", action="store_true")
    parser.add_argument("--browser-collision-target-faces", type=int, default=100_000)
    parser.add_argument("--no-mujoco", action="store_true")
    parser.add_argument("--include-mujoco-visual", action="store_true")
    parser.add_argument("--rebake", action="store_true")
    args = parser.parse_args()

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
        visual_spec=BrowserVisualSpec(
            enabled=not args.no_visual,
            optimizer=args.visual_optimizer,
            simplify_ratio=args.visual_simplify_ratio,
            simplify_error=args.visual_simplify_error,
            texture_format=(
                None if args.visual_texture_format == "none" else args.visual_texture_format
            ),
            max_texture_size=args.visual_max_texture_size,
        ),
        browser_collision_spec=BrowserCollisionSpec(
            enabled=not args.no_browser_collision,
            target_faces=args.browser_collision_target_faces,
        ),
        mujoco_spec=MujocoSceneSpec(
            enabled=not args.no_mujoco,
            include_visual_mesh=args.include_mujoco_visual,
        ),
        rebake=args.rebake,
    )
    print(package.metadata_path)


if __name__ == "__main__":
    cli_main()


__all__ = ["SCENE_PACKAGE_DIR", "cook_scene_package"]

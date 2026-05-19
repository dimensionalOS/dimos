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

This is intentionally not a DimOS runtime module.  It prepares cacheable
files that runtime modules consume through normal config.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from typing import Any

from dimos.simulation.mujoco.collision_spec import CollisionSpec
from dimos.simulation.mujoco.scene_mesh_to_mjcf import load_or_bake
from dimos.simulation.scene_assets.browser_collision import cook_browser_collision
from dimos.simulation.scene_assets.inspect import inspect_scene_asset
from dimos.simulation.scene_assets.mesh_scene import SceneMeshAlignment
from dimos.simulation.scene_assets.spec import (
    BrowserCollisionSpec,
    BrowserVisualSpec,
    MujocoSceneSpec,
    SceneCookSpec,
    ScenePackage,
)
from dimos.simulation.scene_assets.visual_glb import cook_browser_visual
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SCENE_PACKAGE_CACHE_DIR = Path.home() / ".cache" / "dimos" / "scene_packages"
_CACHE_KEY_LEN = 12


def cook_scene_package(
    source_path: str | Path,
    *,
    output_dir: str | Path | None = None,
    alignment: SceneMeshAlignment | None = None,
    robot_mjcf_path: str | Path | None = None,
    meshdir: str | Path | None = None,
    collision_spec: CollisionSpec | None = None,
    visual_spec: BrowserVisualSpec | None = None,
    browser_collision_spec: BrowserCollisionSpec | None = None,
    mujoco_spec: MujocoSceneSpec | None = None,
    rebake: bool = False,
) -> ScenePackage:
    """Cook one source scene into browser and optional MuJoCo artifacts."""
    source = Path(source_path).expanduser().resolve()
    if not source.exists():
        raise FileNotFoundError(f"scene source not found: {source}")

    align = alignment or SceneMeshAlignment()
    visual = visual_spec or BrowserVisualSpec()
    browser_collision = browser_collision_spec or BrowserCollisionSpec()
    mujoco = mujoco_spec or MujocoSceneSpec(enabled=robot_mjcf_path is not None)
    cook_spec = SceneCookSpec(
        source_path=source,
        alignment=align,
        browser_visual=visual,
        browser_collision=browser_collision,
        mujoco=mujoco,
    )

    package_dir = (
        Path(output_dir).expanduser().resolve()
        if output_dir is not None
        else SCENE_PACKAGE_CACHE_DIR / _cache_key(cook_spec, robot_mjcf_path, meshdir)
    )
    browser_dir = package_dir / "browser"
    package_dir.mkdir(parents=True, exist_ok=True)

    stats: dict[str, Any] = {
        "source": inspect_scene_asset(source).to_json_dict(),
        "cook_spec": _cook_spec_json(cook_spec),
    }

    visual_result = cook_browser_visual(
        source,
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
        collision_spec=collision_spec,
        rebake=rebake,
    )
    if browser_collision_result is not None:
        stats["browser_collision"] = browser_collision_result.stats

    mujoco_model_path: Path | None = None
    mujoco_wrapper_path: Path | None = None
    if mujoco.enabled:
        if robot_mjcf_path is None:
            raise ValueError("mujoco cook enabled but robot_mjcf_path was not provided")
        model, wrapper = load_or_bake(
            scene_mesh_path=source,
            robot_mjcf_path=robot_mjcf_path,
            alignment=align,
            meshdir=meshdir,
            collision_spec=collision_spec,
            include_visual_mesh=mujoco.include_visual_mesh,
            rebake=rebake,
        )
        del model
        compiled = wrapper.with_name("compiled.mjb")
        mujoco_model_path = compiled if compiled.exists() else wrapper
        mujoco_wrapper_path = wrapper
        stats["mujoco"] = {
            "model_path": str(mujoco_model_path),
            "wrapper_path": str(mujoco_wrapper_path),
        }

    package = ScenePackage(
        package_dir=package_dir,
        source_path=source,
        alignment=align,
        visual_path=visual_result.path if visual_result else None,
        browser_collision_path=browser_collision_result.path if browser_collision_result else None,
        mujoco_model_path=mujoco_model_path,
        mujoco_wrapper_path=mujoco_wrapper_path,
        metadata_path=package_dir / "scene.meta.json",
        stats=stats,
    )
    package.write_metadata()
    logger.info("scene package cooked: %s", package.metadata_path)
    return package


def _cache_key(
    cook_spec: SceneCookSpec,
    robot_mjcf_path: str | Path | None,
    meshdir: str | Path | None,
) -> str:
    h = hashlib.sha256()
    h.update(cook_spec.source_path.read_bytes())
    h.update(json.dumps(_cook_spec_json(cook_spec), sort_keys=True).encode())
    if robot_mjcf_path is not None:
        robot_path = Path(robot_mjcf_path).expanduser().resolve()
        h.update(robot_path.read_bytes())
    if meshdir is not None:
        h.update(str(Path(meshdir).expanduser().resolve()).encode())
    return h.hexdigest()[:_CACHE_KEY_LEN]


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
    parser = argparse.ArgumentParser(description="Cook a scene asset for DimOS simulators.")
    parser.add_argument("source", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--robot-mjcf", type=Path)
    parser.add_argument("--meshdir", type=Path)
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
        robot_mjcf_path=None if args.no_mujoco else args.robot_mjcf,
        meshdir=args.meshdir,
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
            enabled=not args.no_mujoco and args.robot_mjcf is not None,
            include_visual_mesh=args.include_mujoco_visual,
        ),
        rebake=args.rebake,
    )
    print(package.metadata_path)


if __name__ == "__main__":
    cli_main()


__all__ = ["SCENE_PACKAGE_CACHE_DIR", "cook_scene_package"]

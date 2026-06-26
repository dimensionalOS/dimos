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

"""Small reference consumer for cooked scene packages.

Run:

    python -m dimos.simulation.scenes.demo_scene_package_consumer office
    python -m dimos.simulation.scenes.demo_scene_package_consumer office --compile-g1-mujoco

The point of this module is to make the runtime contract concrete. A consumer
resolves a package, reads the artifact paths it understands, and then passes
those artifacts to its backend. It does not inspect raw source meshes or invoke
the cooking pipeline.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from dimos.simulation.scene_assets.spec import ScenePackage
from dimos.simulation.scenes.catalog import resolve_scene_package
from dimos.utils.data import LfsPath

_G1_ROBOT_MJCF = Path(__file__).resolve().parents[2] / "robot/unitree/g1/assets/g1_29dof.xml"
_G1_ROBOT_MESHDIR = LfsPath("g1_urdf/meshes")


def describe_scene_package(
    scene: str | Path | None,
    *,
    include_stats: bool = False,
) -> dict[str, Any]:
    """Return a JSON-serializable description of the resolved scene package."""
    package = resolve_scene_package(scene)
    if package is None:
        return {
            "scene": None if scene is None else str(scene),
            "enabled": False,
        }

    summary: dict[str, Any] = {
        "scene": str(scene),
        "enabled": True,
        "metadata_path": _path(package.metadata_path),
        "package_dir": _path(package.package_dir),
        "source_path": str(package.source_path),
        "alignment": {
            "scale": package.alignment.scale,
            "translation": package.alignment.translation,
            "rotation_zyx_deg": package.alignment.rotation_zyx_deg,
            "y_up": package.alignment.y_up,
        },
        "artifacts": {
            "browser_visual": _path(package.visual_path),
            "browser_collision": _path(package.browser_collision_path),
            "objects": _path(package.objects_path),
            "mujoco_scene": _path(package.mujoco_scene_path),
        },
        "entity_count": len(package.entities),
        "entity_ids": [
            str(entity.get("id", entity.get("name", "")))
            for entity in package.entities[:20]
            if entity.get("id") or entity.get("name")
        ],
        "stats_keys": sorted(str(key) for key in package.stats),
    }
    if include_stats:
        summary["stats"] = package.stats
    return summary


def g1_mujoco_config_for_scene(scene: str | Path) -> Any:
    """Build the minimal MuJoCo config needed to attach G1 to a scene package."""
    from dimos.simulation.engines.mujoco_sim_module import MujocoSimModuleConfig

    package = _require_scene_package(scene)
    if package.mujoco_scene_path is None:
        raise ValueError(f"scene package has no MuJoCo artifact: {package.metadata_path}")

    return MujocoSimModuleConfig(
        scene_xml=package.mujoco_scene_path,
        robot_mjcf=_G1_ROBOT_MJCF,
        robot_meshdir=_G1_ROBOT_MESHDIR,
        robot_id="",
        scene_entities=package.entities,
        headless=True,
        dof=29,
        enable_color=False,
        enable_depth=False,
        enable_pointcloud=False,
    )


def compile_g1_mujoco_model(scene: str | Path) -> dict[str, int]:
    """Compile G1 attached to a scene package and return basic model counts."""
    from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

    module: MujocoSimModule = object.__new__(MujocoSimModule)
    module.config = g1_mujoco_config_for_scene(scene)
    model = MujocoSimModule._compose_model(module)
    return {
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nbody": int(model.nbody),
        "ngeom": int(model.ngeom),
        "nmesh": int(model.nmesh),
    }


def _require_scene_package(scene: str | Path) -> ScenePackage:
    package = resolve_scene_package(scene)
    if package is None:
        raise ValueError(f"scene is disabled, not a package: {scene}")
    return package


def _path(path: Path | None) -> str | None:
    return None if path is None else str(path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", nargs="?", default="office")
    parser.add_argument(
        "--compile-g1-mujoco",
        action="store_true",
        help="also compile a MuJoCo model by attaching the robot-only G1 MJCF",
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="include full cook stats from scene.meta.json",
    )
    args = parser.parse_args()

    result = describe_scene_package(args.scene, include_stats=args.stats)
    if args.compile_g1_mujoco:
        result["g1_mujoco_compile"] = compile_g1_mujoco_model(args.scene)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

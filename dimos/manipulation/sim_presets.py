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

"""Simulation presets used by manipulation blueprints."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any

from dimos.robot.catalog.galaxea import R1PRO_SIM_MESHDIR, R1PRO_SIM_MJCF_PATH
from dimos.robot.catalog.ufactory import XARM7_SIM_PATH
from dimos.simulation.scene_assets.spec import ScenePackage, load_scene_package
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The R1Pro sim defaults to this scene package so `dimos run r1pro-...` works
# without setting DIMOS_SCENE_PACKAGE_PATH (it exists in the repo data dir).
_R1PRO_DEFAULT_SCENE = "data/scene_packages/dimos_office"


@dataclass(frozen=True)
class MujocoSimPreset:
    robot_config_kwargs: dict[str, Any]
    mujoco_module_kwargs: dict[str, Any]


_XARM7_LEGACY_HOME_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0, -0.7, 0.0)
_XARM7_SCENE_HOME_JOINTS = (0.0, -0.247, 0.0, 0.909, 0.0, 1.15644, 0.0)
_XARM7_SCENE_YAW = math.radians(90.0)
_XARM7_TABLE_STANDOFF_M = 0.45
_XARM7_MJCF_BASE_Z_OFFSET_M = 0.12


def xarm7_mujoco_scene_preset(
    scene_package_env: str = "DIMOS_SCENE_PACKAGE_PATH",
) -> MujocoSimPreset:
    package = _scene_package_from_env(scene_package_env)
    if package is None:
        return MujocoSimPreset(
            robot_config_kwargs={
                "address": str(XARM7_SIM_PATH),
                "base_pose": _base_pose((0.0, 0.0), 0.0, 0.0),
                "home_joints": list(_XARM7_LEGACY_HOME_JOINTS),
            },
            mujoco_module_kwargs={"address": str(XARM7_SIM_PATH)},
        )

    robot_mjcf = Path(str(XARM7_SIM_PATH)).parent / "xarm7.xml"
    spawn_xy, spawn_z = _xarm7_scene_spawn(package)
    return MujocoSimPreset(
        robot_config_kwargs={
            "address": str(robot_mjcf),
            "base_pose": _base_pose(spawn_xy, spawn_z, _XARM7_SCENE_YAW),
            "home_joints": list(_XARM7_SCENE_HOME_JOINTS),
        },
        mujoco_module_kwargs={
            "scene_xml": str(package.mujoco_scene_path),
            "robot_mjcf": str(robot_mjcf),
            "scene_entities": package.entities,
            "spawn_xy": spawn_xy,
            "spawn_z": spawn_z,
            "spawn_yaw": _XARM7_SCENE_YAW,
            "initial_joint_positions": list(_XARM7_SCENE_HOME_JOINTS),
            "render_geom_groups": (0, 1, 2, 3),
        },
    )


# R1Pro is a full-height floor-standing robot (shoulders ~1.45m); it leans at the
# torso to reach a desk. Home = all-zeros (arms hang, torso upright); tuned in
# later phases. Faces +Y toward the desk; base on the floor.
_R1PRO_SCENE_HOME_JOINTS = (0.0,) * 18  # 4 torso + 7 left arm + 7 right arm
_R1PRO_SCENE_YAW = math.radians(90.0)
_R1PRO_TABLE_STANDOFF_M = 0.5
_R1PRO_FLOOR_Z = 0.0


def r1pro_mujoco_scene_preset(
    scene_package_env: str = "DIMOS_SCENE_PACKAGE_PATH",
) -> MujocoSimPreset:
    """Dual-arm R1Pro spawned at the manipulation desk in a scene package.

    Import-safe: with no scene package (``DIMOS_SCENE_PACKAGE_PATH`` unset) this
    DEGRADES to a robot-only preset at the origin rather than raising — the same
    import-time tolerance the xArm preset has — so a module-level call here can't
    crash import of the whole blueprints module (and its xArm blueprints).
    """
    robot_mjcf = str(R1PRO_SIM_MJCF_PATH)
    # R1Pro grippers are direct position servos (ctrlrange == finger joint range),
    # not the xArm's inverted tendon scale.
    common_mujoco_kwargs = {
        "robot_mjcf": robot_mjcf,
        "robot_meshdir": str(R1PRO_SIM_MESHDIR),
        "initial_joint_positions": list(_R1PRO_SCENE_HOME_JOINTS),
        "gripper_ctrl_inverted": False,
        "render_geom_groups": (0, 1, 2, 3),
    }
    package = _r1pro_scene_package(scene_package_env)
    if package is None:
        return MujocoSimPreset(
            robot_config_kwargs={
                "address": robot_mjcf,
                "base_pose": _base_pose((0.0, 0.0), 0.0, 0.0),
                "home_joints": list(_R1PRO_SCENE_HOME_JOINTS),
            },
            mujoco_module_kwargs={"address": robot_mjcf, **common_mujoco_kwargs},
        )

    spawn_xy, spawn_z = _r1pro_scene_spawn(package)
    return MujocoSimPreset(
        robot_config_kwargs={
            "address": robot_mjcf,
            # NOTE: planning base_pose alignment with the spawned MuJoCo pose is
            # handled in the RoboPlan integration phase (Phase 4).
            "base_pose": _base_pose(spawn_xy, spawn_z, _R1PRO_SCENE_YAW),
            "home_joints": list(_R1PRO_SCENE_HOME_JOINTS),
        },
        mujoco_module_kwargs={
            "scene_xml": str(package.mujoco_scene_path),
            "scene_entities": package.entities,
            "spawn_xy": spawn_xy,
            "spawn_z": spawn_z,
            "spawn_yaw": _R1PRO_SCENE_YAW,
            **common_mujoco_kwargs,
        },
    )


def r1pro_scene_obstacles(
    scene_package_env: str = "DIMOS_SCENE_PACKAGE_PATH",
) -> list[dict[str, Any]]:
    """Static box-obstacle specs (desk + graspable objects) from the scene package,
    for seeding the planning world so they render in viser before perception runs.
    initial_pose is the object center and descriptor.extents the AABB size."""
    package = _r1pro_scene_package(scene_package_env)
    if package is None:
        return []
    obstacles: list[dict[str, Any]] = []
    for entity in package.entities:
        eid = str(entity.get("id", ""))
        tags = entity.get("tags", [])
        is_table = eid == "manip_table" or "table" in tags
        if not (is_table or (eid.startswith("manip_") and not is_table)):
            continue
        descriptor = entity.get("descriptor", {})
        extents = descriptor.get("extents")
        if not extents:
            continue
        pose = entity.get("initial_pose", {})
        rgba = descriptor.get("rgba") or (
            [0.55, 0.35, 0.15, 0.7] if is_table else [0.2, 0.7, 0.9, 0.9]
        )
        obstacles.append(
            {
                "name": eid,
                "position": [float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), float(pose.get("z", 0.0))],
                "dimensions": [float(extents[0]), float(extents[1]), float(extents[2])],
                "color": [float(c) for c in rgba],
            }
        )
    return obstacles


def _r1pro_scene_package(env_name: str) -> ScenePackage | None:
    """Load the R1Pro sim scene from the env var or the committed default, warning
    (not raising) if it can't be loaded so import + run stay robust and the
    robot-only fallback is never silent."""
    path = os.environ.get(env_name) or _R1PRO_DEFAULT_SCENE
    meta = Path(path).expanduser()
    if meta.is_dir():
        meta = meta / "scene.meta.json"
    if not meta.exists():
        logger.warning(
            f"R1Pro sim: scene package not found at '{meta}'; spawning robot-only at "
            f"the origin (no desk). Set {env_name} to a scene package directory."
        )
        return None
    try:
        package = load_scene_package(meta)
    except Exception as exc:  # noqa: BLE001 - degrade on any malformed package
        logger.warning(f"R1Pro sim: failed to load scene package '{meta}': {exc}; robot-only.")
        return None
    if package.mujoco_scene_path is None or not package.mujoco_scene_path.exists():
        logger.warning(f"R1Pro sim: scene package '{meta}' has no MuJoCo scene artifact; robot-only.")
        return None
    return package


def _r1pro_scene_spawn(package: ScenePackage) -> tuple[tuple[float, float], float]:
    table = _find_table_entity(package)
    pose = table.get("initial_pose", {})
    table_x = float(pose.get("x", 0.0))
    table_y = float(pose.get("y", 0.0))
    return ((table_x, table_y - _R1PRO_TABLE_STANDOFF_M), _R1PRO_FLOOR_Z)


def _scene_package_from_env(env_name: str) -> ScenePackage | None:
    path = os.environ.get(env_name)
    if not path:
        return None

    metadata_path = Path(path).expanduser()
    if metadata_path.is_dir():
        metadata_path = metadata_path / "scene.meta.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"{env_name} does not exist: {metadata_path}")

    package = load_scene_package(metadata_path)
    if package.mujoco_scene_path is None:
        raise ValueError(f"Scene package has no MuJoCo scene artifact: {metadata_path}")
    if not package.mujoco_scene_path.exists():
        raise FileNotFoundError(
            f"Scene package MuJoCo scene artifact does not exist: {package.mujoco_scene_path}"
        )
    return package


def _xarm7_scene_spawn(package: ScenePackage) -> tuple[tuple[float, float], float]:
    table = _find_table_entity(package)
    pose = table.get("initial_pose", {})
    descriptor = table.get("descriptor", {})
    extents = descriptor.get("extents") or [0.0, 0.0, 0.0]
    table_x = float(pose.get("x", 0.0))
    table_y = float(pose.get("y", 0.0))
    table_z = float(pose.get("z", 0.0))
    table_top_z = table_z + float(extents[2]) / 2.0
    return (
        (table_x, table_y - _XARM7_TABLE_STANDOFF_M),
        table_top_z - _XARM7_MJCF_BASE_Z_OFFSET_M,
    )


def _find_table_entity(package: ScenePackage) -> dict[str, Any]:
    for entity in package.entities:
        if entity.get("id") == "manip_table" or "table" in entity.get("tags", []):
            return entity
    raise ValueError(f"Scene package has no manipulation table entity: {package.metadata_path}")


def _base_pose(xy: tuple[float, float], z: float, yaw: float) -> list[float]:
    return [
        xy[0],
        xy[1],
        z,
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    ]


__all__ = [
    "MujocoSimPreset",
    "r1pro_mujoco_scene_preset",
    "xarm7_mujoco_scene_preset",
]

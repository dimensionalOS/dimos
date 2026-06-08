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

from dimos.robot.catalog.ufactory import XARM7_SIM_PATH
from dimos.simulation.scene_assets.spec import ScenePackage, load_scene_package


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


__all__ = ["MujocoSimPreset", "xarm7_mujoco_scene_preset"]

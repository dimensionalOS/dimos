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

"""G1 GROOT WBC target backed by MuJoCo during development."""

from __future__ import annotations

import logging
import os
from pathlib import Path

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    return default if raw is None or raw == "" else float(raw)


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.lower() in {"1", "true", "yes", "on"}


def _env_xyz(name: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    parts = [float(p.strip()) for p in raw.split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} must be 'x,y,z'")
    return (parts[0], parts[1], parts[2])


def _command_center_blueprints() -> list[Blueprint]:
    if not _env_bool("DIMOS_ENABLE_COMMAND_CENTER", True):
        return []
    try:
        from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
    except ModuleNotFoundError as exc:
        if exc.name not in {"socketio", "fastapi", "uvicorn", "starlette"}:
            raise

        logging.getLogger(__name__).warning(
            "Command Center unavailable; install the web extra to enable WASD controls"
        )
        return []
    return [
        WebsocketVisModule.blueprint(port=int(os.environ.get("DIMOS_COMMAND_CENTER_PORT", "7779")))
    ]


_repo_root = Path(__file__).resolve().parents[6]
_default_mjcf = _repo_root / "data/mujoco_sim/g1_gear_wbc.xml"
_default_scene_mesh = _repo_root / "data/dimos_office_mesh/dimos_office_mesh.glb"
_mjcf_path = os.environ.get("DIMOS_MJCF_PATH") or str(_default_mjcf)
_mjcf_meshdir = os.environ.get("DIMOS_MJCF_MESHDIR")
if _mjcf_meshdir is None and Path(_mjcf_path).expanduser().resolve() == _default_mjcf:
    _mjcf_meshdir = str(_repo_root / "data/g1_urdf/meshes")
_dof = int(os.environ.get("DIMOS_MUJOCO_DOF", "29"))
_scene_mesh_path_override = os.environ.get("DIMOS_SCENE_MESH_PATH") or None
_scene_mesh_path = _scene_mesh_path_override or (
    str(_default_scene_mesh) if _default_scene_mesh.exists() else None
)
_scene_mesh_scale = _env_float(
    "DIMOS_SCENE_MESH_SCALE",
    0.05 if _scene_mesh_path_override else 2.0,
)
_scene_mesh_y_up = _env_bool("DIMOS_SCENE_MESH_Y_UP", bool(_scene_mesh_path_override))
_scene_mesh_rotation = _env_xyz("DIMOS_SCENE_MESH_ROTATION_ZYX_DEG", (0.0, 0.0, 0.0))
_scene_mesh_translation = _env_xyz("DIMOS_SCENE_MESH_TRANSLATION", (0.0, 0.0, 0.0))
_scene_mesh_collision = _env_bool("DIMOS_SCENE_MESH_COLLISION", True)
_scene_mesh_visual = _env_bool("DIMOS_SCENE_MESH_VISUAL", False)
_scene_mesh_bake = _env_bool("DIMOS_SCENE_MESH_BAKE", False)
_enable_depth_cloud = _env_bool("DIMOS_ENABLE_DEPTH_CLOUD", False)
_disable_lidar = _env_bool("DIMOS_DISABLE_LIDAR", False)

_sim_mjcf_path = _mjcf_path
if _scene_mesh_path and _scene_mesh_collision:
    try:
        from dimos.simulation.mujoco.mesh_scene import SceneMeshAlignment
        from dimos.simulation.mujoco.scene_mesh_to_mjcf import load_or_bake

        _, _scene_wrapper = load_or_bake(
            scene_mesh_path=_scene_mesh_path,
            robot_mjcf_path=_mjcf_path,
            alignment=SceneMeshAlignment(
                scale=_scene_mesh_scale,
                rotation_zyx_deg=_scene_mesh_rotation,
                translation=_scene_mesh_translation,
                y_up=_scene_mesh_y_up,
            ),
            meshdir=_mjcf_meshdir,
            include_visual_mesh=_scene_mesh_visual,
            rebake=_scene_mesh_bake,
        )
        _scene_mjb_path = _scene_wrapper.with_name("compiled.mjb")
        _sim_mjcf_path = str(_scene_mjb_path if _scene_mjb_path.exists() else _scene_wrapper)
    except Exception as exc:
        logging.getLogger(__name__).warning(
            "Failed to bake scene mesh into MuJoCo; lidar will only see the base MJCF: %s",
            exc,
        )

g1_groot_wbc = autoconnect(
    MujocoSimModule.blueprint(
        address=_sim_mjcf_path,
        meshdir=_mjcf_meshdir,
        headless=_env_bool("DIMOS_MUJOCO_HEADLESS", True),
        dof=_dof,
        camera_name=os.environ.get("DIMOS_MUJOCO_CAMERA", "head_color"),
        enable_color=False,
        enable_depth=_enable_depth_cloud,
        enable_pointcloud=(not _disable_lidar) or _enable_depth_cloud,
        pointcloud_fps=_env_float("DIMOS_POINTCLOUD_FPS", 2.0),
        lidar_camera_names=(
            []
            if _disable_lidar
            else ["lidar_front_camera", "lidar_left_camera", "lidar_right_camera"]
        ),
        lidar_camera_width=int(os.environ.get("DIMOS_LIDAR_CAMERA_WIDTH", "640")),
        lidar_camera_height=int(os.environ.get("DIMOS_LIDAR_CAMERA_HEIGHT", "360")),
        lidar_voxel_size=_env_float("DIMOS_LIDAR_VOXEL_SIZE", 0.05),
        enable_kinematic_base_control=_env_bool("DIMOS_KINEMATIC_BASE_CONTROL", True),
        enable_kinematic_joint_hold=_env_bool("DIMOS_MUJOCO_KINEMATIC_JOINT_HOLD", True),
    ),
    VoxelGridMapper.blueprint(
        voxel_size=_env_float("DIMOS_GLOBAL_MAP_VOXEL_SIZE", 0.05),
    ),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    *_command_center_blueprints(),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/mujoco/joint_state", JointState),
        ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        ("tele_cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("nav_cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("pointcloud", PointCloud2): LCMTransport("/lidar", PointCloud2),
        ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
        ("global_map", PointCloud2): LCMTransport("/global_map", PointCloud2),
        ("global_costmap", OccupancyGrid): LCMTransport("/global_costmap", OccupancyGrid),
        ("path", PathMsg): LCMTransport("/nav_path", PathMsg),
        ("clicked_point", PointStamped): LCMTransport("/clicked_point", PointStamped),
        ("goal_request", PoseStamped): LCMTransport("/goal_request", PoseStamped),
        ("stop_movement", Bool): LCMTransport("/stop_movement", Bool),
        ("point_goal", PointStamped): LCMTransport("/point_goal", PointStamped),
    }
)

__all__ = ["g1_groot_wbc"]

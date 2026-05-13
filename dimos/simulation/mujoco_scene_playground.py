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

"""MuJoCo scene playground with optional Viser mesh/splat visualization."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointcloud_map_module import PointCloudMapModule
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.viser.viser_render_module import ViserRenderModule


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    return default if raw is None or raw == "" else float(raw)


def _env_xyz(name: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    parts = [float(p.strip()) for p in raw.split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} must be 'x,y,z'")
    return (parts[0], parts[1], parts[2])


def _command_center_blueprints() -> list[Blueprint]:
    if os.environ.get("DIMOS_ENABLE_COMMAND_CENTER", "1") in {"", "0"}:
        return []
    try:
        from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
    except ModuleNotFoundError:
        import logging

        logging.getLogger(__name__).warning(
            "Command Center unavailable; install the web extra to enable WASD controls"
        )
        return []
    return [
        WebsocketVisModule.blueprint(port=int(os.environ.get("DIMOS_COMMAND_CENTER_PORT", "7779")))
    ]


_default_mjcf = Path("data/mujoco_sim/g1_gear_wbc.xml")
_default_scene_mesh = Path("data/dimos_office_mesh/dimos_office_mesh.glb")
_default_splat = Path("data/dimos_office/dimos_office.ply")
_mjcf_path = os.environ.get("DIMOS_MJCF_PATH") or str(_default_mjcf)
_mjcf_meshdir = os.environ.get("DIMOS_MJCF_MESHDIR")
if _mjcf_meshdir is None and Path(_mjcf_path) == _default_mjcf:
    _mjcf_meshdir = "data/g1_urdf/meshes"
_dof = int(os.environ.get("DIMOS_MUJOCO_DOF", "29"))
_scene_mesh_path_override = os.environ.get("DIMOS_SCENE_MESH_PATH") or None
_scene_mesh_path = _scene_mesh_path_override or (
    str(_default_scene_mesh) if _default_scene_mesh.exists() else None
)
_scene_mesh_scale = _env_float(
    "DIMOS_SCENE_MESH_SCALE",
    0.05 if _scene_mesh_path_override else 2.0,
)
_scene_mesh_y_up = (
    os.environ.get(
        "DIMOS_SCENE_MESH_Y_UP",
        "1" if _scene_mesh_path_override else "0",
    )
    != "0"
)
_scene_mesh_rotation = _env_xyz("DIMOS_SCENE_MESH_ROTATION_ZYX_DEG", (0.0, 0.0, 0.0))
_scene_mesh_translation = _env_xyz("DIMOS_SCENE_MESH_TRANSLATION", (0.0, 0.0, 0.0))
_scene_mesh_collision = os.environ.get("DIMOS_SCENE_MESH_COLLISION", "1") not in {"", "0"}
_splat_path_override = os.environ.get("DIMOS_SPLAT_PATH") or None
_splat_path = _splat_path_override or (
    str(_default_splat) if _scene_mesh_path_override is None and _default_splat.exists() else None
)
_splat_alignment_yaml = os.environ.get("DIMOS_SPLAT_ALIGNMENT")
if _splat_alignment_yaml is None and _splat_path == str(_default_splat):
    _office_splat_yaml = Path(tempfile.gettempdir()) / "dimos_office_to_artist_mesh.yaml"
    _office_splat_yaml.write_text(
        "# Maps the original Y-up dimos_office.ply into the artist mesh frame.\n"
        f"scale: {_scene_mesh_scale}\n"
        f"translation: [0.0, 0.0, {0.7734 * _scene_mesh_scale}]\n"
        "rotation_zyx: [164.6633, -0.0865, -95.4786]\n"
        "y_up: false\n"
    )
    _splat_alignment_yaml = str(_office_splat_yaml)
_enable_depth_cloud = os.environ.get("DIMOS_ENABLE_DEPTH_CLOUD", "0").lower() in {
    "1",
    "true",
    "yes",
    "on",
}
_disable_lidar = os.environ.get("DIMOS_DISABLE_LIDAR", "0") not in {"", "0"}

_sim_mjcf_path = _mjcf_path
if _scene_mesh_path and _scene_mesh_collision:
    try:
        from dimos.mapping.mesh_scene import SceneMeshAlignment
        from dimos.mapping.usdz_to_mjcf import bake_scene_mjcf

        _sim_mjcf_path = str(
            bake_scene_mjcf(
                scene_mesh_path=_scene_mesh_path,
                robot_mjcf_path=_mjcf_path,
                alignment=SceneMeshAlignment(
                    scale=_scene_mesh_scale,
                    rotation_zyx_deg=_scene_mesh_rotation,
                    translation=_scene_mesh_translation,
                    y_up=_scene_mesh_y_up,
                ),
                meshdir=_mjcf_meshdir,
            )
        )
    except Exception as exc:
        import logging

        logging.getLogger(__name__).warning(
            "Failed to bake scene mesh into MuJoCo; lidar will only see the base MJCF: %s",
            exc,
        )

mujoco_scene_playground = autoconnect(
    MujocoSimModule.blueprint(
        address=_sim_mjcf_path,
        meshdir=_mjcf_meshdir,
        headless=True,
        dof=_dof,
        camera_name=os.environ.get("DIMOS_MUJOCO_CAMERA", "head_color"),
        enable_color=False,
        enable_depth=True,
        enable_pointcloud=(not _disable_lidar) or _enable_depth_cloud,
        lidar_camera_names=(
            []
            if _disable_lidar
            else ["lidar_front_camera", "lidar_left_camera", "lidar_right_camera"]
        ),
        lidar_camera_width=int(os.environ.get("DIMOS_LIDAR_CAMERA_WIDTH", "640")),
        lidar_camera_height=int(os.environ.get("DIMOS_LIDAR_CAMERA_HEIGHT", "360")),
        lidar_voxel_size=_env_float("DIMOS_LIDAR_VOXEL_SIZE", 0.05),
        enable_kinematic_base_control=os.environ.get("DIMOS_KINEMATIC_BASE_CONTROL", "1")
        not in {"", "0"},
    ),
    MovementManager.blueprint(),
    PointCloudMapModule.blueprint(
        voxel_size=_env_float("DIMOS_GLOBAL_MAP_VOXEL_SIZE", 0.05),
        publish_interval=_env_float("DIMOS_GLOBAL_MAP_PUBLISH_INTERVAL", 0.5),
    ),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    ViserRenderModule.blueprint(
        splat_path=_splat_path,
        mjcf_path=_sim_mjcf_path,
        mjcf_meshdir=_mjcf_meshdir,
        port=int(os.environ.get("DIMOS_VISER_PORT", "8082")),
        alignment_yaml=_splat_alignment_yaml,
        scene_mesh_path=_scene_mesh_path,
        scene_mesh_scale=_scene_mesh_scale,
        scene_mesh_translation=_scene_mesh_translation,
        scene_mesh_rotation_zyx_deg=_scene_mesh_rotation,
        scene_mesh_y_up=_scene_mesh_y_up,
        pointcloud_point_size=_env_float("DIMOS_POINTCLOUD_POINT_SIZE", 0.04),
    ),
    *_command_center_blueprints(),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/mujoco/joint_state", JointState),
        ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        ("tele_cmd_vel", Twist): LCMTransport("/tele_cmd_vel", Twist),
        ("nav_cmd_vel", Twist): LCMTransport("/nav_cmd_vel", Twist),
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("pointcloud", PointCloud2): LCMTransport("/lidar", PointCloud2),
        ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
        ("global_map", PointCloud2): LCMTransport("/global_map", PointCloud2),
        ("pointcloud_overlay", PointCloud2): LCMTransport("/global_map", PointCloud2),
        ("global_costmap", OccupancyGrid): LCMTransport("/global_costmap", OccupancyGrid),
        ("path", PathMsg): LCMTransport("/nav_path", PathMsg),
        ("clicked_point", PointStamped): LCMTransport("/clicked_point", PointStamped),
        ("goal", PointStamped): LCMTransport("/goal", PointStamped),
        ("goal_request", PoseStamped): LCMTransport("/goal_request", PoseStamped),
        ("stop_movement", Bool): LCMTransport("/stop_movement", Bool),
        ("point_goal", PointStamped): LCMTransport("/point_goal", PointStamped),
    }
)

__all__ = ["mujoco_scene_playground"]

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

"""G1 GR00T whole-body-control blueprint.

``dimos --simulation mujoco run g1-groot-wbc`` uses MuJoCo as the whole-body
backend and opens the Babylon viewer with the default ``dimos-office`` scene.
Pass ``--scene <name-or-scene.meta.json>`` to select a cooked scene package;
``--scene none`` starts the bare robot. ``dimos run g1-groot-wbc`` uses the
real G1 DDS connection. The coordinator/task stack is shared; only the
hardware adapter and sim-only modules are gated by configuration.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import logging
import os
from pathlib import Path
import shutil
from typing import Any

from dimos_lcm.std_msgs import Bool

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.g1_groot_wbc_task import (
    ARM_DEFAULT_POSE,
    G1_GROOT_DEFAULT_POSITIONS,
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.experimental.pimsim.entity import EntityStateBatch
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.data import LfsPath
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

logger = logging.getLogger(__name__)

_REPO_ROOT = Path(__file__).resolve().parents[6]
_GROOT_MODEL_DIR = LfsPath("groot")
_MJCF_PATH = LfsPath("mujoco_sim/g1_gear_wbc.xml")
_G1_MESH_DIR = _REPO_ROOT / "data/g1_urdf/meshes"

_SIM_DOF = 29
_SIM_TICK_RATE_HZ = 50.0
_REAL_TICK_RATE_HZ = 500.0
_REAL_ARM_RAMP_SECONDS = 10.0
_SIM_POLICY_DECIMATION = 1
_DEFAULT_COMMAND_CENTER_PORT = 7779
_DEFAULT_BABYLON_PORT = 8091
_DEFAULT_POINTCLOUD_FPS = 2.0
_DEFAULT_LIDAR_VOXEL_SIZE_M = 0.05
_DEFAULT_LIDAR_CAMERA_WIDTH = 640
_DEFAULT_LIDAR_CAMERA_HEIGHT = 360
_DEFAULT_GLOBAL_MAP_VOXEL_SIZE_M = 0.05
_DEFAULT_G1_SPAWN_Z_M = 0.793
_MID360_FRAME_ID = "lidar_link"
_MID360_POINT_RATE = 200_000
_MID360_MIN_RANGE_M = 0.1
_MID360_MAX_RANGE_M = 40.0
_MID360_ELEVATION_MIN_DEG = -7.0
_MID360_ELEVATION_MAX_DEG = 52.0
_MID360_SENSOR_X_M = 0.0002835
_MID360_SENSOR_Y_M = 0.00003
_MID360_SENSOR_Z_M = 0.41618
_MID360_SENSOR_ROLL_DEG = 180.0
_MID360_SENSOR_PITCH_DEG = 2.300348633011322
_MID360_SENSOR_YAW_DEG = 0.0
_RAYTRACE_EXECUTABLE_PATH = (
    _REPO_ROOT / "dimos/mapping/ray_tracing/rust/target/release/voxel_ray_tracing"
)
_SCENE_LIDAR_EXECUTABLE_PATH = (
    _REPO_ROOT / "dimos/simulation/sensors/rust/scene_lidar/target/release/scene_lidar"
)


@dataclass(frozen=True)
class _BackendSelection:
    blueprint: Blueprint
    adapter_type: str
    adapter_address: str | Path
    viewer_mjcf_path: str | Path
    tick_rate: float
    auto_arm: bool
    auto_dry_run: bool
    default_ramp_seconds: float
    decimation: int | None
    arm_holder: TaskConfig | None


def _arm_holder_config() -> TaskConfig:
    return TaskConfig(
        name="servo_arms",
        type="servo",
        joint_names=g1_arms,
        priority=10,
        auto_start=True,
        params={"default_positions": ARM_DEFAULT_POSE},
    )


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    return default if raw is None or raw == "" else float(raw)


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    return default if raw is None or raw == "" else int(raw)


def _babylon_enabled() -> bool:
    return _env_bool("DIMOS_ENABLE_BABYLON", bool(global_config.simulation))


def _raytrace_mapper_available() -> bool:
    return _RAYTRACE_EXECUTABLE_PATH.exists() or shutil.which("cargo") is not None


def _cargo_executable() -> str | None:
    cargo = shutil.which("cargo")
    if cargo is not None:
        return cargo
    cargo_home = Path.home() / ".cargo/bin/cargo"
    return str(cargo_home) if cargo_home.exists() else None


def _native_scene_lidar_available() -> bool:
    return _SCENE_LIDAR_EXECUTABLE_PATH.exists() or _cargo_executable() is not None


def _native_scene_lidar_build_command() -> str | None:
    cargo = _cargo_executable()
    return f"{cargo} build --release" if cargo is not None else None


@lru_cache(maxsize=1)
def _scene_package_config() -> Any | None:
    scene = os.environ.get("DIMOS_SCENE_PACKAGE_PATH") or global_config.scene

    from dimos.simulation.scenes.catalog import resolve_scene_package

    return resolve_scene_package(
        scene,
        robot_mjcf_path=_MJCF_PATH,
        meshdir=_G1_MESH_DIR,
    )


def _native_scene_lidar_enabled(scene_package: Any | None, lidar_disabled: bool) -> bool:
    if lidar_disabled or scene_package is None or scene_package.browser_collision_path is None:
        return False
    if not _env_bool("DIMOS_ENABLE_NATIVE_SCENE_LIDAR", True):
        return False
    if _native_scene_lidar_available():
        return True
    logger.warning(
        "Native scene lidar unavailable; falling back to MuJoCo depth lidar. "
        "Install cargo or build %s to enable it.",
        _SCENE_LIDAR_EXECUTABLE_PATH,
    )
    return False


def _select_backend() -> _BackendSelection:
    if not global_config.simulation:
        return _BackendSelection(
            blueprint=G1WholeBodyConnection.blueprint(release_sport_mode=True),
            adapter_type="transport_lcm",
            adapter_address="",
            viewer_mjcf_path=_MJCF_PATH,
            tick_rate=_REAL_TICK_RATE_HZ,
            auto_arm=False,
            auto_dry_run=True,
            default_ramp_seconds=_REAL_ARM_RAMP_SECONDS,
            decimation=None,
            arm_holder=_arm_holder_config(),
        )

    scene_package = _scene_package_config()
    if scene_package is not None and scene_package.mujoco_model_path is not None:
        sim_mjcf_path = scene_package.mujoco_model_path
        viewer_mjcf_path = scene_package.mujoco_wrapper_path or _MJCF_PATH
    else:
        sim_mjcf_path, viewer_mjcf_path = _MJCF_PATH, _MJCF_PATH
    lidar_disabled = _env_bool("DIMOS_DISABLE_LIDAR", False)
    depth_cloud_enabled = _env_bool("DIMOS_ENABLE_DEPTH_CLOUD", False)
    native_scene_lidar_enabled = _native_scene_lidar_enabled(scene_package, lidar_disabled)

    from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

    backend = MujocoSimModule.blueprint(
        address=sim_mjcf_path,
        meshdir=str(_G1_MESH_DIR),
        headless=_env_bool("DIMOS_MUJOCO_HEADLESS", True),
        dof=_SIM_DOF,
        camera_name=os.environ.get("DIMOS_MUJOCO_CAMERA", "head_color"),
        enable_color=False,
        enable_depth=depth_cloud_enabled,
        enable_pointcloud=depth_cloud_enabled
        or ((not lidar_disabled) and not native_scene_lidar_enabled),
        pointcloud_fps=_env_float("DIMOS_POINTCLOUD_FPS", _DEFAULT_POINTCLOUD_FPS),
        lidar_camera_names=(
            []
            if lidar_disabled or native_scene_lidar_enabled
            else ["lidar_front_camera", "lidar_left_camera", "lidar_right_camera"]
        ),
        renderer_max_geom=_env_int("DIMOS_MUJOCO_RENDERER_MAX_GEOM", 0),
        lidar_camera_width=_env_int("DIMOS_LIDAR_CAMERA_WIDTH", _DEFAULT_LIDAR_CAMERA_WIDTH),
        lidar_camera_height=_env_int("DIMOS_LIDAR_CAMERA_HEIGHT", _DEFAULT_LIDAR_CAMERA_HEIGHT),
        lidar_voxel_size=_env_float("DIMOS_LIDAR_VOXEL_SIZE", _DEFAULT_LIDAR_VOXEL_SIZE_M),
        enable_kinematic_base_control=_env_bool("DIMOS_KINEMATIC_BASE_CONTROL", False),
        enable_kinematic_joint_hold=_env_bool("DIMOS_MUJOCO_KINEMATIC_JOINT_HOLD", False),
        inject_legacy_assets=True,
        spawn_xy=global_config.mujoco_start_pos_float,
        spawn_z=_env_float("DIMOS_MUJOCO_START_Z", _DEFAULT_G1_SPAWN_Z_M),
        reset_joint_positions=G1_GROOT_DEFAULT_POSITIONS,
    )
    return _BackendSelection(
        blueprint=backend,
        adapter_type="sim_mujoco_g1",
        adapter_address=sim_mjcf_path,
        viewer_mjcf_path=viewer_mjcf_path,
        tick_rate=_SIM_TICK_RATE_HZ,
        auto_arm=True,
        auto_dry_run=False,
        default_ramp_seconds=0.0,
        decimation=_SIM_POLICY_DECIMATION,
        arm_holder=_arm_holder_config(),
    )


def _coordinator_blueprint(selection: _BackendSelection) -> tuple[Blueprint, str]:
    cmd_vel_topic = "/cmd_vel" if global_config.simulation else "/g1/cmd_vel"
    task_configs = [
        TaskConfig(
            name="groot_wbc",
            type="g1_groot_wbc",
            joint_names=g1_legs_waist,
            priority=50,
            auto_start=True,
            params={
                "model_path": _GROOT_MODEL_DIR,
                "hardware_id": "g1",
                "auto_arm": selection.auto_arm,
                "auto_dry_run": selection.auto_dry_run,
                "default_ramp_seconds": selection.default_ramp_seconds,
                "decimation": selection.decimation,
            },
        ),
        *([selection.arm_holder] if selection.arm_holder is not None else []),
    ]

    coordinator = ControlCoordinator.blueprint(
        tick_rate=selection.tick_rate,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[
            HardwareComponent(
                hardware_id="g1",
                hardware_type=HardwareType.WHOLE_BODY,
                joints=g1_joints,
                adapter_type=selection.adapter_type,
                address=selection.adapter_address,
                auto_enable=True,
                wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
            ),
        ],
        tasks=task_configs,
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
            ("twist_command", Twist): LCMTransport(cmd_vel_topic, Twist),
            ("motor_states", JointState): LCMTransport("/g1/motor_states", JointState),
            ("imu", Imu): LCMTransport("/g1/imu", Imu),
            ("motor_command", MotorCommandArray): LCMTransport(
                "/g1/motor_command", MotorCommandArray
            ),
        }
    )
    return coordinator, cmd_vel_topic


def _websocket_blueprint(cmd_vel_topic: str) -> Blueprint:
    return WebsocketVisModule.blueprint(
        port=_env_int("DIMOS_COMMAND_CENTER_PORT", _DEFAULT_COMMAND_CENTER_PORT)
    ).transports(
        {
            ("tele_cmd_vel", Twist): LCMTransport(cmd_vel_topic, Twist),
        }
    )


def _sim_support_blueprints() -> tuple[Blueprint, ...]:
    if not global_config.simulation:
        return ()

    from dimos.mapping.costmapper import CostMapper
    from dimos.mapping.ray_tracing.module import PoseStampedToOdometry, RayTracingVoxelMap
    from dimos.mapping.static_costmap import StaticCostmapModule
    from dimos.mapping.voxels import VoxelGridMapper
    from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner

    lidar_disabled = _env_bool("DIMOS_DISABLE_LIDAR", False)
    scene_package = _scene_package_config()
    native_scene_lidar_enabled = _native_scene_lidar_enabled(scene_package, lidar_disabled)
    global_map_voxel_size = _env_float(
        "DIMOS_GLOBAL_MAP_VOXEL_SIZE", _DEFAULT_GLOBAL_MAP_VOXEL_SIZE_M
    )
    map_backend = os.environ.get("DIMOS_GLOBAL_MAP_BACKEND", "raytrace").lower()
    raytrace_mapper_available = _raytrace_mapper_available()
    scene_lidar_publish_sensor_frame = _env_bool(
        "DIMOS_SCENE_LIDAR_SENSOR_FRAME",
        map_backend == "raytrace" and raytrace_mapper_available,
    )
    scene_lidar_scan_model = os.environ.get("DIMOS_SCENE_LIDAR_SCAN_MODEL", "mid360")
    scene_lidar_frame_id = os.environ.get("DIMOS_SCENE_LIDAR_FRAME_ID", _MID360_FRAME_ID)
    scene_lidar_sensor_x = _env_float("DIMOS_SCENE_LIDAR_SENSOR_X", _MID360_SENSOR_X_M)
    scene_lidar_sensor_y = _env_float("DIMOS_SCENE_LIDAR_SENSOR_Y", _MID360_SENSOR_Y_M)
    scene_lidar_sensor_z = _env_float("DIMOS_SCENE_LIDAR_SENSOR_Z", _MID360_SENSOR_Z_M)
    scene_lidar_sensor_roll = _env_float(
        "DIMOS_SCENE_LIDAR_SENSOR_ROLL_DEG", _MID360_SENSOR_ROLL_DEG
    )
    scene_lidar_sensor_pitch = _env_float(
        "DIMOS_SCENE_LIDAR_SENSOR_PITCH_DEG", _MID360_SENSOR_PITCH_DEG
    )
    scene_lidar_sensor_yaw = _env_float("DIMOS_SCENE_LIDAR_SENSOR_YAW_DEG", _MID360_SENSOR_YAW_DEG)

    lidar_stack: tuple[Blueprint, ...] = ()
    if native_scene_lidar_enabled:
        from dimos.simulation.sensors.scene_lidar import SceneLidarModule

        assert scene_package is not None
        lidar_stack = (
            SceneLidarModule.blueprint(
                build_command=_native_scene_lidar_build_command(),
                scene_metadata_path=str(scene_package.metadata_path),
                collision_path=str(scene_package.browser_collision_path),
                scan_model=scene_lidar_scan_model,
                frame_id=scene_lidar_frame_id,
                publish_sensor_frame=scene_lidar_publish_sensor_frame,
                hz=_env_float("DIMOS_SCENE_LIDAR_HZ", 10.0),
                point_rate=_env_int("DIMOS_SCENE_LIDAR_POINT_RATE", _MID360_POINT_RATE),
                horizontal_samples=_env_int("DIMOS_SCENE_LIDAR_HORIZONTAL_SAMPLES", 720),
                vertical_samples=_env_int("DIMOS_SCENE_LIDAR_VERTICAL_SAMPLES", 16),
                elevation_min_deg=_env_float(
                    "DIMOS_SCENE_LIDAR_ELEVATION_MIN_DEG", _MID360_ELEVATION_MIN_DEG
                ),
                elevation_max_deg=_env_float(
                    "DIMOS_SCENE_LIDAR_ELEVATION_MAX_DEG", _MID360_ELEVATION_MAX_DEG
                ),
                min_range=_env_float("DIMOS_SCENE_LIDAR_MIN_RANGE", _MID360_MIN_RANGE_M),
                max_range=_env_float("DIMOS_SCENE_LIDAR_MAX_RANGE", _MID360_MAX_RANGE_M),
                sensor_x=scene_lidar_sensor_x,
                sensor_y=scene_lidar_sensor_y,
                sensor_z=scene_lidar_sensor_z,
                sensor_roll_deg=scene_lidar_sensor_roll,
                sensor_pitch_deg=scene_lidar_sensor_pitch,
                sensor_yaw_deg=scene_lidar_sensor_yaw,
                yaw_offset_deg=_env_float("DIMOS_SCENE_LIDAR_YAW_OFFSET_DEG", 0.0),
                output_voxel_size=_env_float("DIMOS_SCENE_LIDAR_OUTPUT_VOXEL_SIZE", 0.03),
                support_floor=_env_bool(
                    "DIMOS_SCENE_LIDAR_SUPPORT_FLOOR",
                    global_config.simulation in ("babylon", "pimsim"),
                ),
                support_floor_z=_env_float("DIMOS_SCENE_SUPPORT_FLOOR_Z", 0.0),
                support_floor_size=_env_float("DIMOS_SCENE_SUPPORT_FLOOR_SIZE", 0.0),
            ).transports(
                {
                    ("pose", PoseStamped): LCMTransport("/odom", PoseStamped),
                    ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
                    # Dynamic-entity batch from BabylonSceneViewerModule
                    # (Add button in HUD spawns; rust lidar folds entity
                    # primitives into per-ray analytical intersections).
                    ("entity_states", EntityStateBatch): LCMTransport(
                        "/entity_state_batch", EntityStateBatch
                    ),
                }
            ),
        )

    if lidar_disabled or scene_package is None:
        mapping_stack: tuple[Blueprint, ...] = (StaticCostmapModule.blueprint(),)
    elif map_backend in {"voxel", "python"}:
        mapping_stack = (
            VoxelGridMapper.blueprint(voxel_size=global_map_voxel_size).transports(
                {("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2)}
            ),
            CostMapper.blueprint(),
        )
    elif not raytrace_mapper_available:
        logger.warning(
            "Rust ray-tracing mapper unavailable; falling back to Python VoxelGridMapper. "
            "Install cargo or build %s to enable it.",
            _RAYTRACE_EXECUTABLE_PATH,
        )
        mapping_stack = (
            VoxelGridMapper.blueprint(voxel_size=global_map_voxel_size).transports(
                {("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2)}
            ),
            CostMapper.blueprint(),
        )
    else:
        mapping_stack = (
            PoseStampedToOdometry.blueprint().transports(
                {
                    ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
                    ("odometry", Odometry): LCMTransport("/odometry", Odometry),
                }
            ),
            RayTracingVoxelMap.blueprint(
                voxel_size=global_map_voxel_size,
                max_range=_env_float("DIMOS_RAYTRACE_MAX_RANGE", 30.0),
                ray_subsample=_env_int("DIMOS_RAYTRACE_SUBSAMPLE", 1),
                shadow_depth=_env_float("DIMOS_RAYTRACE_SHADOW_DEPTH", 0.2),
                grace_depth=_env_float("DIMOS_RAYTRACE_GRACE_DEPTH", 0.2),
                min_health=_env_int("DIMOS_RAYTRACE_MIN_HEALTH", -2),
                max_health=_env_int("DIMOS_RAYTRACE_MAX_HEALTH", 1),
                transform_sensor_frame=_env_bool(
                    "DIMOS_RAYTRACE_TRANSFORM_SENSOR_FRAME",
                    native_scene_lidar_enabled and scene_lidar_publish_sensor_frame,
                ),
                sensor_x=scene_lidar_sensor_x,
                sensor_y=scene_lidar_sensor_y,
                sensor_z=scene_lidar_sensor_z,
                sensor_roll_deg=scene_lidar_sensor_roll,
                sensor_pitch_deg=scene_lidar_sensor_pitch,
                sensor_yaw_deg=scene_lidar_sensor_yaw,
            ).transports(
                {
                    ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
                    ("odometry", Odometry): LCMTransport("/odometry", Odometry),
                    ("global_map", PointCloud2): LCMTransport("/global_map", PointCloud2),
                }
            ),
            CostMapper.blueprint(),
        )

    return (
        *lidar_stack,
        *mapping_stack,
        ReplanningAStarPlanner.blueprint(),
    )


def _babylon_blueprint(viewer_mjcf_path: str | Path, cmd_vel_topic: str) -> Blueprint | None:
    """Build the BabylonSceneViewerModule blueprint for either backend.

    Sim mode optionally overlays a scene-mesh visual; real mode uses the bare
    G1 MJCF. Simulation starts Babylon by default; set
    ``DIMOS_ENABLE_BABYLON=0`` to suppress it.
    """
    if not _babylon_enabled():
        return None

    from dimos.experimental.pimsim.module import BabylonSceneViewerModule
    from dimos.simulation.mujoco.model import get_assets

    kwargs: dict[str, Any] = dict(
        mjcf_path=viewer_mjcf_path,
        assets=get_assets(),
        port=_env_int("DIMOS_BABYLON_PORT", _DEFAULT_BABYLON_PORT),
    )
    if global_config.simulation:
        scene_package = _scene_package_config()
        if scene_package is not None and scene_package.visual_path is not None:
            scene_visual_path = str(scene_package.visual_path)
            browser_collision_path = (
                str(scene_package.browser_collision_path)
                if scene_package.browser_collision_path is not None
                else None
            )
            kwargs.update(
                scene_path=scene_visual_path,
                scene_scale=scene_package.alignment.scale,
                scene_translation=scene_package.alignment.translation,
                scene_rotation_zyx_deg=scene_package.alignment.rotation_zyx_deg,
                scene_y_up=scene_package.alignment.y_up,
                browser_collision_path=browser_collision_path,
                initial_entities=scene_package.entities,
            )
            # Optional gaussian splat sitting next to the package, served as
            # a static asset; browser loads it on demand via the Splat toggle.
            splat_dir = Path(scene_package.package_dir) / "splat"
            splat_ply = splat_dir / "scene.ply"
            if splat_ply.exists():
                import yaml as _yaml

                splat_alignment: dict[str, Any] = {}
                alignment_yaml_path = splat_dir / "alignment.yaml"
                if alignment_yaml_path.exists():
                    splat_alignment = _yaml.safe_load(alignment_yaml_path.read_text()) or {}
                kwargs.update(
                    splat_path=str(splat_ply),
                    splat_alignment=splat_alignment,
                )
    kwargs.update(
        pointcloud_hz=_env_float("DIMOS_BABYLON_POINTCLOUD_HZ", 2.0),
        pointcloud_max_points=_env_int("DIMOS_BABYLON_POINTCLOUD_MAX_POINTS", 70000),
    )
    # Babylon-as-physics mode: integrate cmd_vel locally, publish sim_odom,
    # let the rust scene_lidar consume it.  No MuJoCo at runtime.
    # "pimsim" is the preferred alias going forward; "babylon" stays accepted.
    babylon_is_physics = global_config.simulation in ("babylon", "pimsim")
    if babylon_is_physics:
        kwargs.update(
            enable_sim=True,
            sim_rate=_env_float("DIMOS_BABYLON_SIM_RATE_HZ", 100.0),
            vehicle_height=_env_float("DIMOS_BABYLON_VEHICLE_HEIGHT", 0.75),
            step_offset=_env_float("DIMOS_BABYLON_STEP_OFFSET", 0.22),
            support_floor=_env_bool("DIMOS_BABYLON_SUPPORT_FLOOR", True),
            support_floor_z=_env_float("DIMOS_SCENE_SUPPORT_FLOOR_Z", 0.0),
            support_floor_size=_env_float("DIMOS_SCENE_SUPPORT_FLOOR_SIZE", 0.0),
            lock_z=True,
        )

    bp = BabylonSceneViewerModule.blueprint(**kwargs).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("path", PathMsg): LCMTransport("/nav_path", PathMsg),
            ("pointcloud_overlay", PointCloud2): LCMTransport("/global_map", PointCloud2),
            ("cmd_vel", Twist): LCMTransport(cmd_vel_topic, Twist),
            ("clicked_point", PointStamped): LCMTransport("/clicked_point", PointStamped),
            ("point_goal", PointStamped): LCMTransport("/point_goal", PointStamped),
            ("workspace_image", Image): LCMTransport("/workspace_image", Image),
            # Dynamic-entity batch out — picked up by SceneLidarModule
            # subscriber so the lidar pointcloud includes Havok entities.
            ("entity_state_batch", EntityStateBatch): LCMTransport(
                "/entity_state_batch", EntityStateBatch
            ),
        }
    )

    if babylon_is_physics:
        # In physics mode, sim_odom IS the canonical /odom — every
        # downstream consumer (scene_lidar pose, mapping, planner)
        # reads from it.
        bp = bp.transports({("sim_odom", PoseStamped): LCMTransport("/odom", PoseStamped)})
    return bp


def _arm_teleop_blueprint() -> Blueprint | None:
    """Bridge the babylon slider HUD to the coordinator's ``servo_arms`` task.

    Implements ``HumanoidControlSpec``; the viewer auto-wires it. Only worth
    starting when Babylon is enabled (nothing else consumes the spec).
    """
    if not _babylon_enabled():
        return None
    from dimos.robot.unitree.g1.arm_teleop import G1ArmTeleop

    return G1ArmTeleop.blueprint().transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
        }
    )


def _compose_scene_splat_alignment_yaml(
    scene_package: Any, splat_alignment_yaml: Path | None
) -> Path:
    """Bake scene-package alignment into a source-frame splat alignment yaml.

    The browser path applies the scene package transform at the scene root and
    therefore expects ``alignment.yaml`` to remain source-frame relative. The
    splat camera renders directly in dimos world coordinates, so it needs the
    scene-package transform composed into the splat alignment once up front.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    import yaml

    from dimos.visualization.viser.splat import SplatAlignment

    raw_alignment = (
        SplatAlignment.from_yaml(splat_alignment_yaml)
        if splat_alignment_yaml is not None and splat_alignment_yaml.exists()
        else SplatAlignment(y_up=False)
    )

    def _scene_rotation_matrix() -> np.ndarray:
        alignment = scene_package.alignment
        rz, ry, rx = (np.deg2rad(angle) for angle in alignment.rotation_zyx_deg)
        cz, sz = np.cos(rz), np.sin(rz)
        cy, sy = np.cos(ry), np.sin(ry)
        cx, sx = np.cos(rx), np.sin(rx)
        rotate_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
        rotate_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
        rotate_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
        matrix = rotate_z @ rotate_y @ rotate_x
        if scene_package.alignment.y_up:
            matrix = matrix @ np.array(
                [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
                dtype=np.float64,
            )
        return matrix

    scene_rotation = _scene_rotation_matrix()
    raw_rotation = raw_alignment.world_from_splat().astype(np.float64)
    merged_rotation = scene_rotation @ raw_rotation
    merged_scale = float(scene_package.alignment.scale) * float(raw_alignment.scale)
    raw_translation = np.asarray(raw_alignment.translation, dtype=np.float64)
    scene_translation = np.asarray(scene_package.alignment.translation, dtype=np.float64)
    merged_translation = (
        float(scene_package.alignment.scale) * (scene_rotation @ raw_translation)
        + scene_translation
    )

    rz, ry, rx = R.from_matrix(merged_rotation).as_euler("ZYX", degrees=True)
    runtime_alignment_yaml = Path("/tmp") / (
        f"dimos_runtime_splat_alignment_{Path(scene_package.package_dir).name}.yaml"
    )
    runtime_alignment_yaml.write_text(
        yaml.safe_dump(
            {
                "scale": merged_scale,
                "translation": [float(v) for v in merged_translation],
                "rotation_zyx": [float(rz), float(ry), float(rx)],
                "y_up": False,
            },
            sort_keys=False,
        )
    )
    return runtime_alignment_yaml


def _splat_camera_blueprint() -> Blueprint | None:
    """Render a Gaussian splat from the robot's camera pose into /camera_image.

    Off by default; opt-in with ``DIMOS_ENABLE_SPLAT_CAMERA=1``. Only fires
    when the active scene package has ``splat/scene.ply`` next to its
    metadata (e.g. ``--scene office-splat``). The macOS MLX backend lives
    in ``dimos/visualization/viser/splat_camera.py``.
    """
    if not _env_bool("DIMOS_ENABLE_SPLAT_CAMERA", False):
        return None
    scene_package = _scene_package_config()
    if scene_package is None or scene_package.package_dir is None:
        return None
    splat_ply = Path(scene_package.package_dir) / "splat" / "scene.ply"
    if not splat_ply.exists():
        logger.info(
            "Splat camera enabled but %s missing; cook the scene with splat first.",
            splat_ply,
        )
        return None
    alignment_yaml = Path(scene_package.package_dir) / "splat" / "alignment.yaml"
    runtime_alignment_yaml = _compose_scene_splat_alignment_yaml(scene_package, alignment_yaml)

    from dimos.visualization.viser.camera import g1_d435_default, g1_d435_forward
    from dimos.visualization.viser.splat_camera import SplatCameraModule

    # Use the C++ Metal kernel by default (~2x perf, monkey-patched to drop
    # training-divergence fog blobs that otherwise blur the foreground).
    # Set DIMOS_MLX_RASTERIZER=python to fall back to the pure-python path.
    os.environ.setdefault("DIMOS_MLX_RASTERIZER", "cpp")

    camera_spec = (
        g1_d435_forward() if _env_bool("DIMOS_SPLAT_CAMERA_FORWARD", True) else g1_d435_default()
    )
    return SplatCameraModule.blueprint(
        splat_path=str(splat_ply),
        mjcf_path=str(_MJCF_PATH),
        alignment_yaml=str(runtime_alignment_yaml),
        camera_spec=camera_spec,
        render_hz=_env_float("DIMOS_SPLAT_RENDER_HZ", 10.0),
    ).transports(
        {
            ("color_image", Image): LCMTransport("/camera_image", Image),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        }
    )


def _camera_bridge_blueprint() -> Blueprint | None:
    """Pull the forward / head-mounted v4l2 camera into ``/camera_image``."""
    host = os.environ.get("DIMOS_ROBOT_CAMERA_HOST")
    if not host:
        return None
    from dimos.hardware.sensors.camera.tcp_jpeg import TcpJpegCameraModule

    return TcpJpegCameraModule.blueprint(
        host=host,
        port=_env_int("DIMOS_ROBOT_CAMERA_PORT", 5000),
    ).transports(
        {
            ("video", Image): LCMTransport("/camera_image", Image),
        }
    )


def _workspace_camera_bridge_blueprint() -> Blueprint | None:
    """Pull the workspace / down-looking camera into ``/workspace_image``.

    Same TCP-JPEG protocol as the forward camera, just on a different port
    (defaults to 5001). Enabled when ``DIMOS_ROBOT_WORKSPACE_CAMERA_HOST``
    is set; defaults the host to ``DIMOS_ROBOT_CAMERA_HOST`` since both
    cameras almost always live on the same machine.
    """
    host = os.environ.get(
        "DIMOS_ROBOT_WORKSPACE_CAMERA_HOST",
        os.environ.get("DIMOS_ROBOT_CAMERA_HOST", ""),
    )
    if not host or not _env_bool("DIMOS_ENABLE_WORKSPACE_CAMERA", True):
        return None
    # Distinct class so the module coordinator deploys this alongside the
    # forward camera (it deduplicates by class, not instance).
    from dimos.hardware.sensors.camera.tcp_jpeg import WorkspaceTcpJpegCameraModule

    # Remap the inherited ``video`` stream to ``video_workspace`` so the
    # autoconnect transport dict — keyed globally by (stream_name, type) —
    # doesn't collide with the forward TcpJpegCameraModule's ``video`` Out.
    # Without this remap the last-merged transport wins and BOTH cameras end
    # up publishing to /workspace_image.
    return (
        WorkspaceTcpJpegCameraModule.blueprint(
            host=host,
            port=_env_int("DIMOS_ROBOT_WORKSPACE_CAMERA_PORT", 5001),
        )
        .remappings([(WorkspaceTcpJpegCameraModule, "video", "video_workspace")])
        .transports(
            {
                ("video_workspace", Image): LCMTransport("/workspace_image", Image),
            }
        )
    )


def _quest_teleop_blueprint(cmd_vel_topic: str) -> Blueprint | None:
    if not _env_bool("DIMOS_ENABLE_QUEST_TELEOP", False):
        return None
    from dimos.robot.unitree.g1.quest_teleop import G1QuestTeleopModule

    return G1QuestTeleopModule.blueprint(
        server_port=_env_int("DIMOS_QUEST_TELEOP_PORT", 8443),
        right_stick_mode=os.environ.get("DIMOS_QUEST_RIGHT_STICK_MODE", "yaw").strip().lower()
        or "yaw",
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
            ("cmd_vel", Twist): LCMTransport(cmd_vel_topic, Twist),
            ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
            # Forward the camera bridge feeds into the WebXR client so the
            # operator sees the robot's view as floating quads in VR. The
            # color_image goes to the in-front quad; the workspace_image
            # goes to a lower quad (the down-looking realsense by default).
            ("color_image", Image): LCMTransport("/camera_image", Image),
            ("workspace_image", Image): LCMTransport("/workspace_image", Image),
        }
    )


if global_config.simulation in ("babylon", "pimsim"):
    # Browser-physics nav stack. Babylon owns the robot's kinematic base
    # (cmd_vel → sim_odom) and the Havok entity world; the rust scene
    # lidar consumes both. No MuJoCo, no coordinator, no GR00T policy
    # (joint-level control needs a real physics sim).
    _cmd_vel_topic = "/cmd_vel"
    _babylon = _babylon_blueprint(_MJCF_PATH, _cmd_vel_topic)
    if _babylon is None:
        raise RuntimeError(
            f"--simulation {global_config.simulation} requested but Babylon viewer "
            "is disabled (DIMOS_ENABLE_BABYLON=0?)"
        )
    _splat_camera = _splat_camera_blueprint()
    _optional_pimsim = tuple(bp for bp in (_splat_camera,) if bp is not None)
    g1_groot_wbc = autoconnect(
        _babylon,
        _websocket_blueprint(_cmd_vel_topic),
        *_sim_support_blueprints(),
        *_optional_pimsim,
    ).transports(
        {
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("cmd_vel", Twist): LCMTransport(_cmd_vel_topic, Twist),
            ("nav_cmd_vel", Twist): LCMTransport(_cmd_vel_topic, Twist),
            ("pointcloud", PointCloud2): LCMTransport("/lidar", PointCloud2),
            ("global_map", PointCloud2): LCMTransport("/global_map", PointCloud2),
            ("global_costmap", OccupancyGrid): LCMTransport("/global_costmap", OccupancyGrid),
            ("path", PathMsg): LCMTransport("/nav_path", PathMsg),
            ("clicked_point", PointStamped): LCMTransport("/clicked_point", PointStamped),
            ("point_goal", PointStamped): LCMTransport("/point_goal", PointStamped),
            ("goal_request", PoseStamped): LCMTransport("/goal_request", PoseStamped),
            ("stop_movement", Bool): LCMTransport("/stop_movement", Bool),
        }
    )
else:
    _backend_selection = _select_backend()
    _coordinator, _cmd_vel_topic = _coordinator_blueprint(_backend_selection)
    _babylon = _babylon_blueprint(_MJCF_PATH, _cmd_vel_topic)
    _teleop = _arm_teleop_blueprint()
    _quest = _quest_teleop_blueprint(_cmd_vel_topic)
    _camera_bridge = _camera_bridge_blueprint()
    _workspace_camera = _workspace_camera_bridge_blueprint()
    _splat_camera = _splat_camera_blueprint()
    _optional = tuple(
        bp
        for bp in (_babylon, _teleop, _quest, _camera_bridge, _workspace_camera, _splat_camera)
        if bp is not None
    )

    g1_groot_wbc = autoconnect(
        _backend_selection.blueprint,
        _coordinator,
        _websocket_blueprint(_cmd_vel_topic),
        *_sim_support_blueprints(),
        *_optional,
    ).transports(
        {
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("cmd_vel", Twist): LCMTransport(_cmd_vel_topic, Twist),
            ("nav_cmd_vel", Twist): LCMTransport(_cmd_vel_topic, Twist),
            ("pointcloud", PointCloud2): LCMTransport("/lidar", PointCloud2),
            ("global_map", PointCloud2): LCMTransport("/global_map", PointCloud2),
            ("global_costmap", OccupancyGrid): LCMTransport("/global_costmap", OccupancyGrid),
            ("path", PathMsg): LCMTransport("/nav_path", PathMsg),
            ("clicked_point", PointStamped): LCMTransport("/clicked_point", PointStamped),
            ("point_goal", PointStamped): LCMTransport("/point_goal", PointStamped),
            ("goal_request", PoseStamped): LCMTransport("/goal_request", PoseStamped),
            ("stop_movement", Bool): LCMTransport("/stop_movement", Bool),
        }
    )

__all__ = ["g1_groot_wbc"]

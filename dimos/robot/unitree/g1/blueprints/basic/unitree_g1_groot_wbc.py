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

"""Unitree G1 GR00T whole-body-control blueprint.

One blueprint, ``--simulation`` flag picks the backend:

Real hardware (default):
    G1WholeBodyConnection (DDS rt/lowstate <-> rt/lowcmd) + transport_lcm
    whole-body adapter. 500 Hz tick. Safety profile: unarmed + dry-run on
    start; activate explicitly through ControlCoordinator RPC after
    verifying commands. The policy ramps from the current pose to its
    bent-knee default over 10 s before taking torque control. The 14 arm
    joints are held at the relaxed GR00T-trained default via a lower-priority
    servo task.

Sim (``--simulation``):
    MujocoSimModule (in-process MuJoCo + SHM) + sim_mujoco_g1 adapter.
    50 Hz tick (matches the rate the policy was trained at). No arming
    ramp and no dry-run. The 14 arm joints are still held with the same
    lower-priority servo task as hardware so headless and viewer runs do not
    depend on incidental startup timing.

Usage:
    dimos run unitree-g1-groot-wbc                 # real hardware
    dimos --simulation mujoco run unitree-g1-groot-wbc    # sim
    dimos --simulation mujoco --scene none run unitree-g1-groot-wbc
    dimos --simulation mujoco --scene office run unitree-g1-groot-wbc
    dimos --simulation mujoco --scene supermarket run unitree-g1-groot-wbc

Overrides (replace the old env-var dance):
    dimos run unitree-g1-groot-wbc \\
        -o g1wholebodyconnection.network_interface=enp2s0
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    ARM_DEFAULT_POSE,
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.g1_rerun import (
    G1_RERUN_ROOT,
    g1_costmap,
    g1_urdf_joint_state,
    g1_urdf_static_robot,
)
from dimos.utils.data import LfsPath
from dimos.visualization.rerun.scene_package import scene_package_static_entities
from dimos.visualization.vis_module import vis_module

# Lazy data handles. LfsPath only triggers the LFS pull on first
# str()/open(); using ``get_data(...)`` at import time would block the
# whole CLI on a multi-GB download every time the module is imported.
_GROOT_MODEL_DIR = LfsPath("groot")
_MJCF_PATH = LfsPath("mujoco_sim/g1_gear_wbc.xml")
_ROBOT_ONLY_MJCF_PATH = Path(__file__).resolve().parents[2] / "assets" / "g1_29dof.xml"
_ROBOT_MESHDIR = LfsPath("g1_urdf/meshes")

_adapter_address: str | Path
_cmd_vel_topic = "/cmd_vel" if global_config.simulation else "/g1/cmd_vel"
_MUJOCO_LIDAR_CAMERAS = (
    "lidar_front_camera",
    "lidar_left_camera",
    "lidar_right_camera",
)
_MUJOCO_LIDAR_CAMERA = _MUJOCO_LIDAR_CAMERAS[0]
# Robot geoms occupy groups 0/1. The legacy floor uses group 2, and cooked
# scene packages/entities use group 3, so lidar should render world geometry.
_MUJOCO_LIDAR_GEOM_GROUPS = (2, 3)
assert G1.height_clearance is not None and G1.width_clearance is not None
_MUJOCO_LIDAR_BASE_KWARGS: dict[str, Any] = {
    "width": 320,
    "height": 240,
    "fps": 2,
    "enable_color": False,
    "enable_depth": False,
    "enable_pointcloud": True,
    "pointcloud_fps": 1.0,
    "enable_mujoco_lidar": True,
    "mujoco_lidar_geom_groups": list(_MUJOCO_LIDAR_GEOM_GROUPS),
    "mujoco_lidar_raycast_width": 64,
    "mujoco_lidar_raycast_height": 32,
    "mujoco_lidar_robot_exclusion_radius": G1.width_clearance,
}
_G1_SUPERMARKET_COMPOSED_MJB = (
    "mujoco/composed/unitree-g1-groot-wbc_spawn_9p2_11p8_yaw_m1p57_static_only_lidar.mjb"
)
_G1_NAV_VOXEL_RESOLUTION = 0.05
_G1_NAV_OVERHEAD_SAFETY_MARGIN = 0.2
_G1_NAV_MAX_STEP_HEIGHT = 0.10
_G1_NAV_ROTATION_DIAMETER = 0.8
_G1_NAV_SAFE_RADIUS_MARGIN = 0.6


def _mujoco_lidar_kwargs(camera_name: str, camera_names: tuple[str, ...]) -> dict[str, Any]:
    return {
        "camera_name": camera_name,
        "mujoco_lidar_camera_names": list(camera_names),
        **_MUJOCO_LIDAR_BASE_KWARGS,
    }


if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("unitree-g1-groot-wbc only supports --simulation mujoco")

if global_config.simulation == "mujoco":
    from dimos.mapping.voxels import VoxelGridMapper
    from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
    from dimos.simulation.engines.robot_sim_binding import (
        RobotSimSpec,
        mjcf_joint_names_from_hardware,
    )

    _g1_sim_joints = tuple(g1_joints)
    _g1_sim_spec = RobotSimSpec(
        robot_id="g1",
        hardware_joints=_g1_sim_joints,
        root_body_names=("pelvis",),
        root_joint_names=("floating_base_joint",),
        require_floating_base=True,
        model_joint_names=mjcf_joint_names_from_hardware(_g1_sim_joints),
        imu_gyro_names=(
            "imu-pelvis-angular-velocity",
            "imu-torso-angular-velocity",
            "imu-angular-velocity",
            "gyro_pelvis",
            "imu_gyro",
        ),
        imu_accel_names=(
            "imu-pelvis-linear-acceleration",
            "imu-torso-linear-acceleration",
            "imu-linear-acceleration",
            "accelerometer_pelvis",
            "imu_accel",
        ),
        require_imu=True,
    )

    def _legacy_mujoco_backend() -> Any:
        return MujocoSimModule.blueprint(
            address=_MJCF_PATH,
            headless=True,
            dof=29,
            **_mujoco_lidar_kwargs(_MUJOCO_LIDAR_CAMERA, _MUJOCO_LIDAR_CAMERAS),
            inject_legacy_assets=True,
            robot_sim_spec=_g1_sim_spec,
        )

    def _scene_mujoco_backend() -> tuple[Any, str | Path]:
        if global_config.scene is None:
            return _legacy_mujoco_backend(), _MJCF_PATH

        scene_path = Path(str(global_config.scene)).expanduser()
        if scene_path.suffix.lower() == ".mjb":
            if not scene_path.exists():
                raise FileNotFoundError(f"MuJoCo binary scene not found: {scene_path}")
            return (
                MujocoSimModule.blueprint(
                    address=scene_path,
                    headless=True,
                    dof=29,
                    **_mujoco_lidar_kwargs(_MUJOCO_LIDAR_CAMERA, _MUJOCO_LIDAR_CAMERAS),
                    robot_sim_spec=_g1_sim_spec,
                ),
                scene_path,
            )

        from dimos.simulation.scenes.catalog import resolve_scene_package

        package = resolve_scene_package(global_config.scene)
        if package is None:
            return _legacy_mujoco_backend(), _MJCF_PATH
        if package.mujoco_scene_path is None:
            raise ValueError(f"scene package has no MuJoCo scene artifact: {package.metadata_path}")

        composed_scene = _precomposed_g1_scene(package.package_dir)
        if composed_scene is not None:
            return (
                MujocoSimModule.blueprint(
                    address=composed_scene,
                    headless=True,
                    dof=29,
                    **_mujoco_lidar_kwargs(_MUJOCO_LIDAR_CAMERA, _MUJOCO_LIDAR_CAMERAS),
                    robot_sim_spec=_g1_sim_spec,
                ),
                composed_scene,
            )

        return (
            MujocoSimModule.blueprint(
                scene_xml=package.mujoco_scene_path,
                robot_mjcf=_ROBOT_ONLY_MJCF_PATH,
                robot_meshdir=_ROBOT_MESHDIR,
                robot_id="",
                scene_entities=package.entities,
                headless=True,
                dof=29,
                **_mujoco_lidar_kwargs(_MUJOCO_LIDAR_CAMERA, _MUJOCO_LIDAR_CAMERAS),
                robot_sim_spec=_g1_sim_spec,
            ),
            _ROBOT_ONLY_MJCF_PATH,
        )

    def _precomposed_g1_scene(package_dir: Path) -> Path | None:
        if "supermarket" not in package_dir.name:
            return None
        candidate = package_dir / _G1_SUPERMARKET_COMPOSED_MJB
        return candidate if candidate.exists() else None

    # Sim backend: MuJoCo engine via SHM.
    _backend, _adapter_address = _scene_mujoco_backend()
    # MujocoSimModule's ``odom`` Out is the sole producer of ``/odom``
    # now - the coordinator no longer polls the whole-body adapter for
    # base pose (read_odom was dropped from the Protocol). autoconnect
    # maps ``(odom, PoseStamped)`` to ``/odom`` by default; no override.
    _adapter_type = "sim_mujoco_g1"
    _tick_rate = 50.0
    _auto_arm = True
    _auto_dry_run = False
    _default_ramp_seconds = 0.0
    _decimation: int | None = 1
    _arm_holder = TaskConfig(
        name="servo_arms",
        type="servo",
        joint_names=g1_arms,
        priority=10,
        auto_start=True,
        params={"default_positions": ARM_DEFAULT_POSE},
    )
    _mapper = VoxelGridMapper.blueprint(emit_every=1)
    _nav_stack = autoconnect(
        _mapper,
        CostMapper.blueprint(
            config=HeightCostConfig(
                resolution=_G1_NAV_VOXEL_RESOLUTION,
                can_pass_under=G1.height_clearance + _G1_NAV_OVERHEAD_SAFETY_MARGIN,
                can_climb=_G1_NAV_MAX_STEP_HEIGHT,
            ),
            initial_safe_radius_meters=G1.width_clearance + _G1_NAV_SAFE_RADIUS_MARGIN,
        ),
        ReplanningAStarPlanner.blueprint(
            robot_width=G1.width_clearance,
            robot_rotation_diameter=_G1_NAV_ROTATION_DIAMETER,
        ),
        MovementManager.blueprint(),
    )
    _remappings = [
        (VoxelGridMapper, "lidar", "pointcloud"),
        (ControlCoordinator, "twist_command", "cmd_vel"),
    ]
else:
    from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection

    # Real-hw backend: DDS connection module + transport_lcm adapter.
    _backend = G1WholeBodyConnection.blueprint(release_sport_mode=True)
    _adapter_type = "transport_lcm"
    _adapter_address = ""
    _tick_rate = 500.0
    # Real hardware: come up unarmed + dry-run; operator must click
    # Activate (10 s ramp) after verifying commands.
    _auto_arm = False
    _auto_dry_run = True
    _default_ramp_seconds = 10.0
    _decimation = None  # task default (10) pairs with 500 Hz tick.
    # Real hardware needs the arms held -- kd damping alone would let
    # them sag toward singular configurations between trajectories.
    _arm_holder = TaskConfig(
        name="servo_arms",
        type="servo",
        joint_names=g1_arms,
        priority=10,
        auto_start=True,
        params={"default_positions": ARM_DEFAULT_POSE},
    )
    _nav_stack = MovementManager.blueprint()
    _remappings = [(ControlCoordinator, "twist_command", "cmd_vel")]


def _g1_groot_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="G1 GR00T WBC",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.0),
            ),
        ),
        rrb.TimePanel(state="collapsed"),
    )


def _g1_nav_path(path: NavPath) -> Any:
    return path.to_rerun(z_offset=0.3)


_static_rerun_entities = {
    # MujocoSimModule logs odom as a Transform3D at world/odom; the robot
    # mesh lives underneath and link transforms are driven by joint state.
    G1_RERUN_ROOT: g1_urdf_static_robot(root_path=G1_RERUN_ROOT),
}
_static_rerun_entities.update(scene_package_static_entities(global_config.scene))

_rerun_config = {
    "blueprint": _g1_groot_rerun_blueprint,
    "visual_override": {
        # This blueprint uses raycast lidar, so suppress raw camera streams
        # in Rerun.
        "world/color_image": None,
        "world/camera_info": None,
        "world/depth_image": None,
        "world/depth_camera_info": None,
        "world/coordinator_joint_state": g1_urdf_joint_state(root_path=G1_RERUN_ROOT),
        "world/global_costmap": g1_costmap,
        "world/navigation_costmap": g1_costmap,
        "world/path": _g1_nav_path,
    },
    "max_hz": {
        "world/coordinator_joint_state": 20.0,
        "world/global_map": 1.0,
        "world/global_costmap": 2.0,
        "world/navigation_costmap": 2.0,
        # The planner publishes an empty Path() immediately before the new
        # planned path. Throttling this entity drops the real path.
        "world/path": 0,
    },
    "static": _static_rerun_entities,
}


def _viewer() -> Any:
    return vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config)


_coordinator = ControlCoordinator.blueprint(
    tick_rate=_tick_rate,
    hardware=[
        HardwareComponent(
            hardware_id="g1",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=g1_joints,
            adapter_type=_adapter_type,
            address=_adapter_address,
            wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
        ),
    ],
    tasks=[
        TaskConfig(
            name="groot_wbc",
            type="g1_groot_wbc",
            joint_names=g1_legs_waist,
            priority=50,
            auto_start=True,
            params={
                "model_path": _GROOT_MODEL_DIR,
                "hardware_id": "g1",
                "auto_arm": _auto_arm,
                "auto_dry_run": _auto_dry_run,
                "default_ramp_seconds": _default_ramp_seconds,
                "decimation": _decimation,
            },
        ),
        *([_arm_holder] if _arm_holder is not None else []),
    ],
).transports(
    {
        ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
        ("cmd_vel", Twist): LCMTransport(_cmd_vel_topic, Twist),
        # Real-hw only: the transport_lcm adapter speaks to
        # G1WholeBodyConnection over these topics. autoconnect already
        # matches by (name, type) so sim doesn't need them -- they're
        # harmless when the sim engine doesn't expose those ports.
        ("motor_states", JointState): LCMTransport("/g1/motor_states", JointState),
        ("imu", Imu): LCMTransport("/g1/imu", Imu),
        ("motor_command", MotorCommandArray): LCMTransport("/g1/motor_command", MotorCommandArray),
    }
)

unitree_g1_groot_wbc = (
    autoconnect(_backend, _coordinator, _nav_stack, _viewer())
    .remappings(_remappings)
    .global_config(robot_model="unitree_g1")
)

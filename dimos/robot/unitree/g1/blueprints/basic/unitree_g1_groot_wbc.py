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
    dimos --simulation mujoco --scene-package none run unitree-g1-groot-wbc
    dimos --simulation mujoco --scene-package office run unitree-g1-groot-wbc
    dimos --simulation mujoco --scene-package supermarket run unitree-g1-groot-wbc

Overrides (replace the old env-var dance):
    dimos run unitree-g1-groot-wbc \\
        -o g1wholebodyconnection.network_interface=enp2s0
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, cast

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
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.g1_rerun import (
    G1_RERUN_ROOT,
    g1_costmap,
    g1_urdf_joint_state,
    g1_urdf_static_joints,
    g1_urdf_static_robot,
)
from dimos.robot.unitree.g1.manipulation import (
    G1_ARM_SIDES,
    g1_arm_model_config,
    g1_arm_trajectory_task,
)
from dimos.simulation.scene_assets.spec import ScenePackage
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
_G1_NUM_MOTORS = len(g1_joints)
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
    # Base-frame scans + the odometry output = the LIO contract, so the
    # raytracing mapper wires identically in sim and on real hardware.
    "mujoco_lidar_frame": "base",
}
_G1_COMPOSED_MJB_KEY = "unitree-g1-groot-wbc_spawn_9p2_11p8_yaw_m1p57_static_only_lidar"
_G1_COMPOSED_MJB_ROBOT = "unitree-g1-groot-wbc"
_G1_COMPOSED_MJB_ENTITY_POLICY = "static-only"
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
    from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
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
            dof=_G1_NUM_MOTORS,
            **_mujoco_lidar_kwargs(_MUJOCO_LIDAR_CAMERA, _MUJOCO_LIDAR_CAMERAS),
            inject_legacy_assets=True,
            robot_sim_spec=_g1_sim_spec,
        )

    def _scene_mujoco_backend() -> tuple[Any, str | Path]:
        if global_config.scene_package is None:
            return _legacy_mujoco_backend(), _MJCF_PATH

        scene_path = Path(str(global_config.scene_package)).expanduser()
        if scene_path.suffix.lower() == ".mjb":
            if not scene_path.exists():
                raise FileNotFoundError(f"MuJoCo binary scene not found: {scene_path}")
            return (
                MujocoSimModule.blueprint(
                    address=scene_path,
                    headless=True,
                    dof=_G1_NUM_MOTORS,
                    **_mujoco_lidar_kwargs(_MUJOCO_LIDAR_CAMERA, _MUJOCO_LIDAR_CAMERAS),
                    robot_sim_spec=_g1_sim_spec,
                ),
                scene_path,
            )

        from dimos.simulation.scenes.catalog import resolve_scene_package

        package = resolve_scene_package(global_config.scene_package)
        if package is None:
            return _legacy_mujoco_backend(), _MJCF_PATH
        if package.mujoco_scene_path is None:
            raise ValueError(f"scene package has no MuJoCo scene artifact: {package.metadata_path}")

        composed_scene = _precomposed_g1_scene(package)
        if composed_scene is not None:
            return (
                MujocoSimModule.blueprint(
                    address=composed_scene,
                    headless=True,
                    dof=_G1_NUM_MOTORS,
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
                dof=_G1_NUM_MOTORS,
                **_mujoco_lidar_kwargs(_MUJOCO_LIDAR_CAMERA, _MUJOCO_LIDAR_CAMERAS),
                robot_sim_spec=_g1_sim_spec,
            ),
            _ROBOT_ONLY_MJCF_PATH,
        )

    def _precomposed_g1_scene(package: ScenePackage) -> Path | None:
        candidate = package.mujoco_composed_binary_path(
            key=_G1_COMPOSED_MJB_KEY,
            robot=_G1_COMPOSED_MJB_ROBOT,
            entity_policy=_G1_COMPOSED_MJB_ENTITY_POLICY,
        )
        if candidate is None:
            return None
        if not candidate.exists():
            raise FileNotFoundError(
                f"scene package declares a composed MuJoCo binary that is missing: {candidate}"
            )
        return candidate

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
    _n_workers = 2  # sim: keep the default worker count
    _arm_holder = TaskConfig(
        name="servo_arms",
        type="servo",
        joint_names=g1_arms,
        priority=10,
        auto_start=True,
        params={"default_positions": ARM_DEFAULT_POSE},
    )
    # Same mapper as real hardware for sim/real parity: the sim lidar is
    # configured to publish base-frame scans (mujoco_lidar_frame below) and
    # the sim module publishes ground-truth Odometry, so the raytracer gets
    # exactly the contract the LIO provides on the robot. Lidar frames
    # arrive at ~1 Hz, so emit per frame.
    _mapper = RayTracingVoxelMap.blueprint(
        voxel_size=_G1_NAV_VOXEL_RESOLUTION,
        emit_every=0,
        global_emit_every=1,
    )
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
        (RayTracingVoxelMap, "lidar", "pointcloud"),
        (ControlCoordinator, "twist_command", "cmd_vel"),
    ]
else:
    import os

    from dimos.hardware.sensors.lidar.pointlio.module import PointLio
    from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
    from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection

    # Real-hw backend: DDS connection module + transport_lcm adapter.
    _backend = G1WholeBodyConnection.blueprint(release_sport_mode=True)
    _adapter_type = "transport_lcm"
    _adapter_address = ""
    # 100 Hz outer loop: policy runs at 50 Hz (decimation 2) and lowcmd
    # streams at 100 Hz. The onboard Jetson can't sustain a 500 Hz Python
    # tick loop -- it collapses to ~90 Hz and starves the policy of fresh
    # state, so balance decays. 100 Hz holds cleanly and matches the
    # reference deployment's rate.
    _tick_rate = 100.0
    # Real hardware: come up unarmed + dry-run; operator must click
    # Activate (10 s ramp) after verifying commands.
    _auto_arm = False
    _auto_dry_run = True
    _default_ramp_seconds = 10.0
    _decimation = 2  # 100 Hz tick / 2 = 50 Hz policy (training + sim rate).
    # Give each heavy module (Rerun bridge, connection, coordinator, websocket
    # servers) its own process. The bridge is already dedicated_worker, but
    # with too few workers the other modules crowd one process and the
    # system-wide contention starves the bridge's gRPC senders -- the viewer
    # falls behind. Isolating everything keeps it streaming live.
    _n_workers = 10
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
    # Real-hw nav: the unitree-g1-nav-simple middle (raytracing voxel map ->
    # height costmap -> replanning A*) fed by the Livox MID-360 through
    # Point-LIO (the go2 nav-3d front-end; our FAST-LIO2 build segfaulted and
    # Point-LIO is the actively maintained LIO), executed through the
    # coordinator's twist_command by the WBC policy instead of sport mode.
    # The LIO world frame originates at the lidar boot pose (ground ~1.2 m
    # below z=0); visuals account for it below.
    _nav_stack = autoconnect(
        PointLio.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=_G1_NAV_VOXEL_RESOLUTION,
            # No /local_map: it is the MLS planner's incremental feed and has
            # no consumer here -- it was 4.5 MB/s of pure bus + encode waste.
            emit_every=0,
            # Global map at ~1 Hz (lidar runs ~4-5 Hz effective). CostMapper
            # recomputes per emit, so this also paces the costmap. Full-rate
            # emits saturated the Orin (load ~26/8) and starved the LIO.
            global_emit_every=4,
        ),
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
    _remappings = [(ControlCoordinator, "twist_command", "cmd_vel")]

# Arm manipulation planner (sim and real: plans are kinematic either way and
# execution is gated behind the coordinator's arm/dry-run state on hardware).
# Plans against the pelvis-rooted arm models from the reachability registry
# and executes through the per-arm trajectory tasks below. Viser is the
# interactive test surface; execute stays disabled unless
# -o manipulationmodule.visualization.allow_plan_execute=true is passed.
# Reachability map overlays are opt-in:
#   -o manipulationmodule.visualization.reachability_maps.g1-left=<map.npz>
_manipulation_stack: tuple[Any, ...] = (
    planner(
        robots=[g1_arm_model_config(side) for side in G1_ARM_SIDES],
        world_backend="mujoco",
        kinematics={"backend": "mink"},
        floor_z=0.0,
        # Deploy-branch defaults: viser reachable over the LAN and execute
        # enabled (still gated by the coordinator's arm/dry-run state).
        # Mainline keeps host=127.0.0.1 and allow_plan_execute=False;
        # baked in here because -o overrides replace this whole dict, so
        # per-flag opt-ins would silently drop backend/port.
        visualization={
            "backend": "viser",
            "port": 8095,
            "host": "0.0.0.0",
            "allow_plan_execute": True,
        },
    ),
)


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


# Robot mesh root. Sim: MujocoSimModule publishes /odom (PoseStamped), which
# the bridge logs as a Transform3D at world/odom -- the mesh lives underneath.
# Real hw: the LIO publishes /odometry (Odometry) instead, logged at
# world/odometry, so the mesh roots there and _g1_real_odometry_root shapes
# the transform.
_G1_ROOT = G1_RERUN_ROOT if global_config.simulation == "mujoco" else "world/odometry/g1"


# The LIO world frame originates at the lidar's boot pose (the physical
# ground sits ~1.2 m below z=0). Like the nav blueprints' sensor-attached
# robot marker, everything drawn for the robot derives from the sensor pose;
# here the offsets come from the URDF itself instead of magic numbers.
_G1_URDF_PATH = Path(__file__).resolve().parents[2] / "g1.urdf"
# Nominal standing pelvis height; matches G1GrootWBCTask's height_cmd.
_G1_NOMINAL_PELVIS_Z = 0.74
_g1_pelvis_mid360_cache: list[Any] = []


def _g1_pelvis_to_mid360() -> Any:
    """Rest-pose pelvis->mid360_link transform from the G1 URDF (cached)."""
    if not _g1_pelvis_mid360_cache:
        import numpy as np
        import yourdfpy

        urdf = yourdfpy.URDF.load(str(_G1_URDF_PATH), load_meshes=False)
        urdf.update_cfg(np.zeros(len(urdf.actuated_joint_names)))
        _g1_pelvis_mid360_cache.append(urdf.get_transform("mid360_link", "pelvis"))
    return _g1_pelvis_mid360_cache[0]


def _g1_real_odometry_root(odom: Any) -> Any:
    """Robot-mesh root: pelvis pose composed from LIO odometry.

    The odometry tracks the mid360 body frame, so the pelvis pose is
    T_world_mid360 @ inverse(pelvis->mid360 at rest). Waist articulation
    between pelvis and head is ignored (rest offset), the same approximation
    as the nav blueprints' fixed sensor-attached marker.
    """
    import numpy as np
    import rerun as rr

    from dimos.msgs.geometry_msgs.Quaternion import Quaternion

    t_world_mid360 = np.eye(4)
    # The MID-360 is physically mounted upside down on the G1 head (the
    # URDF's mid360_joint doesn't carry the flip), so un-roll the sensor
    # body by pi about x before composing: Rx(pi) == diag(1, -1, -1).
    t_world_mid360[:3, :3] = odom.orientation.to_rotation_matrix() @ np.diag([1.0, -1.0, -1.0])
    t_world_mid360[:3, 3] = (odom.x, odom.y, odom.z)
    t_world_pelvis = t_world_mid360 @ np.linalg.inv(_g1_pelvis_to_mid360())
    q = Quaternion.from_rotation_matrix(t_world_pelvis[:3, :3])
    return rr.Transform3D(
        translation=t_world_pelvis[:3, 3].tolist(),
        rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
    )


def _g1_real_ground_z() -> float:
    """Ground height in the LIO boot frame: -(mount z + nominal pelvis z)."""
    return -(float(_g1_pelvis_to_mid360()[2, 3]) + _G1_NOMINAL_PELVIS_Z)


def _g1_real_costmap(grid: Any) -> Any:
    """Costmap rendered on the actual ground plane of the boot frame."""
    return g1_costmap(grid, z_offset=_g1_real_ground_z() + 0.02)


_static_rerun_entities: dict[str, Any] = {
    _G1_ROOT: g1_urdf_static_robot(root_path=_G1_ROOT),
    # Fixed-joint (and rest-pose) link transforms, logged once. The per-frame
    # animator (g1_urdf_joint_state) then only updates the movable joints.
    f"{_G1_ROOT}/_joint_rest": g1_urdf_static_joints(root_path=_G1_ROOT),
}
_static_rerun_entities.update(scene_package_static_entities(global_config.scene_package))

_rerun_config = {
    # Cap the in-RAM recording so a late-connecting viewer replays a small
    # recent slice instead of a multi-GB backlog; 512MB fits the onboard
    # Jetson. This knob also sets the auto-spawned viewer's budget, and the
    # 1 Hz full-snapshot nav streams (global map, costmaps) fill a small
    # budget in minutes -- the viewer GC then evicts them between updates
    # ("the map disappears"). Sim runs on the workstation, so give it room.
    "memory_limit": "512MB" if global_config.simulation != "mujoco" else "8GB",
    "blueprint": _g1_groot_rerun_blueprint,
    "visual_override": {
        # This blueprint uses raycast lidar, so suppress raw camera streams
        # in Rerun.
        "world/color_image": None,
        # Raw scans are in the sensor/base frame (LIO contract), not world --
        # the voxel map is the live view. Same rationale as world/lidar on
        # real hardware below.
        "world/pointcloud": None,
        "world/camera_info": None,
        "world/depth_image": None,
        "world/depth_camera_info": None,
        "world/coordinator_joint_state": g1_urdf_joint_state(root_path=_G1_ROOT),
        "world/global_costmap": g1_costmap,
        "world/navigation_costmap": g1_costmap,
        "world/path": _g1_nav_path,
    },
    "max_hz": {
        # 50 Hz mesh animation. The 15 Hz cap predates the newest_first serve
        # fix; with serving healthy the produce rate no longer outruns the
        # viewer. Drop back toward 15 if the viewer ever drifts behind again.
        "world/coordinator_joint_state": 50.0,
        # Raw whole-body state streams arrive at ~440 Hz. The mesh animates
        # from coordinator_joint_state (above); these are only useful as debug
        # plots, so throttle them hard instead of flooding Rerun's store.
        "world/g1/imu": 10.0,
        "world/g1/motor_states": 10.0,
        "world/g1/motor_command": 10.0,
        "world/odometry": 15.0,
        "world/global_map": 1.0,
        "world/global_costmap": 2.0,
        "world/navigation_costmap": 2.0,
        # The planner publishes an empty Path() immediately before the new
        # planned path. Throttling this entity drops the real path.
        "world/path": 0,
    },
    "static": _static_rerun_entities,
}

if global_config.simulation != "mujoco":
    # Real hw: root the robot mesh on FAST-LIO odometry, draw the costmap on
    # the boot frame's actual ground plane, and throttle the raw registered
    # scan over the WiFi link (the voxel map is the useful view).
    _rerun_config["visual_override"]["world/odometry"] = _g1_real_odometry_root
    _rerun_config["visual_override"]["world/global_costmap"] = _g1_real_costmap
    _rerun_config["visual_override"]["world/navigation_costmap"] = _g1_real_costmap
    # The raw FAST-LIO scan is in the (upside-down) sensor frame, not the
    # world frame -- registration happens in RayTracingVoxelMap. Hide it like
    # the nav blueprints do; the voxel map is the live view.
    _rerun_config["visual_override"]["world/lidar"] = None


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
        # Per-arm trajectory execution for the ManipulationModule. Idle tasks
        # emit nothing (servo_arms keeps the hold); an executing task outranks
        # the hold for its 7 joints only.
        *(g1_arm_trajectory_task(side) for side in G1_ARM_SIDES),
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
    autoconnect(_backend, _coordinator, _nav_stack, *_manipulation_stack, _viewer())
    .remappings(cast("Any", _remappings))
    .global_config(robot_model="unitree_g1", n_workers=_n_workers)
)

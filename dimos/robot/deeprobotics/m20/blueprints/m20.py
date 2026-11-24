# Copyright 2026 Dimensional Inc.
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


"""The M20: one blueprint for the real robot and its simulation, picked with ``--choice M20Mode=...``.

``real`` (default) runs on the robot on its own Rust drivers with dimSLAM, the voxel map, MLS planning, local planner,
follower and continuous relocalization, with its own choices (M20Joints, M20Walker, M20BodyFilter, M20BodyImu,
M20Cameras, M20Slam). ``sim`` drives the simulated walking M20 with the same navigation stack to a clicked goal.
``sim-bench`` is ``sim`` with one nav-bench case driven against it (``dimos nav-bench`` runs it).

Real mode runs the M20 on its own Rust drivers, with no vendor DrDDS stack: dimSLAM in place of the vendor LIO.

AOS (a ``dimos host`` tagged ``board=aos``) runs the body IMU, the cameras and, with ``--choice M20Joints=controller``, the
motor-bus joint state and batteries (the M20 controller; needs rl_deploy with the ecan shim). NOS (``board=nos``) takes the Airys' multicast
on its own lidar NIC and runs both Airy pairs off it: full sweeps with the body cut, for the map and relocalization,
and 10 ms slices of each sweep for dimSLAM beside them (``--choice M20Slam=gos`` runs dimSLAM on GOS on the full sweeps).
Everything else runs on the machine that starts the run (GOS): the ray tracer, planners and relocalization.

Both Airys publish onto one ``lidar`` stream (their IMUs onto ``lidar_imu``), each scan in its own frame (front_lidar,
rear_lidar), with the robot's body already cut out by the lidar driver (the URDF's collision geometry at the standing
pose, or at the controller's joint state with ``--choice M20Joints=controller``): dimSLAM tells the lidars apart by frame_id, the ray
tracer traces each from its own lidar's origin, and relocalization places each by tf at its stamp.
``--choice M20BodyFilter=python`` cuts it with the Python UrdfManagerPointClearing instead (lidars onto ``lidar_raw``,
the filter onto ``lidar``). dimSLAM runs on the rear lidar's IMU alone (two IMUs ran under real time
on the RK3588). Each Airy is namespaced but publishes its mount onto the one global tf. ``--choice M20Cameras=off`` runs no cameras; the body IMU (``imu``) is sniffed off the vendor
yesense_node by default; ``--choice M20BodyImu=serial`` reads the UART instead, with yesense_node stopped. Nothing commands the legs
unless ``--choice M20Walker=rl_deploy|controller`` gives cmd_vel a walker.
"""

from __future__ import annotations

import math
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, MetaBlueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.rockchip_h264.module import RockchipH264Cameras
from dimos.hardware.sensors.imu.yesense.module import YesenseImu
from dimos.hardware.sensors.lidar.robosense.module import Airy
from dimos.mapping.dim_slam.dim_slam import (
    DimSlam,
    ImuConfig,
    InitialStds,
    LidarConfig,
    ProcessNoise,
)
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.cloud_relocalizer import MID360_NAV
from dimos.mapping.relocalization.module import PremapModule
from dimos.navigation.bench.driver import EpisodeDriver
from dimos.navigation.embodiment.base import Embodiment
from dimos.navigation.embodiment.m20 import M20
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.native import TrajectoryFollowerNative
from dimos.robot.deeprobotics.m20 import constants
from dimos.robot.deeprobotics.m20.body_imu_sniff.module import M20BodyImuSniff
from dimos.robot.deeprobotics.m20.cmd_vel.module import M20CmdVel
from dimos.robot.deeprobotics.m20.constants import (
    GOS_SLAM_CPUS,
    MAP_FRAME,
    NOS_SLAM_CPUS,
    PLANNER_VIZ_HZ,
    PLANNER_VIZ_SWEEP_CELLS,
    VOXEL_SIZE_M,
    WALL_CLEARANCE_M,
    WORLD_FRAME,
)
from dimos.robot.deeprobotics.m20.controller.dock_detector import M20DockDetector
from dimos.robot.deeprobotics.m20.controller.module import M20Controller
from dimos.robot.deeprobotics.m20.controller.service import M20ControllerClient
from dimos.robot.deeprobotics.m20.model import (
    M20_MODEL,
    STAND_JOINT_POSITIONS,
    M20StandingJointState,
    M20StandingJointStateConfig,
)
from dimos.robot.deeprobotics.m20.rerun import m20_sim_rerun_config
from dimos.robot.urdf_manager import UrdfManager
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing
from dimos.simulation.m20_sim.world import SIM_M20, SimM20World
from dimos.visualization.vis_module import vis_module

# --- navigation and relocalization (as m20 ran them on the vendor's sensors) ---


def m20_nav_stack(
    embodiment: Embodiment = M20,
    lidar: str = "slam_body_points",
    pre_trace_subsample: int = 5,
    ray_subsample: int = 5,
    emit_every: int = 1,
) -> MetaBlueprint:
    """Voxel map, planners and follower for an M20 whose step and body limits are `embodiment`'s; the ray tracer traces the scans on `lidar`."""
    return autoconnect(
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_M,
            max_range=25.0,
            # 1 keeps every point: a lidar that voxelizes its own scans needs no more thinning here
            pre_trace_subsample=pre_trace_subsample,
            ray_subsample=ray_subsample,
            emit_every=emit_every,
            global_emit_every=50,
            world_frame=WORLD_FRAME,
            worker_threads=3,
        ).remappings([(RayTracingVoxelMap, "lidar", lidar)]),
        MLSPlannerNative.blueprint(
            world_frame=WORLD_FRAME,
            voxel_size=VOXEL_SIZE_M,
            robot_height=embodiment.height,
            start_z_offset_m=embodiment.base_height,
            wall_clearance_m=WALL_CLEARANCE_M,
            wall_buffer_m=0.85,
            wall_buffer_weight=100.0,
            step_threshold_m=embodiment.steppable,
            viz_publish_hz=PLANNER_VIZ_HZ,
            viz_sweep_cells=PLANNER_VIZ_SWEEP_CELLS,
            worker_threads=2,
        ).remappings(
            [
                (MLSPlannerNative, "global_map", "global_map_unused"),
                (MLSPlannerNative, "path", "planner_path"),
            ]
        ),
        LocalPlannerNative.blueprint(embodiment=embodiment, world_frame=WORLD_FRAME),
        TrajectoryFollowerNative.blueprint(embodiment=embodiment),
        MovementManager.blueprint(),
    )


def m20_relocalization(
    lidar: str = "slam_body_points",
    attempt_after_frames: list[str] | None = None,
    attempt_every_sweeps: int = 1,
) -> Blueprint:
    """Continuous relocalization against the premap, matching a live map built from the scans on `lidar` (each placed by tf at its stamp)."""
    return PremapModule.blueprint(
        world_frame=WORLD_FRAME,
        map_frame="premap",
        # at 0.2 m live voxels a right office match scores ~0.5-0.7 and a wrong one ~0.25-0.35
        # one core: a whole-premap search on every core starved MovementManager and the planners on GOS (98% CPU, 2026-11-21)
        relocalize=MID360_NAV.model_copy(update={"fitness_threshold": 0.65, "threads": 1}),
        # body scans placed in map (odometry), so a world -> map jump leaves the kept live map as it was
        live_cloud="lidar",
        relocalize_once=False,
        new_map=True,
        # a standing robot keeps ~6-7k live voxels at 0.2 m, so the first match waits for 2k, under the 20k default
        min_local_points=2_000,
        # coarse enough that a robot that roams the office keeps ~10k points, not ~500k (a global search in seconds, not minutes)
        live_voxel_m=0.2,
        match_radius_m=15.0,
        # a lost fix keeps refining by ICP and never falls back to a whole-premap search: that search pegged 4 GOS cores
        global_after_refusals=0,
        attempt_after_frames=attempt_after_frames or [],
        attempt_every_sweeps=attempt_every_sweeps,
    ).remappings([(PremapModule, "lidar", lidar)])


# --- dimSLAM ---


def m20_dim_slam(
    lidar_frames: tuple[str, ...] = ("front_lidar", "rear_lidar"),
    imu_frames: tuple[str, ...] = ("rear_lidar",),
    # the RK3588's A76 cores (4-7): on its A55s (0-3) the filter takes ~100% of a core and falls seconds behind
    cpus: tuple[int, ...] = (4, 5, 6, 7),
    **overrides: Any,
) -> Blueprint:
    """``imu_frames``: the first drives and levels the filter. The rear Airy's IMU, its gyro bias
    ~0.006 rad/s against the front's ~0.043; both together ran under real time on the GOS RK3588."""
    return DimSlam.blueprint(
        # no cameras: lidar-inertial only
        camera_mode=None,
        odom_frame_id=MAP_FRAME,
        output_frame_id="base_link",
        # m^2: Point-LIO's 0.01 jittered 4 mm per scan at rest on the Airys; 0.05 steadied it to 1.1 mm
        lidars=[LidarConfig(frame_id=frame, point_variance=0.05) for frame in lidar_frames],
        # an Airy IMU at rest (90 s, 200 Hz): gyro variance ~5e-7 rad^2/s^2, accel ~1e-4 m^2/s^4 per axis
        imus=[
            # Point-LIO's leveling: 100 samples, no gyro gate
            ImuConfig(
                frame_id=frame,
                gyro_variance=1e-6,
                accel_variance=1e-4,
                init_samples=100,
                init_gyro_limit=0.0,
            )
            for frame in imu_frames
        ],
        # a walking dog's body changes motion far slower than Point-LIO's handheld defaults allow
        process_noise=ProcessNoise(velocity=2.0, angular_velocity=100.0, specific_force=50.0),
        # Point-LIO's starting covariance (variance 0.01 per block, biases 1e-3) and gravity, its 99% chi-squared gate
        # on sources, and no runaway gate: what this filter was tuned with
        initial_stds=InitialStds(
            position=math.sqrt(0.01),
            velocity=math.sqrt(0.01),
            rotation=math.sqrt(0.01),
            bias=math.sqrt(0.001),
        ),
        initial_gravity_estimate=9.81,
        outlier_rejection_allowed_variance=-1.0,
        max_position_m=0.0,
        cpus=list(cpus),
        **overrides,
    ).remappings([(DimSlam, "odometry", "slam_odom")])


# --- real mode: the robot on its own drivers ---

# The `dimos host` tags of the boards: AOS has the IMU, cameras and motor bus wired to it, NOS the lidars' network.
AOS = {"board": "aos"}
NOS = {"board": "nos"}


def _airy(
    frame: str,
    group: str,
    lidar_ip: str,
    ports: tuple[int, int, int],
    mount_xyz: tuple[float, float, float],
    mount_rpy: tuple[float, float, float],
    lidar: str = "lidar",
    imu: str = "lidar_imu",
    suffix: str = "",
    publish_mount: bool = True,
    **settings: Any,
) -> MetaBlueprint:
    """One of the M20's Airys onto `lidar` and `imu`, namespaced by its frame + `suffix` (tf and the joint state global)."""
    msop_port, difop_port, imu_port = ports
    return (
        Airy.blueprint(
            frame_id=frame,
            # the IMU samples are rotated into the lidar's axes, so they share its frame
            imu_frame_id=frame,
            group=group,
            lidar_ip=lidar_ip,
            msop_port=msop_port,
            difop_port=difop_port,
            imu_port=imu_port,
            # a second pair on the same lidars leaves the mount to the first, so each mount has one publisher
            mount_parent_frame_id="base_link" if publish_mount else None,
            mount_xyz=mount_xyz,
            mount_rpy=mount_rpy,
            # the vendor driver's config.yaml: 0.1 to 60 m, PTP-synced lidar clock (settings may narrow the range)
            **{"min_distance": 0.1, "max_distance": 60.0, "use_lidar_clock": True, **settings},
        )
        .remappings(
            [
                (Airy, "lidar", lidar),
                (Airy, "imu", imu),
                (Airy, "joint_state", "coordinator_joint_state"),
            ]
        )
        .namespace(frame + suffix, expose={lidar, imu, "coordinator_joint_state", "tf"})
    )


def _airys(front: dict[str, Any], rear: dict[str, Any], **both: Any) -> MetaBlueprint:
    """Both Airys at their mounts, `both` settings to each and `front` / `rear` to one."""
    c = constants
    return autoconnect(
        _airy(
            c.FRONT_LIDAR_FRAME,
            c.FRONT_LIDAR_GROUP,
            c.FRONT_LIDAR_IP,
            c.FRONT_LIDAR_PORTS,
            c.FRONT_LIDAR_XYZ_M,
            c.FRONT_LIDAR_RPY,
            **both,
            **front,
        ),
        _airy(
            c.REAR_LIDAR_FRAME,
            c.REAR_LIDAR_GROUP,
            c.REAR_LIDAR_IP,
            c.REAR_LIDAR_PORTS,
            c.REAR_LIDAR_XYZ_M,
            c.REAR_LIDAR_RPY,
            **both,
            **rear,
        ),
    )


def _dim_slam_on_nos() -> Blueprint:
    """dimSLAM on NOS, next to its own pair of Airy drivers. NOS's lidar NIC gets the multicast as early as AOS (13.8 ms
    after the lidar's stamp on both), so the slices never cross a board, and each 10 ms slice is fused as soon as it is
    measured instead of after the whole 100 ms sweep (walk #2 replay: 198 poses/s at the same CPU and accuracy as full
    sweeps, each pose's work 1.5 ms p50 / 8.7 ms p95 against 56 ms mean)."""
    return autoconnect(
        # the lidars' own stream (body cut, full sweeps) still feeds the map and relocalization from AOS; this pair
        # is dimSLAM's alone: raw slices of ~179 blocks (10 ms of a 10 Hz sweep), voxelized as the ray tracer's
        _airys(
            {},
            {},
            lidar="lidar_slices",
            imu="lidar_slices_imu",
            suffix="_slices",
            publish_mount=False,
            body_filter=False,
            voxel=VOXEL_SIZE_M,
            split_blocks=179,
        ).hosted(tags=NOS),
        m20_dim_slam(
            cpus=NOS_SLAM_CPUS,
            # a pose per slice is ~200/s: every 4th keeps the newest pose at most 40 ms stale for its readers
            publish_every=4,
        )
        .remappings([(DimSlam, "lidar", "lidar_slices"), (DimSlam, "imu", "lidar_slices_imu")])
        .hosted(tags=NOS),
    )


def _dock_detector_on_nos() -> Blueprint:
    """The controller's dock pose and obstacle check: drivers of both Airys of their own (the shared stream drops
    intensity, which the reflector posts are found by, and voxels away the points charge_manager's obstacle check
    counts), out to the detector's range, and the detector beside them, so their full sweeps never leave NOS; only ~10
    small poses and obstacle readings a second cross to the controller on AOS."""
    c = constants
    dock_driver = {
        "lidar": "dock_lidar",
        "imu": "dock_lidar_imu",
        "suffix": "_dock",
        "publish_mount": False,
        "point_format": "full",
        "body_filter": False,
        "enable_imu": False,
        # the vendor driver's (NOS node_driver config.yaml, 0.2 to 60 m): what reflective_column_node and
        # charge_manager's obstacle check see
        "min_distance": 0.2,
        "max_distance": 60.0,
        # rounded as the vendor driver's build rounds (fused multiply-adds): reflective_column_node's points to the bit
        "fused_multiply_add": True,
    }
    return autoconnect(
        _airy(
            c.FRONT_LIDAR_FRAME,
            c.FRONT_LIDAR_GROUP,
            c.FRONT_LIDAR_IP,
            c.FRONT_LIDAR_PORTS,
            c.FRONT_LIDAR_XYZ_M,
            c.FRONT_LIDAR_RPY,
            **dock_driver,
        ).hosted(tags=NOS),
        _airy(
            c.REAR_LIDAR_FRAME,
            c.REAR_LIDAR_GROUP,
            c.REAR_LIDAR_IP,
            c.REAR_LIDAR_PORTS,
            c.REAR_LIDAR_XYZ_M,
            c.REAR_LIDAR_RPY,
            **dock_driver,
        ).hosted(tags=NOS),
        M20DockDetector.blueprint()
        .remappings([(M20DockDetector, "lidar", "dock_lidar")])
        .hosted(tags=NOS),
    )


def _python_body_filter(**config: Any) -> Blueprint:
    """The Python filter: the body cut out of every raw scan, at the joint state nearest its stamp, onto lidar."""
    return UrdfManagerPointClearing.blueprint(model=M20_MODEL, **config).remappings(
        [
            (UrdfManagerPointClearing, "pointcloud", "lidar_raw"),
            (UrdfManagerPointClearing, "filtered_pointcloud", "lidar"),
        ]
    )


def _lidars_and_body_filter(joints: str) -> ChoiceAtom:
    """Both lidars and where the body is cut out of their scans, for one joint source (standing or controller)."""
    controller = M20ControllerClient if joints == "service" else M20Controller
    joint_source: Blueprint = (
        M20StandingJointState.blueprint()
        if joints == "standing"
        else controller.blueprint()
        .remappings([(controller, "joint_state", "coordinator_joint_state")])
        .hosted(tags=AOS)
    )
    # the lidars' voxel thins the scans on the wire: the rear Airy sees ~5x the front's points at
    # the ray tracer's voxel, so it voxelizes coarser to keep both under m20's one vendor cloud; x, y, z and
    # offset_time on the wire (nothing past the body cut reads intensity or ring)
    front, rear = {"voxel": VOXEL_SIZE_M}, {"voxel": 0.12}
    # on NOS, which takes the Airys' multicast on its own lidar NIC like AOS (and its cores 0-3 are idle), so the
    # scans skip the AOS -> NOS hop; the controller's joint state still comes from AOS over zenoh. The body is the
    # URDF's collision geometry, at the standing pose unless the controller's joint state poses it
    in_driver = _airys(
        front,
        rear,
        point_format="minimal",
        body_filter=True,
        body_filter_urdf=M20_MODEL.load().xml,
        body_filter_joint_positions=dict(STAND_JOINT_POSITIONS),
        body_filter_joint_source="fixed" if joints == "standing" else "joint_state",
    ).hosted(tags=NOS)
    python = autoconnect(
        _airys(front, rear, lidar="lidar_raw", point_format="minimal", body_filter=False).hosted(
            tags=NOS
        ),
        joint_source,
        # a fixed pose republished every period is matched within that period
        _python_body_filter(state_tolerance_s=M20StandingJointStateConfig().publish_period_s)
        if joints == "standing"
        else _python_body_filter(),
    )
    return ChoiceAtom(
        name="M20BodyFilter",
        default="lidar",
        choices={
            # cut in each lidar's driver on AOS before publishing: no extra hop
            "lidar": autoconnect(joint_source, in_driver),
            "python": python,
        },
    )


# the M20 Yesense's output frame (yesense_node's data_proc length 0x31)
YESENSE_FRAME_BYTES = 0x31

_m20_real = autoconnect(
    # the body pose comes from the standing pose or the motor bus (the controller; in its default tap mode it needs
    # rl_deploy with the ecan shim)
    ChoiceAtom(
        name="M20Joints",
        default="standing",
        choices={
            "standing": _lidars_and_body_filter("standing"),
            "controller": _lidars_and_body_filter("controller"),
            # the m20-controller service on AOS (rust/service), which keeps holding the robot between runs
            "service": _lidars_and_body_filter("service"),
        },
    ),
    # who takes cmd_vel. off (default): nobody, nothing walks. rl_deploy: the vendor walking controller, through its
    # REAL_STEER topic (needs rl_deploy running, basic_server stopped). controller: the M20 controller owns the motor
    # bus (rl_deploy stopped) with its walking policy on; its gains stay off (allow_gains) until an operator turns them
    # on, so it runs the state machine without moving anything. With M20Joints=controller it is the same module.
    ChoiceAtom(
        name="M20Walker",
        default="off",
        choices={
            "off": autoconnect(),
            "rl_deploy": M20CmdVel.blueprint().hosted(tags=AOS),
            "controller": autoconnect(
                M20Controller.blueprint(
                    mode="owner",
                    policy=True,
                    # the vendor bridge's cmd_vel timeout (it resends at 10 Hz, so rl_deploy's 0.15 s never fires)
                    command_timeout_s=0.4,
                ).hosted(tags=AOS),
                # dock(): the reflector posts' pose from the front lidar
                _dock_detector_on_nos(),
            ),
            # the controller as a board service (rust/service): a run that stops leaves the robot as it is, as rl_deploy
            "service": autoconnect(
                M20ControllerClient.blueprint().hosted(tags=AOS), _dock_detector_on_nos()
            ),
        },
    ),
    # sniff: the vendor yesense_node's samples copied off its DDS traffic (needs it running); serial: the UART read
    # directly (needs yesense_node stopped, it holds the port)
    ChoiceAtom(
        name="M20BodyImu",
        default="sniff",
        choices={
            "sniff": M20BodyImuSniff.blueprint().hosted(tags=AOS),
            # the Yesense on AOS's 16550 UART, at its mount on the M20
            "serial": YesenseImu.blueprint(
                device="/dev/ttyS9",
                # AOS's vendor kernel hands this UART's bytes only to syscall 0x1cb (what yesense_node calls); read(2)
                # gets none of them
                read_syscall=0x1CB,
                # one frame per sync as yesense_node reads: a 256 B read waits to fill (5 frames, 25 ms bursts)
                frame_sync_bytes=YESENSE_FRAME_BYTES,
                frame_id=constants.IMU_FRAME,
                imu_xyz=constants.IMU_XYZ_M,
                mount_rpy_rad=constants.IMU_MOUNT_RPY_RAD,
            ).hosted(tags=AOS),
            "off": autoconnect(),
        },
    ),
    ChoiceAtom(
        name="M20Cameras",
        default="relay",
        choices={
            # the AOS board's two cameras at their full 1080p30 MJPEG, at their centers in base_link from the Deep
            # Robotics docs; the provisional M20/Whale focal length is at 1280 px wide (the vendor publishes no
            # factory calibration) and the driver scales it to the stream's width (x1.5 at 1920); the bitrate is
            # the 720p 4 Mb/s scaled by the pixel count (x2.25); no deskew
            "relay": RockchipH264Cameras.blueprint(
                width=1920,
                height=1080,
                fps=30,
                bitrate_bps=9_000_000,
                front_camera_xyz=(0.37646, 0.0, 0.03738),
                rear_camera_xyz=(-0.37646, 0.0, 0.03738),
                focal_length_px=607.0,
                focal_length_width_px=1280,
                deskew=False,
            ).hosted(tags=AOS),
            "off": autoconnect(),
        },
    ),
    # nos: dimSLAM beside its own lidar drivers on NOS, fed 10 ms slices; gos: on GOS, fed AOS's full sweeps
    ChoiceAtom(
        name="M20Slam",
        default="nos",
        choices={
            "nos": _dim_slam_on_nos(),
            "gos": m20_dim_slam(cpus=GOS_SLAM_CPUS).remappings([(DimSlam, "imu", "lidar_imu")]),
        },
    ),
    # the robot's links on tf and its model for viewers, from whichever joint state the run has, as m20
    UrdfManager.blueprint(model=M20_MODEL),
    # m20's 5/5 subsampling on top of the lidars' voxel, as m20 traces the vendor cloud; the two Airys publish a cloud
    # each per sweep, so a local map every 2nd cloud and a relocalization once both have landed keep m20's one per sweep
    m20_nav_stack(lidar="lidar", emit_every=2),
    # ... and, as Jeff asked, an attempt about once a second: every 10th sweep of the Airys' 10 Hz (a count, not a timer)
    m20_relocalization(
        lidar="lidar",
        attempt_after_frames=[constants.FRONT_LIDAR_FRAME, constants.REAR_LIDAR_FRAME],
        attempt_every_sweeps=10,
    ),
).global_config(
    n_workers=5,
    transport="zenoh",
)


# --- the modes ---

_m20_sim = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=m20_sim_rerun_config()),
    SimM20World.blueprint().remappings(
        [
            (SimM20World, "lidar", "slam_body_points"),
            (SimM20World, "odometry", "slam_odom"),
            # the sim's measured joint angles pose the same URDF the real robot shows
            (SimM20World, "joint_state", "coordinator_joint_state"),
        ]
    ),
    UrdfManager.blueprint(model=M20_MODEL),
    m20_nav_stack(SIM_M20),
    # gossip off: with it on, zenoh's pending-connection bug keeps native modules spawned together from linking
).global_config(zenoh_gossip=False, n_workers=9)

_m20_sim_bench = autoconnect(
    _m20_sim, SimM20World.blueprint(wait_for_scene=True), EpisodeDriver.blueprint()
).global_config(n_workers=10)

m20 = autoconnect(
    ChoiceAtom(
        name="M20Mode",
        default="real",
        choices={"real": _m20_real, "sim": _m20_sim, "sim-bench": _m20_sim_bench},
    )
)

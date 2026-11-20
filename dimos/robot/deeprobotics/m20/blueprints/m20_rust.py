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

"""The real M20 (``m20``'s default mode) on its own Rust drivers, with no vendor DrDDS stack: dimSLAM in place of the vendor LIO.

AOS (a ``dimos host`` tagged ``board=aos``) runs the hardware: both Airys, the Yesense body IMU, the cameras and,
with ``--choice M20Joints=ecan``, the motor-bus joint state and batteries (needs rl_deploy with the ecan shim).
NOS (``board=nos``) runs dimSLAM beside its own pair of Airy drivers, which read the same multicast on NOS's lidar
NIC and hand it 10 ms slices of each sweep (``--choice M20Slam=gos`` runs it on GOS on AOS's full sweeps instead).
Everything else runs on the machine that starts the run (GOS): the ray tracer, planners and relocalization.

Both Airys publish onto one ``lidar`` stream (their IMUs onto ``lidar_imu``), each scan in its own frame (front_lidar,
rear_lidar), with the robot's body already cut out by the lidar driver (the URDF's collision geometry at the standing
pose, or at ecan's joint state with ``--choice M20Joints=ecan``): dimSLAM tells the lidars apart by frame_id, the ray
tracer traces each from its own lidar's origin, and relocalization places each by tf at its stamp.
``--choice M20BodyFilter=python`` cuts it with the Python UrdfManagerPointClearing instead (lidars onto ``lidar_raw``,
the filter onto ``lidar``). dimSLAM runs on the rear lidar's IMU alone (two IMUs ran under real time
on the RK3588). Each Airy is namespaced but publishes its mount onto the one global tf. The cameras' module replaces ``m20-camera.service``
(``--choice M20Cameras=off`` leaves them to it); the body IMU (``imu``) is sniffed off the vendor
yesense_node by default; ``--choice M20BodyImu=serial`` reads the UART instead, with yesense_node stopped. Nothing commands the legs.
"""

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, MetaBlueprint, autoconnect
from dimos.hardware.sensors.camera.rockchip_h264.module import RockchipH264Cameras
from dimos.hardware.sensors.imu.yesense.module import YesenseImu
from dimos.hardware.sensors.lidar.robosense.module import Airy
from dimos.mapping.dim_slam_lio.dim_slam_lio import DimSlamLio
from dimos.robot.deeprobotics.m20 import constants
from dimos.robot.deeprobotics.m20.blueprints._nav import (
    VOXEL_SIZE_M,
    m20_nav_stack,
    m20_relocalization,
)
from dimos.robot.deeprobotics.m20.blueprints.m20_slam import m20_dim_slam
from dimos.robot.deeprobotics.m20.body_imu_sniff.module import M20BodyImuSniff
from dimos.robot.deeprobotics.m20.constants import GOS_SLAM_CPUS, NOS_SLAM_CPUS
from dimos.robot.deeprobotics.m20.ecan.module import M20Ecan
from dimos.robot.deeprobotics.m20.model import (
    M20_MODEL,
    STAND_JOINT_POSITIONS,
    M20StandingJointState,
    M20StandingJointStateConfig,
)
from dimos.robot.urdf_manager import UrdfManager
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing

# The `dimos host` tag of the board the lidars, IMU, cameras and motor bus are wired to.
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
            # the vendor driver's config.yaml: 0.1 to 60 m, PTP-synced lidar clock
            min_distance=0.1,
            max_distance=60.0,
            use_lidar_clock=True,
            **settings,
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
        .remappings(
            [(DimSlamLio, "lidar", "lidar_slices"), (DimSlamLio, "imu", "lidar_slices_imu")]
        )
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
    """Both lidars and where the body is cut out of their scans, for one joint source (standing or ecan)."""
    joint_source: Blueprint = (
        M20StandingJointState.blueprint()
        if joints == "standing"
        else M20Ecan.blueprint()
        .remappings([(M20Ecan, "joint_state", "coordinator_joint_state")])
        .hosted(tags=AOS)
    )
    # the lidars' voxel thins the scans on the wire: the rear Airy sees ~5x the front's points at
    # the ray tracer's voxel, so it voxelizes coarser to keep both under m20's one vendor cloud; x, y, z and
    # offset_time on the wire (nothing past the body cut reads intensity or ring)
    front, rear = {"voxel": VOXEL_SIZE_M}, {"voxel": 0.12}
    # on NOS, which takes the Airys' multicast on its own lidar NIC like AOS (and its cores 0-3 are idle), so the
    # scans skip the AOS -> NOS hop; ecan's joint state still comes from AOS over zenoh. The body is the URDF's
    # collision geometry, at the standing pose unless ecan's joint state poses it
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


_m20_real = autoconnect(
    # the body pose comes from the standing pose or the motor bus (ecan, needs rl_deploy with the shim)
    ChoiceAtom(
        name="M20Joints",
        default="standing",
        choices={
            "standing": _lidars_and_body_filter("standing"),
            "ecan": _lidars_and_body_filter("ecan"),
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
            # the AOS board's two cameras, at their centers in base_link from the Deep Robotics docs, with the
            # provisional M20/Whale focal length at 1280 px wide (the vendor publishes no factory calibration)
            "relay": RockchipH264Cameras.blueprint(
                front_camera_xyz=(0.37646, 0.0, 0.03738),
                rear_camera_xyz=(-0.37646, 0.0, 0.03738),
                focal_length_px=607.0,
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
            "gos": m20_dim_slam(cpus=GOS_SLAM_CPUS).remappings([(DimSlamLio, "imu", "lidar_imu")]),
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

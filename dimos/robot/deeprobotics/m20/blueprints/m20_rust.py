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

"""The M20 on its own Rust drivers, with no vendor DrDDS stack: ``m20`` with dimSLAM in place of the vendor LIO.

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

from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, autoconnect
from dimos.mapping.dim_slam_lio.dim_slam_lio import DimSlamLio
from dimos.robot.deeprobotics.m20.blueprints._nav import (
    VOXEL_SIZE_M,
    m20_nav_stack,
    m20_relocalization,
)
from dimos.robot.deeprobotics.m20.blueprints.m20_slam import m20_dim_slam
from dimos.robot.deeprobotics.m20.body_imu_sniff.module import M20BodyImuSniff
from dimos.robot.deeprobotics.m20.camera.module import M20CameraRelay
from dimos.robot.deeprobotics.m20.constants import GOS_SLAM_CPUS, NOS_SLAM_CPUS
from dimos.robot.deeprobotics.m20.ecan.module import M20Ecan
from dimos.robot.deeprobotics.m20.imu.module import M20Imu
from dimos.robot.deeprobotics.m20.lidar import m20_lidars
from dimos.robot.deeprobotics.m20.model import (
    M20_MODEL,
    M20StandingJointState,
    M20StandingJointStateConfig,
)
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing

# The `dimos host` tag of the board the lidars, IMU, cameras and motor bus are wired to.
AOS = {"board": "aos"}
NOS = {"board": "nos"}


def _dim_slam_on_nos() -> Blueprint:
    """dimSLAM on NOS, next to its own pair of Airy drivers. NOS's lidar NIC gets the multicast as early as AOS (13.8 ms
    after the lidar's stamp on both), so the slices never cross a board, and each 10 ms slice is fused as soon as it is
    measured instead of after the whole 100 ms sweep (walk #2 replay: 198 poses/s at the same CPU and accuracy as full
    sweeps, each pose's work 1.5 ms p50 / 8.7 ms p95 against 56 ms mean)."""
    return autoconnect(
        # the lidars' own stream (body cut, full sweeps) still feeds the map and relocalization from AOS; this pair
        # is dimSLAM's alone: raw 36 degree slices (10 ms of a 10 Hz sweep), cut at the same azimuths on both lidars so
        # dimSLAM never waits on the other lidar's offset slice, voxelized as the ray tracer's
        m20_lidars(
            lidar="lidar_slices",
            imu="lidar_slices_imu",
            namespace_suffix="_slices",
            # AOS's pair publishes the mounts
            publish_mounts=False,
            body_filter=False,
            voxel=VOXEL_SIZE_M,
            split_sectors=10,
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
    # the lidars voxelize at the ray tracer's voxel, the only thinning its scans get
    in_driver = m20_lidars(
        voxel=VOXEL_SIZE_M,
        body_filter_joint_source="fixed" if joints == "standing" else "joint_state",
    ).hosted(tags=AOS)
    python = autoconnect(
        m20_lidars(lidar="lidar_raw", voxel=VOXEL_SIZE_M, body_filter=False).hosted(tags=AOS),
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
            "lidar": in_driver if joints == "standing" else autoconnect(joint_source, in_driver),
            "python": python,
        },
    )


m20_rust = autoconnect(
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
            "serial": M20Imu.blueprint().hosted(tags=AOS),
            "off": autoconnect(),
        },
    ),
    ChoiceAtom(
        name="M20Cameras",
        default="relay",
        choices={"relay": M20CameraRelay.blueprint().hosted(tags=AOS), "off": autoconnect()},
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
    m20_nav_stack(lidar="lidar", pre_trace_subsample=1, ray_subsample=1),
    m20_relocalization(lidar="lidar"),
).global_config(
    n_workers=5,
    transport="zenoh",
)

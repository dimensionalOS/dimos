#!/usr/bin/env python3
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

"""R1 Pro ControlCoordinator, ROS connection, and Point-LIO on the chassis Mid-360.

Mirrors ``unitree_g1_coordinator.py``: the 18-DOF upper body goes through
the generic whole-body transport adapter, the holonomic chassis through the
twist-base transport adapter. ``base_link`` is placed by Point-LIO, not wheel odometry.

Usage:
    dimos run r1pro-coordinator
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, TransportSpec, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.mapping.dim_slam.dim_slam import Covariance, DimSlam, SourceConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic, Zenoh
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_CHASSIS_LIDAR_HOST_IP,
    R1PRO_CHASSIS_LIDAR_IP,
    R1PRO_WHEEL_ODOM_FRAME,
)
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS, R1ProConnection
from dimos.robot.galaxea.r1pro.head_cameras import (
    HeadCameraInfo,
    HeadLeftCamera,
    HeadRightCamera,
)
from dimos.robot.galaxea.r1pro.lio import PoseFromOdometry, R1ProLio, R1ProLioConfig, R1ProPointLio
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_chassis_joints = make_twist_base_joints("chassis")


def _r1pro_rerun_blueprint() -> Any:
    """Two-tab viewer layout: main (head stereo + 3D) and all cameras + depth.

    Entity paths assume the bridge's default ``entity_prefix="world"``.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    main_tab = rrb.Horizontal(
        rrb.Vertical(
            rrb.Spatial2DView(origin="world/head_left_color", name="Head left"),
            rrb.Spatial2DView(origin="world/head_right_color", name="Head right"),
        ),
        rrb.Spatial3DView(
            origin="world",
            name="3D",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.5),
            ),
        ),
        column_shares=[1, 2],
        name="Head + 3D",
    )

    cameras_tab = rrb.Grid(
        # Row 1 — RGB.
        rrb.Spatial2DView(origin="world/head_left_color", name="Head left"),
        rrb.Spatial2DView(origin="world/head_right_color", name="Head right"),
        rrb.Spatial2DView(origin="world/wrist_left_color", name="Wrist left"),
        rrb.Spatial2DView(origin="world/wrist_right_color", name="Wrist right"),
        # Row 2 — depth.
        rrb.Spatial2DView(origin="world/head_depth", name="Head depth"),
        rrb.Spatial2DView(origin="world/wrist_left_depth", name="Wrist left depth"),
        rrb.Spatial2DView(origin="world/wrist_right_depth", name="Wrist right depth"),
        grid_columns=4,
        name="All cameras",
    )

    return rrb.Blueprint(
        rrb.Tabs(main_tab, cameras_tab),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


# Per-entity rate caps for the rerun bridge (visualization only — the
# coordinator sees full rate). Sized for an on-robot deployment viewed over
# WiFi. Keys are rerun entity paths.
_RERUN_MAX_HZ = {
    "world/head_left_color": 5.0,
    "world/head_right_color": 5.0,
    "world/wrist_left_color": 5.0,
    "world/wrist_right_color": 5.0,
    # Raw float32 Points3D; uncapped it out-bytes every camera stream.
    "world/lidar": 5.0,
}


# Per-topic overrides for the rerun bridge (None = suppress entirely).
_RERUN_VISUAL_OVERRIDE = {
    # Raw depth frames are the heaviest payloads on the viewer link.
    "world/wrist_left_depth": None,
    "world/wrist_right_depth": None,
    "world/head_depth": None,
}


rerun_config = {
    "blueprint": _r1pro_rerun_blueprint,
    "pubsubs": [Zenoh()],
    "rerun_open": global_config.rerun_open,
    "rerun_web": global_config.rerun_web,
    # A live viewer needs only a small rolling buffer, and every viewer
    # (re)connect replays the whole buffer before going live.
    "memory_limit": "256MB",
    "max_hz": _RERUN_MAX_HZ,
    "visual_override": _RERUN_VISUAL_OVERRIDE,
}


def r1pro_visualization() -> Blueprint:
    if global_config.viewer == "rerun":
        return autoconnect(
            RerunBridgeModule.blueprint(**rerun_config),
            RerunWebSocketServer.blueprint(),
        )
    if global_config.viewer == "none":
        return Blueprint(blueprints=())
    raise ValueError(f"Unsupported viewer: {global_config.viewer}")


def _zenoh_transport(
    topic: str,
    msg_type: type,
    *,
    latest_wins: bool = False,
) -> TransportSpec:
    return ZenohTransport.spec(
        ZenohTopic(
            f"dimos/{topic.lstrip('/')}",
            msg_type,
            qos=QOS_LATEST_WINS if latest_wins else None,
        )
    )


def _head_cameras() -> list[Blueprint]:
    """Both head eyes and their intrinsics; see ``head_cameras``."""
    return [HeadLeftCamera.blueprint(), HeadRightCamera.blueprint(), HeadCameraInfo.blueprint()]


def r1pro_control(
    *,
    tasks: Sequence[TaskConfig] | None = None,
    **connection: Any,
) -> Blueprint:
    """R1ProConnection, ControlCoordinator and the head cameras.

    ``tasks`` overrides the default task set (whole-body trajectory + chassis
    velocity); transports and remappings stay identical either way.
    Other keyword args are ``R1ProConnectionConfig`` fields.
    """
    resolved_tasks = (
        list(tasks)
        if tasks is not None
        else [
            joint_trajectory_task(R1PRO_UPPER_BODY_JOINTS),
            TaskConfig(
                name="vel_chassis",
                type="velocity",
                joint_names=_chassis_joints,
                priority=10,
            ),
        ]
    )

    return (
        autoconnect(
            R1ProConnection.blueprint(**connection),
            *_head_cameras(),
            ControlCoordinator.blueprint(
                tick_rate=100,
                hardware=[
                    HardwareComponent(
                        hardware_id="r1pro",
                        hardware_type=HardwareType.WHOLE_BODY,
                        joints=R1PRO_UPPER_BODY_JOINTS,
                        adapter_type="transport_lcm",
                    ),
                    HardwareComponent(
                        hardware_id="chassis",
                        hardware_type=HardwareType.BASE,
                        joints=_chassis_joints,
                        adapter_type="transport_lcm",
                    ),
                ],
                tasks=resolved_tasks,
            ),
        )
        # Rename so the chassis adapter owns the canonical /chassis/* names.
        .remappings(
            [
                (R1ProConnection, "cmd_vel", "chassis_cmd_vel"),
                (R1ProConnection, "odom", "chassis_odom"),
                (HeadLeftCamera, "jpeg_out", "head_left_color"),
                (HeadRightCamera, "jpeg_out", "head_right_color"),
                (HeadCameraInfo, "left_info", "head_left_info"),
                (HeadCameraInfo, "right_info", "head_right_info"),
            ]
        )
        .transports(
            {
                # WholeBody bridge (hw_id="r1pro"). TransportWholeBodyAdapter
                # builds /{hw}/motor_states|imu|motor_command itself, so these
                # three topics are fixed by hardware_id, not a naming choice.
                # Only one IMU goes to /r1pro/imu.
                ("motor_states", JointState): _zenoh_transport("/r1pro/motor_states", JointState),
                ("imu_chassis", Imu): _zenoh_transport("/r1pro/imu", Imu),
                ("imu_torso", Imu): _zenoh_transport("/imu_torso", Imu),
                ("motor_command", MotorCommandArray): _zenoh_transport(
                    "/r1pro/motor_command", MotorCommandArray
                ),
                # Twist bridge (hw_id="chassis").
                ("chassis_cmd_vel", Twist): _zenoh_transport("/chassis/cmd_vel", Twist),
                ("chassis_odom", PoseStamped): _zenoh_transport("/chassis/odom", PoseStamped),
                # Wheel odometry (pose + twist) for navigation consumers.
                ("odometry", Odometry): _zenoh_transport("/odometry", Odometry),
                # Public Twist bus: any module's cmd_vel Out drives the
                # coordinator's twist_command In.
                ("cmd_vel", Twist): _zenoh_transport("/cmd_vel", Twist),
                ("twist_command", Twist): _zenoh_transport("/cmd_vel", Twist),
                # Sensor pass-throughs.
                ("head_left_color", CompressedImage): _zenoh_transport(
                    "/head_left_color", CompressedImage, latest_wins=True
                ),
                ("head_right_color", CompressedImage): _zenoh_transport(
                    "/head_right_color", CompressedImage, latest_wins=True
                ),
                ("head_left_info", CameraInfo): _zenoh_transport("/head_left_info", CameraInfo),
                ("head_right_info", CameraInfo): _zenoh_transport("/head_right_info", CameraInfo),
                ("head_depth", Image): _zenoh_transport("/head_depth", Image, latest_wins=True),
                ("wrist_left_color", CompressedImage): _zenoh_transport(
                    "/wrist_left_color", CompressedImage, latest_wins=True
                ),
                ("wrist_left_depth", Image): _zenoh_transport(
                    "/wrist_left_depth", Image, latest_wins=True
                ),
                ("wrist_right_color", CompressedImage): _zenoh_transport(
                    "/wrist_right_color", CompressedImage, latest_wins=True
                ),
                ("wrist_right_depth", Image): _zenoh_transport(
                    "/wrist_right_depth", Image, latest_wins=True
                ),
                # ControlCoordinator outs.
                ("coordinator_joint_state", JointState): _zenoh_transport(
                    "/coordinator/joint_state", JointState
                ),
                ("joint_command", JointState): _zenoh_transport("/r1pro/joint_command", JointState),
            }
        )
        .global_config(transport="zenoh")
    )


def r1pro_lidar_odometry(mount_under_base: bool = False) -> Blueprint:
    """Our Mid-360 driver (per-point times, its own IMU) into Point-LIO, plus the mount tf."""
    lidar_frame = R1ProLioConfig.model_fields["lidar_frame"].default
    return autoconnect(
        mid360_for_pointlio(
            frame_id=lidar_frame,
            lidar_ip=R1PRO_CHASSIS_LIDAR_IP,
            host_ip=R1PRO_CHASSIS_LIDAR_HOST_IP,
        ),
        R1ProPointLio.blueprint(
            sensor_frame_id=lidar_frame,
            # Standing still at 0.01 the pose stepped ~7 mm a scan (the robot jittered in the viewer); 1.0 took it to 2.5 mm.
            lidar_meas_cov=1.0,
        ).remappings([(R1ProPointLio, "odometry", "pointlio_odometry")]),
        R1ProLio.blueprint(mount_under_base=mount_under_base),
    ).remappings(
        [
            (R1ProLio, "odometry", "pointlio_odometry"),
            # The name the planners already read, unless something else owns base_link's pose.
            (R1ProLio, "pose", "pointlio_base_pose" if mount_under_base else "chassis_odom"),
        ]
    )


# Without an IMU dimSLAM blends the sources' deltas by inverse variance: Point-LIO gets ~99% of x, y and yaw, and z,
# roll and pitch outright. The wheels mostly carry the pose while Point-LIO is silent (dimSLAM's 0.5 s timeout). Replaying
# a 30 m R1 drive, the fused pose stayed within 2.5 cm (mean) of Point-LIO's.
POINTLIO_SOURCE = SourceConfig(
    parent_frame_id="odom",
    child_frame_id="base_link",
    pose_variances=Covariance(x=1e-5, y=1e-5, z=1e-5, roll=1e-5, pitch=1e-5, yaw=1e-5),
)
WHEEL_SOURCE = SourceConfig(
    parent_frame_id=R1PRO_WHEEL_ODOM_FRAME,
    child_frame_id="base_link",
    pose_variances=Covariance(x=1e-3, y=1e-3, yaw=1e-3),
)


def r1pro_fused_odometry() -> Blueprint:
    """Point-LIO (carried to base_link) and wheel odometry into dimSLAM, which owns odom -> base_link.

    Needs ``r1pro_control(odom_frame_id=R1PRO_WHEEL_ODOM_FRAME)`` with its ``odometry`` remapped to ``slam_sources``.
    No cameras or IMUs: cuVSLAM on the head stereo would compete with Point-LIO for the Orin's cores. To add them, set
    ``cameras`` and remap ``dimslam_image``/``dimslam_camera_info`` onto the head stereo streams.
    """
    return autoconnect(
        r1pro_lidar_odometry(mount_under_base=True),
        DimSlam.blueprint(
            # Required, but nothing feeds the camera inputs, so the tracker never starts.
            camera_mode="stereo",
            odom_sources=[POINTLIO_SOURCE, WHEEL_SOURCE],
        ),
        PoseFromOdometry.blueprint(),
    ).remappings(
        [
            # Point-LIO's own odom -> lidar edge would give base_link's subtree a second root.
            (R1ProPointLio, "tf", "pointlio_tf"),
            (R1ProLio, "base_odometry", "slam_sources"),
            (DimSlam, "odom_sources", "slam_sources"),
            (DimSlam, "odometry", "slam_odometry"),
            (DimSlam, "image", "dimslam_image"),
            (DimSlam, "camera_info", "dimslam_camera_info"),
            (DimSlam, "depth_image", "dimslam_depth_image"),
            (DimSlam, "depth_camera_info", "dimslam_depth_camera_info"),
            (DimSlam, "depth_cloud", "dimslam_depth_cloud"),
            (DimSlam, "imu", "dimslam_imu"),
            (PoseFromOdometry, "odometry", "slam_odometry"),
            # The name the planners already read.
            (PoseFromOdometry, "pose", "chassis_odom"),
        ]
    )


# n_workers keeps the 100 Hz coordinator tick loop out of the interpreter running the connection's sensor threads.
r1pro_coordinator = autoconnect(
    r1pro_visualization(),
    # Off, so base_link has exactly one parent: Point-LIO's, through the mount.
    r1pro_control(publish_odom_tf=False),
    r1pro_lidar_odometry(),
).global_config(n_workers=4)

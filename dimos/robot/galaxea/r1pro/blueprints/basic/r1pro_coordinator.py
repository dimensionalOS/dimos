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

The R1 Pro is driven through ``R1ProConnection`` alone: the 18-DOF upper body
takes positions and the holonomic chassis takes speeds, both through the
coordinator's per-robot ports — no transport adapters in between.
``base_link`` is placed by Point-LIO, not wheel odometry.

Usage:
    dimos run r1pro-coordinator
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In, Out, TransportSpec
from dimos.core.transport import ZenohTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic, Zenoh
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_CHASSIS_LIDAR_HOST_IP,
    R1PRO_CHASSIS_LIDAR_IP,
    R1PRO_COORDINATOR_MODEL,
)
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_GRIPPER_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnection,
)
from dimos.robot.galaxea.r1pro.head_cameras import (
    HeadCameraInfo,
    HeadLeftCamera,
    HeadRightCamera,
)
from dimos.robot.galaxea.r1pro.lio import R1ProLio, R1ProLioConfig
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


class R1ProCoordinator(ControlCoordinator):
    """The R1 Pro driven through ``R1ProConnection``, robot name "r1pro".

    ``r1pro_position_command`` carries the 18 upper-body joints' targets and
    the two jaws' openings, ``r1pro_cmd_vel`` the chassis' speeds;
    ``r1pro_joint_state`` is the joint feedback and ``r1pro_imu`` the chassis
    IMU (the torso IMU stays on the bus, as before). The jaws are
    command-only joints: never held, never zero-filled, and their feedback
    never gates readiness.
    """

    r1pro_position_command: Out[JointState]
    r1pro_cmd_vel: Out[Twist]
    r1pro_joint_state: In[JointState]
    r1pro_imu: In[Imu]


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
    """R1ProConnection, R1ProCoordinator and the head cameras.

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
                joint_names=R1PRO_BASE_JOINTS,
                priority=10,
            ),
        ]
    )

    return (
        autoconnect(
            R1ProConnection.blueprint(**connection),
            *_head_cameras(),
            R1ProCoordinator.blueprint(
                instance_name="ControlCoordinator",
                tick_rate=100,
                model=R1PRO_COORDINATOR_MODEL,
                command_only_joints=list(R1PRO_GRIPPER_JOINTS),
                tasks=resolved_tasks,
            ),
        )
        # The connection's ports onto the coordinator's per-robot ports, and
        # the head cameras onto their public names.
        .remappings(
            [
                (R1ProConnection, "position_command", "r1pro_position_command"),
                (R1ProConnection, "cmd_vel", "r1pro_cmd_vel"),
                (R1ProCoordinator, "r1pro_joint_state", "joint_state"),
                # The chassis IMU feeds the coordinator's sensor state, as it
                # did through the whole-body adapter; the torso IMU stays a
                # plain bus stream.
                (R1ProCoordinator, "r1pro_imu", "imu_chassis"),
                # Keeps its name for the navigation consumers that read it.
                (R1ProConnection, "odom", "chassis_odom"),
                (HeadLeftCamera, "jpeg_out", "head_left_color"),
                (HeadRightCamera, "jpeg_out", "head_right_color"),
                (HeadCameraInfo, "left_info", "head_left_info"),
                (HeadCameraInfo, "right_info", "head_right_info"),
            ]
        )
        .transports(
            {
                # Both IMUs stay on the bus under their old topics.
                ("imu_chassis", Imu): _zenoh_transport("/r1pro/imu", Imu),
                ("imu_torso", Imu): _zenoh_transport("/imu_torso", Imu),
                # Wheel odometry (pose + twist) for navigation consumers.
                ("odometry", Odometry): _zenoh_transport("/odometry", Odometry),
                ("chassis_odom", PoseStamped): _zenoh_transport("/chassis/odom", PoseStamped),
                # Public Twist bus: any module's cmd_vel Out drives the
                # coordinator's twist_command In, which the chassis velocity
                # task claims.
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
            }
        )
        .global_config(transport="zenoh")
    )


def r1pro_lidar_odometry() -> Blueprint:
    """Our Mid-360 driver (per-point times, its own IMU) into Point-LIO, plus the mount tf."""
    lidar_frame = R1ProLioConfig.model_fields["lidar_frame"].default
    return autoconnect(
        mid360_for_pointlio(
            frame_id=lidar_frame,
            lidar_ip=R1PRO_CHASSIS_LIDAR_IP,
            host_ip=R1PRO_CHASSIS_LIDAR_HOST_IP,
        ),
        PointLio.blueprint(sensor_frame_id=lidar_frame).remappings(
            [(PointLio, "odometry", "pointlio_odometry")]
        ),
        R1ProLio.blueprint(),
    ).remappings(
        [
            (R1ProLio, "odometry", "pointlio_odometry"),
            # The name the planners already read.
            (R1ProLio, "pose", "chassis_odom"),
        ]
    )


# n_workers keeps the 100 Hz coordinator tick loop out of the interpreter running the connection's sensor threads.
r1pro_coordinator = autoconnect(
    r1pro_visualization(),
    # Off, so base_link has exactly one parent: Point-LIO's, through the mount.
    r1pro_control(publish_odom_tf=False),
    r1pro_lidar_odometry(),
).global_config(n_workers=4)

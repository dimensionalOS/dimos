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

"""R1 Pro ControlCoordinator: R1ProConnection Module + transport_lcm bridges.

Mirrors ``unitree_g1_coordinator.py``: the 18-DOF upper body goes through
the generic whole-body transport adapter, the holonomic chassis through the
twist-base transport adapter.

Usage:
    dimos run r1pro-coordinator
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import JpegLcmTransport, LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS, R1ProConnection
from dimos.visualization.vis_module import vis_module

_chassis_joints = make_twist_base_joints("chassis")


def _r1pro_rerun_blueprint() -> Any:
    """Two-tab viewer layout: main (head stereo + 3D) and all cameras + depth.

    Entity paths assume the bridge's default ``entity_prefix="world"`` — so
    LCM topic ``/r1pro/head_left_color`` lands at ``world/r1pro/head_left_color``.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    main_tab = rrb.Horizontal(
        rrb.Vertical(
            rrb.Spatial2DView(origin="world/r1pro/head_left_color", name="Head left"),
            rrb.Spatial2DView(origin="world/r1pro/head_right_color", name="Head right"),
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
        rrb.Spatial2DView(origin="world/r1pro/head_left_color", name="Head left"),
        rrb.Spatial2DView(origin="world/r1pro/head_right_color", name="Head right"),
        rrb.Spatial2DView(origin="world/r1pro/wrist_left_color", name="Wrist left"),
        rrb.Spatial2DView(origin="world/r1pro/wrist_right_color", name="Wrist right"),
        # Row 2 — depth.
        rrb.Spatial2DView(origin="world/r1pro/head_depth", name="Head depth"),
        rrb.Spatial2DView(origin="world/r1pro/wrist_left_depth", name="Wrist left depth"),
        rrb.Spatial2DView(origin="world/r1pro/wrist_right_depth", name="Wrist right depth"),
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
# WiFi. Keys are rerun entity paths (entity_prefix "world" + the LCM topic).
_RERUN_MAX_HZ = {
    "world/r1pro/head_left_color": 5.0,
    "world/r1pro/head_right_color": 5.0,
    "world/r1pro/wrist_left_color": 5.0,
    "world/r1pro/wrist_right_color": 5.0,
    # Raw float32 Points3D; uncapped it out-bytes every camera stream.
    "world/r1pro/lidar": 5.0,
}


def _compress_color_for_viewer(msg: Any) -> Any:
    """Re-encode a color frame to JPEG before it enters the rerun stream —
    raw RGB frames overwhelm a WiFi-connected viewer."""
    import rerun as rr

    try:
        data = getattr(msg, "data", None)
        if data is None:
            return msg  # not an Image — let the bridge's default conversion run
        image = rr.Image(data)
        compress = getattr(image, "compress", None)
        return compress(jpeg_quality=75) if compress is not None else image
    except Exception:
        return msg  # fall back to the bridge's default conversion


# Per-topic overrides for the rerun bridge (None = suppress entirely).
_RERUN_VISUAL_OVERRIDE = {
    "world/r1pro/head_left_color": _compress_color_for_viewer,
    "world/r1pro/head_right_color": _compress_color_for_viewer,
    "world/r1pro/wrist_left_color": _compress_color_for_viewer,
    "world/r1pro/wrist_right_color": _compress_color_for_viewer,
    # Raw depth frames are the heaviest payloads on the viewer link.
    "world/r1pro/wrist_left_depth": None,
    "world/r1pro/wrist_right_depth": None,
    "world/r1pro/head_depth": None,
}


rerun_config = {
    "blueprint": _r1pro_rerun_blueprint,
    "pubsubs": [LCM()],
    # A live viewer needs only a small rolling buffer, and every viewer
    # (re)connect replays the whole buffer before going live.
    "memory_limit": "256MB",
    "max_hz": _RERUN_MAX_HZ,
    "visual_override": _RERUN_VISUAL_OVERRIDE,
}


def r1pro_control(
    *,
    tasks: Sequence[TaskConfig] | None = None,
) -> Blueprint:
    """R1ProConnection + ControlCoordinator wired over LCM transports.

    ``tasks`` overrides the default task set (whole-body servo + chassis
    velocity); transports and remappings stay identical either way.
    """
    resolved_tasks = (
        list(tasks)
        if tasks is not None
        else [
            TaskConfig(
                name="servo_r1pro",
                type="servo",
                joint_names=R1PRO_UPPER_BODY_JOINTS,
                priority=10,
            ),
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
            R1ProConnection.blueprint(),
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
            ]
        )
        .transports(
            {
                # WholeBody bridge (hw_id="r1pro"). TransportWholeBodyAdapter
                # subscribes /{hw}/imu — only one IMU goes there.
                ("motor_states", JointState): LCMTransport("/r1pro/motor_states", JointState),
                ("imu_chassis", Imu): LCMTransport("/r1pro/imu", Imu),
                ("imu_torso", Imu): LCMTransport("/r1pro/imu_torso", Imu),
                ("motor_command", MotorCommandArray): LCMTransport(
                    "/r1pro/motor_command", MotorCommandArray
                ),
                # Twist bridge (hw_id="chassis").
                ("chassis_cmd_vel", Twist): LCMTransport("/chassis/cmd_vel", Twist),
                ("chassis_odom", PoseStamped): LCMTransport("/chassis/odom", PoseStamped),
                # Wheel odometry (pose + twist) for navigation consumers.
                ("odometry", Odometry): LCMTransport("/r1pro/odometry", Odometry),
                # Public Twist bus: any module's cmd_vel Out drives the
                # coordinator's twist_command In.
                ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
                ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
                # Sensor pass-throughs.
                ("head_left_color", Image): JpegLcmTransport("/r1pro/head_left_color", Image),
                ("head_right_color", Image): JpegLcmTransport("/r1pro/head_right_color", Image),
                ("head_depth", Image): LCMTransport("/r1pro/head_depth", Image),
                ("lidar", PointCloud2): LCMTransport("/r1pro/lidar", PointCloud2),
                ("wrist_left_color", Image): JpegLcmTransport("/r1pro/wrist_left_color", Image),
                ("wrist_left_depth", Image): LCMTransport("/r1pro/wrist_left_depth", Image),
                ("wrist_right_color", Image): JpegLcmTransport("/r1pro/wrist_right_color", Image),
                ("wrist_right_depth", Image): LCMTransport("/r1pro/wrist_right_depth", Image),
                # ControlCoordinator outs.
                ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
                ("joint_command", JointState): LCMTransport("/r1pro/joint_command", JointState),
            }
        )
    )


# n_workers keeps the 100 Hz coordinator tick loop out of the interpreter
# that runs the connection's camera-decode threads.
r1pro_coordinator = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    r1pro_control(),
).global_config(n_workers=4)


__all__ = ["r1pro_control", "r1pro_coordinator", "rerun_config"]

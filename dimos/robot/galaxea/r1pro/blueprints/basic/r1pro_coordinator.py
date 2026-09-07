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

"""R1 Pro ControlCoordinator and ROS connection.

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
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, TransportSpec, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.core.transport_factory import make_transport
from dimos.hardware.spec import JointLimits
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic, Zenoh
from dimos.robot.galaxea.r1pro.connection import (
    GRIPPER_POSITION_RANGE,
    R1PRO_COMMAND_JOINTS,
    R1PRO_GRIPPER_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnection,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_chassis_joints = make_twist_base_joints("chassis")


def r1pro_whole_body_hardware() -> HardwareComponent:
    """The 18-DOF upper body plus both grippers, behind the ROS bridge."""
    lo, hi = GRIPPER_POSITION_RANGE
    n_arm = len(R1PRO_UPPER_BODY_JOINTS)
    n_gripper = len(R1PRO_GRIPPER_JOINTS)
    return HardwareComponent(
        hardware_id="r1pro",
        hardware_type=HardwareType.WHOLE_BODY,
        joints=R1PRO_COMMAND_JOINTS,
        adapter_type="transport_lcm",
        adapter_kwargs={"transport_cls": make_transport},
        # GripperControlTask normalizes 0..1 against these. The arm and torso
        # joints are left unknown: the vendor driver enforces their limits and
        # nothing here needs to resolve them.
        limits=JointLimits(
            position_lower=[None] * n_arm + [lo] * n_gripper,
            position_upper=[None] * n_arm + [hi] * n_gripper,
            velocity_max=[None] * (n_arm + n_gripper),
        ),
    )


def r1pro_gripper_tasks(priority: int = 20, *, per_hand: bool = False) -> list[TaskConfig]:
    """One task per gripper.

    By default both listen on the coordinator's ``gripper_command``, which
    routes broadcast, so a single normalized command drives both hands
    together — what the one-button operator UIs send.

    ``per_hand`` binds each task to its own ``{side}_gripper_command`` instead,
    which is what a VR operator needs: ArmTeleopModule publishes each
    controller's analog trigger on those streams, and every other arm blueprint
    in the tree binds them the same way. Without it the trigger value reaches
    the coordinator and is dropped, because no task is bound to the port.
    """
    tasks = []
    for joint in R1PRO_GRIPPER_JOINTS:
        name = joint.rsplit("/", 1)[-1]
        bind = {"gripper_command": f"{name}_command"} if per_hand else None
        tasks.append(
            TaskConfig(
                name=name,
                type="gripper",
                joint_names=[joint],
                priority=priority,
                **({"stream_bind": bind} if bind else {}),
            )
        )
    return tasks


def r1pro_chassis_hardware() -> HardwareComponent:
    """The holonomic chassis as twist-base virtual joints."""
    return HardwareComponent(
        hardware_id="chassis",
        hardware_type=HardwareType.BASE,
        joints=_chassis_joints,
        adapter_type="transport_lcm",
        adapter_kwargs={"transport_cls": make_transport},
    )


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


def r1pro_control(
    *,
    tasks: Sequence[TaskConfig] | None = None,
    coordinator_cls: type[ControlCoordinator] = ControlCoordinator,
) -> Blueprint:
    """R1ProConnection and ControlCoordinator.

    ``tasks`` overrides the default task set (whole-body trajectory + chassis
    velocity); transports and remappings stay identical either way.
    ``coordinator_cls`` selects a subclass that carries extra input ports, such
    as ``TeleopControlCoordinator``.
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
            *r1pro_gripper_tasks(),
        ]
    )

    return (
        autoconnect(
            R1ProConnection.blueprint(),
            coordinator_cls.blueprint(
                tick_rate=100,
                hardware=[
                    r1pro_whole_body_hardware(),
                    r1pro_chassis_hardware(),
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
                ("head_depth", Image): _zenoh_transport("/head_depth", Image, latest_wins=True),
                ("lidar", PointCloud2): _zenoh_transport("/lidar", PointCloud2, latest_wins=True),
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


# n_workers keeps the 100 Hz coordinator tick loop out of the interpreter
# that runs the connection's sensor threads.
r1pro_coordinator = autoconnect(
    r1pro_visualization(),
    r1pro_control(),
).global_config(n_workers=4)

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

"""Mobile manipulation coordinator blueprints.

Usage:
    dimos run coordinator-mock-twist-base                # Mock holonomic base
    dimos run coordinator-mobile-manip-mock              # Mock arm + base
    dimos run coordinator-flowbase                       # FlowBase holonomic base (Portal RPC)
    dimos run coordinator-flowbase-keyboard-teleop       # FlowBase + WASD pygame teleop
    dimos run coordinator-flowbase-vis                   # FlowBase + Rerun visualization
    dimos run coordinator-flowbase-nav                   # FlowBase + FastLio2 + nav stack (click-to-drive)
"""

from __future__ import annotations

import os

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_twist_base_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.navigation.nav_stack.modules.simple_planner.simple_planner import SimplePlanner
from dimos.robot.catalog.ufactory import xarm7 as _catalog_xarm7
from dimos.robot.unitree.g1.config import G1_LOCAL_PLANNER_PRECOMPUTED_PATHS
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module

_base_joints = make_twist_base_joints("base")


def _mock_twist_base(hw_id: str = "base") -> HardwareComponent:
    """Mock holonomic twist base (3-DOF: vx, vy, wz)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type="mock_twist_base",
    )


def _flowbase_twist_base(
    hw_id: str = "base",
    address: str = "172.6.2.20:11323",
) -> HardwareComponent:
    """FlowBase holonomic platform via Portal RPC (3-DOF: vx, vy, wz)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type="flowbase",
        address=address,
    )


# Mock holonomic twist base (3-DOF: vx, vy, wz)
coordinator_mock_twist_base = ControlCoordinator.blueprint(
    hardware=[_mock_twist_base()],
    tasks=[
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
    }
)

# FlowBase holonomic twist base (3-DOF: vx, vy, wz) over Portal RPC
coordinator_flowbase = ControlCoordinator.blueprint(
    hardware=[_flowbase_twist_base()],
    tasks=[
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
    }
)

# FlowBase + WASD pygame keyboard teleop in a single blueprint
coordinator_flowbase_keyboard_teleop = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_flowbase_twist_base()],
        tasks=[
            TaskConfig(
                name="vel_base",
                type="velocity",
                joint_names=_base_joints,
                priority=10,
            ),
        ],
    ),
    KeyboardTeleop.blueprint(),
).transports(
    {
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# FlowBase + Rerun + WebSocket dashboard (on-screen joystick at http://localhost:7779)
# rerun_open="both" → native Rerun GUI + dashboard served at /. Without this, / would
# redirect to the deprecated /command-center route, which returns "not built".
coordinator_flowbase_vis = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_flowbase_twist_base()],
        tasks=[
            TaskConfig(
                name="vel_base",
                type="velocity",
                joint_names=_base_joints,
                priority=10,
            ),
        ],
    ),
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config={"rerun_open": "both"},
    ),
).transports(
    {
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        # Route the dashboard joystick's Twist to the coordinator's /cmd_vel topic.
        ("tele_cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# FlowBase + Livox MID-360 + FastLio2 SLAM + nav stack with click-to-drive in Rerun.
# Rerun's clicked_point: Out[PointStamped] (from RerunWebSocketServer) is remapped
# straight into SimplePlanner's goal: In[PointStamped] — no MovementManager needed.
# PathFollower.cmd_vel is renamed to "nav_cmd_vel" inside create_nav_stack; we route
# both nav_cmd_vel and the coordinator's twist_command to LCM /cmd_vel so the existing
# JointVelocityTask passthrough drives the FlowBase adapter.
#
# Override LIDAR network at runtime: LIDAR_HOST_IP=<your-ip> LIDAR_IP=<lidar-ip>.
# MID-360 mount: 20cm forward (+x), 20cm right (-y in ROS), 10cm above base center (+z).
# Identity orientation assumes level mount; refine quaternion if tilted.
_flowbase_mid360_mount = Pose(0.20, -0.20, 0.10, *Quaternion.from_euler(Vector3(0, 0, 0)))

coordinator_flowbase_nav = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.1.5"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.1.189"),
            mount=_flowbase_mid360_mount,
            map_freq=1.0,
            config="default.yaml",
        ),
        create_nav_stack(
            planner="simple",
            vehicle_height=0.5,  # FlowBase platform clearance — tune if needed
            max_speed=0.8,  # conservative starting point
            terrain_analysis={
                # MID-360 is mounted ~10cm above base (close to floor); G1 has it at ~1.2m.
                # Looser thresholds avoid classifying floor noise as obstacles.
                "obstacle_height_threshold": 0.15,
                "ground_height_threshold": 0.10,
                "sensor_range": 20,
            },
            local_planner={
                # Reusing G1's precomputed paths until FlowBase-specific ones exist.
                "paths_dir": str(G1_LOCAL_PLANNER_PRECOMPUTED_PATHS),
                "publish_free_paths": False,
            },
            simple_planner={
                "cell_size": 0.2,
                "obstacle_height_threshold": 0.15,
                "inflation_radius": 0.3,  # FlowBase footprint smaller than G1's 0.5
                "lookahead_distance": 2.0,
                "replan_rate": 5.0,
                "replan_cooldown": 2.0,
            },
        ),
        ControlCoordinator.blueprint(
            hardware=[_flowbase_twist_base()],
            tasks=[
                TaskConfig(
                    name="vel_base",
                    type="velocity",
                    joint_names=_base_joints,
                    priority=10,
                ),
            ],
        ),
        # Rerun directly (no vis_module helper — we don't want WebsocketVisModule's
        # dashboard/command-center). RerunWebSocketServer owns the click → clicked_point
        # bridge for clicks made inside the Rerun 3D viewer itself.
        # rerun_open="both" → spawn the native viewer AND serve the web viewer (so the
        # Wayland/Vulkan native path is bypassable by opening http://localhost:9878/).
        RerunBridgeModule.blueprint(
            **nav_stack_rerun_config({"memory_limit": "1GB"}, vis_throttle=0.5),
            rerun_open="both",
        ),
        RerunWebSocketServer.blueprint(),
    )
    .remappings(
        [
            (FastLio2, "lidar", "registered_scan"),
            # Rerun click → planner goal, skipping MovementManager.
            (SimplePlanner, "goal", "clicked_point"),
        ]
    )
    .transports(
        {
            # nav_cmd_vel (PathFollower output, remapped inside create_nav_stack) and
            # coordinator's twist_command both ride /cmd_vel.
            ("nav_cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(n_workers=8)
)


# Mock arm (7-DOF) + mock holonomic base (3-DOF)
_mock_arm_cfg = _catalog_xarm7(name="arm")

coordinator_mobile_manip_mock = ControlCoordinator.blueprint(
    hardware=[_mock_arm_cfg.to_hardware_component(), _mock_twist_base()],
    tasks=[
        _mock_arm_cfg.to_task_config(task_name="traj_arm"),
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
    }
)


__all__ = [
    "coordinator_flowbase",
    "coordinator_flowbase_keyboard_teleop",
    "coordinator_flowbase_nav",
    "coordinator_flowbase_vis",
    "coordinator_mobile_manip_mock",
    "coordinator_mock_twist_base",
]

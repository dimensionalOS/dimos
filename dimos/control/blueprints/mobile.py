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
    dimos run coordinator-flowbase-nav                   # FlowBase + FastLio2 + nav stack (click-to-drive)
"""

from __future__ import annotations

import os

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_joints,
    make_twist_base_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.robot.unitree.g1.config import G1_LOCAL_PLANNER_PRECOMPUTED_PATHS
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

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
    address: str | None = None,
) -> HardwareComponent:
    """FlowBase holonomic platform via Portal RPC (3-DOF: vx, vy, wz).

    Address defaults to ``FlowBaseAdapter.DEFAULT_ADDRESS`` when ``None``.
    """
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
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

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
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

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
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

# FlowBase + Livox MID-360 + FastLio2 SLAM + nav stack with click-to-drive in Rerun. The velocity
# sink is ControlCoordinator + FlowBaseAdapter

coordinator_flowbase_nav = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.1.5"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.1.189"),
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
        # MovementManager: subscribes clicked_point + nav_cmd_vel + tele_cmd_vel,
        # publishes muxed cmd_vel + goal (+ way_point, disconnected below).
        MovementManager.blueprint(),
        # FlowBase driver: ControlCoordinator with the existing JointVelocityTask
        # passthrough; receives Twist from MovementManager on LCM /cmd_vel.
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
        RerunBridgeModule.blueprint(
            **nav_stack_rerun_config({"memory_limit": "1GB"}, vis_throttle=0.5),
            rerun_open="native",
        ),
        RerunWebSocketServer.blueprint(),
    )
    .remappings(
        [
            (FastLio2, "lidar", "registered_scan"),
            # SimplePlanner / FarPlanner owns way_point — disconnect MovementManager's
            # redundant pass-through copy (matches unitree-g1-nav-onboard).
            (MovementManager, "way_point", "_mgr_way_point_unused"),
            # MovementManager.cmd_vel publishes to LCM /cmd_vel by default; the
            # coordinator's twist_command listens on the same name.
            (ControlCoordinator, "twist_command", "cmd_vel"),
        ]
    )
    .global_config(n_workers=8)
)


# Mock arm (7-DOF) + mock holonomic base (3-DOF)
_mock_arm_hw = HardwareComponent(
    hardware_id="arm",
    hardware_type=HardwareType.MANIPULATOR,
    joints=make_joints("arm", 7),
    adapter_type="mock",
)

coordinator_mobile_manip_mock = ControlCoordinator.blueprint(
    hardware=[_mock_arm_hw, _mock_twist_base()],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=_mock_arm_hw.joints,
            priority=10,
        ),
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

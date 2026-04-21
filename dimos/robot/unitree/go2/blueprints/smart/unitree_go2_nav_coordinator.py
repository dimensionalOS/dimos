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

"""Go2 navigation with ControlCoordinator-based path following + Rerun viz.

Composes:
    unitree_go2_basic   — GO2Connection + Rerun + WebSocket viz
    ControlCoordinator  — 100 Hz tick loop with transport_lcm adapter
    Nav stack           — VoxelMapper + CostMapper + A* planner + Frontier explorer

The path follower task is declared here in the blueprint via TaskConfig.
The nav module is controller-agnostic — it discovers the task by name
at startup and routes paths to it.  To swap controllers, change the
TaskConfig type here (no nav module changes needed).

Usage:
    dimos run unitree-go2-nav-coordinator
    dimos --simulation run unitree-go2-nav-coordinator
"""

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection

_go2_joints = make_twist_base_joints("go2")

unitree_go2_nav_coordinator = (
    autoconnect(
        unitree_go2_basic,
        ControlCoordinator.blueprint(
            hardware=[
                HardwareComponent(
                    hardware_id="go2",
                    hardware_type=HardwareType.BASE,
                    joints=_go2_joints,
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="vel_go2",
                    type="velocity",
                    joint_names=_go2_joints,
                    priority=10,
                ),
                # Path follower — swap type to "path_follower" for PurePursuit+PID
                TaskConfig(
                    name="reactive_path_follower",
                    type="reactive_path_follower",
                    joint_names=_go2_joints,
                    priority=20,
                ),
            ],
        ),
        VoxelGridMapper.blueprint(voxel_size=0.1),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        WavefrontFrontierExplorer.blueprint(),
    )
    .remappings(
        [
            (GO2Connection, "cmd_vel", "go2_cmd_vel"),
            (GO2Connection, "odom", "go2_odom"),
        ]
    )
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("go2_cmd_vel", Twist): LCMTransport("/go2/cmd_vel", Twist),
            ("go2_odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
            ("odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(n_workers=7, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_nav_coordinator"]

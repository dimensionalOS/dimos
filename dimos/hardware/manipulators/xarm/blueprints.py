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

"""Blueprints for XArm manipulator using B-lite architecture.

Usage:
    # Run via CLI:
    dimos run xarm-servo           # Driver only
    dimos run xarm-trajectory      # Driver + Joint trajectory controller

    # Or programmatically:
    from dimos.hardware.manipulators.xarm.blueprints import xarm_trajectory
    coordinator = xarm_trajectory.build()
    coordinator.loop()
"""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.manipulators.xarm.arm import xarm as xarm_blueprint
from dimos.manipulation.control import cartesian_motion_controller, joint_trajectory_controller
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import (
    JointCommand,
    JointState,
    RobotState,
)
from dimos.msgs.trajectory_msgs import JointTrajectory

# =============================================================================
# XArm6 Servo Control Blueprint
# =============================================================================
# XArm driver in servo mode - publishes joint states, accepts joint commands.
# =============================================================================

xarm_servo = xarm_blueprint(
    ip="192.168.1.185",
    dof=6,
    control_rate=100.0,
    monitor_rate=10.0,
).transports(
    {
        # Joint state feedback (position, velocity, effort)
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        # Robot state feedback (mode, state, errors)
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
    }
)

# =============================================================================
# XArm5 Servo Control Blueprint
# =============================================================================

xarm5_servo = xarm_blueprint(
    ip="192.168.1.185",
    dof=5,
    control_rate=100.0,
    monitor_rate=10.0,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
    }
)

# =============================================================================
# XArm7 Servo Control Blueprint
# =============================================================================

xarm7_servo = xarm_blueprint(
    ip="192.168.1.185",
    dof=7,
    control_rate=100.0,
    monitor_rate=10.0,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
    }
)

# =============================================================================
# XArm Trajectory Control Blueprint (Driver + Trajectory Controller)
# =============================================================================
# Combines XArm driver with JointTrajectoryController for trajectory execution.
# The controller receives JointTrajectory messages and executes them.
# =============================================================================

xarm_trajectory = autoconnect(
    xarm_blueprint(
        ip="192.168.2.235",
        dof=7,
        control_rate=100.0,  # Higher rate for smoother trajectory execution
        monitor_rate=10.0,
    ),
    joint_trajectory_controller(
        control_frequency=100.0,
    ),
).transports(
    {
        # Shared topics between driver and controller
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
        ("joint_position_command", JointCommand): LCMTransport(
            "/xarm/joint_position_command", JointCommand
        ),
        # Trajectory input topic
        ("trajectory", JointTrajectory): LCMTransport("/trajectory", JointTrajectory),
    }
)

# =============================================================================
# XArm7 Trajectory Control Blueprint
# =============================================================================

xarm7_trajectory = autoconnect(
    xarm_blueprint(
        ip="192.168.1.185",
        dof=7,
        control_rate=100.0,
        monitor_rate=10.0,
    ),
    joint_trajectory_controller(
        control_frequency=100.0,
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
        ("joint_position_command", JointCommand): LCMTransport(
            "/xarm/joint_position_command", JointCommand
        ),
        ("trajectory", JointTrajectory): LCMTransport("/trajectory", JointTrajectory),
    }
)

# =============================================================================
# XArm Cartesian Control Blueprint (Driver + Cartesian Controller)
# =============================================================================
# Combines XArm driver with CartesianMotionController for Cartesian space control.
# The controller receives target_pose and converts to joint commands via IK.
# =============================================================================

xarm_cartesian = autoconnect(
    xarm_blueprint(
        ip="192.168.1.185",
        dof=6,
        control_rate=100.0,
        monitor_rate=10.0,
    ),
    cartesian_motion_controller(
        control_frequency=20.0,
        position_kp=5.0,
        position_ki=0.0,
        position_kd=0.1,
        max_linear_velocity=0.2,
        max_angular_velocity=1.0,
    ),
).transports(
    {
        # Shared topics between driver and controller
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
        ("joint_position_command", JointCommand): LCMTransport(
            "/xarm/joint_position_command", JointCommand
        ),
        # Controller-specific topics
        ("target_pose", PoseStamped): LCMTransport("/target_pose", PoseStamped),
        ("current_pose", PoseStamped): LCMTransport("/xarm/current_pose", PoseStamped),
    }
)


__all__ = [
    "xarm5_servo",
    "xarm7_servo",
    "xarm7_trajectory",
    "xarm_cartesian",
    "xarm_servo",
    "xarm_trajectory",
]

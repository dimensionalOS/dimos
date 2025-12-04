# Copyright 2025 Dimensional Inc.
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

"""
Blueprints for Piper manipulator control systems.

This module provides declarative blueprints for configuring Piper servo control,
following the same pattern used for xArm and other manipulators.

Usage:
    # Run via CLI:
    dimos run piper-servo           # Driver only
    dimos run piper-cartesian       # Driver + Cartesian motion controller
    dimos run piper-trajectory      # Driver + Joint trajectory controller

    # Or programmatically:
    from dimos.hardware.manipulators.piper.piper_blueprints import piper_servo
    coordinator = piper_servo.build()
    coordinator.loop()
"""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.manipulators.piper.piper_driver import piper_driver
from dimos.manipulation.control import cartesian_motion_controller, joint_trajectory_controller
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.msgs.trajectory_msgs import JointTrajectory

# =============================================================================
# Piper Servo Control Blueprint
# =============================================================================
# PiperDriver configured for servo control mode.
# Publishes joint states and robot state, listens for joint commands.
# =============================================================================

piper_servo = piper_driver(
    can_name="can0",
    enable_on_start=True,
    control_frequency=100.0,
    joint_state_rate=100.0,
    robot_state_rate=10.0,
).transports(
    {
        # Joint state feedback (position, velocity, effort)
        ("joint_state", JointState): LCMTransport("/piper/joint_states", JointState),
        # Robot state feedback (mode, state, errors)
        ("robot_state", RobotState): LCMTransport("/piper/robot_state", RobotState),
        # Position commands input
        ("joint_position_command", JointCommand): LCMTransport(
            "/piper/joint_position_command", JointCommand
        ),
        # Velocity commands input
        ("joint_velocity_command", JointCommand): LCMTransport(
            "/piper/joint_velocity_command", JointCommand
        ),
    }
)

# =============================================================================
# Piper Cartesian Control Blueprint (Driver + Controller)
# =============================================================================
# Combines PiperDriver with CartesianMotionController for Cartesian space control.
# The controller receives target_pose and converts to joint commands via IK.
# =============================================================================

piper_cartesian = autoconnect(
    piper_driver(
        can_name="can0",
        enable_on_start=True,
        control_frequency=100.0,
        joint_state_rate=100.0,
        robot_state_rate=10.0,
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
        ("joint_state", JointState): LCMTransport("/piper/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/piper/robot_state", RobotState),
        ("joint_position_command", JointCommand): LCMTransport(
            "/piper/joint_position_command", JointCommand
        ),
        # Controller-specific topics
        ("target_pose", PoseStamped): LCMTransport("/target_pose", PoseStamped),
        ("current_pose", PoseStamped): LCMTransport("/piper/current_pose", PoseStamped),
    }
)

# =============================================================================
# Piper Trajectory Control Blueprint (Driver + Trajectory Controller)
# =============================================================================
# Combines PiperDriver with JointTrajectoryController for trajectory execution.
# The controller receives JointTrajectory messages and executes them at 100Hz.
# =============================================================================

piper_trajectory = autoconnect(
    piper_driver(
        can_name="can0",
        enable_on_start=True,
        control_frequency=100.0,
        joint_state_rate=100.0,
        robot_state_rate=10.0,
    ),
    joint_trajectory_controller(
        control_frequency=100.0,
    ),
).transports(
    {
        # Shared topics between driver and controller
        ("joint_state", JointState): LCMTransport("/piper/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/piper/robot_state", RobotState),
        ("joint_position_command", JointCommand): LCMTransport(
            "/piper/joint_position_command", JointCommand
        ),
        # Trajectory input topic
        ("trajectory", JointTrajectory): LCMTransport("/trajectory", JointTrajectory),
    }
)

__all__ = ["piper_cartesian", "piper_servo", "piper_trajectory"]

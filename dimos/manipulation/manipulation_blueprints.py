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
Blueprints for manipulation module integration.

This module provides declarative blueprints for combining the ManipulationModule
with arm drivers and trajectory controllers.

Usage:
    # Programmatically:
    from dimos.manipulation.manipulation_blueprints import xarm6_manipulation
    coordinator = xarm6_manipulation.build()
    coordinator.loop()

    # Or build your own composition:
    from dimos.manipulation import manipulation_module
    from dimos.hardware.manipulators.xarm import xarm_driver
    from dimos.manipulation.control import joint_trajectory_controller
    from dimos.core.blueprints import autoconnect

    my_system = autoconnect(
        xarm_driver(ip="192.168.1.235", dof=6),
        manipulation_module(
            robot_urdf_path="...",
            joint_names=[...],
            enable_viz=True,
        ),
        joint_trajectory_controller(control_frequency=100.0),
    ).transports({...})
"""

from pathlib import Path

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.manipulators.piper.piper_driver import piper_driver
from dimos.hardware.manipulators.xarm.xarm_driver import xarm_driver
from dimos.manipulation.control import joint_trajectory_controller
from dimos.manipulation.manipulation_module import manipulation_module
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.msgs.trajectory_msgs import JointTrajectory


# Path to xarm URDF
def _get_xarm_urdf_path() -> str:
    """Get path to xarm URDF."""
    base = Path(__file__).parent.parent / "hardware" / "manipulators"
    return str(base / "xarm/xarm_description/urdf/xarm_device.urdf.xacro")


def _get_xarm_package_paths() -> dict[str, str]:
    """Get package paths for xarm xacro resolution."""
    base = Path(__file__).parent.parent / "hardware" / "manipulators"
    return {"xarm_description": str(base / "xarm/xarm_description")}


# Path to piper URDF
def _get_piper_urdf_path() -> str:
    """Get path to piper URDF."""
    base = Path(__file__).parent.parent / "hardware" / "manipulators"
    return str(base / "piper/piper_description/urdf/piper_description.xacro")


def _get_piper_package_paths() -> dict[str, str]:
    """Get package paths for piper xacro resolution."""
    base = Path(__file__).parent.parent / "hardware" / "manipulators"
    return {"piper_description": str(base / "piper/piper_description")}


# =============================================================================
# XArm6 Full Manipulation Stack
# =============================================================================
# Combines:
#   - XArmDriver: Hardware interface
#   - ManipulationModule: Motion planning (Drake RRT-Connect)
#   - JointTrajectoryController: Trajectory execution at 100Hz
#
# Data flow:
#   Driver.joint_state ──► ManipulationModule.joint_state (planning world sync)
#   ManipulationModule.trajectory ──► Controller.trajectory (planned path)
#   Controller.joint_position_command ──► Driver.joint_position_command (execution)
# =============================================================================

xarm6_manipulation = autoconnect(
    xarm_driver(
        ip="192.168.1.235",
        dof=6,
        has_gripper=False,
        has_force_torque=False,
        control_rate=100,
        monitor_rate=10,
        connection_type="sim",  # Use "hardware" for real robot
    ),
    manipulation_module(
        robot_urdf_path=_get_xarm_urdf_path(),
        robot_name="xarm6",
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        end_effector_link="link6",
        base_link="link_base",
        max_velocity=1.0,
        max_acceleration=2.0,
        planning_timeout=10.0,
        enable_viz=True,
        package_paths=_get_xarm_package_paths(),
        xacro_args={"dof": "6", "limited": "true"},
    ),
    joint_trajectory_controller(
        control_frequency=100.0,
    ),
).transports(
    {
        # Joint state from driver → ManipulationModule and Controller
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        # Robot state from driver
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
        # Trajectory from ManipulationModule → Controller
        ("trajectory", JointTrajectory): LCMTransport("/xarm/trajectory", JointTrajectory),
        # Commands from Controller → Driver
        ("joint_position_command", JointCommand): LCMTransport(
            "/xarm/joint_position_command", JointCommand
        ),
    }
)


# =============================================================================
# XArm7 Full Manipulation Stack
# =============================================================================

xarm7_manipulation = autoconnect(
    xarm_driver(
        ip="192.168.1.235",
        dof=7,
        has_gripper=False,
        has_force_torque=False,
        control_rate=100,
        monitor_rate=10,
        connection_type="sim",  # Use "hardware" for real robot
    ),
    manipulation_module(
        robot_urdf_path=_get_xarm_urdf_path(),
        robot_name="xarm7",
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
        end_effector_link="link7",
        base_link="link_base",
        max_velocity=1.0,
        max_acceleration=2.0,
        planning_timeout=10.0,
        enable_viz=True,
        package_paths=_get_xarm_package_paths(),
        xacro_args={"dof": "7", "limited": "true"},
    ),
    joint_trajectory_controller(
        control_frequency=100.0,
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
        ("trajectory", JointTrajectory): LCMTransport("/xarm/trajectory", JointTrajectory),
        ("joint_position_command", JointCommand): LCMTransport(
            "/xarm/joint_position_command", JointCommand
        ),
    }
)


# =============================================================================
# ManipulationModule Only (for integration with existing driver/controller)
# =============================================================================
# Use this when you already have a driver and controller running separately.
# Just connects the planning module to existing LCM topics.
# =============================================================================

xarm6_planner_only = manipulation_module(
    robot_urdf_path=_get_xarm_urdf_path(),
    robot_name="xarm6",
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    end_effector_link="link6",
    base_link="link_base",
    max_velocity=1.0,
    max_acceleration=2.0,
    planning_timeout=10.0,
    enable_viz=True,
    package_paths=_get_xarm_package_paths(),
    xacro_args={"dof": "6", "limited": "true"},
).transports(
    {
        # Subscribe to joint state from driver
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        # Publish trajectory for controller
        ("trajectory", JointTrajectory): LCMTransport("/xarm/trajectory", JointTrajectory),
    }
)


# =============================================================================
# Piper Full Manipulation Stack (Simulation Mode)
# =============================================================================
# Combines:
#   - PiperDriver: Hardware interface (simulation mode)
#   - ManipulationModule: Motion planning (Drake RRT-Connect)
#   - JointTrajectoryController: Trajectory execution at 100Hz
#
# Data flow:
#   Driver.joint_state ──► ManipulationModule.joint_state (planning world sync)
#   ManipulationModule.trajectory ──► Controller.trajectory (planned path)
#   Controller.joint_position_command ──► Driver.joint_position_command (execution)
# =============================================================================

piper_manipulation = autoconnect(
    piper_driver(
        can_port="can0",
        has_gripper=True,
        enable_on_start=True,
        control_rate=100,
        monitor_rate=10,
        connection_type="sim",  # Simulation mode
    ),
    manipulation_module(
        robot_urdf_path=_get_piper_urdf_path(),
        robot_name="piper",
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        end_effector_link="link6",
        base_link="arm_base",
        max_velocity=1.0,
        max_acceleration=2.0,
        planning_timeout=10.0,
        enable_viz=True,
        package_paths=_get_piper_package_paths(),
    ),
    joint_trajectory_controller(
        control_frequency=100.0,
    ),
).transports(
    {
        # Joint state from driver → ManipulationModule and Controller
        ("joint_state", JointState): LCMTransport("/piper/joint_states", JointState),
        # Robot state from driver
        ("robot_state", RobotState): LCMTransport("/piper/robot_state", RobotState),
        # Trajectory from ManipulationModule → Controller
        ("trajectory", JointTrajectory): LCMTransport("/piper/trajectory", JointTrajectory),
        # Commands from Controller → Driver
        ("joint_position_command", JointCommand): LCMTransport(
            "/piper/joint_position_command", JointCommand
        ),
    }
)


__all__ = [
    "xarm6_manipulation",
    "xarm6_planner_only",
    "xarm7_manipulation",
    "piper_manipulation",
]

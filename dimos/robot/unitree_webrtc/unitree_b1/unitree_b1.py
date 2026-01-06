#!/usr/bin/env python3
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

# Copyright 2025 Dimensional Inc.

"""
Unitree B1 quadruped robot with simplified UDP control.
Uses standard Twist interface for velocity commands.
"""

import logging
import os

from dimos import core
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.resource import Resource
from dimos.msgs.geometry_msgs import PoseStamped, TwistStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.std_msgs import Int32
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.robot import Robot
from dimos.robot.ros_bridge import BridgeDirection, ROSBridge
from dimos.robot.unitree_webrtc.unitree_b1.connection import (
    B1ConnectionModule,
    MockB1ConnectionModule,
)
from dimos.skills.skills import SkillLibrary
from dimos.types.robot_capabilities import RobotCapability
from dimos.utils.logging_config import setup_logger

# Handle ROS imports for environments where ROS is not available like CI
try:
    from geometry_msgs.msg import (  # type: ignore[import-untyped]
        TwistStamped as ROSTwistStamped,  # type: ignore[attr-defined]
    )
    from nav_msgs.msg import Odometry as ROSOdometry  # type: ignore[attr-defined, import-untyped]
    from tf2_msgs.msg import TFMessage as ROSTFMessage  # type: ignore[attr-defined, import-untyped]

    ROS_AVAILABLE = True
except ImportError:
    ROSTwistStamped = None  # type: ignore[assignment, misc]
    ROSOdometry = None  # type: ignore[assignment, misc]
    ROSTFMessage = None  # type: ignore[assignment, misc]
    ROS_AVAILABLE = False

logger = setup_logger(level=logging.INFO)


class UnitreeB1(Robot, Resource):
    """Unitree B1 quadruped robot with UDP control.

    Simplified architecture:
    - Connection module handles Twist → B1Command conversion
    - Standard /cmd_vel interface for navigation compatibility
    - Optional joystick module for testing
    """

    def __init__(
        self,
        ip: str = "192.168.123.14",
        port: int = 9090,
        output_dir: str | None = None,
        skill_library: SkillLibrary | None = None,
        enable_joystick: bool = False,
        enable_ros_bridge: bool = True,
        test_mode: bool = False,
    ) -> None:
        """Initialize the B1 robot.

        Args:
            ip: Robot IP address (or server running joystick_server_udp)
            port: UDP port for joystick server (default 9090)
            output_dir: Directory for saving outputs
            skill_library: Skill library instance (optional)
            enable_joystick: Enable pygame joystick control module
            enable_ros_bridge: Enable ROS bridge for external control
            test_mode: Test mode - print commands instead of sending UDP
        """
        super().__init__()
        self.ip = ip
        self.port = port
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.enable_joystick = enable_joystick
        self.enable_ros_bridge = enable_ros_bridge
        self.test_mode = test_mode
        self.capabilities = [RobotCapability.LOCOMOTION]
        self.connection = None
        self.joystick = None
        self.ros_bridge = None
        self._dimos = ModuleCoordinator(n=2)

        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Robot outputs will be saved to: {self.output_dir}")

    def start(self) -> None:
        """Start the B1 robot - initialize DimOS, deploy modules, and start them."""

        logger.info("Initializing DimOS...")
        self._dimos.start()

        logger.info("Deploying connection module...")
        if self.test_mode:
            self.connection = self._dimos.deploy(MockB1ConnectionModule, self.ip, self.port)  # type: ignore[assignment]
        else:
            self.connection = self._dimos.deploy(B1ConnectionModule, self.ip, self.port)  # type: ignore[assignment]

        # Configure LCM transports for connection (matching G1 pattern)
        self.connection.cmd_vel.transport = core.LCMTransport("/cmd_vel", TwistStamped)  # type: ignore[attr-defined]
        self.connection.mode_cmd.transport = core.LCMTransport("/b1/mode", Int32)  # type: ignore[attr-defined]
        self.connection.odom_in.transport = core.LCMTransport("/state_estimation", Odometry)  # type: ignore[attr-defined]
        self.connection.odom_pose.transport = core.LCMTransport("/odom", PoseStamped)  # type: ignore[attr-defined]

        # Deploy joystick move_vel control
        if self.enable_joystick:
            from dimos.robot.unitree_webrtc.unitree_b1.joystick_module import JoystickModule

            self.joystick = self._dimos.deploy(JoystickModule)  # type: ignore[assignment]
            self.joystick.twist_out.transport = core.LCMTransport("/cmd_vel", TwistStamped)  # type: ignore[attr-defined]
            self.joystick.mode_out.transport = core.LCMTransport("/b1/mode", Int32)  # type: ignore[attr-defined]
            logger.info("Joystick module deployed - pygame window will open")

        self._dimos.start_all_modules()

        self.connection.idle()  # type: ignore[attr-defined]  # Start in IDLE mode for safety
        logger.info("B1 started in IDLE mode (safety)")

        # Deploy ROS bridge if enabled (matching G1 pattern)
        if self.enable_ros_bridge:
            self._deploy_ros_bridge()

        logger.info(f"UnitreeB1 initialized - UDP control to {self.ip}:{self.port}")
        if self.enable_joystick:
            logger.info("Pygame joystick module enabled for testing")
        if self.enable_ros_bridge:
            logger.info("ROS bridge enabled for external control")

    def stop(self) -> None:
        self._dimos.stop()
        if self.ros_bridge:
            self.ros_bridge.stop()

    def _deploy_ros_bridge(self) -> None:
        """Deploy and configure ROS bridge (matching G1 implementation)."""
        self.ros_bridge = ROSBridge("b1_ros_bridge")  # type: ignore[assignment]

        # Add /cmd_vel topic from ROS to DIMOS
        self.ros_bridge.add_topic(  # type: ignore[attr-defined]
            "/cmd_vel", TwistStamped, ROSTwistStamped, direction=BridgeDirection.ROS_TO_DIMOS
        )

        # Add /state_estimation topic from ROS to DIMOS (external odometry)
        self.ros_bridge.add_topic(  # type: ignore[attr-defined]
            "/state_estimation", Odometry, ROSOdometry, direction=BridgeDirection.ROS_TO_DIMOS
        )

        # Add /tf topic from ROS to DIMOS
        self.ros_bridge.add_topic(  # type: ignore[attr-defined]
            "/tf", TFMessage, ROSTFMessage, direction=BridgeDirection.ROS_TO_DIMOS
        )

        self.ros_bridge.start()  # type: ignore[attr-defined]

        logger.info("ROS bridge deployed: /cmd_vel, /state_estimation, /tf (ROS → DIMOS)")

    # Robot control methods (standard interface)
    def move(self, twist_stamped: TwistStamped, duration: float = 0.0) -> None:
        """Send movement command to robot using timestamped Twist.

        Args:
            twist_stamped: TwistStamped message with linear and angular velocities
            duration: How long to move (not used for B1)
        """
        if self.connection:
            self.connection.move(twist_stamped, duration)

    def stand(self) -> None:
        """Put robot in stand mode."""
        if self.connection:
            self.connection.stand()
            logger.info("B1 switched to STAND mode")

    def walk(self) -> None:
        """Put robot in walk mode."""
        if self.connection:
            self.connection.walk()
            logger.info("B1 switched to WALK mode")

    def idle(self) -> None:
        """Put robot in idle mode."""
        if self.connection:
            self.connection.idle()
            logger.info("B1 switched to IDLE mode")


def main() -> None:
    """Main entry point for testing B1 robot."""
    import argparse

    parser = argparse.ArgumentParser(description="Unitree B1 Robot Control")
    parser.add_argument("--ip", default="192.168.12.1", help="Robot IP address")
    parser.add_argument("--port", type=int, default=9090, help="UDP port")
    parser.add_argument("--joystick", action="store_true", help="Enable pygame joystick control")
    parser.add_argument("--ros-bridge", action="store_true", default=True, help="Enable ROS bridge")
    parser.add_argument(
        "--no-ros-bridge", dest="ros_bridge", action="store_false", help="Disable ROS bridge"
    )
    parser.add_argument("--output-dir", help="Output directory for logs/data")
    parser.add_argument(
        "--test", action="store_true", help="Test mode - print commands instead of UDP"
    )

    args = parser.parse_args()

    robot = UnitreeB1(  # type: ignore[abstract]
        ip=args.ip,
        port=args.port,
        output_dir=args.output_dir,
        enable_joystick=args.joystick,
        enable_ros_bridge=args.ros_bridge,
        test_mode=args.test,
    )

    robot.start()

    try:
        if args.joystick:
            print("\n" + "=" * 50)
            print("B1 JOYSTICK CONTROL")
            print("=" * 50)
            print("Focus the pygame window to control")
            print("Press keys in pygame window:")
            print("  0/1/2 = Idle/Stand/Walk modes")
            print("  WASD = Move/Turn")
            print("  JL = Strafe")
            print("  Space/Q = Emergency Stop")
            print("  ESC = Quit pygame (then Ctrl+C to exit)")
            print("=" * 50 + "\n")

            import time

            while True:
                time.sleep(1)
        else:
            # Manual control example
            print("\nB1 Robot ready for commands")
            print("Use robot.idle(), robot.stand(), robot.walk() to change modes")
            if args.ros_bridge:
                print("ROS bridge active - listening for /cmd_vel and /state_estimation")
            else:
                print("Use robot.move(TwistStamped(...)) to send velocity commands")
            print("Press Ctrl+C to exit\n")

            import time

            while True:
                time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()

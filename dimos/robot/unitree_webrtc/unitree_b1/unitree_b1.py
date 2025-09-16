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

import os
import logging
from typing import Optional

from dimos import core
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.std_msgs import Int32
from dimos.protocol.pubsub.lcmpubsub import LCM
from dimos.robot.robot import Robot
from dimos.robot.unitree_webrtc.unitree_b1.connection import (
    B1ConnectionModule,
    TestB1ConnectionModule,
)
from dimos.skills.skills import SkillLibrary
from dimos.types.robot_capabilities import RobotCapability
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_b1", level=logging.INFO)


class UnitreeB1(Robot):
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
        output_dir: str = None,
        skill_library: Optional[SkillLibrary] = None,
        enable_joystick: bool = False,
        test_mode: bool = False,
    ):
        """Initialize the B1 robot.

        Args:
            ip: Robot IP address (or server running joystick_server_udp)
            port: UDP port for joystick server (default 9090)
            output_dir: Directory for saving outputs
            skill_library: Skill library instance (optional)
            enable_joystick: Enable pygame joystick control module
            test_mode: Test mode - print commands instead of sending UDP
        """
        super().__init__()
        self.ip = ip
        self.port = port
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.enable_joystick = enable_joystick
        self.test_mode = test_mode
        self.capabilities = [RobotCapability.LOCOMOTION]
        self.connection = None
        self.joystick = None

        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Robot outputs will be saved to: {self.output_dir}")

    def start(self):
        """Start the B1 robot - initialize DimOS, deploy modules, and start them."""

        logger.info("Initializing DimOS...")
        self.dimos = core.start(2)

        logger.info("Deploying connection module...")
        if self.test_mode:
            self.connection = self.dimos.deploy(TestB1ConnectionModule, self.ip, self.port)
        else:
            self.connection = self.dimos.deploy(B1ConnectionModule, self.ip, self.port)

        # Configure LCM transports for connection
        self.connection.cmd_vel.transport = core.LCMTransport("/cmd_vel", Twist)
        self.connection.mode_cmd.transport = core.LCMTransport("/b1/mode", Int32)

        # Deploy joystick move_vel control
        if self.enable_joystick:
            from dimos.robot.unitree_webrtc.unitree_b1.joystick_module import JoystickModule

            self.joystick = self.dimos.deploy(JoystickModule)
            self.joystick.twist_out.transport = core.LCMTransport("/cmd_vel", Twist)
            self.joystick.mode_out.transport = core.LCMTransport("/b1/mode", Int32)
            logger.info("Joystick module deployed - pygame window will open")

        self.connection.start()
        self.connection.idle()  # Start in IDLE mode for safety
        logger.info("B1 started in IDLE mode (safety)")

        if self.joystick:
            self.joystick.start()

        logger.info(f"UnitreeB1 initialized - UDP control to {self.ip}:{self.port}")
        if self.enable_joystick:
            logger.info("Pygame joystick module enabled for testing")

    # Robot control methods (standard interface)
    def move(self, twist: Twist, duration: float = 0.0):
        """Send movement command to robot using standard Twist.

        Args:
            twist: Twist message with linear and angular velocities
            duration: How long to move (not used for B1)
        """
        if self.connection:
            self.connection.move(twist, duration)

    def stop(self):
        """Stop all robot movement."""
        if self.connection:
            self.connection.stop()

    def stand(self):
        """Put robot in stand mode."""
        if self.connection:
            self.connection.stand()
            logger.info("B1 switched to STAND mode")

    def walk(self):
        """Put robot in walk mode."""
        if self.connection:
            self.connection.walk()
            logger.info("B1 switched to WALK mode")

    def idle(self):
        """Put robot in idle mode."""
        if self.connection:
            self.connection.idle()
            logger.info("B1 switched to IDLE mode")

    def cleanup(self):
        """Clean up robot resources."""
        logger.info("Cleaning up B1 robot...")

        # Stop robot movement
        self.stop()

        # Clean up modules
        if self.connection:
            self.connection.cleanup()

        logger.info("B1 cleanup complete")

    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()


def main():
    """Main entry point for testing B1 robot."""
    import argparse

    parser = argparse.ArgumentParser(description="Unitree B1 Robot Control")
    parser.add_argument("--ip", default="192.168.12.1", help="Robot IP address")
    parser.add_argument("--port", type=int, default=9090, help="UDP port")
    parser.add_argument("--joystick", action="store_true", help="Enable pygame joystick control")
    parser.add_argument("--output-dir", help="Output directory for logs/data")
    parser.add_argument(
        "--test", action="store_true", help="Test mode - print commands instead of UDP"
    )

    args = parser.parse_args()

    robot = UnitreeB1(
        ip=args.ip,
        port=args.port,
        output_dir=args.output_dir,
        enable_joystick=args.joystick,
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
            print("Use robot.move(Twist(...)) to send velocity commands")
            print("Press Ctrl+C to exit\n")

            import time

            while True:
                time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        robot.cleanup()


if __name__ == "__main__":
    main()

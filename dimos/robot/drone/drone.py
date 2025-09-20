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

"""Main Drone robot class for DimOS."""

import os
import time
import logging
from typing import Optional

from dimos import core
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.std_msgs import String
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.protocol import pubsub

# LCM not needed in orchestrator - modules handle communication
from dimos.robot.robot import Robot
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.robot.drone.camera_module import DroneCameraModule
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.types.robot_capabilities import RobotCapability
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class Drone(Robot):
    """Generic MAVLink-based drone with video and depth capabilities."""

    def __init__(
        self,
        connection_string: str = "udp:0.0.0.0:14550",
        video_port: int = 5600,
        camera_intrinsics: Optional[list] = None,
        output_dir: str = None,
        outdoor: bool = False,
    ):
        """Initialize drone robot.

        Args:
            connection_string: MAVLink connection string
            video_port: UDP port for video stream
            camera_intrinsics: Camera intrinsics [fx, fy, cx, cy]
            output_dir: Directory for outputs
            outdoor: Use GPS only mode (no velocity integration)
        """
        super().__init__()

        self.connection_string = connection_string
        self.video_port = video_port
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.outdoor = outdoor

        if camera_intrinsics is None:
            # Assuming 1920x1080 with typical FOV
            self.camera_intrinsics = [1000.0, 1000.0, 960.0, 540.0]
        else:
            self.camera_intrinsics = camera_intrinsics

        self.capabilities = [
            RobotCapability.LOCOMOTION,  # Aerial locomotion
            RobotCapability.VISION,
        ]

        self.dimos = None
        self.connection = None
        self.camera = None
        self.foxglove_bridge = None

        self._setup_directories()

    def _setup_directories(self):
        """Setup output directories."""
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Drone outputs will be saved to: {self.output_dir}")

    def start(self):
        """Start the drone system with all modules."""
        logger.info("Starting Drone robot system...")

        # Start DimOS cluster
        self.dimos = core.start(4)

        # Deploy modules
        self._deploy_connection()
        self._deploy_camera()
        self._deploy_visualization()

        # Start modules
        self._start_modules()

        logger.info("Drone system initialized and started")
        logger.info("Foxglove visualization available at http://localhost:8765")

    def _deploy_connection(self):
        """Deploy and configure connection module."""
        logger.info("Deploying connection module...")

        self.connection = self.dimos.deploy(
            DroneConnectionModule,
            connection_string=self.connection_string,
            video_port=self.video_port,
            outdoor=self.outdoor,
        )

        # Configure LCM transports
        self.connection.odom.transport = core.LCMTransport("/drone/odom", PoseStamped)
        self.connection.status.transport = core.LCMTransport("/drone/status", String)
        self.connection.telemetry.transport = core.LCMTransport("/drone/telemetry", String)
        self.connection.video.transport = core.LCMTransport("/drone/video", Image)
        self.connection.movecmd.transport = core.LCMTransport("/drone/cmd_vel", Vector3)

        logger.info("Connection module deployed")

    def _deploy_camera(self):
        """Deploy and configure camera module."""
        logger.info("Deploying camera module...")

        self.camera = self.dimos.deploy(DroneCameraModule, camera_intrinsics=self.camera_intrinsics)

        # Configure LCM transports
        self.camera.color_image.transport = core.LCMTransport("/drone/color_image", Image)
        self.camera.depth_image.transport = core.LCMTransport("/drone/depth_image", Image)
        self.camera.depth_colorized.transport = core.LCMTransport("/drone/depth_colorized", Image)
        self.camera.camera_info.transport = core.LCMTransport("/drone/camera_info", CameraInfo)
        self.camera.camera_pose.transport = core.LCMTransport("/drone/camera_pose", PoseStamped)

        # Connect video from connection module to camera module
        self.camera.video.connect(self.connection.video)

        logger.info("Camera module deployed")

    def _deploy_visualization(self):
        """Deploy visualization tools."""
        logger.info("Setting up Foxglove bridge...")
        self.foxglove_bridge = FoxgloveBridge()

    def _start_modules(self):
        """Start all deployed modules."""
        logger.info("Starting modules...")

        # Start connection first
        result = self.connection.start()
        if not result:
            logger.warning("Connection module failed to start (no drone connected?)")

        # Start camera
        result = self.camera.start()
        if not result:
            logger.warning("Camera module failed to start")

        # Start Foxglove
        self.foxglove_bridge.start()

        logger.info("All modules started")

    # Robot control methods

    def get_odom(self) -> Optional[PoseStamped]:
        """Get current odometry.

        Returns:
            Current pose or None
        """
        return self.connection.get_odom()

    def get_status(self) -> dict:
        """Get drone status.

        Returns:
            Status dictionary
        """
        return self.connection.get_status()

    def move(self, vector: Vector3, duration: float = 0.0):
        """Send movement command.

        Args:
            vector: Velocity vector [x, y, z] in m/s
            duration: How long to move (0 = continuous)
        """
        self.connection.move(vector, duration)

    def takeoff(self, altitude: float = 3.0) -> bool:
        """Takeoff to altitude.

        Args:
            altitude: Target altitude in meters

        Returns:
            True if takeoff initiated
        """
        return self.connection.takeoff(altitude)

    def land(self) -> bool:
        """Land the drone.

        Returns:
            True if land command sent
        """
        return self.connection.land()

    def arm(self) -> bool:
        """Arm the drone.

        Returns:
            True if armed successfully
        """
        return self.connection.arm()

    def disarm(self) -> bool:
        """Disarm the drone.

        Returns:
            True if disarmed successfully
        """
        return self.connection.disarm()

    def set_mode(self, mode: str) -> bool:
        """Set flight mode.

        Args:
            mode: Mode name (STABILIZE, GUIDED, LAND, RTL, etc.)

        Returns:
            True if mode set successfully
        """
        return self.connection.set_mode(mode)

    def fly_to(self, lat: float, lon: float, alt: float) -> bool:
        """Fly to GPS coordinates.

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters (relative to home)

        Returns:
            True if command sent successfully
        """
        return self.connection.fly_to(lat, lon, alt)

    def get_single_rgb_frame(self, timeout: float = 2.0) -> Optional[Image]:
        """Get a single RGB frame from camera.

        Args:
            timeout: Timeout in seconds (currently unused as we get latest frame)

        Returns:
            Image message or None
        """
        if self.connection:
            return self.connection.get_single_frame()
        return None

    def stop(self):
        """Stop the drone system."""
        logger.info("Stopping drone system...")

        if self.connection:
            self.connection.stop()

        if self.camera:
            self.camera.stop()

        if self.foxglove_bridge:
            self.foxglove_bridge.stop()

        if self.dimos:
            self.dimos.shutdown()

        logger.info("Drone system stopped")


def main():
    """Main entry point for drone system."""
    import argparse

    parser = argparse.ArgumentParser(description="DimOS Drone System")
    parser.add_argument("--replay", action="store_true", help="Use recorded data for testing")
    parser.add_argument(
        "--test", action="store_true", help="Run test commands (takeoff, land, capture frame)"
    )
    parser.add_argument(
        "--outdoor",
        action="store_true",
        help="Outdoor mode - use GPS only, no velocity integration",
    )
    args = parser.parse_args()

    # Configure logging
    setup_logger("dimos.robot.drone", level=logging.INFO)

    # Suppress verbose loggers
    logging.getLogger("distributed").setLevel(logging.WARNING)
    logging.getLogger("asyncio").setLevel(logging.WARNING)

    if args.replay:
        connection = "replay"
        print("\n🔄 REPLAY MODE - Using drone replay data")
    else:
        connection = os.getenv("DRONE_CONNECTION", "udp:0.0.0.0:14550")
    video_port = int(os.getenv("DRONE_VIDEO_PORT", "5600"))

    print(f"""
╔══════════════════════════════════════════╗
║         DimOS Mavlink Drone Runner       ║
╠══════════════════════════════════════════╣
║  MAVLink: {connection:<30} ║
║  Video:   UDP port {video_port:<22} ║
║  Foxglove: http://localhost:8765        ║
╚══════════════════════════════════════════╝
    """)

    # Configure LCM
    pubsub.lcm.autoconf()

    drone = Drone(connection_string=connection, video_port=video_port, outdoor=args.outdoor)

    drone.start()

    print("\n✓ Drone system started successfully!")
    print("\nLCM Topics:")
    print("  • /drone/odom           - Odometry (PoseStamped)")
    print("  • /drone/status         - Status (String/JSON)")
    print("  • /drone/telemetry      - Full telemetry (String/JSON)")
    print("  • /drone/color_image    - RGB Video (Image)")
    print("  • /drone/depth_image    - Depth estimation (Image)")
    print("  • /drone/depth_colorized - Colorized depth (Image)")
    print("  • /drone/camera_info    - Camera calibration")
    print("  • /drone/cmd_vel        - Movement commands (Vector3)")

    # Start interactive command thread
    import threading
    import re

    def command_loop():
        """Interactive command loop for testing."""
        print("\n" + "=" * 60)
        print("INTERACTIVE MODE - Type commands:")
        print("  fly_to(lat, lon, alt) - Fly to GPS coordinates")
        print("  takeoff(alt) - Takeoff to altitude")
        print("  land() - Land the drone")
        print("  arm() - Arm the drone")
        print("  disarm() - Disarm the drone")
        print("  status - Show current status")
        print("  quit - Exit the program")
        print("=" * 60 + "\n")

        while True:
            try:
                cmd = input("drone> ").strip()

                if cmd.lower() == "quit":
                    print("Exiting...")
                    drone.stop()
                    import sys

                    sys.exit(0)

                # Parse fly_to command
                fly_to_match = re.match(
                    r"fly_to\s*\(\s*([-.\d]+)\s*,\s*([-.\d]+)\s*,\s*([-.\d]+)\s*\)", cmd
                )
                if fly_to_match:
                    lat = float(fly_to_match.group(1))
                    lon = float(fly_to_match.group(2))
                    alt = float(fly_to_match.group(3))
                    print(f"Flying to: lat={lat}, lon={lon}, alt={alt}m")
                    result = drone.fly_to(lat, lon, alt)
                    print(f"Result: {result}")
                    continue

                # Parse takeoff command
                takeoff_match = re.match(r"takeoff\s*\(\s*([-.\d]+)\s*\)", cmd)
                if takeoff_match:
                    alt = float(takeoff_match.group(1))
                    print(f"Taking off to {alt}m")
                    result = drone.takeoff(alt)
                    print(f"Result: {result}")
                    continue

                # Parse simple commands
                if cmd == "land()":
                    print("Landing...")
                    result = drone.land()
                    print(f"Result: {result}")
                elif cmd == "arm()":
                    print("Arming...")
                    result = drone.arm()
                    print(f"Result: {result}")
                elif cmd == "disarm()":
                    print("Disarming...")
                    result = drone.disarm()
                    print(f"Result: {result}")
                elif cmd == "status":
                    print("Status:", drone.get_status())
                elif cmd:
                    print(f"Unknown command: {cmd}")

            except Exception as e:
                print(f"Error: {e}")

    # Start command thread
    cmd_thread = threading.Thread(target=command_loop, daemon=True)
    cmd_thread.start()

    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down drone system...")
        drone.stop()
        print("✓ Drone system stopped cleanly")


if __name__ == "__main__":
    main()

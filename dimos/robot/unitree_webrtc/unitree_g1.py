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

"""
Unitree G1 humanoid robot.
Minimal implementation using WebRTC connection for robot control.
"""

import os
import time
import logging
from typing import Optional

from dimos import core
from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE, DEFAULT_CAPACITY_DEPTH_IMAGE
from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Twist, TwistStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs import CameraInfo, PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol import pubsub
from dimos.protocol.pubsub.lcmpubsub import LCM
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.robot.ros_bridge import ROSBridge, BridgeDirection
from geometry_msgs.msg import TwistStamped as ROSTwistStamped
from nav_msgs.msg import Odometry as ROSOdometry
from sensor_msgs.msg import PointCloud2 as ROSPointCloud2
from tf2_msgs.msg import TFMessage as ROSTFMessage
from dimos.skills.skills import SkillLibrary
from dimos.robot.robot import Robot

from dimos.types.robot_capabilities import RobotCapability
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_g1", level=logging.INFO)

try:
    from dimos.hardware.camera.zed import ZEDModule
except ImportError:
    logger.warning("ZEDModule not found. Please install pyzed to use ZED camera functionality.")
    ZEDModule = None

# Suppress verbose loggers
logging.getLogger("aiortc.codecs.h264").setLevel(logging.ERROR)
logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("websockets.server").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)
logging.getLogger("asyncio").setLevel(logging.ERROR)


class G1ConnectionModule(Module):
    """Simplified connection module for G1 - uses WebRTC for control."""

    movecmd: In[TwistStamped] = None
    odom_in: In[Odometry] = None

    odom_pose: Out[PoseStamped] = None
    ip: str
    connection_type: str = "webrtc"

    def __init__(self, ip: str = None, connection_type: str = "webrtc", *args, **kwargs):
        self.ip = ip
        self.connection_type = connection_type
        self.connection = None
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        """Start the connection and subscribe to sensor streams."""
        # Use the exact same UnitreeWebRTCConnection as Go2
        self.connection = UnitreeWebRTCConnection(self.ip)
        self.movecmd.subscribe(self.move)
        self.odom_in.subscribe(self._publish_odom_pose)

    def _publish_odom_pose(self, msg: Odometry):
        self.odom_pose.publish(
            PoseStamped(
                ts=msg.ts,
                frame_id=msg.frame_id,
                position=msg.pose.pose.position,
                orientation=msg.pose.orientation,
            )
        )

    @rpc
    def move(self, twist_stamped: TwistStamped, duration: float = 0.0):
        """Send movement command to robot."""
        twist = Twist(linear=twist_stamped.linear, angular=twist_stamped.angular)
        self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict):
        """Forward WebRTC publish requests to connection."""
        return self.connection.publish_request(topic, data)


class UnitreeG1(Robot):
    """Unitree G1 humanoid robot."""

    def __init__(
        self,
        ip: str,
        output_dir: str = None,
        websocket_port: int = 7779,
        skill_library: Optional[SkillLibrary] = None,
        recording_path: str = None,
        replay_path: str = None,
        enable_joystick: bool = False,
        enable_connection: bool = True,
        enable_ros_bridge: bool = True,
        enable_camera: bool = False,
    ):
        """Initialize the G1 robot.

        Args:
            ip: Robot IP address
            output_dir: Directory for saving outputs
            websocket_port: Port for web visualization
            skill_library: Skill library instance
            recording_path: Path to save recordings (if recording)
            replay_path: Path to replay recordings from (if replaying)
            enable_joystick: Enable pygame joystick control
            enable_connection: Enable robot connection module
            enable_ros_bridge: Enable ROS bridge
            enable_camera: Enable ZED camera module
        """
        super().__init__()
        self.ip = ip
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.recording_path = recording_path
        self.replay_path = replay_path
        self.enable_joystick = enable_joystick
        self.enable_connection = enable_connection
        self.enable_ros_bridge = enable_ros_bridge
        self.enable_camera = enable_camera
        self.websocket_port = websocket_port
        self.lcm = LCM()

        # Initialize skill library with G1 robot type
        if skill_library is None:
            from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills

            skill_library = MyUnitreeSkills(robot_type="g1")
        self.skill_library = skill_library

        # Set robot capabilities
        self.capabilities = [RobotCapability.LOCOMOTION]

        # Module references
        self.dimos = None
        self.connection = None
        self.websocket_vis = None
        self.foxglove_bridge = None
        self.joystick = None
        self.ros_bridge = None
        self.zed_camera = None

        self._setup_directories()

    def _setup_directories(self):
        """Setup output directories."""
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Robot outputs will be saved to: {self.output_dir}")

    def start(self):
        """Start the robot system with all modules."""
        self.dimos = core.start(4)  # 2 workers for connection and visualization

        if self.enable_connection:
            self._deploy_connection()

        self._deploy_visualization()

        if self.enable_camera:
            self._deploy_camera()

        if self.enable_joystick:
            self._deploy_joystick()

        if self.enable_ros_bridge:
            self._deploy_ros_bridge()

        self._start_modules()

        self.lcm.start()

        logger.info("UnitreeG1 initialized and started")
        logger.info(f"WebSocket visualization available at http://localhost:{self.websocket_port}")

    def _deploy_connection(self):
        """Deploy and configure the connection module."""
        self.connection = self.dimos.deploy(G1ConnectionModule, self.ip)

        # Configure LCM transports
        self.connection.movecmd.transport = core.LCMTransport("/cmd_vel", TwistStamped)
        self.connection.odom_in.transport = core.LCMTransport("/state_estimation", Odometry)
        self.connection.odom_pose.transport = core.LCMTransport("/odom", PoseStamped)

    def _deploy_camera(self):
        """Deploy and configure the ZED camera module (real or fake based on replay_path)."""

        if self.replay_path:
            # Use FakeZEDModule for replay
            from dimos.hardware.fake_zed_module import FakeZEDModule

            logger.info(f"Deploying FakeZEDModule for replay from: {self.replay_path}")
            self.zed_camera = self.dimos.deploy(
                FakeZEDModule,
                recording_path=self.replay_path,
                frame_id="zed_camera",
            )
        else:
            # Use real ZEDModule (with optional recording)
            logger.info("Deploying ZED camera module...")
            self.zed_camera = self.dimos.deploy(
                ZEDModule,
                camera_id=0,
                resolution="HD720",
                depth_mode="NEURAL",
                fps=30,
                enable_tracking=True,  # Enable for G1 pose estimation
                enable_imu_fusion=True,
                set_floor_as_origin=True,
                publish_rate=30.0,
                frame_id="zed_camera",
                recording_path=self.recording_path,  # Pass recording path if provided
            )

        # Configure ZED LCM transports (same for both real and fake)
        self.zed_camera.color_image.transport = core.pSHMTransport(
            "/zed/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
        )
        self.zed_camera.depth_image.transport = core.pSHMTransport(
            "/zed/depth_image", default_capacity=DEFAULT_CAPACITY_DEPTH_IMAGE
        )
        self.zed_camera.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)
        self.zed_camera.pose.transport = core.LCMTransport("/zed/pose", PoseStamped)

        logger.info("ZED camera module configured")

    def _deploy_visualization(self):
        """Deploy and configure visualization modules."""
        # Deploy WebSocket visualization module
        self.websocket_vis = self.dimos.deploy(WebsocketVisModule, port=self.websocket_port)
        self.websocket_vis.movecmd_stamped.transport = core.LCMTransport("/cmd_vel", TwistStamped)

        # Note: robot_pose connection removed since odom was removed from G1ConnectionModule

        # Deploy Foxglove bridge
        self.foxglove_bridge = FoxgloveBridge(
            shm_channels=[
                "/zed/color_image#sensor_msgs.Image",
                "/zed/depth_image#sensor_msgs.Image",
            ]
        )

    def _deploy_joystick(self):
        """Deploy joystick control module."""
        from dimos.robot.unitree_webrtc.g1_joystick_module import G1JoystickModule

        logger.info("Deploying G1 joystick module...")
        self.joystick = self.dimos.deploy(G1JoystickModule)
        self.joystick.twist_out.transport = core.LCMTransport("/cmd_vel", Twist)
        logger.info("Joystick module deployed - pygame window will open")

    def _deploy_ros_bridge(self):
        """Deploy and configure ROS bridge."""
        self.ros_bridge = ROSBridge("g1_ros_bridge")

        # Add /cmd_vel topic from ROS to DIMOS
        self.ros_bridge.add_topic(
            "/cmd_vel", TwistStamped, ROSTwistStamped, direction=BridgeDirection.ROS_TO_DIMOS
        )

        # Add /state_estimation topic from ROS to DIMOS
        self.ros_bridge.add_topic(
            "/state_estimation", Odometry, ROSOdometry, direction=BridgeDirection.ROS_TO_DIMOS
        )

        # Add /tf topic from ROS to DIMOS
        self.ros_bridge.add_topic(
            "/tf", TFMessage, ROSTFMessage, direction=BridgeDirection.ROS_TO_DIMOS
        )

        # Add /registered_scan topic from ROS to DIMOS
        self.ros_bridge.add_topic(
            "/registered_scan", PointCloud2, ROSPointCloud2, direction=BridgeDirection.ROS_TO_DIMOS
        )

        logger.info(
            "ROS bridge deployed: /cmd_vel, /state_estimation, /tf, /registered_scan (ROS → DIMOS)"
        )

    def _start_modules(self):
        """Start all deployed modules."""
        if self.connection:
            self.connection.start()
        self.websocket_vis.start()
        self.foxglove_bridge.start()

        if self.joystick:
            self.joystick.start()

        # Initialize skills after connection is established
        if self.skill_library is not None:
            for skill in self.skill_library:
                if hasattr(skill, "__name__"):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

    def move(self, twist_stamped: TwistStamped, duration: float = 0.0):
        """Send movement command to robot."""
        self.connection.move(twist_stamped, duration)

    def get_odom(self) -> PoseStamped:
        """Get the robot's odometry."""
        # Note: odom functionality removed from G1ConnectionModule
        return None

    def shutdown(self):
        """Shutdown the robot and clean up resources."""
        logger.info("Shutting down UnitreeG1...")

        # Shutdown ROS bridge if it exists
        if self.ros_bridge is not None:
            try:
                self.ros_bridge.shutdown()
                logger.info("ROS bridge shut down successfully")
            except Exception as e:
                logger.error(f"Error shutting down ROS bridge: {e}")

        # Stop other modules if needed
        if self.websocket_vis:
            try:
                self.websocket_vis.stop()
            except Exception as e:
                logger.error(f"Error stopping websocket vis: {e}")

        logger.info("UnitreeG1 shutdown complete")


def main():
    """Main entry point for testing."""
    import os
    import argparse
    from dotenv import load_dotenv

    load_dotenv()

    parser = argparse.ArgumentParser(description="Unitree G1 Humanoid Robot Control")
    parser.add_argument("--ip", default=os.getenv("ROBOT_IP"), help="Robot IP address")
    parser.add_argument("--joystick", action="store_true", help="Enable pygame joystick control")
    parser.add_argument("--camera", action="store_true", help="Enable ZED camera module")
    parser.add_argument("--output-dir", help="Output directory for logs/data")
    parser.add_argument("--record", help="Path to save recording")
    parser.add_argument("--replay", help="Path to replay recording from")

    args = parser.parse_args()

    pubsub.lcm.autoconf()

    robot = UnitreeG1(
        ip=args.ip,
        output_dir=args.output_dir,
        recording_path=args.record,
        replay_path=args.replay,
        enable_joystick=args.joystick,
        enable_camera=args.camera,
        enable_connection=os.getenv("ROBOT_IP") is not None,
        enable_ros_bridge=True,
    )
    robot.start()

    try:
        if args.joystick:
            print("\n" + "=" * 50)
            print("G1 HUMANOID JOYSTICK CONTROL")
            print("=" * 50)
            print("Focus the pygame window to control")
            print("Keys:")
            print("  WASD = Forward/Back/Strafe")
            print("  QE = Turn Left/Right")
            print("  Space = Emergency Stop")
            print("  ESC = Quit pygame (then Ctrl+C to exit)")
            print("=" * 50 + "\n")

        logger.info("G1 robot running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        robot.shutdown()


if __name__ == "__main__":
    main()

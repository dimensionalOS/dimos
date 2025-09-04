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
Unitree G1 humanoid robot with ZED camera integration.
Minimal implementation using WebRTC connection for robot control and ZED for vision.
"""

import os
import time
import logging
from typing import Optional

from dimos import core
from dimos.core import Module, In, Out, rpc
from dimos.hardware.zed_camera import ZEDModule
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Twist, Vector3, Quaternion
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.protocol import pubsub
from dimos.protocol.pubsub.lcmpubsub import LCM
from dimos.protocol.tf import TF
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import SkillLibrary
from dimos.robot.robot import Robot
from dimos.types.robot_capabilities import RobotCapability
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_g1", level=logging.INFO)

# Suppress verbose loggers
logging.getLogger("aiortc.codecs.h264").setLevel(logging.ERROR)
logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("websockets.server").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)
logging.getLogger("asyncio").setLevel(logging.ERROR)


class G1ConnectionModule(Module):
    """Simplified connection module for G1 - uses WebRTC for control, no video."""

    movecmd: In[Twist] = None
    odom: Out[PoseStamped] = None
    ip: str
    connection_type: str = "webrtc"

    _odom: PoseStamped = None

    def __init__(self, ip: str = None, connection_type: str = "webrtc", *args, **kwargs):
        self.ip = ip
        self.connection_type = connection_type
        self.tf = TF()
        self.connection = None
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        """Start the connection and subscribe to sensor streams."""
        # Use the exact same UnitreeWebRTCConnection as Go2
        self.connection = UnitreeWebRTCConnection(self.ip)

        # Subscribe only to odometry (no video/lidar for G1)
        self.connection.odom_stream().subscribe(self._publish_tf)
        self.movecmd.subscribe(self.move)

    def _publish_tf(self, msg):
        """Publish odometry and TF transforms."""
        self._odom = msg
        self.odom.publish(msg)
        self.tf.publish(Transform.from_pose("base_link", msg))

        # Publish ZED camera transform relative to robot base
        zed_transform = Transform(
            translation=Vector3(0.0, 0.0, 1.5),  # ZED mounted at ~1.5m height on G1
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="zed_camera",
            ts=time.time(),
        )
        self.tf.publish(zed_transform)

    @rpc
    def get_odom(self) -> Optional[PoseStamped]:
        """Get the robot's odometry."""
        return self._odom

    @rpc
    def move(self, twist: Twist, duration: float = 0.0):
        """Send movement command to robot."""
        self.connection.move(twist, duration)

    @rpc
    def standup(self):
        """Make the robot stand up."""
        return self.connection.standup()

    @rpc
    def liedown(self):
        """Make the robot lie down."""
        return self.connection.liedown()

    @rpc
    def publish_request(self, topic: str, data: dict):
        """Forward WebRTC publish requests to connection."""
        return self.connection.publish_request(topic, data)


class UnitreeG1(Robot):
    """Unitree G1 humanoid robot with ZED camera for vision."""

    def __init__(
        self,
        ip: str,
        output_dir: str = None,
        skill_library: Optional[SkillLibrary] = None,
        recording_path: str = None,
        replay_path: str = None,
    ):
        """Initialize the G1 robot.

        Args:
            ip: Robot IP address
            output_dir: Directory for saving outputs
            skill_library: Skill library instance
            recording_path: Path to save recordings (if recording)
            replay_path: Path to replay recordings from (if replaying)
        """
        super().__init__()
        self.ip = ip
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.recording_path = recording_path
        self.replay_path = replay_path
        self.lcm = LCM()

        # Initialize skill library with G1 robot type
        if skill_library is None:
            from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills

            skill_library = MyUnitreeSkills(robot_type="g1")
        self.skill_library = skill_library

        # Set robot capabilities
        self.capabilities = [RobotCapability.LOCOMOTION, RobotCapability.VISION]

        # Module references
        self.dimos = None
        self.connection = None
        self.zed_camera = None
        self.foxglove_bridge = None

        self._setup_directories()

    def _setup_directories(self):
        """Setup output directories."""
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Robot outputs will be saved to: {self.output_dir}")

    def start(self):
        """Start the robot system with all modules."""
        self.dimos = core.start(2)  # 2 workers for connection and ZED modules

        self._deploy_connection()
        self._deploy_camera()
        self._deploy_visualization()
        self._start_modules()

        self.lcm.start()

        logger.info("UnitreeG1 initialized and started")
        logger.info("ZED camera module deployed for vision")

    def _deploy_connection(self):
        """Deploy and configure the connection module."""
        self.connection = self.dimos.deploy(G1ConnectionModule, self.ip)

        # Configure LCM transports
        self.connection.odom.transport = core.LCMTransport("/g1/odom", PoseStamped)
        self.connection.movecmd.transport = core.LCMTransport("/g1/cmd_vel", Twist)

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
        self.zed_camera.color_image.transport = core.LCMTransport("/zed/color_image", Image)
        self.zed_camera.depth_image.transport = core.LCMTransport("/zed/depth_image", Image)
        self.zed_camera.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)
        self.zed_camera.pose.transport = core.LCMTransport("/zed/pose", PoseStamped)

        logger.info("ZED camera module configured")

    def _deploy_visualization(self):
        """Deploy visualization tools."""
        self.foxglove_bridge = FoxgloveBridge()

    def _start_modules(self):
        """Start all deployed modules."""
        self.connection.start()
        self.zed_camera.start()
        self.foxglove_bridge.start()

        # Initialize skills after connection is established
        if self.skill_library is not None:
            for skill in self.skill_library:
                if hasattr(skill, "__name__"):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

    def get_single_rgb_frame(self, timeout: float = 2.0) -> Image:
        """Get a single RGB frame from the ZED camera."""
        from dimos.protocol.pubsub.lcmpubsub import Topic

        topic = Topic("/zed/color_image", Image)
        return self.lcm.wait_for_message(topic, timeout=timeout)

    def move(self, twist: Twist, duration: float = 0.0):
        """Send movement command to robot."""
        self.connection.move(twist, duration)

    def get_odom(self) -> PoseStamped:
        """Get the robot's odometry."""
        return self.connection.get_odom()

    def standup(self):
        """Make the robot stand up."""
        return self.connection.standup()

    def liedown(self):
        """Make the robot lie down."""
        return self.connection.liedown()


def main():
    """Main entry point for testing."""
    import os
    from dotenv import load_dotenv

    load_dotenv()

    ip = os.getenv("ROBOT_IP")
    if not ip:
        logger.error("ROBOT_IP not set in environment")
        return

    pubsub.lcm.autoconf()

    robot = UnitreeG1(ip=ip)
    robot.start()

    try:
        logger.info("G1 robot running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")


if __name__ == "__main__":
    main()

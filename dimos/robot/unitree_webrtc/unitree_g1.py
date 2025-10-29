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

import logging
import os
import time

from dimos_lcm.foxglove_msgs import SceneUpdate
from geometry_msgs.msg import PoseStamped as ROSPoseStamped, TwistStamped as ROSTwistStamped
from nav_msgs.msg import Odometry as ROSOdometry
from reactivex.disposable import Disposable
from sensor_msgs.msg import Joy as ROSJoy, PointCloud2 as ROSPointCloud2
from tf2_msgs.msg import TFMessage as ROSTFMessage

from dimos import core
from dimos.agents2 import Agent
from dimos.agents2.cli.human import HumanInput
from dimos.agents2.skills.ros_navigation import RosNavigation
from dimos.agents2.spec import Model, Provider
from dimos.core import In, Module, Out, rpc
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.resource import Resource
from dimos.hardware.camera import zed
from dimos.hardware.camera.module import CameraModule
from dimos.hardware.camera.webcam import Webcam
from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    TwistStamped,
    Vector3,
)
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs import CameraInfo, Image, Joy, PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.moduleDB import ObjectDBModule
from dimos.perception.spatial_perception import SpatialMemory
from dimos.protocol import pubsub
from dimos.protocol.pubsub.lcmpubsub import LCM
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.robot.robot import Robot
from dimos.robot.ros_bridge import BridgeDirection, ROSBridge
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.rosnav import NavigationModule
from dimos.robot.unitree_webrtc.unitree_g1_skill_container import UnitreeG1SkillContainer
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import SkillLibrary
from dimos.types.robot_capabilities import RobotCapability
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_g1", level=logging.INFO)

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

    def __init__(
        self, ip: str | None = None, connection_type: str = "webrtc", *args, **kwargs
    ) -> None:
        self.ip = ip
        self.connection_type = connection_type
        self.connection = None
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self) -> None:
        """Start the connection and subscribe to sensor streams."""

        super().start()

        # Use the exact same UnitreeWebRTCConnection as Go2
        self.connection = UnitreeWebRTCConnection(self.ip)
        self.connection.start()
        unsub = self.movecmd.subscribe(self.move)
        self._disposables.add(Disposable(unsub))
        unsub = self.odom_in.subscribe(self._publish_odom_pose)
        self._disposables.add(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        self.connection.stop()
        super().stop()

    def _publish_odom_pose(self, msg: Odometry) -> None:
        self.odom_pose.publish(
            PoseStamped(
                ts=msg.ts,
                frame_id=msg.frame_id,
                position=msg.pose.pose.position,
                orientation=msg.pose.orientation,
            )
        )

    @rpc
    def move(self, twist_stamped: TwistStamped, duration: float = 0.0) -> None:
        """Send movement command to robot."""
        twist = Twist(linear=twist_stamped.linear, angular=twist_stamped.angular)
        self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict):
        """Forward WebRTC publish requests to connection."""
        return self.connection.publish_request(topic, data)


class UnitreeG1(Robot, Resource):
    """Unitree G1 humanoid robot."""

    def __init__(
        self,
        ip: str,
        output_dir: str | None = None,
        websocket_port: int = 7779,
        skill_library: SkillLibrary | None = None,
        recording_path: str | None = None,
        replay_path: str | None = None,
        enable_joystick: bool = False,
        enable_connection: bool = True,
        enable_ros_bridge: bool = True,
        enable_perception: bool = False,
        enable_camera: bool = False,
    ) -> None:
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
            enable_camera: Enable web camera module
        """
        super().__init__()
        self.ip = ip
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.recording_path = recording_path
        self.replay_path = replay_path
        self.enable_joystick = enable_joystick
        self.enable_connection = enable_connection
        self.enable_ros_bridge = enable_ros_bridge
        self.enable_perception = enable_perception
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
        self._dimos = ModuleCoordinator(n=4)
        self.connection = None
        self.websocket_vis = None
        self.foxglove_bridge = None
        self.spatial_memory_module = None
        self.joystick = None
        self.ros_bridge = None
        self.camera = None
        self._ros_nav = None
        self._setup_directories()

    def _setup_directories(self) -> None:
        """Setup directories for spatial memory storage."""
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Robot outputs will be saved to: {self.output_dir}")

        # Initialize memory directories
        self.memory_dir = os.path.join(self.output_dir, "memory")
        os.makedirs(self.memory_dir, exist_ok=True)

        # Initialize spatial memory properties
        self.spatial_memory_dir = os.path.join(self.memory_dir, "spatial_memory")
        self.spatial_memory_collection = "spatial_memory"
        self.db_path = os.path.join(self.spatial_memory_dir, "chromadb_data")
        self.visual_memory_path = os.path.join(self.spatial_memory_dir, "visual_memory.pkl")

        # Create spatial memory directories
        os.makedirs(self.spatial_memory_dir, exist_ok=True)
        os.makedirs(self.db_path, exist_ok=True)

    def _deploy_detection(self, goto) -> None:
        detection = self._dimos.deploy(
            ObjectDBModule, goto=goto, camera_info=zed.CameraInfo.SingleWebcam
        )

        detection.image.connect(self.camera.image)
        detection.pointcloud.transport = core.LCMTransport("/map", PointCloud2)

        detection.annotations.transport = core.LCMTransport("/annotations", ImageAnnotations)
        detection.detections.transport = core.LCMTransport("/detections", Detection2DArray)

        detection.scene_update.transport = core.LCMTransport("/scene_update", SceneUpdate)
        detection.target.transport = core.LCMTransport("/target", PoseStamped)
        detection.detected_pointcloud_0.transport = core.LCMTransport(
            "/detected/pointcloud/0", PointCloud2
        )
        detection.detected_pointcloud_1.transport = core.LCMTransport(
            "/detected/pointcloud/1", PointCloud2
        )
        detection.detected_pointcloud_2.transport = core.LCMTransport(
            "/detected/pointcloud/2", PointCloud2
        )

        detection.detected_image_0.transport = core.LCMTransport("/detected/image/0", Image)
        detection.detected_image_1.transport = core.LCMTransport("/detected/image/1", Image)
        detection.detected_image_2.transport = core.LCMTransport("/detected/image/2", Image)

        self.detection = detection

    def start(self) -> None:
        self.lcm.start()
        self._dimos.start()

        if self.enable_connection:
            self._deploy_connection()

        self._deploy_visualization()

        if self.enable_joystick:
            self._deploy_joystick()

        if self.enable_ros_bridge:
            self._deploy_ros_bridge()

        self.nav = self._dimos.deploy(NavigationModule)
        self.nav.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)
        self.nav.goal_pose.transport = core.LCMTransport("/goal_pose", PoseStamped)
        self.nav.cancel_goal.transport = core.LCMTransport("/cancel_goal", Bool)
        self.nav.joy.transport = core.LCMTransport("/joy", Joy)
        self.nav.start()

        self._deploy_camera()
        self._deploy_detection(self.nav.go_to)

        if self.enable_perception:
            self._deploy_perception()

        self.lcm.start()

        # Setup agent with G1 skills
        logger.info("Setting up agent with G1 skills...")

        agent = Agent(
            system_prompt="You are a helpful assistant controlling a Unitree G1 humanoid robot. You can control the robot's arms, movement modes, and navigation.",
            model=Model.GPT_4O,
            provider=Provider.OPENAI,
        )

        # Register G1-specific skill container
        g1_skills = UnitreeG1SkillContainer(robot=self)
        agent.register_skills(g1_skills)

        human_input = self._dimos.deploy(HumanInput)
        agent.register_skills(human_input)

        if self.enable_perception:
            agent.register_skills(self.detection)

        # Register ROS navigation
        self._ros_nav = RosNavigation(self)
        self._ros_nav.start()
        agent.register_skills(self._ros_nav)

        agent.run_implicit_skill("human")
        agent.start()

        # For logging
        skills = [tool.name for tool in agent.get_tools()]
        logger.info(f"Agent configured with {len(skills)} skills: {', '.join(skills)}")

        agent.loop_thread()

        logger.info("UnitreeG1 initialized and started")
        logger.info(f"WebSocket visualization available at http://localhost:{self.websocket_port}")
        self._start_modules()

    def stop(self) -> None:
        self._dimos.stop()
        if self._ros_nav:
            self._ros_nav.stop()
        self.lcm.stop()

    def _deploy_connection(self) -> None:
        """Deploy and configure the connection module."""
        self.connection = self._dimos.deploy(G1ConnectionModule, self.ip)

        # Configure LCM transports
        self.connection.movecmd.transport = core.LCMTransport("/cmd_vel", TwistStamped)
        self.connection.odom_in.transport = core.LCMTransport("/state_estimation", Odometry)
        self.connection.odom_pose.transport = core.LCMTransport("/odom", PoseStamped)

    def _deploy_camera(self) -> None:
        """Deploy and configure a standard webcam module."""
        logger.info("Deploying standard webcam module...")

        self.camera = self._dimos.deploy(
            CameraModule,
            transform=Transform(
                translation=Vector3(0.05, 0.0, 0.0),
                rotation=Quaternion.from_euler(Vector3(0.0, 0.2, 0.0)),
                frame_id="sensor",
                child_frame_id="camera_link",
            ),
            hardware=lambda: Webcam(
                camera_index=0,
                frequency=15,
                stereo_slice="left",
                camera_info=zed.CameraInfo.SingleWebcam,
            ),
        )

        self.camera.image.transport = core.LCMTransport("/image", Image)
        self.camera.camera_info.transport = core.LCMTransport("/camera_info", CameraInfo)
        logger.info("Webcam module configured")

    def _deploy_visualization(self) -> None:
        """Deploy and configure visualization modules."""
        # Deploy WebSocket visualization module
        self.websocket_vis = self._dimos.deploy(WebsocketVisModule, port=self.websocket_port)
        self.websocket_vis.movecmd_stamped.transport = core.LCMTransport("/cmd_vel", TwistStamped)

        # Note: robot_pose connection removed since odom was removed from G1ConnectionModule

        # Deploy Foxglove bridge
        self.foxglove_bridge = FoxgloveBridge(
            shm_channels=[
                "/zed/color_image#sensor_msgs.Image",
                "/zed/depth_image#sensor_msgs.Image",
            ]
        )
        self.foxglove_bridge.start()

    def _deploy_perception(self) -> None:
        self.spatial_memory_module = self._dimos.deploy(
            SpatialMemory,
            collection_name=self.spatial_memory_collection,
            db_path=self.db_path,
            visual_memory_path=self.visual_memory_path,
            output_dir=self.spatial_memory_dir,
        )

        self.spatial_memory_module.color_image.connect(self.camera.image)
        self.spatial_memory_module.odom.transport = core.LCMTransport("/odom", PoseStamped)

        logger.info("Spatial memory module deployed and connected")

    def _deploy_joystick(self) -> None:
        """Deploy joystick control module."""
        from dimos.robot.unitree_webrtc.g1_joystick_module import G1JoystickModule

        logger.info("Deploying G1 joystick module...")
        self.joystick = self._dimos.deploy(G1JoystickModule)
        self.joystick.twist_out.transport = core.LCMTransport("/cmd_vel", Twist)
        logger.info("Joystick module deployed - pygame window will open")

    def _deploy_ros_bridge(self) -> None:
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

        from std_msgs.msg import Bool as ROSBool

        from dimos.msgs.std_msgs import Bool

        # Navigation control topics from autonomy stack
        self.ros_bridge.add_topic(
            "/goal_pose", PoseStamped, ROSPoseStamped, direction=BridgeDirection.DIMOS_TO_ROS
        )
        self.ros_bridge.add_topic(
            "/cancel_goal", Bool, ROSBool, direction=BridgeDirection.DIMOS_TO_ROS
        )
        self.ros_bridge.add_topic(
            "/goal_reached", Bool, ROSBool, direction=BridgeDirection.ROS_TO_DIMOS
        )

        self.ros_bridge.add_topic("/joy", Joy, ROSJoy, direction=BridgeDirection.DIMOS_TO_ROS)

        self.ros_bridge.add_topic(
            "/registered_scan",
            PointCloud2,
            ROSPointCloud2,
            direction=BridgeDirection.ROS_TO_DIMOS,
            remap_topic="/map",
        )

        self.ros_bridge.start()

        logger.info(
            "ROS bridge deployed: /cmd_vel, /state_estimation, /tf, /registered_scan (ROS → DIMOS)"
        )

    def _start_modules(self) -> None:
        """Start all deployed modules."""
        self._dimos.start_all_modules()

        # Initialize skills after connection is established
        if self.skill_library is not None:
            for skill in self.skill_library:
                if hasattr(skill, "__name__"):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

    def move(self, twist_stamped: TwistStamped, duration: float = 0.0) -> None:
        """Send movement command to robot."""
        self.connection.move(twist_stamped, duration)

    def get_odom(self) -> PoseStamped:
        """Get the robot's odometry."""
        # Note: odom functionality removed from G1ConnectionModule
        return None

    @property
    def spatial_memory(self) -> SpatialMemory | None:
        return self.spatial_memory_module


def main() -> None:
    """Main entry point for testing."""
    import argparse
    import os

    from dotenv import load_dotenv

    load_dotenv()

    parser = argparse.ArgumentParser(description="Unitree G1 Humanoid Robot Control")
    parser.add_argument("--ip", default=os.getenv("ROBOT_IP"), help="Robot IP address")
    parser.add_argument("--joystick", action="store_true", help="Enable pygame joystick control")
    parser.add_argument("--camera", action="store_true", help="Enable usb camera module")
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
        enable_perception=True,
    )
    robot.start()

    # time.sleep(7)
    # print("Starting navigation...")
    # print(
    #     robot.nav.go_to(
    #         PoseStamped(
    #             ts=time.time(),
    #             frame_id="map",
    #             position=Vector3(0.0, 0.0, 0.03),
    #             orientation=Quaternion(0, 0, 0, 0),
    #         ),
    #         timeout=10,
    #     ),
    # )
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
        robot.stop()


if __name__ == "__main__":
    main()

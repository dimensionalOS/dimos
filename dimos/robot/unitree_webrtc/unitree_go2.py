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


import functools
import logging
import os
import time
import warnings
from typing import Callable, Optional

from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3, Quaternion
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.msgs.sensor_msgs import Image
from dimos.perception.spatial_perception import SpatialMemory
from dimos.protocol import pubsub
from dimos.protocol.tf import TF
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
from dimos.navigation.global_planner import AstarPlanner
from dimos.navigation.local_planner.holonomic_local_planner import HolonomicLocalPlanner
from dimos.navigation.bt_navigator.navigator import BehaviorTreeNavigator
from dimos.navigation.frontier_exploration import WavefrontFrontierExplorer
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import AbstractRobotSkill, SkillLibrary
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay
from dimos_lcm.std_msgs import Bool

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_go2", level=logging.INFO)

# Suppress verbose loggers
logging.getLogger("aiortc.codecs.h264").setLevel(logging.ERROR)
logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("websockets.server").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)
logging.getLogger("asyncio").setLevel(logging.ERROR)
logging.getLogger("root").setLevel(logging.WARNING)

# Suppress warnings
warnings.filterwarnings("ignore", message="coroutine.*was never awaited")
warnings.filterwarnings("ignore", message="H264Decoder.*failed to decode")


class FakeRTC:
    """Fake WebRTC connection for testing with recorded data."""

    def __init__(self, *args, **kwargs):
        data = get_data("unitree_office_walk")

    def connect(self):
        pass

    def standup(self):
        print("standup suppressed")

    def liedown(self):
        print("liedown suppressed")

    @functools.cache
    def lidar_stream(self):
        print("lidar stream start")
        lidar_store = TimedSensorReplay("unitree_office_walk/lidar", autocast=LidarMessage.from_msg)
        return lidar_store.stream()

    @functools.cache
    def odom_stream(self):
        print("odom stream start")
        odom_store = TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)
        return odom_store.stream()

    @functools.cache
    def video_stream(self):
        print("video stream start")
        video_store = TimedSensorReplay(
            "unitree_office_walk/video", autocast=lambda x: Image.from_numpy(x).to_rgb()
        )
        return video_store.stream()

    def move(self, vector: Vector3, duration: float = 0.0):
        pass

    def publish_request(self, topic: str, data: dict):
        """Fake publish request for testing."""
        return {"status": "ok", "message": "Fake publish"}


class ConnectionModule(Module):
    """Module that handles robot sensor data and movement commands."""

    movecmd: In[Vector3] = None
    odom: Out[PoseStamped] = None
    lidar: Out[LidarMessage] = None
    video: Out[Image] = None
    ip: str
    playback: bool

    _odom: PoseStamped = None
    _lidar: LidarMessage = None

    def __init__(self, ip: str = None, playback: bool = False, *args, **kwargs):
        self.ip = ip
        self.playback = playback
        self.tf = TF()
        self.connection = None
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        """Start the connection and subscribe to sensor streams."""
        if self.playback:
            self.connection = FakeRTC(self.ip)
        else:
            self.connection = UnitreeWebRTCConnection(self.ip)

        # Connect sensor streams to outputs
        self.connection.lidar_stream().subscribe(self.lidar.publish)
        self.connection.odom_stream().subscribe(self._publish_tf)
        self.connection.video_stream().subscribe(self.video.publish)
        self.movecmd.subscribe(self.move)

    def _publish_tf(self, msg):
        self._odom = msg
        self.odom.publish(msg)
        self.tf.publish(Transform.from_pose("base_link", msg))
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )
        self.tf.publish(camera_link)

    @rpc
    def get_odom(self) -> Optional[PoseStamped]:
        """Get the robot's odometry.

        Returns:
            The robot's odometry
        """
        return self._odom

    @rpc
    def move(self, vector: Vector3, duration: float = 0.0):
        """Send movement command to robot."""
        self.connection.move(vector, duration)

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
        """Publish a request to the WebRTC connection.
        Args:
            topic: The RTC topic to publish to
            data: The data dictionary to publish
        Returns:
            The result of the publish request
        """
        return self.connection.publish_request(topic, data)


class UnitreeGo2:
    """Full Unitree Go2 robot with navigation and perception capabilities."""

    def __init__(
        self,
        ip: str,
        output_dir: str = None,
        websocket_port: int = 7779,
        skill_library: Optional[SkillLibrary] = None,
        playback: bool = False,
    ):
        """Initialize the robot system.

        Args:
            ip: Robot IP address (or None for fake connection)
            output_dir: Directory for saving outputs (default: assets/output)
            enable_perception: Whether to enable spatial memory/perception
            websocket_port: Port for web visualization
            skill_library: Skill library instance
            playback: If True, use recorded data instead of real robot connection
        """
        self.ip = ip
        self.playback = playback or (ip is None)  # Auto-enable playback if no IP provided
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.websocket_port = websocket_port

        # Initialize skill library
        if skill_library is None:
            skill_library = MyUnitreeSkills()
        self.skill_library = skill_library

        self.dimos = None
        self.connection = None
        self.mapper = None
        self.global_planner = None
        self.local_planner = None
        self.navigator = None
        self.frontier_explorer = None
        self.websocket_vis = None
        self.foxglove_bridge = None
        self.spatial_memory_module = None

        self._setup_directories()

    def _setup_directories(self):
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

    def start(self):
        """Start the robot system with all modules."""
        self.dimos = core.start(4)

        self._deploy_connection()
        self._deploy_mapping()
        self._deploy_navigation()
        self._deploy_visualization()
        self._deploy_perception()

        self._start_modules()

        logger.info("UnitreeGo2 initialized and started")
        logger.info(f"WebSocket visualization available at http://localhost:{self.websocket_port}")

    def _deploy_connection(self):
        """Deploy and configure the connection module."""
        self.connection = self.dimos.deploy(ConnectionModule, self.ip, playback=self.playback)

        self.connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
        self.connection.odom.transport = core.LCMTransport("/odom", PoseStamped)
        self.connection.video.transport = core.LCMTransport("/video", Image)
        self.connection.movecmd.transport = core.LCMTransport("/cmd_vel", Vector3)

    def _deploy_mapping(self):
        """Deploy and configure the mapping module."""
        self.mapper = self.dimos.deploy(Map, voxel_size=0.5, global_publish_interval=2.5)

        self.mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
        self.mapper.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)
        self.mapper.local_costmap.transport = core.LCMTransport("/local_costmap", OccupancyGrid)

        self.mapper.lidar.connect(self.connection.lidar)

    def _deploy_navigation(self):
        """Deploy and configure navigation modules."""
        self.global_planner = self.dimos.deploy(AstarPlanner)
        self.local_planner = self.dimos.deploy(HolonomicLocalPlanner)
        self.navigator = self.dimos.deploy(BehaviorTreeNavigator, local_planner=self.local_planner)
        self.frontier_explorer = self.dimos.deploy(WavefrontFrontierExplorer)

        self.navigator.goal.transport = core.LCMTransport("/navigation_goal", PoseStamped)
        self.navigator.goal_request.transport = core.LCMTransport("/goal_request", PoseStamped)
        self.navigator.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)
        self.navigator.global_costmap.transport = core.LCMTransport(
            "/global_costmap", OccupancyGrid
        )
        self.global_planner.path.transport = core.LCMTransport("/global_path", Path)
        self.local_planner.cmd_vel.transport = core.LCMTransport("/cmd_vel", Vector3)
        self.frontier_explorer.goal_request.transport = core.LCMTransport(
            "/goal_request", PoseStamped
        )
        self.frontier_explorer.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)

        self.global_planner.target.connect(self.navigator.goal)

        self.global_planner.global_costmap.connect(self.mapper.global_costmap)
        self.global_planner.odom.connect(self.connection.odom)

        self.local_planner.path.connect(self.global_planner.path)
        self.local_planner.local_costmap.connect(self.mapper.local_costmap)
        self.local_planner.odom.connect(self.connection.odom)

        self.connection.movecmd.connect(self.local_planner.cmd_vel)

        self.navigator.odom.connect(self.connection.odom)

        self.frontier_explorer.costmap.connect(self.mapper.global_costmap)
        self.frontier_explorer.odometry.connect(self.connection.odom)

    def _deploy_visualization(self):
        """Deploy and configure visualization modules."""
        self.websocket_vis = self.dimos.deploy(WebsocketVisModule, port=self.websocket_port)
        self.websocket_vis.click_goal.transport = core.LCMTransport("/goal_request", PoseStamped)

        self.websocket_vis.robot_pose.connect(self.connection.odom)
        self.websocket_vis.path.connect(self.global_planner.path)
        self.websocket_vis.global_costmap.connect(self.mapper.global_costmap)

        self.foxglove_bridge = FoxgloveBridge()

    def _deploy_perception(self):
        """Deploy and configure the spatial memory module."""
        self.spatial_memory_module = self.dimos.deploy(
            SpatialMemory,
            collection_name=self.spatial_memory_collection,
            db_path=self.db_path,
            visual_memory_path=self.visual_memory_path,
            output_dir=self.spatial_memory_dir,
        )

        self.spatial_memory_module.video.connect(self.connection.video)
        self.spatial_memory_module.odom.connect(self.connection.odom)

        logger.info("Spatial memory module deployed and connected")

    def _start_modules(self):
        """Start all deployed modules in the correct order."""
        self.connection.start()
        self.mapper.start()
        self.global_planner.start()
        self.local_planner.start()
        self.navigator.start()
        self.frontier_explorer.start()
        self.websocket_vis.start()
        self.foxglove_bridge.start()

        if self.spatial_memory_module:
            self.spatial_memory_module.start()

        # Initialize skills after connection is established
        if self.skill_library is not None:
            for skill in self.skill_library:
                if isinstance(skill, AbstractRobotSkill):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

    def move(self, vector: Vector3, duration: float = 0.0):
        """Send movement command to robot."""
        self.connection.move(vector, duration)

    def explore(self) -> bool:
        """Start autonomous frontier exploration.

        Returns:
            True if exploration started successfully
        """
        return self.frontier_explorer.explore()

    def navigate_to(self, pose: PoseStamped, blocking: bool = True):
        """Navigate to a target pose.

        Args:
            pose: Target pose to navigate to
            blocking: If True, block until goal is reached. If False, return immediately.

        Returns:
            If blocking=True: True if navigation was successful, False otherwise
            If blocking=False: True if goal was accepted, False otherwise
        """

        logger.info(
            f"Navigating to pose: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )
        return self.navigator.set_goal(pose, blocking=blocking)

    def stop_exploration(self) -> bool:
        """Stop autonomous exploration.

        Returns:
            True if exploration was stopped
        """
        return self.frontier_explorer.stop_exploration()

    def cancel_navigation(self) -> bool:
        """Cancel the current navigation goal.

        Returns:
            True if goal was cancelled
        """
        return self.navigator.cancel_goal()

    @property
    def spatial_memory(self) -> Optional[SpatialMemory]:
        """Get the robot's spatial memory module.

        Returns:
            SpatialMemory module instance or None if perception is disabled
        """
        return self.spatial_memory_module

    def get_skills(self):
        """Get the robot's skill library.

        Returns:
            The robot's skill library for adding/managing skills
        """
        return self.skill_library

    def get_odom(self) -> PoseStamped:
        """Get the robot's odometry.

        Returns:
            The robot's odometry
        """
        return self.connection.get_odom()


def main():
    """Main entry point."""
    ip = os.getenv("ROBOT_IP")

    pubsub.lcm.autoconf()

    robot = UnitreeGo2(ip=ip, websocket_port=7779, playback=False)
    robot.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")


if __name__ == "__main__":
    main()

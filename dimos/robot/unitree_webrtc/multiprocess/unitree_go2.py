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
import threading
import time
import warnings
from typing import Callable, Optional

from reactivex import Observable
from reactivex import operators as ops

from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
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
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import getter_streaming
from dimos.utils.testing import TimedSensorReplay
from dimos_lcm.std_msgs import Bool

logger = setup_logger("dimos.robot.unitree_webrtc.multiprocess.unitree_go2", level=logging.INFO)

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


class FakeRTC(UnitreeWebRTCConnection):
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

    def move(self, vector: Vector3):
        pass


class ConnectionModule(UnitreeWebRTCConnection, Module):
    """Module that handles robot sensor data and movement commands."""

    movecmd: In[Vector3] = None
    odom: Out[PoseStamped] = None
    lidar: Out[LidarMessage] = None
    video: Out[Image] = None
    ip: str

    _odom: Callable[[], Odometry]
    _lidar: Callable[[], LidarMessage]

    def __init__(self, ip: str, *args, **kwargs):
        self.ip = ip
        self.tf = TF()
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        """Start the connection and subscribe to sensor streams."""
        super().__init__(self.ip)

        self.lidar_stream().subscribe(self.lidar.publish)
        self.odom_stream().subscribe(self.odom.publish)
        self.video_stream().subscribe(self.video.publish)
        self.tf_stream().subscribe(self.tf.publish)
        self.movecmd.subscribe(self.move)

        self._odom = getter_streaming(self.odom_stream())
        self._lidar = getter_streaming(self.lidar_stream())

    @rpc
    def move(self, vector: Vector3):
        """Send movement command to robot."""
        super().move(vector)


class UnitreeGo2:
    """Full Unitree Go2 robot with navigation and perception capabilities."""

    def __init__(
        self,
        ip: str,
        output_dir: str = None,
        websocket_port: int = 7779,
    ):
        """Initialize the robot system.

        Args:
            ip: Robot IP address (or None for fake connection)
            output_dir: Directory for saving outputs (default: assets/output)
            enable_perception: Whether to enable spatial memory/perception
            websocket_port: Port for web visualization
        """
        self.ip = ip
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.websocket_port = websocket_port

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
        if self.ip:
            self.connection = self.dimos.deploy(ConnectionModule, self.ip)
        else:
            logger.info("No robot IP provided, using fake connection with recorded data")
            self.connection = self.dimos.deploy(ConnectionModule.__class__.__bases__[0], "fake")
            self.connection.__class__ = type(
                "FakeConnectionModule", (FakeRTC, Module), dict(ConnectionModule.__dict__)
            )

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
        self.navigator = self.dimos.deploy(BehaviorTreeNavigator)
        self.frontier_explorer = self.dimos.deploy(WavefrontFrontierExplorer)

        self.navigator.goal.transport = core.LCMTransport("/navigation_goal", PoseStamped)
        self.navigator.goal_request.transport = core.LCMTransport("/goal_request", PoseStamped)
        self.navigator.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)
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
        self.navigator.connect_local_planner(self.local_planner)

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

    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Navigate to a specific pose.

        Args:
            x: Target X position in meters
            y: Target Y position in meters
            yaw: Target orientation in radians (default: 0)

        Returns:
            True if navigation goal was accepted
        """
        if not self.navigator:
            raise RuntimeError("Navigation not initialized. Call start() first.")

        goal = PoseStamped()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = 0.0
        goal.orientation.w = 1.0
        goal.frame_id = "map"
        goal.ts = time.time()

        return self.navigator.set_goal(goal)

    def explore(self) -> bool:
        """Start autonomous frontier exploration.

        Returns:
            True if exploration started successfully
        """
        if not self.frontier_explorer:
            raise RuntimeError("Frontier explorer not initialized. Call start() first.")
        return self.frontier_explorer.explore()

    def stop_exploration(self) -> bool:
        """Stop autonomous exploration.

        Returns:
            True if exploration was stopped
        """
        if not self.frontier_explorer:
            return False
        return self.frontier_explorer.stop_exploration()

    def cancel_navigation(self) -> bool:
        """Cancel the current navigation goal.

        Returns:
            True if goal was cancelled
        """
        if not self.navigator:
            return False
        return self.navigator.cancel_goal()

    @property
    def spatial_memory(self) -> Optional[SpatialMemory]:
        """Get the robot's spatial memory module.

        Returns:
            SpatialMemory module instance or None if perception is disabled
        """
        return self.spatial_memory_module


def main():
    """Main entry point."""
    ip = os.getenv("ROBOT_IP")

    pubsub.lcm.autoconf()

    robot = UnitreeGo2(ip=ip, websocket_port=7779)
    robot.start()

    robot.explore()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")


if __name__ == "__main__":
    main()

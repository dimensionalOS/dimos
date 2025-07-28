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


import asyncio
import functools
import logging
import os
import threading
import time
import warnings
from typing import Callable, Optional

from reactivex import Observable
from reactivex import operators as ops

import dimos.core.colors as colors
from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Transform, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.perception.spatial_perception import SpatialMemory
from dimos.protocol import pubsub
from dimos.protocol.tf import TF
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.robot.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.robot.global_planner import AstarPlanner
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection, VideoMessage
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.costmap import Costmap
from dimos.types.vector import Vector
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import getter_streaming
from dimos.utils.testing import TimedSensorReplay

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


# can be swapped in for UnitreeWebRTCConnection
class FakeRTC(UnitreeWebRTCConnection):
    def __init__(self, *args, **kwargs):
        # ensures we download msgs from lfs store
        data = get_data("unitree_office_walk")

    def connect(self): ...

    def standup(self):
        print("standup supressed")

    def liedown(self):
        print("liedown supressed")

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

    def move(self, vector: Vector):
        print("move supressed", vector)


class ConnectionModule(FakeRTC, Module):
    movecmd: In[Vector3] = None
    odom: Out[Vector3] = None
    lidar: Out[LidarMessage] = None
    video: Out[VideoMessage] = None
    ip: str

    _odom: Callable[[], Odometry]
    _lidar: Callable[[], LidarMessage]

    @rpc
    def move(self, vector: Vector3):
        super().move(vector)

    def __init__(self, ip: str, *args, **kwargs):
        self.ip = ip
        self.tf = TF()
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        # Initialize the parent WebRTC connection
        super().__init__(self.ip)
        self.tf = TF()
        # Connect sensor streams to LCM outputs
        self.lidar_stream().subscribe(self.lidar.publish)
        self.odom_stream().subscribe(self.odom.publish)
        self.video_stream().subscribe(self.video.publish)
        self.tf_stream().subscribe(self.tf.publish)

        # Connect LCM input to robot movement commands
        self.movecmd.subscribe(self.move)

        # Set up streaming getters for latest sensor data
        self._odom = getter_streaming(self.odom_stream())
        self._lidar = getter_streaming(self.lidar_stream())

    @rpc
    def get_local_costmap(self) -> Costmap:
        return self._lidar().costmap()

    @rpc
    def get_odom(self) -> Odometry:
        return self._odom()

    @rpc
    def get_pos(self) -> Vector:
        return self._odom().position


class ControlModule(Module):
    plancmd: Out[Pose] = None

    @rpc
    def start(self):
        def plancmd():
            time.sleep(4)
            print(colors.red("requesting global plan"))
            self.plancmd.publish(Pose(0, 0, 0, 0, 0, 0, 1))

        thread = threading.Thread(target=plancmd, daemon=True)
        thread.start()


class UnitreeGo2Light:
    def __init__(
        self,
        ip: str,
        output_dir: str = os.path.join(os.getcwd(), "assets", "output"),
    ):
        self.output_dir = output_dir
        self.ip = ip
        self.dimos = None
        self.connection = None
        self.mapper = None
        self.local_planner = None
        self.global_planner = None
        self.frontier_explorer = None
        self.foxglove_bridge = None
        self.ctrl = None

        # Spatial Memory Initialization ======================================
        # Create output directory
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

        # Create spatial memory directory
        os.makedirs(self.spatial_memory_dir, exist_ok=True)
        os.makedirs(self.db_path, exist_ok=True)

        self.spatial_memory_module = None
        # ==============================================================

    async def start(self):
        self.dimos = core.start(4)

        # Connection Module - Robot sensor data interface via WebRTC ===================
        self.connection = self.dimos.deploy(ConnectionModule, self.ip)

        # This enables LCM transport
        # Ensures system multicast, udp sizes are auto-adjusted if needed

        # Configure ConnectionModule LCM transport outputs for sensor data streams
        # OUTPUT: LiDAR point cloud data to /lidar topic
        self.connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
        # OUTPUT: Robot odometry/pose data to /odom topic
        self.connection.odom.transport = core.LCMTransport("/odom", PoseStamped)
        # OUTPUT: Camera video frames to /video topic
        self.connection.video.transport = core.LCMTransport("/video", Image)
        # ======================================================================
        # self.connection.tf.transport = core.LCMTransport("/tf", LidarMessage)

        # Map Module - Point cloud accumulation and costmap generation =========
        self.mapper = self.dimos.deploy(Map, voxel_size=0.5, global_publish_interval=2.5)

        # OUTPUT: Accumulated point cloud map to /global_map topic
        self.mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)

        # Connect ConnectionModule OUTPUT lidar to Map INPUT lidar for point cloud accumulation
        self.mapper.lidar.connect(self.connection.lidar)
        # ====================================================================

        # Local planner Module, LCM transport & connection ================
        self.local_planner = self.dimos.deploy(
            VFHPurePursuitPlanner,
            get_costmap=self.connection.get_local_costmap,
        )

        # Connects odometry LCM stream to BaseLocalPlanner odometry input
        self.local_planner.odom.connect(self.connection.odom)

        # Configures BaseLocalPlanner movecmd output to /move LCM topic
        self.local_planner.movecmd.transport = core.LCMTransport("/move", Vector3)

        # Connects connection.movecmd input to local_planner.movecmd output
        self.connection.movecmd.connect(self.local_planner.movecmd)
        # ===================================================================

        # Global Planner Module ===============
        self.global_planner = self.dimos.deploy(
            AstarPlanner,
            get_costmap=self.mapper.costmap,
            get_robot_pos=self.connection.get_pos,
            set_local_nav=self.local_planner.navigate_path_local,
        )

        # Spatial Memory Module ======================================
        self.spatial_memory_module = self.dimos.deploy(
            SpatialMemory,
            collection_name=self.spatial_memory_collection,
            db_path=self.db_path,
            visual_memory_path=self.visual_memory_path,
            output_dir=self.spatial_memory_dir,
        )

        # Connect video and odometry streams to spatial memory
        self.spatial_memory_module.video.connect(self.connection.video)
        self.spatial_memory_module.odom.connect(self.connection.odom)

        # Start the spatial memory module
        self.spatial_memory_module.start()

        logger.info("Spatial memory module deployed and connected")
        # ==============================================================

        # Configure AstarPlanner OUTPUT path: Out[Path] to /global_path LCM topic
        self.global_planner.path.transport = core.pLCMTransport("/global_path")
        # ======================================

        # Global Planner Control Module ===========================
        # Debug module that sends (0,0,0) goal after 4 second delay
        self.ctrl = self.dimos.deploy(ControlModule)

        # Configure ControlModule OUTPUT to publish goal coordinates to /global_target
        self.ctrl.plancmd.transport = core.LCMTransport("/global_target", Vector3)

        # Connect ControlModule OUTPUT to AstarPlanner INPUT - triggers A* planning when goal received
        self.global_planner.target.connect(self.ctrl.plancmd)
        # ==========================================

        # Visualization ============================
        self.foxglove_bridge = FoxgloveBridge()
        # ==========================================

        self.frontier_explorer = WavefrontFrontierExplorer(
            set_goal=self.global_planner.set_goal,
            get_costmap=self.mapper.costmap,
            get_robot_pos=self.connection.get_pos,
        )

        # Prints full module IO
        print("\n")
        for module in [
            self.connection,
            self.mapper,
            self.local_planner,
            self.global_planner,
            self.ctrl,
        ]:
            print(module.io(), "\n")

        # Start modules =============================
        self.mapper.start()
        self.connection.start()
        self.local_planner.start()
        self.global_planner.start()
        self.foxglove_bridge.start()
        # self.ctrl.start() # DEBUG

        await asyncio.sleep(2)
        print("querying system")
        print(self.mapper.costmap())
        logger.info("UnitreeGo2Light initialized and started")

    def get_pose(self) -> dict:
        """Get the current pose (position and rotation) of the robot.

        Returns:
            Dictionary containing:
                - position: Vector (x, y, z)
                - rotation: Vector (roll, pitch, yaw) in radians
        """
        if not self.connection:
            raise RuntimeError("Connection not established. Call start() first.")
        odom = self.connection.get_odom()
        position = Vector(odom.x, odom.y, odom.z)
        rotation = Vector(odom.roll, odom.pitch, odom.yaw)
        return {"position": position, "rotation": rotation}

    def move(self, velocity: Vector, duration: float = 0.0) -> bool:
        """Move the robot using velocity commands.

        Args:
            velocity: Velocity vector [x, y, yaw]
            duration: Duration to apply command (seconds)

        Returns:
            bool: True if movement succeeded
        """
        if not self.connection:
            raise RuntimeError("Connection not established. Call start() first.")
        self.connection.move(Vector3(velocity.x, velocity.y, velocity.z))
        if duration > 0:
            time.sleep(duration)
            self.connection.move(Vector3(0, 0, 0))  # Stop
        return True

    def explore(self, stop_event=None) -> bool:
        """Start autonomous frontier exploration.

        Args:
            stop_event: Optional threading.Event to signal when exploration should stop

        Returns:
            bool: True if exploration completed successfully
        """
        if not self.frontier_explorer:
            raise RuntimeError("Frontier explorer not initialized. Call start() first.")
        return self.frontier_explorer.explore(stop_event=stop_event)

    def standup(self):
        """Make the robot stand up."""
        if self.connection and hasattr(self.connection, "standup"):
            return self.connection.standup()

    def liedown(self):
        """Make the robot lie down."""
        if self.connection and hasattr(self.connection, "liedown"):
            return self.connection.liedown()

    @property
    def costmap(self):
        """Access to the costmap for navigation."""
        if not self.mapper:
            raise RuntimeError("Mapper not initialized. Call start() first.")
        return self.mapper.costmap

    @property
    def spatial_memory(self) -> Optional[SpatialMemory]:
        """Get the robot's spatial memory module.

        Returns:
            SpatialMemory module instance or None if perception is disabled
        """
        return self.spatial_memory_module

    def get_video_stream(self, fps: int = 30) -> Observable:
        """Get the video stream with rate limiting and processing.

        Args:
            fps: Frames per second for rate limiting

        Returns:
            Observable stream of video frames
        """
        # Import required modules for LCM subscription
        from reactivex import create
        from reactivex.disposable import Disposable

        from dimos.msgs.sensor_msgs import Image
        from dimos.protocol.pubsub.lcmpubsub import LCM, Topic

        lcm_instance = LCM()
        lcm_instance.start()

        topic = Topic("/video", Image)

        def subscribe(observer, scheduler=None):
            unsubscribe_fn = lcm_instance.subscribe(topic, lambda msg, _: observer.on_next(msg))

            return Disposable(unsubscribe_fn)

        return create(subscribe).pipe(
            ops.map(
                lambda img: img.data if hasattr(img, "data") else img
            ),  # Convert Image message to numpy array
            ops.sample(1.0 / fps),
        )


async def run_light_robot():
    """Run the lightweight robot without GPU modules."""
    ip = os.getenv("ROBOT_IP")
    pubsub.lcm.autoconf()

    robot = UnitreeGo2Light(ip)

    await robot.start()

    pose = robot.get_pose()
    print(f"Robot position: {pose['position']}")
    print(f"Robot rotation: {pose['rotation']}")
    robot.explore()
    # Keep the program running
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    import os

    print("Running UnitreeGo2Light...")
    asyncio.run(run_light_robot())

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
import threading
import time
from typing import Callable

from reactivex import operators as ops

import dimos.core.colors as colors
from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.robot.global_planner import AstarPlanner
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.robot.unitree_webrtc.connection import VideoMessage, UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.costmap import Costmap
from dimos.types.vector import Vector
from dimos.utils.data import get_data
from dimos.utils.reactive import getter_streaming
from dimos.utils.testing import TimedSensorReplay
from dimos.robot.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
import os
import logging
import warnings
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.multiprocess.unitree_go2", level=logging.INFO)

# Configure logging levels
os.environ["DIMOS_LOG_LEVEL"] = "WARNING"

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
    def video_stream(self, freq_hz=0.5):
        print("video stream start")
        video_store = TimedSensorReplay("unitree_office_walk/video", autocast=Image.from_numpy)
        return video_store.stream().pipe(ops.sample(freq_hz))

    def move(self, vector: Vector):
        print("move supressed", vector)


class ConnectionModule(UnitreeWebRTCConnection, Module):
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
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        # Initialize the parent WebRTC connection
        super().__init__(self.ip)

        # Connect sensor streams to LCM outputs
        self.lidar_stream().subscribe(self.lidar.publish)
        self.odom_stream().subscribe(self.odom.publish)
        self.video_stream().subscribe(self.video.publish)

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
    plancmd: Out[Vector3] = None

    @rpc
    def start(self):
        def plancmd():
            time.sleep(4)
            print(colors.red("requesting global plan"))
            self.plancmd.publish(Vector3(0, 0, 0))

        thread = threading.Thread(target=plancmd, daemon=True)
        thread.start()


async def run(ip):
    dimos = core.start(4)

    # Connection Module - Robot sensor data interface via WebRTC ===================
    connection = dimos.deploy(ConnectionModule, ip)

    # This enables LCM transport
    # Ensures system multicast, udp sizes are auto-adjusted if needed
    pubsub.lcm.autoconf()

    # Configure ConnectionModule LCM transport outputs for sensor data streams
    # OUTPUT: LiDAR point cloud data to /lidar topic
    connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
    # OUTPUT: Robot odometry/pose data to /odom topic
    connection.odom.transport = core.LCMTransport("/odom", PoseStamped)
    # OUTPUT: Camera video frames to /video topic
    connection.video.transport = core.LCMTransport("/video", Image)
    # ======================================================================

    # Map Module - Point cloud accumulation and costmap generation =========
    mapper = dimos.deploy(Map, voxel_size=0.5, global_publish_interval=2.5)

    # OUTPUT: Accumulated point cloud map to /global_map topic
    mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)

    # Connect ConnectionModule OUTPUT lidar to Map INPUT lidar for point cloud accumulation
    mapper.lidar.connect(connection.lidar)
    # ====================================================================

    # Local planner Module, LCM transport & connection ================
    local_planner = dimos.deploy(
        VFHPurePursuitPlanner,
        get_costmap=connection.get_local_costmap,
    )

    # Connects odometry LCM stream to BaseLocalPlanner odometry input
    local_planner.odom.connect(connection.odom)

    # Configures BaseLocalPlanner movecmd output to /move LCM topic
    local_planner.movecmd.transport = core.LCMTransport("/move", Vector3)

    # Connects connection.movecmd input to local_planner.movecmd output
    connection.movecmd.connect(local_planner.movecmd)
    # ===================================================================

    # Global Planner Module ===============
    global_planner = dimos.deploy(
        AstarPlanner,
        get_costmap=mapper.costmap,
        get_robot_pos=connection.get_pos,
        set_local_nav=local_planner.navigate_path_local,
    )

    # Configure AstarPlanner OUTPUT path: Out[Path] to /global_path LCM topic
    global_planner.path.transport = core.pLCMTransport("/global_path")
    # ======================================

    # Global Planner Control Module ===========================
    # Debug module that sends (0,0,0) goal after 4 second delay
    ctrl = dimos.deploy(ControlModule)

    # Configure ControlModule OUTPUT to publish goal coordinates to /global_target
    ctrl.plancmd.transport = core.LCMTransport("/global_target", Vector3)

    # Connect ControlModule OUTPUT to AstarPlanner INPUT - triggers A* planning when goal received
    global_planner.target.connect(ctrl.plancmd)
    # ==========================================

    # Visualization ============================
    foxglove_bridge = FoxgloveBridge()
    # ==========================================

    frontier_explorer = WavefrontFrontierExplorer(
        set_goal=global_planner.set_goal,
        get_costmap=mapper.costmap,
        get_robot_pos=connection.get_pos,
    )

    # Prints full module IO
    print("\n")
    for module in [connection, mapper, local_planner, global_planner, ctrl]:
        print(module.io().result(), "\n")

    # Start modules =============================
    mapper.start()
    connection.start()
    local_planner.start()
    global_planner.start()
    foxglove_bridge.start()
    # ctrl.start() # DEBUG

    await asyncio.sleep(2)
    frontier_explorer.explore()
    print("querying system")
    print(mapper.costmap())
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    import os

    asyncio.run(run(os.getenv("ROBOT_IP")))
    # asyncio.run(run("192.168.9.140"))

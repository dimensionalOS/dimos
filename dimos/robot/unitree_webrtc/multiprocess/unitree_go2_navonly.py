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

import dimos.core.colors as colors
from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Transform, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid, Path
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
        video_store = TimedSensorReplay("unitree_office_walk/video", autocast=Image.from_numpy)
        return video_store.stream()

    def move(self, vector: Vector):
        ...
        # print("move supressed", vector)


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
        # Connect sensor streams to LCM outputs
        self.lidar_stream().subscribe(self.lidar.publish)
        self.odom_stream().subscribe(self.odom.publish)
        # self.video_stream().subscribe(self.video.publish)
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
            while True:
                time.sleep(0.5)
                print(colors.red("requesting global plan"))
                self.plancmd.publish(
                    PoseStamped(
                        ts=time.time(),
                        position=(0, 0, 0),
                        orientation=(0, 0, 0, 1),
                    )
                )

        thread = threading.Thread(target=plancmd, daemon=True)
        thread.start()


class UnitreeGo2Light:
    ip: str

    def __init__(self, ip: str):
        self.ip = ip

    def start(self):
        dimos = core.start(4)

        connection = dimos.deploy(ConnectionModule, self.ip)
        connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
        connection.odom.transport = core.LCMTransport("/odom", PoseStamped)
        connection.video.transport = core.LCMTransport("/video", Image)
        connection.movecmd.transport = core.LCMTransport("/mov", Vector3)

        mapper = dimos.deploy(Map, voxel_size=0.5, global_publish_interval=2.5)

        mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
        mapper.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)

        mapper.lidar.connect(connection.lidar)

        global_planner = dimos.deploy(
            AstarPlanner,
            get_costmap=mapper.costmap,
            get_robot_pos=connection.get_pos,
            set_local_nav=print,
        )

        ctrl = dimos.deploy(ControlModule)

        ctrl.plancmd.transport = core.LCMTransport("/global_target", PoseStamped)
        global_planner.path.transport = core.LCMTransport("/global_path", Path)
        global_planner.target.connect(ctrl.plancmd)
        foxglove_bridge = FoxgloveBridge()

        connection.start()
        mapper.start()
        global_planner.start()
        foxglove_bridge.start()
        ctrl.start()


if __name__ == "__main__":
    import os

    ip = os.getenv("ROBOT_IP")
    pubsub.lcm.autoconf()
    robot = UnitreeGo2Light(ip)
    robot.start()

    while True:
        time.sleep(1)

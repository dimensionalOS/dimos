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
from dimos.protocol import pubsub
from dimos.protocol.tf import TF
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
from dimos.navigation.global_planner import AstarPlanner
from dimos.navigation.local_planner.holonomic_local_planner import HolonomicLocalPlanner
from dimos.navigation.bt_navigator.navigator import BehaviorTreeNavigator
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection, VideoMessage
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
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

    def move(self, vector: Vector3):
        pass
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
        self.video_stream().subscribe(self.video.publish)
        self.tf_stream().subscribe(self.tf.publish)

        # Connect LCM input to robot movement commands
        self.movecmd.subscribe(self.move)

        # Set up streaming getters for latest sensor data
        self._odom = getter_streaming(self.odom_stream())
        self._lidar = getter_streaming(self.lidar_stream())


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
        connection.movecmd.transport = core.LCMTransport("/cmd_vel", Vector3)

        mapper = dimos.deploy(Map, voxel_size=0.5, global_publish_interval=2.5)

        mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
        mapper.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)
        mapper.local_costmap.transport = core.LCMTransport("/local_costmap", OccupancyGrid)

        mapper.lidar.connect(connection.lidar)

        # Deploy global planner with new module-based approach
        global_planner = dimos.deploy(AstarPlanner)

        # Deploy local planner
        local_planner = dimos.deploy(HolonomicLocalPlanner)

        # Deploy behavior tree navigator
        navigator = dimos.deploy(BehaviorTreeNavigator)

        # Deploy frontier exploration module
        frontier_explorer = dimos.deploy(WavefrontFrontierExplorer)

        # Set up transports
        navigator.goal.transport = core.LCMTransport("/navigation_goal", PoseStamped)
        navigator.goal_request.transport = core.LCMTransport("/goal_request", PoseStamped)
        global_planner.path.transport = core.LCMTransport("/global_path", Path)
        local_planner.cmd_vel.transport = core.LCMTransport("/cmd_vel", Vector3)
        frontier_explorer.goal_request.transport = core.LCMTransport("/goal_request", PoseStamped)

        # Connect navigator to global planner
        global_planner.target.connect(navigator.goal)

        # Connect global planner inputs
        global_planner.global_costmap.connect(mapper.global_costmap)
        global_planner.odom.connect(connection.odom)

        # Connect local planner inputs
        local_planner.path.connect(global_planner.path)
        local_planner.local_costmap.connect(mapper.local_costmap)
        local_planner.odom.connect(connection.odom)

        # Connect local planner output to robot movement
        connection.movecmd.connect(local_planner.cmd_vel)

        # Connect navigator to odometry
        navigator.odom.connect(connection.odom)

        # Connect navigator to local planner for goal monitoring
        navigator.connect_local_planner(local_planner)

        # Deploy WebSocket visualization module
        websocket_vis = dimos.deploy(WebsocketVisModule, port=7779)

        # Set up transport for click output
        websocket_vis.click_goal.transport = core.LCMTransport("/click_goal", PoseStamped)

        # Connect visualization inputs
        websocket_vis.robot_pose.connect(connection.odom)
        websocket_vis.path.connect(global_planner.path)
        websocket_vis.global_costmap.connect(mapper.global_costmap)

        # Connect frontier explorer inputs
        frontier_explorer.costmap.connect(mapper.global_costmap)
        frontier_explorer.odometry.connect(connection.odom)

        # Note: frontier_explorer.goal_request and websocket_vis.click_goal both
        # publish to the same /goal_request topic which navigator subscribes to

        foxglove_bridge = FoxgloveBridge()

        connection.start()
        mapper.start()
        global_planner.start()
        local_planner.start()
        navigator.start()
        frontier_explorer.start()
        websocket_vis.start()
        foxglove_bridge.start()

        logger.info(f"WebSocket visualization available at http://localhost:7779")


if __name__ == "__main__":
    import os

    ip = os.getenv("ROBOT_IP")
    pubsub.lcm.autoconf()
    robot = UnitreeGo2Light(ip)
    robot.start()

    while True:
        time.sleep(1)

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
import time
from typing import Callable

from reactivex import operators as ops

from dimos import core
from dimos.core import In, Module, Out
from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.robot.global_planner import AstarPlanner
from dimos.robot.local_planner.simple import SimplePlanner
from dimos.robot.unitree_webrtc.connection import VideoMessage, WebRTCRobot
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.costmap import Costmap
from dimos.types.vector import Vector
from dimos.utils.reactive import backpressure, getter_streaming
from dimos.utils.testing import TimedSensorReplay


class FakeRTC(WebRTCRobot):
    def connect(self): ...

    @functools.cache
    def lidar_stream(self):
        print("lidar stream start")
        lidar_store = TimedSensorReplay("unitree_office_walk/lidar", autocast=LidarMessage.from_msg)
        return backpressure(lidar_store.stream())

    @functools.cache
    def odom_stream(self):
        print("odom stream start")
        odom_store = TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)
        return backpressure(odom_store.stream())

    @functools.cache
    def video_stream(self):
        print("video stream start")
        video_store = TimedSensorReplay("unitree_office_walk/video", autocast=Image.from_numpy)
        return backpressure(video_store.stream().pipe(ops.sample(0.25)))

    def move(self, vector: Vector):
        print("move supressed", vector)


class ConnectionModule(FakeRTC, Module):
    movecmd: In[Vector] = None
    odom: Out[Vector3] = None
    lidar: Out[LidarMessage] = None
    video: Out[VideoMessage] = None
    ip: str

    _odom: Callable[[], Odometry]
    _lidar: Callable[[], LidarMessage]

    def __init__(self, ip: str):
        Module.__init__(self)
        self.ip = ip

    def start(self):
        # Since TimedSensorReplay is now non-blocking, we can subscribe directly
        self.lidar_stream().subscribe(self.lidar.publish)
        self.odom_stream().subscribe(self.odom.publish)
        self.video_stream().subscribe(self.video.publish)
        self.movecmd.subscribe(print)
        self._odom = getter_streaming(self.odom_stream())
        self._lidar = getter_streaming(self.lidar_stream())

    def get_local_costmap(self) -> Costmap:
        return self._lidar().costmap()

    def get_odom(self) -> Odometry:
        return self._odom()

    def get_pos(self) -> Vector:
        print("GETPOS")
        return self._odom().position

    def move(self, vector: Vector):
        print("move command received:", vector)


class ControlModule(Module):
    plancmd: Out[Vector3] = None

    def start(self):
        time.sleep(5)
        print("requesting global nav")
        self.plancmd.publish(Vector3([0, 0, 0]))


class Unitree:
    def __init__(self, ip: str):
        self.ip = ip

    def start(self):
        dimos = None
        if not dimos:
            dimos = core.start(2)

        connection = dimos.deploy(ConnectionModule, self.ip)

        # ensures system multicast, udp sizes are auto-adjusted if needed
        pubsub.lcm.autoconf()

        connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
        connection.odom.transport = core.LCMTransport("/odom", Odometry)
        connection.video.transport = core.LCMTransport("/video", Image)

        map = dimos.deploy(Map, voxel_size=0.5)
        map.lidar.connect(connection.lidar)

        local_planner = dimos.deploy(
            SimplePlanner,
            get_costmap=lambda: connection.get_local_costmap().result(),
            get_robot_pos=lambda: connection.get_pos().result(),
        )

        global_planner = dimos.deploy(
            AstarPlanner,
            get_costmap=lambda: map.costmap().result(),
            get_robot_pos=lambda: connection.get_pos().result(),
        )

        local_planner.path.connect(global_planner.path)
        local_planner.movecmd.connect(connection.movecmd)

        ctrl = dimos.deploy(ControlModule)
        ctrl.plancmd.transport = core.LCMTransport("/global_target", Vector3)
        ctrl.plancmd.connect(global_planner.target)

        # we review the structure
        print("\n")
        for module in [connection, map, global_planner, local_planner, ctrl]:
            print(module.io().result(), "\n")

        # start systems
        map.start().result()
        connection.start().result()
        local_planner.start().result()
        global_planner.start().result()
        ctrl.start()
        print("running")
        time.sleep(2)

        print(map.costmap().result())


if __name__ == "__main__":
    unitree = Unitree("Bla")
    unitree.start()
    time.sleep(30)

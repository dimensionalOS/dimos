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
import contextvars
import functools
import time
from typing import Callable

from dask.distributed import get_client, get_worker
from distributed import get_worker
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler

import dimos.core.colors as colors
from dimos import core
from dimos.core import In, Module, Out, rpc
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
from dimos.utils.data import get_data
from dimos.utils.reactive import backpressure, getter_streaming
from dimos.utils.testing import TimedSensorReplay


# can be swapped in for WebRTCRobot
class FakeRTC(WebRTCRobot):
    def connect(self): ...

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
        return video_store.stream().pipe(ops.sample(0.5))

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

    def __init__(self, ip: str, *args, **kwargs):
        Module.__init__(self, *args, **kwargs)
        self.ip = ip

    @rpc
    def start(self):
        # ensure that LFS data is available
        data = get_data("unitree_office_walk")
        # Since TimedSensorReplay is now non-blocking, we can subscribe directly
        self.lidar_stream().subscribe(self.lidar.publish)
        self.odom_stream().subscribe(self.odom.publish)
        self.video_stream().subscribe(self.video.publish)

        print("movecmd sub")
        self.movecmd.subscribe(print)
        print("sub ok")
        self._odom = getter_streaming(self.odom_stream())
        self._lidar = getter_streaming(self.lidar_stream())

        print("ConnectionModule started")

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

    async def start(self):
        async def plancmd():
            await asyncio.sleep(4)
            print(colors.red("requesting global plan"))
            self.plancmd.publish(Vector3([0, 0, 0]))

        asyncio.create_task(plancmd())


class Unitree:
    def __init__(self, ip: str):
        self.ip = ip

    async def start(self):
        dimos = None
        if not dimos:
            dimos = core.start(4)

        connection = dimos.deploy(ConnectionModule, self.ip)

        # This enables LCM transport
        # Ensures system multicast, udp sizes are auto-adjusted if needed
        pubsub.lcm.autoconf()

        connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
        connection.odom.transport = core.LCMTransport("/odom", Odometry)
        connection.video.transport = core.LCMTransport("/video", Image)
        connection.movecmd.transport = core.LCMTransport("/move", Vector3)

        mapper = dimos.deploy(Map, voxel_size=0.5)

        local_planner = dimos.deploy(
            SimplePlanner,
            get_costmap=connection.get_local_costmap,
            get_robot_pos=connection.get_pos,
        )

        global_planner = dimos.deploy(
            AstarPlanner,
            get_costmap=mapper.costmap,
            get_robot_pos=connection.get_pos,
        )

        global_planner.path.transport = core.pLCMTransport("/global_path")

        local_planner.path.connect(global_planner.path)
        local_planner.movecmd.connect(connection.movecmd)

        ctrl = dimos.deploy(ControlModule)

        mapper.lidar.connect(connection.lidar)

        ctrl.plancmd.transport = core.LCMTransport("/global_target", Vector3)
        global_planner.target.connect(ctrl.plancmd)

        # we review the structure
        print("\n")
        for module in [connection, mapper, local_planner, global_planner, ctrl]:
            print(module.io().result(), "\n")

        print(colors.green("starting mapper"))
        mapper.start()

        print(colors.green("starting connection"))
        connection.start()

        print(colors.green("local planner start"))
        local_planner.start()

        print(colors.green("starting global planner"))
        global_planner.start()

        print(colors.green("starting ctrl"))
        ctrl.start()

        print(colors.red("READY"))

        await asyncio.sleep(3)

        print("querying system")
        print(mapper.costmap())
        # global_planner.dask_receive_msg("target", Vector3([0, 0, 0])).result()
        time.sleep(20)


if __name__ == "__main__":
    unitree = Unitree("Bla")
    asyncio.run(unitree.start())
    time.sleep(30)

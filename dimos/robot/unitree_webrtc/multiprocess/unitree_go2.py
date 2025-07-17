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
import threading
import time
from typing import Callable

from dask.distributed import get_client, get_worker
from distributed import get_worker
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler

import dimos.core.colors as colors
from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.robot.foxglove_bridge import FoxgloveBridge
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


class RealRTC(WebRTCRobot): ...


# inherit RealRTC instead of FakeRTC to run the real robot
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
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        super().__init__(self.ip)
        # ensure that LFS data is available
        self.lidar_stream().subscribe(self.lidar.publish)
        self.odom_stream().subscribe(self.odom.publish)
        self.video_stream().subscribe(self.video.publish)
        self.movecmd.subscribe(self.move)
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
    connection = dimos.deploy(ConnectionModule, ip)

    # This enables LCM transport
    # Ensures system multicast, udp sizes are auto-adjusted if needed
    # TODO: this doesn't seem to work atm and LCMTransport instantiation can fail
    pubsub.lcm.autoconf()

    connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
    connection.odom.transport = core.LCMTransport("/odom", PoseStamped)
    connection.video.transport = core.LCMTransport("/video", Image)

    mapper = dimos.deploy(Map, voxel_size=0.5, global_publish_interval=2.5)

    mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)

    local_planner = dimos.deploy(
        SimplePlanner,
        get_costmap=connection.get_local_costmap,
    )

    global_planner = dimos.deploy(
        AstarPlanner,
        get_costmap=mapper.costmap,
        get_robot_pos=connection.get_pos,
    )

    global_planner.path.transport = core.pLCMTransport("/global_path")

    local_planner.path.connect(global_planner.path)

    local_planner.odom.connect(connection.odom)

    local_planner.movecmd.transport = core.LCMTransport("/move", Vector3)
    local_planner.movecmd.connect(connection.movecmd)

    ctrl = dimos.deploy(ControlModule)

    mapper.lidar.connect(connection.lidar)

    ctrl.plancmd.transport = core.LCMTransport("/global_target", Vector3)

    global_planner.target.connect(ctrl.plancmd)

    foxglove_bridge = FoxgloveBridge()

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

    print(colors.green("starting foxglove bridge"))
    foxglove_bridge.start()

    # uncomment to move the bot
    print(colors.green("starting ctrl"))
    ctrl.start()

    print(colors.red("READY"))

    await asyncio.sleep(10)
    print("querying system")
    print(mapper.costmap())
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    import os

    asyncio.run(run(os.getenv("ROBOT_IP")))
    # asyncio.run(run("192.168.9.140"))

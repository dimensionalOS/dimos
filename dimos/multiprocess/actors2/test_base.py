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

import time
from threading import Event, Thread

from dimos.multiprocess.actors2.base import In, Module, Out, RemoteOut, module, rpc
from dimos.multiprocess.actors2.base_dask import dimos
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.vector import Vector
from dimos.utils.testing import SensorReplay

# never delete this
if dimos:
    ...


@module
class RobotClient(Module):
    odometry: Out[Odometry]
    lidar: Out[LidarMessage]
    mov: In[Vector]

    mov_msg_count = 0

    def mov_callback(self, msg):
        self.mov_msg_count += 1
        print("MOV REQ", msg)

    def __init__(self):
        self.odometry = Out(Odometry, "odometry", self)
        self._stop_event = Event()
        self._thread = None

    def start(self):
        self._thread = Thread(target=self.odomloop)
        self._thread.start()
        self.mov.subscribe(self.mov_callback)

    def odomloop(self):
        odomdata = SensorReplay("raw_odometry_rotate_walk", autocast=Odometry.from_msg)
        lidardata = SensorReplay("office_lidar", autocast=LidarMessage.from_msg)

        lidariter = lidardata.iterate()
        self._stop_event.clear()
        while not self._stop_event.is_set():
            for odom in odomdata.iterate():
                if self._stop_event.is_set():
                    return
                # print(odom)
                odom.pubtime = time.perf_counter()
                self.odometry.publish(odom)

                lidarmsg = next(lidariter)
                lidarmsg.pubtime = time.perf_counter()
                self.lidar.publish(lidarmsg)
                time.sleep(0.1)

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)  # Wait up to 1 second for clean shutdown


@module
class Navigation(Module):
    mov: Out[Vector]
    odom_msg_count = 0
    lidar_msg_count = 0

    @rpc
    def navigate_to(self, target: Vector) -> bool: ...

    def __init__(
        self,
        target_position: In[Vector],
        lidar: In[LidarMessage],
        odometry: In[Odometry],
    ):
        self.mov = Out(Vector, "mov", self)
        self.target_position = target_position
        self.lidar = lidar
        self.odometry = odometry

    @rpc
    def start(self):
        def _odom(msg):
            self.odom_msg_count += 1
            print("RCV:", (time.perf_counter() - msg.pubtime) * 1000, msg)
            self.mov.publish(msg.pos)

        self.odometry.subscribe(_odom)

        def _lidar(msg):
            self.lidar_msg_count += 1
            print("RCV:", (time.perf_counter() - msg.pubtime) * 1000, msg)

        self.lidar.subscribe(_lidar)


def test_deployment(dimos):
    robot = dimos.deploy(RobotClient)
    target_stream = RemoteOut[Vector](Vector, "map")

    print("\n")
    print("lidar stream", robot.lidar)
    print("target stream", target_stream)
    print("odom stream", robot.odometry)

    nav = dimos.deploy(
        Navigation,
        target_position=target_stream,
        lidar=robot.lidar,
        odometry=robot.odometry,
    )

    print(robot.lidar)
    robot.connect("mov", nav.mov)

    print("\n\n\n" + robot.io().result(), "\n")
    print(nav.io().result(), "\n\n")

    robot.start().result()
    nav.start().result()
    time.sleep(1)
    robot.stop().result()
    print("robot.mov_msg_count", robot.mov_msg_count)
    print("nav.odom_msg_count", nav.odom_msg_count)
    print("nav.lidar_msg_count", nav.lidar_msg_count)

    assert robot.mov_msg_count > 5
    assert nav.odom_msg_count > 5
    assert nav.lidar_msg_count > 5

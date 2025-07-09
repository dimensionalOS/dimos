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

import pytest

from dimos.core import (
    In,
    LCMTransport,
    Module,
    Out,
    RemoteOut,
    ZenohTransport,
    dimos,
    pLCMTransport,
    rpc,
    start,
)
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.vector import Vector
from dimos.utils.testing import SensorReplay

# never delete this line
if dimos:
    ...


class RobotClient(Module):
    odometry: Out[Odometry] = None
    lidar: Out[LidarMessage] = None
    mov: In[Vector] = None

    mov_msg_count = 0

    def mov_callback(self, msg):
        self.mov_msg_count += 1

    def __init__(self):
        super().__init__()
        print(self)
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
                print(odom)
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


class Navigation(Module):
    mov: Out[Vector] = None
    lidar: In[LidarMessage] = None
    target_position: In[Vector] = None
    odometry: In[Odometry] = None

    odom_msg_count = 0
    lidar_msg_count = 0

    @rpc
    def navigate_to(self, target: Vector) -> bool: ...

    def __init__(self):
        super().__init__()

    @rpc
    def start(self):
        def _odom(msg):
            self.odom_msg_count += 1
            print("RCV:", (time.perf_counter() - msg.pubtime) * 1000, msg)
            self.mov.publish(msg.pos)

        self.odometry.subscribe(_odom)

        def _lidar(msg):
            self.lidar_msg_count += 1
            if hasattr(msg, "pubtime"):
                print("RCV:", (time.perf_counter() - msg.pubtime) * 1000, msg)
            else:
                print("RCV: unknown time", msg)

        self.lidar.subscribe(_lidar)


@pytest.mark.tool
def test_deployment(dimos):
    robot = dimos.deploy(RobotClient)
    target_stream = RemoteOut[Vector](Vector, "target")

    print("\n")
    print("lidar stream", robot.lidar)
    print("target stream", target_stream)
    print("odom stream", robot.odometry)

    nav = dimos.deploy(Navigation)

    # this one encodes proper LCM messages
    robot.lidar.transport = LCMTransport("/lidar", LidarMessage)
    # odometry & mov using just a pickle over LCM
    robot.odometry.transport = pLCMTransport("/odom")
    nav.mov.transport = pLCMTransport("/mov")

    nav.lidar.connect(robot.lidar)
    nav.odometry.connect(robot.odometry)
    robot.mov.connect(nav.mov)

    print("\n" + robot.io().result() + "\n")
    print("\n" + nav.io().result() + "\n")
    robot.start().result()
    nav.start().result()

    time.sleep(1)
    robot.stop().result()

    print("robot.mov_msg_count", robot.mov_msg_count)
    print("nav.odom_msg_count", nav.odom_msg_count)
    print("nav.lidar_msg_count", nav.lidar_msg_count)

    assert robot.mov_msg_count >= 8
    assert nav.odom_msg_count >= 8
    assert nav.lidar_msg_count >= 8


if __name__ == "__main__":
    client = start(1)  # single process for CI memory
    test_deployment(client)

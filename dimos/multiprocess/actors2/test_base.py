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
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.path import Path
from dimos.types.vector import Vector
from dimos.utils.testing import SensorReplay


@module
class RobotClient(Module):
    odometry: Out[Odometry]

    def __init__(self):
        self.odometry = Out(Odometry, "odometry", self)
        self._stop_event = Event()
        self._thread = None

    @rpc
    def start(self):
        self._thread = Thread(target=self.odomloop)
        self._thread.start()

    def odomloop(self):
        odomdata = SensorReplay("raw_odometry_rotate_walk", autocast=Odometry.from_msg)
        self._stop_event.clear()
        while not self._stop_event.is_set():
            for odom in odomdata.iterate():
                if self._stop_event.is_set():
                    return
                odom.pubtime = time.perf_counter()
                self.odometry.publish(odom)
                time.sleep(0.1)

    @rpc
    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)


@module
class Navigation(Module):
    target_path: Out[Path]

    @rpc
    def navigate_to(self, target: Vector) -> bool: ...

    def __init__(
        self,
        target_position: In[Vector],
        map_stream: In[Map],
        odometry: In[Odometry],
    ):
        self.target_position = target_position
        self.map_stream = map_stream
        self.odometry = odometry
        self.target_path = Out(Path, "target_path")

    @rpc
    def start(self):
        print("navigation odom stream is, subscribing", self.odometry)
        self.odometry.subscribe(print)


def test_introspection():
    """Test introspection of the Navigation module."""
    assert hasattr(Navigation, "inputs")
    assert hasattr(Navigation, "rpcs")
    print("\n\n\n" + Navigation.io(), "\n\n")


def test_stream_introspection():
    nav = Navigation(1, 2, 3)

    print(nav.target_path)


def test_instance_introspection():
    robot = RobotClient()
    print(robot)

    map_stream = Out[Map](Map, "map")
    target_stream = Out[Vector](Vector, "map")
    print("\n")
    print("map stream", map_stream)
    print("target stream", target_stream)
    print("odom stream", robot.odometry)

    nav = Navigation(target_position=target_stream, map_stream=map_stream, odometry=robot.odometry)
    """Test introspection of the Navigation module."""
    assert hasattr(nav, "inputs")
    assert hasattr(nav, "rpcs")
    print("\n\n\n" + nav.io(), "\n\n")


#    nav.start()


def test_deployment(dimos):
    robot = dimos.deploy(RobotClient)

    map_stream = RemoteOut[Map](Map, "map")
    target_stream = RemoteOut[Vector](Vector, "map")
    odom_stream = robot.odometry
    print("\n")
    print("map stream", map_stream)
    print("target stream", target_stream)
    print("odom stream", odom_stream)

    # # print(type(odom_stream.owner))
    # # print(type(robot))

    # # out = Out(Odometry, "odometry", robot)

    nav = dimos.deploy(
        Navigation, target_position=target_stream, map_stream=map_stream, odometry=odom_stream
    )

    # # """Test introspection of the Navigation module."""
    # # assert hasattr(nav, "inputs")
    # # assert hasattr(nav, "rpcs")
    print("\n\n\n" + robot.io().result(), "\n")
    print(nav.io().result(), "\n\n")

    robot.start().result()
    # time.sleep(2)

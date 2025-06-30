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

import time
from threading import Event, Thread

from dimos.multiprocess.actors2.base import dimos
from dimos.multiprocess.actors2.meta2 import ActorReference, In, Module, Out, module, rpc
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.path import Path
from dimos.types.vector import Vector
from dimos.utils.testing import SensorReplay

if dimos:  # otherwise ruff deletes the import
    ...


# we have these references but we actually want them to be just syntax sugar that facilitates
# actors talking to each other, for example
#
# actor2 asks actor1 to subscribe to it's output1, and return it to actor1 input2 - this is a very flat message.
# let's start with that?
#
# what we are getting in our inputs is actor2 50% of the data, we get actor1 and output1 reference


@module
class RobotClient(Module):
    odometry: Out[Odometry]

    def __init__(self):
        self.odometry = Out(Odometry, "odometry", self)

        self._stop_event = Event()
        self._thread = None

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
                # print(odom)
                odom.pubtime = time.perf_counter()
                self.odometry.publish(odom)
                time.sleep(0.1)

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)  # Wait up to 1 second for clean shutdown


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

    def start(self):
        print("navigation odom stream is, subscribing", self.odometry)
        self.odometry.subscribe(print)


def test_introspect():
    """Test introspection of the Navigation module."""
    assert hasattr(Navigation, "inputs")
    assert hasattr(Navigation, "outputs")
    assert hasattr(Navigation, "rpcs")
    print("\n\n\n" + Navigation.io(), "\n\n")


def test_introspect_instance(dimos):
    robot = dimos.deploy(RobotClient)

    print(robot.odometry)

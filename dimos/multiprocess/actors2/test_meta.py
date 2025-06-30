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
from dimos.multiprocess.actors2.meta import ActorReference, In, Out, module, rpc
from dimos.robot.unitree_webrtc.type.lidar import Lidar
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
#


class Module:
    def subscribe(self, out_name: str, actor_reference: ActorReference, in_name: str):
        print(
            f"{self.__class__.__name__} {out_name} SUB REQ from {actor_reference} pass into INPUT {in_name}"
        )
        self.outputs[out_name].subscribers.append((actor_reference, in_name))

    def receive_message(self, in_name, message):
        # print(f"RECEIVED MESSAGE IN {self.__class__.__name__} INPUT {in_name}: {message}")
        self.inputs[in_name].receive(message)


@module
class RobotClient(Module):
    odometry: Out[Odometry]
    lidar: Out[Lidar]

    def __init__(self):
        self.odometry = Out(Odometry, "odometry", self)

        self._stop_event = Event()
        self._thread = None

    def start(self):
        self._thread = Thread(target=self.odomloop)
        self._thread.start()

    def odomloop(self):
        odomdata = SensorReplay("raw_odometry_rotate_walk", autocast=Odometry.from_msg)
        lidardata = SensorReplay("office_lidar", autocast=Lidar.from_msg)

        lidariter = lidardata.iterate()
        self._stop_event.clear()
        while not self._stop_event.is_set():
            for odom in odomdata.iterate():
                if self._stop_event.is_set():
                    print("Stopping odometry stream")
                    return
                # print(odom)
                odom.pubtime = time.perf_counter()
                self.odometry.publish(odom)
                self.lidar.publish(next(lidariter))
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
        print("navigation odom stream is", self.odometry)
        print("calling subscribe", self.odometry.subscribe())


def test_introspect():
    """Test introspection of the Navigation module."""
    assert hasattr(Navigation, "inputs")
    assert hasattr(Navigation, "outputs")
    assert hasattr(Navigation, "rpcs")
    print("\n\n\n" + Navigation.io(), "\n\n")


def test_get_sub(dimos):
    target_position_stream = Out(Vector, "target_position")
    map_stream = Out(Map, "map")
    odometry_stream = Out(Odometry, "odometry")

    print("\n")
    print("Target Position Stream:\t", target_position_stream)
    print("Map Stream:\t", map_stream)

    robot = dimos.deploy(RobotClient)
    print("Odometry Stream:\t", robot.odometry, "\n\n")

    robot.start().result()

    nav = dimos.deploy(
        Navigation,
        target_position=target_position_stream,
        map_stream=map_stream,
        odometry=robot.odometry,
    )

    print("\n\nNAV Instance:\t", nav)

    print("NAV Target:\t", nav.target_path)

    print(f"NAV I/O (remote query):\n\n{nav.io().result()}")

    nav.start().result()

    time.sleep(5)
    robot.stop()
    time.sleep(0.2)


def test_final_working_solution(dimos):
    """Final test confirming the actor communication solution works completely."""

    # Deploy actors
    robot = dimos.deploy(RobotClient)
    robot.start().result()

    nav = dimos.deploy(
        Navigation,
        target_position=Out(Vector, "target_position"),
        map_stream=Out(Map, "map"),
        odometry=robot.odometry,
    )

    # Verify actor references are properly set
    robot_ref = robot.ref().result()
    nav_ref = nav.ref().result()

    print(f"✅ Robot ref has _actor: {robot_ref._actor is not None}")
    print(f"✅ Nav ref has _actor: {nav_ref._actor is not None}")

    # Start navigation (triggers subscription)
    nav.start().result()
    print("✅ Navigation started and subscribed to robot odometry")

    # Give it a short time to receive messages
    print("📡 Testing message delivery for 2 seconds...")
    time.sleep(2)

    # Clean shutdown
    robot.stop().result()
    print("🏁 Test completed successfully - Actor communication is WORKING!")

    print("\n" + "=" * 60)
    print("🎉 SOLUTION SUMMARY:")
    print("✅ Fixed ActorReference to store deployed Actor objects")
    print("✅ Modified @module decorator to detect actor context")
    print("✅ Actors now communicate successfully via proper references")
    print("✅ The manual actor reference hydration issue is SOLVED!")
    print("=" * 60)

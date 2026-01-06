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

import pytest
from reactivex.disposable import Disposable

from dimos.core import (
    In,
    LCMTransport,
    Module,
    Out,
    pLCMTransport,
    rpc,
    start,
)
from dimos.core.testing import MockRobotClient, dimos
from dimos.msgs.geometry_msgs import Vector3
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry

assert dimos


class Navigation(Module):
    mov: Out[Vector3]
    lidar: In[LidarMessage]
    target_position: In[Vector3]
    odometry: In[Odometry]

    odom_msg_count = 0
    lidar_msg_count = 0

    @rpc
    def navigate_to(self, target: Vector3) -> bool: ...

    def __init__(self) -> None:
        super().__init__()

    @rpc
    def start(self) -> None:
        def _odom(msg) -> None:
            self.odom_msg_count += 1
            print("RCV:", (time.perf_counter() - msg.pubtime) * 1000, msg)
            self.mov.publish(msg.position)

        unsub = self.odometry.subscribe(_odom)
        self._disposables.add(Disposable(unsub))

        def _lidar(msg) -> None:
            self.lidar_msg_count += 1
            if hasattr(msg, "pubtime"):
                print("RCV:", (time.perf_counter() - msg.pubtime) * 1000, msg)
            else:
                print("RCV: unknown time", msg)

        unsub = self.lidar.subscribe(_lidar)
        self._disposables.add(Disposable(unsub))


def test_classmethods() -> None:
    # Test class property access
    class_rpcs = Navigation.rpcs
    print("Class rpcs:", class_rpcs)
    # Test instance property access
    nav = Navigation()
    instance_rpcs = nav.rpcs
    print("Instance rpcs:", instance_rpcs)

    # Assertions
    assert isinstance(class_rpcs, dict), "Class rpcs should be a dictionary"
    assert isinstance(instance_rpcs, dict), "Instance rpcs should be a dictionary"
    assert class_rpcs == instance_rpcs, "Class and instance rpcs should be identical"

    # Check that we have the expected RPC methods
    assert "navigate_to" in class_rpcs, "navigate_to should be in rpcs"
    assert "start" in class_rpcs, "start should be in rpcs"
    assert len(class_rpcs) == 9

    # Check that the values are callable
    assert callable(class_rpcs["navigate_to"]), "navigate_to should be callable"
    assert callable(class_rpcs["start"]), "start should be callable"

    # Check that they have the __rpc__ attribute
    assert hasattr(class_rpcs["navigate_to"], "__rpc__"), (
        "navigate_to should have __rpc__ attribute"
    )
    assert hasattr(class_rpcs["start"], "__rpc__"), "start should have __rpc__ attribute"

    nav._close_module()


@pytest.mark.module
def test_basic_deployment(dimos) -> None:
    robot = dimos.deploy(MockRobotClient)

    print("\n")
    print("lidar stream", robot.lidar)
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

    robot.start()
    nav.start()

    time.sleep(1)
    robot.stop()

    print("robot.mov_msg_count", robot.mov_msg_count)
    print("nav.odom_msg_count", nav.odom_msg_count)
    print("nav.lidar_msg_count", nav.lidar_msg_count)

    assert robot.mov_msg_count >= 8
    assert nav.odom_msg_count >= 8
    assert nav.lidar_msg_count >= 8

    dimos.shutdown()


if __name__ == "__main__":
    client = start(1)  # single process for CI memory
    test_deployment(client)

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
    rpc,
    start,
)
from dimos.msgs.geometry_msgs import Vector3
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.testing import SensorReplay


@pytest.fixture
def dimos():
    """Fixture to create a Dimos client for testing."""
    client = start(2)
    yield client
    client.stop()


class MockRobotClient(Module):
    odometry: Out[Odometry] = None
    lidar: Out[LidarMessage] = None
    mov: In[Vector3] = None

    mov_msg_count = 0

    def mov_callback(self, msg):
        self.mov_msg_count += 1

    def __init__(self):
        super().__init__()
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

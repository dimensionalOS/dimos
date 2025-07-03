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

import itertools
import time

import pytest

from dimos.msgs.sensor_msgs import PointCloud2
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.testing import SensorReplay


def test_init():
    lidar = SensorReplay("office_lidar")

    for raw_frame in itertools.islice(lidar.iterate(), 5):
        assert isinstance(raw_frame, dict)
        frame = LidarMessage.from_msg(raw_frame)
        assert isinstance(frame, LidarMessage)
        data = frame.to_pointcloud2().lcm_encode()
        assert len(data) > 0
        assert isinstance(data, bytes)


@pytest.mark.tool
def test_publish():
    lcm = LCM()
    lcm.start()

    topic = Topic(topic="/lidar", lcm_type=PointCloud2)
    lidar = SensorReplay("office_lidar", autocast=LidarMessage.from_msg)

    while True:
        for frame in lidar.iterate():
            print(frame)
            lcm.publish(topic, frame.to_pointcloud2())
            time.sleep(0.1)

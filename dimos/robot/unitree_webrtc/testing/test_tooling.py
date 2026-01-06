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

from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.reactive import backpressure
from dimos.utils.testing import TimedSensorReplay


@pytest.mark.tool
def test_replay_all() -> None:
    lidar_store = TimedSensorReplay("unitree/lidar", autocast=LidarMessage.from_msg)
    odom_store = TimedSensorReplay("unitree/odom", autocast=Odometry.from_msg)
    video_store = TimedSensorReplay("unitree/video")

    backpressure(odom_store.stream()).subscribe(print)
    backpressure(lidar_store.stream()).subscribe(print)
    backpressure(video_store.stream()).subscribe(print)

    print("Replaying for 3 seconds...")
    time.sleep(3)
    print("Stopping replay after 3 seconds")

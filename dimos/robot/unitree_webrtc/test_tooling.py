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

import os
import sys
import time

import pytest
from dotenv import load_dotenv

from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.reactive import backpressure
from dimos.utils.testing import TimedSensorReplay, TimedSensorStorage


@pytest.mark.tool
def test_record_all():
    from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2

    load_dotenv()
    robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")

    print("Robot is standing up...")

    robot.standup()

    lidar_store = TimedSensorStorage("unitree/lidar")
    odom_store = TimedSensorStorage("unitree/odom")
    video_store = TimedSensorStorage("unitree/video")

    lidar_store.save_stream(robot.raw_lidar_stream()).subscribe(print)
    odom_store.save_stream(robot.raw_odom_stream()).subscribe(print)
    video_store.save_stream(robot.video_stream()).subscribe(print)

    print("Recording, CTRL+C to kill")

    try:
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Robot is lying down...")
        robot.liedown()
        print("Exit")
        sys.exit(0)


@pytest.mark.tool
def test_replay_all():
    lidar_store = TimedSensorReplay("unitree/lidar", autocast=LidarMessage.from_msg)
    odom_store = TimedSensorReplay("unitree/odom", autocast=Odometry.from_msg)
    video_store = TimedSensorReplay("unitree/video")

    backpressure(odom_store.stream()).subscribe(print)
    backpressure(lidar_store.stream()).subscribe(print)
    backpressure(video_store.stream()).subscribe(print)

    print("Replaying for 3 seconds...")
    time.sleep(3)
    print("Stopping replay after 3 seconds")

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
import reactivex.operators as ops

from dimos.robot.unitree_webrtc.testing.multimock import Multimock
from dimos.robot.unitree_webrtc.testing.helpers import show3d_stream
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage


@pytest.mark.tool
def test_record_lidar():
    from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2

    load_dotenv()
    robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")

    print("Robot is standing up...")
    robot.standup()

    lidar_store = Multimock("athens_lidar")
    odom_store = Multimock("athens_odom")
    lidar_store.consume(robot.raw_lidar_stream()).subscribe(print)
    odom_store.consume(robot.raw_odom_stream()).subscribe(print)

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
def test_replay_recording():
    from dimos.robot.unitree_webrtc.type.odometry import position_from_odom

    odom_stream = Multimock("athens_odom").stream().pipe(ops.map(position_from_odom))
    odom_stream.subscribe(lambda x: print(x))

    map = Map()

    def lidarmsg(msg):
        frame = LidarMessage.from_msg(msg)
        map.add_frame(frame)
        return [map, map.costmap.smudge()]

    global_map_stream = Multimock("athens_lidar").stream().pipe(ops.map(lidarmsg))
    show3d_stream(global_map_stream.pipe(ops.map(lambda x: x[0])), clearframe=True).run()


@pytest.mark.tool
def compare_events():
    odom_events = Multimock("athens_odom").list()

    map = Map()

    def lidarmsg(msg):
        frame = LidarMessage.from_msg(msg)
        map.add_frame(frame)
        return [map, map.costmap.smudge()]

    global_map_stream = Multimock("athens_lidar").stream().pipe(ops.map(lidarmsg))
    show3d_stream(global_map_stream.pipe(ops.map(lambda x: x[0])), clearframe=True).run()

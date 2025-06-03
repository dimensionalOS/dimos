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
import cv2
import os
import asyncio
from dotenv import load_dotenv
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2, Color
from dimos.robot.unitree_webrtc.testing.helpers import show3d_stream
from dimos.web.websocket_vis.server import WebsocketVis
from dimos.types.vector import Vector
import logging
import open3d as o3d
import reactivex.operators as ops
import numpy as np
import time
import threading

# logging.basicConfig(level=logging.DEBUG)

load_dotenv()
robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")

websocket_vis = WebsocketVis()
websocket_vis.start()
websocket_vis.connect(robot.global_planner.vis_stream())


def msg_handler(msgtype, data):
    if msgtype == "click":
        try:
            robot.global_planner.set_goal(Vector(data["position"]))
        except Exception as e:
            print(f"Error setting goal: {e}")
            return


def threaded_msg_handler(msgtype, data):
    thread = threading.Thread(target=msg_handler, args=(msgtype, data))
    thread.daemon = True
    thread.start()


websocket_vis.msg_handler = threaded_msg_handler

print("standing up")
robot.standup()
print("robot is up")


def newmap(msg):
    return ["costmap", robot.map.costmap.smudge()]


websocket_vis.connect(robot.map_stream.pipe(ops.map(newmap)))
websocket_vis.connect(robot.odom_stream().pipe(ops.map(lambda pos: ["robot_pos", pos.pos.to_2d()])))

try:
    while True:
        #        robot.move_vel(Vector(0.1, 0.1, 0.1))
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopping robot")
    robot.liedown()

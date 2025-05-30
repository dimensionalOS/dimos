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
import threading
import time

import reactivex.operators as ops
from dotenv import load_dotenv

from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.types.vector import Vector
from dimos.web.websocket_vis.server import WebsocketVis

# logging.basicConfig(level=logging.DEBUG)

load_dotenv()
robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")

websocket_vis = WebsocketVis()
websocket_vis.start()
websocket_vis.connect(robot.global_planner.vis_stream())


def msg_handler(msgtype, data):
    if msgtype == "click":
        try:
            print(data)
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

# Keep the script running and handle graceful shutdown
shutdown_event = threading.Event()


# def signal_handler(signum, frame):
#     print("\nShutdown signal received. Cleaning up...")
#     shutdown_event.set()


# # Register signal handlers for graceful shutdown
# signal.signal(signal.SIGINT, signal_handler)
# signal.signal(signal.SIGTERM, signal_handler)


print("Robot is running. Press Ctrl+C to stop.")

try:
    # Keep the main thread alive while the robot loop runs in the background
    while not shutdown_event.is_set():
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Shutting down...")
finally:
    print("Stopping robot...")
    robot.liedown()
    print("Robot stopped.")

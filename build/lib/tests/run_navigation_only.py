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
from dotenv import load_dotenv
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.robot.unitree_webrtc.testing.helpers import show3d_stream
from dimos.web.websocket_vis.server import WebsocketVis
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.types.vector import Vector
import reactivex.operators as ops
import time
import threading
import asyncio
import atexit
import signal
import sys
import warnings
import logging
# logging.basicConfig(level=logging.DEBUG)

# Filter out known WebRTC warnings that don't affect functionality
warnings.filterwarnings("ignore", message="coroutine.*was never awaited")
warnings.filterwarnings("ignore", message=".*RTCSctpTransport.*")

# Set up logging to reduce asyncio noise
logging.getLogger("asyncio").setLevel(logging.ERROR)

load_dotenv()
robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="normal", enable_perception=False)


# Add graceful shutdown handling to prevent WebRTC task destruction errors
def cleanup_robot():
    print("Cleaning up robot connection...")
    try:
        # Make cleanup non-blocking to avoid hangs
        def quick_cleanup():
            try:
                robot.liedown()
            except:
                pass

        # Run cleanup in a separate thread with timeout
        cleanup_thread = threading.Thread(target=quick_cleanup)
        cleanup_thread.daemon = True
        cleanup_thread.start()
        cleanup_thread.join(timeout=3.0)  # Max 3 seconds for cleanup

        # Force stop the robot's WebRTC connection
        try:
            robot.stop()
        except:
            pass

    except Exception as e:
        print(f"Error during cleanup: {e}")
        # Continue anyway


atexit.register(cleanup_robot)


def signal_handler(signum, frame):
    print("Received shutdown signal, cleaning up...")
    try:
        cleanup_robot()
    except:
        pass
    # Force exit if cleanup hangs
    os._exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

websocket_vis = WebsocketVis()
websocket_vis.start()
websocket_vis.connect(robot.global_planner.vis_stream())


def msg_handler(msgtype, data):
    if msgtype == "click":
        print(f"Received click at position: {data['position']}")

        try:
            print("Setting goal...")

            # Instead of disabling visualization, make it timeout-safe
            original_vis = robot.global_planner.vis

            def safe_vis(name, drawable):
                """Visualization wrapper that won't block on timeouts"""
                try:
                    # Use a separate thread for visualization to avoid blocking
                    def vis_update():
                        try:
                            original_vis(name, drawable)
                        except Exception as e:
                            print(f"Visualization update failed (non-critical): {e}")

                    vis_thread = threading.Thread(target=vis_update)
                    vis_thread.daemon = True
                    vis_thread.start()
                    # Don't wait for completion - let it run asynchronously
                except Exception as e:
                    print(f"Visualization setup failed (non-critical): {e}")

            robot.global_planner.vis = safe_vis
            robot.global_planner.set_goal(Vector(data["position"]))
            robot.global_planner.vis = original_vis

            print("Goal set successfully")
        except Exception as e:
            print(f"Error setting goal: {e}")
            import traceback

            traceback.print_exc()


def threaded_msg_handler(msgtype, data):
    print(f"Processing message: {msgtype}")

    # Create a dedicated event loop for goal setting to avoid conflicts
    def run_with_dedicated_loop():
        try:
            # Use asyncio.run which creates and manages its own event loop
            # This won't conflict with the robot's or websocket's event loops
            async def async_msg_handler():
                msg_handler(msgtype, data)

            asyncio.run(async_msg_handler())
            print("Goal setting completed successfully")
        except Exception as e:
            print(f"Error in goal setting thread: {e}")
            import traceback

            traceback.print_exc()

    thread = threading.Thread(target=run_with_dedicated_loop)
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

local_planner_viz_stream = robot.local_planner_viz_stream.pipe(ops.share())

# Add RobotWebInterface with video stream
streams = {"unitree_video": robot.get_video_stream(), "local_planner_viz": local_planner_viz_stream}
web_interface = RobotWebInterface(port=5555, **streams)
web_interface.run()

try:
    while True:
        #        robot.move_vel(Vector(0.1, 0.1, 0.1))
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopping robot")
    robot.liedown()
except Exception as e:
    print(f"Unexpected error in main loop: {e}")
    import traceback

    traceback.print_exc()
finally:
    print("Cleaning up...")
    cleanup_robot()

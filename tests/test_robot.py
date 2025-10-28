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

from reactivex import operators as RxOps

from dimos.robot.local_planner.local_planner import navigate_to_goal_local
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.web.robot_web_interface import RobotWebInterface


def main():
    print("Initializing Unitree Go2 robot with local planner visualization...")

    # Initialize the robot with webrtc interface
    robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")

    # Get the camera stream
    video_stream = robot.get_video_stream()

    # The local planner visualization stream is created during robot initialization
    local_planner_stream = robot.local_planner_viz_stream

    local_planner_stream = local_planner_stream.pipe(
        RxOps.share(),
        RxOps.map(lambda x: x if x is not None else None),
        RxOps.filter(lambda x: x is not None),
    )

    goal_following_thread = None
    try:
        # Set up web interface with both streams
        streams = {"camera": video_stream, "local_planner": local_planner_stream}

        # Create and start the web interface
        web_interface = RobotWebInterface(port=5555, **streams)

        # Wait for initialization
        print("Waiting for camera and systems to initialize...")
        time.sleep(2)

        # Start the goal following test in a separate thread
        print("Starting navigation to local goal (2m ahead) in a separate thread...")
        goal_following_thread = threading.Thread(
            target=navigate_to_goal_local,
            kwargs={"robot": robot, "goal_xy_robot": (3.0, 0.0), "distance": 0.0, "timeout": 300},
            daemon=True,
        )
        goal_following_thread.start()

        print("Robot streams running")
        print("Web interface available at http://localhost:5555")
        print("Press Ctrl+C to exit")

        # Start web server (blocking call)
        web_interface.run()

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        print("Cleaning up...")
        # Make sure the robot stands down safely
        try:
            robot.liedown()
        except:
            pass
        print("Test completed")


if __name__ == "__main__":
    main()

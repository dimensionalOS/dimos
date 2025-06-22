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

import tests.test_header
import os

import time
from dotenv import load_dotenv
from dimos.agents.cerebras_agent import CerebrasAgent
from dimos.agents.claude_agent import ClaudeAgent
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2

# from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.web.websocket_vis.server import WebsocketVis
from dimos.skills.observe_stream import ObserveStream
from dimos.skills.observe import Observe
from dimos.skills.kill_skill import KillSkill
from dimos.skills.navigation import NavigateWithText, GetPose, NavigateToGoal, Explore
from dimos.skills.visual_navigation_skills import FollowHuman
import reactivex as rx
import reactivex.operators as ops
from dimos.stream.audio.pipelines import tts, stt
import threading
import json
from dimos.types.vector import Vector
from dimos.skills.speak import Speak

from dimos.perception.object_detection_stream import ObjectDetectionStream
from dimos.perception.detection2d.detic_2d_det import Detic2DDetector
from dimos.utils.reactive import backpressure
import asyncio
import atexit
import signal
import sys
import warnings
import logging

# Filter out known WebRTC warnings that don't affect functionality
warnings.filterwarnings("ignore", message="coroutine.*was never awaited")
warnings.filterwarnings("ignore", message=".*RTCSctpTransport.*")

# Set up logging to reduce asyncio noise
logging.getLogger("asyncio").setLevel(logging.ERROR)

# Load API key from environment
load_dotenv()

# Allow command line arguments to control spatial memory parameters
import argparse


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Run the robot with optional spatial memory parameters"
    )
    parser.add_argument(
        "--new-memory", action="store_true", help="Create a new spatial memory from scratch"
    )
    parser.add_argument(
        "--spatial-memory-dir", type=str, help="Directory for storing spatial memory data"
    )
    return parser.parse_args()


args = parse_arguments()

# Initialize robot with spatial memory parameters - using WebRTC mode instead of "ai"
robot = UnitreeGo2(
    ip=os.getenv("ROBOT_IP"),
    mode="normal",
)


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

# Initialize WebSocket visualization
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


def newmap(msg):
    return ["costmap", robot.map.costmap.smudge()]


websocket_vis.connect(robot.map_stream.pipe(ops.map(newmap)))
websocket_vis.connect(robot.odom_stream().pipe(ops.map(lambda pos: ["robot_pos", pos.pos.to_2d()])))

# Create a subject for agent responses
agent_response_subject = rx.subject.Subject()
agent_response_stream = agent_response_subject.pipe(ops.share())
local_planner_viz_stream = robot.local_planner_viz_stream.pipe(ops.share())

# Initialize object detection stream
min_confidence = 0.6
class_filter = None  # No class filtering
min_confidence = 0.99  # temporarily disable detections
# detector = Detic2DDetector(vocabulary=None, threshold=min_confidence)

# Create video stream from robot's camera
video_stream = robot.get_video_stream()  # WebRTC doesn't use ROS video stream

# # Initialize ObjectDetectionStream with robot
# object_detector = ObjectDetectionStream(
#     camera_intrinsics=robot.camera_intrinsics,
#     min_confidence=min_confidence,
#     class_filter=class_filter,
#     get_pose=robot.get_pose,
#     detector=detector,
#     video_stream=video_stream,
# )

# # Create visualization stream for web interface
# viz_stream = backpressure(object_detector.get_stream()).pipe(
#     ops.share(),
#     ops.map(lambda x: x["viz_frame"] if x is not None else None),
#     ops.filter(lambda x: x is not None),
# )

# # Get the formatted detection stream
# formatted_detection_stream = object_detector.get_formatted_stream().pipe(
#     ops.filter(lambda x: x is not None)
# )


# Create a direct mapping that combines detection data with locations
def combine_with_locations(object_detections):
    # Get locations from spatial memory
    try:
        spatial_memory = robot.get_spatial_memory()
        if spatial_memory is None:
            # If spatial memory is disabled, just return the object detections
            return object_detections

        locations = spatial_memory.get_robot_locations()

        # Format the locations section
        locations_text = "\n\nSaved Robot Locations:\n"
        if locations:
            for loc in locations:
                locations_text += f"- {loc.name}: Position ({loc.position[0]:.2f}, {loc.position[1]:.2f}, {loc.position[2]:.2f}), "
                locations_text += f"Rotation ({loc.rotation[0]:.2f}, {loc.rotation[1]:.2f}, {loc.rotation[2]:.2f})\n"
        else:
            locations_text += "None\n"

        # Simply concatenate the strings
        return object_detections + locations_text
    except Exception as e:
        print(f"Error adding locations: {e}")
        return object_detections


# Create the combined stream with a simple pipe operation
# enhanced_data_stream = formatted_detection_stream.pipe(ops.map(combine_with_locations), ops.share())

streams = {
    "unitree_video": robot.get_video_stream(),  # Changed from get_ros_video_stream to get_video_stream for WebRTC
    "local_planner_viz": local_planner_viz_stream,
    # "object_detection": viz_stream,  # Uncommented object detection
}
text_streams = {
    "agent_responses": agent_response_stream,
}

web_interface = RobotWebInterface(port=5555, text_streams=text_streams, **streams)

# stt_node = stt()

# Read system query from prompt.txt file
with open(
    os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets/agent/prompt.txt"), "r"
) as f:
    system_query = f.read()

# Create a ClaudeAgent instance
agent = ClaudeAgent(
    dev_name="test_agent",
    # input_query_stream=stt_node.emit_text(),
    input_query_stream=web_interface.query_stream,
    skills=robot.get_skills(),
    system_query=system_query,
    model_name="claude-3-7-sonnet-latest",
    thinking_budget_tokens=0,
)

# tts_node = tts()
# tts_node.consume_text(agent.get_response_observable())

robot_skills = robot.get_skills()
robot_skills.add(ObserveStream)
robot_skills.add(Observe)
robot_skills.add(KillSkill)
robot_skills.add(NavigateWithText)
robot_skills.add(FollowHuman)
robot_skills.add(GetPose)
# robot_skills.add(Speak)
robot_skills.add(NavigateToGoal)
robot_skills.add(Explore)

robot_skills.create_instance("ObserveStream", robot=robot, agent=agent)
robot_skills.create_instance("Observe", robot=robot, agent=agent)
robot_skills.create_instance("KillSkill", robot=robot, skill_library=robot_skills)
robot_skills.create_instance("NavigateWithText", robot=robot)
robot_skills.create_instance("FollowHuman", robot=robot)
robot_skills.create_instance("GetPose", robot=robot)
robot_skills.create_instance("NavigateToGoal", robot=robot)
robot_skills.create_instance("Explore", robot=robot)
# robot_skills.create_instance("Speak", tts_node=tts_node)

# Subscribe to agent responses and send them to the subject
agent.get_response_observable().subscribe(lambda x: agent_response_subject.on_next(x))

print("ObserveStream and Kill skills registered and ready for use")
print("Created memory.txt file")

# Start web interface in a separate thread to avoid blocking
web_thread = threading.Thread(target=web_interface.run)
web_thread.daemon = True
web_thread.start()

try:
    while True:
        # Main loop - can add robot movement or other logic here
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

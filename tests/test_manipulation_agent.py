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

import datetime
import os

import cv2
from dotenv import load_dotenv
from openai import OpenAI
import reactivex as rx
import reactivex.operators as ops
from reactivex.subject import BehaviorSubject

from dimos.agents.claude_agent import ClaudeAgent
from dimos.perception.detection2d.detic_2d_det import Detic2DDetector
from dimos.perception.object_detection_stream import ObjectDetectionStream
from dimos.robot.robot import MockManipulationRobot
from dimos.skills.manipulation.manipulate_skill import Manipulate
from dimos.skills.manipulation.rotation_constraint_skill import RotationConstraintSkill
from dimos.skills.manipulation.translation_constraint_skill import TranslationConstraintSkill
from dimos.skills.skills import SkillLibrary
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure
from dimos.web.robot_web_interface import RobotWebInterface

# Initialize logger for the agent module
logger = setup_logger()

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
    return parser.parse_args()


args = parse_arguments()


# Set up the manipulation skills library
manipulation_skills = SkillLibrary()

robot = MockManipulationRobot(skill_library=manipulation_skills)

# Add the skills to the library
manipulation_skills.add(TranslationConstraintSkill)
manipulation_skills.add(RotationConstraintSkill)
manipulation_skills.add(Manipulate)

# Create instances with appropriate parameters
manipulation_skills.create_instance("TranslationConstraintSkill", robot=robot)
manipulation_skills.create_instance("RotationConstraintSkill", robot=robot)
manipulation_skills.create_instance("Manipulate", robot=robot)


# Create a subject for agent responses
agent_response_subject = rx.subject.Subject()
agent_response_stream = agent_response_subject.pipe(ops.share())

# Initialize object detection stream
detector = Detic2DDetector()


# Initialize test video stream
# video_stream = VideoProvider(
#     dev_name="UnitreeGo2",
#     video_source=f"{os.getcwd()}/assets/trimmed_video_office.mov"
# ).capture_video_as_observable(realtime=False, fps=1)

# Initialize ObjectDetectionStream with robot
object_detector = ObjectDetectionStream(
    camera_intrinsics=robot.camera_intrinsics,
    detector=detector,
    video_stream=robot.video_stream,
    disable_depth=True,
)

# Create visualization stream for web interface (detection visualization)
viz_stream = backpressure(object_detector.get_stream()).pipe(
    ops.share(),
    ops.map(lambda x: x["viz_frame"] if x is not None else None),
    ops.filter(lambda x: x is not None),
)


# Helper function to draw a manipulation point on a frame
def draw_point_on_frame(frame, x, y):
    # Draw a circle at the manipulation point
    cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)  # Red circle
    cv2.circle(frame, (x, y), 12, (255, 255, 255), 2)  # White border

    # Add text with coordinates
    cv2.putText(
        frame, f"({x},{y})", (x + 15, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
    )

    return frame


# Function to add manipulation point to stream frames
def draw_manipulation_point(frame):
    try:
        if frame is None or latest_manipulation_point.value is None:
            return frame

        # Make a copy to avoid modifying the original frame
        viz_frame = frame.copy()

        # Get the latest manipulation point coordinates
        x, y = latest_manipulation_point.value

        # Draw the point using our helper function
        draw_point_on_frame(viz_frame, x, y)

        return viz_frame
    except Exception as e:
        logger.error(f"Error drawing manipulation point: {e}")
        return frame


# Create manipulation point visualization stream
manipulation_viz_stream = robot.video_stream.pipe(
    ops.map(draw_manipulation_point), ops.filter(lambda x: x is not None), ops.share()
)

# Get the formatted detection stream
formatted_detection_stream = object_detector.get_formatted_stream().pipe(
    ops.filter(lambda x: x is not None)
)


# Create a direct mapping that combines detection data with locations
def combine_with_locations(object_detections):
    # Get locations from spatial memory
    try:
        locations = robot.get_spatial_memory().get_robot_locations()

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
enhanced_data_stream = formatted_detection_stream.pipe(ops.map(combine_with_locations), ops.share())

streams = {
    "unitree_video": robot.video_stream,
    "object_detection": viz_stream,
    "manipulation_point": manipulation_viz_stream,
}
text_streams = {
    "agent_responses": agent_response_stream,
}

web_interface = RobotWebInterface(port=5555, text_streams=text_streams, **streams)

# stt_node = stt()

# Read system query from prompt.txt file
with open(
    os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets", "agent", "prompt.txt")
) as f:
    system_query = f.read()


# Create response subject
response_subject = rx.subject.Subject()

# Create behavior subjects to store the current frame and latest manipulation point
# BehaviorSubject stores the latest value and provides it to new subscribers
current_frame_subject = BehaviorSubject(None)
latest_manipulation_point = BehaviorSubject(None)  # Will store (x, y) tuple


# Function to parse manipulation point coordinates from VLM response
def process_manipulation_point(response, frame):
    logger.info(f"Processing manipulation point with response: {response}")
    try:
        # Parse coordinates from response (format: "x,y")
        coords = response.strip().split(",")
        if len(coords) != 2:
            logger.error(f"Invalid coordinate format: {response}")
            return

        x, y = int(coords[0]), int(coords[1])

        # Update the latest manipulation point subject with the new coordinates
        latest_manipulation_point.on_next((x, y))

        # Save a static image with the point for reference
        save_manipulation_point_image(frame, x, y)

    except Exception as e:
        logger.error(f"Error processing manipulation point: {e}")


# Function to save a static image with manipulation point visualization
def save_manipulation_point_image(frame, x, y):
    try:
        if frame is None:
            logger.error("Cannot save manipulation point image: frame is None")
            return

        # Create a copy of the frame for static image saving
        visualization = frame.copy()

        # Draw the manipulation point
        draw_point_on_frame(visualization, x, y)

        # Create directory if it doesn't exist
        output_dir = os.path.join(os.getcwd(), "assets", "agent", "manipulation_agent")
        os.makedirs(output_dir, exist_ok=True)

        # Save image with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = os.path.join(output_dir, f"manipulation_point_{timestamp}.jpg")
        cv2.imwrite(output_path, visualization)

        logger.info(f"Saved manipulation point visualization to {output_path}")
    except Exception as e:
        logger.error(f"Error saving manipulation point image: {e}")


# Subscribe to video stream to capture current frame
# Use `current_frame_subject` BehaviorSubject to store the latest frame for manipulation point visualization
robot.video_stream.subscribe(
    on_next=lambda frame: current_frame_subject.on_next(
        frame.copy() if frame is not None else None
    ),
    on_error=lambda error: logger.error(f"Error in video stream: {error}"),
)

# Create Qwen client
qwen_client = OpenAI(
    base_url="https://dashscope-intl.aliyuncs.com/compatible-mode/v1",
    api_key=os.getenv("ALIBABA_API_KEY"),
)

# Create temporary agent for processing
manipulation_vlm = ClaudeAgent(
    dev_name="QwenSingleFrameAgent",
    # openai_client=qwen_client,
    # model_name="qwen2.5-vl-72b-instruct",
    # tokenizer=HuggingFaceTokenizer(model_name=f"Qwen/qwen2.5-vl-72b-instruct"),
    # max_output_tokens_per_request=100,
    system_query="You are a robot that is trying to perform a manipulation task. ",
    # input_video_stream=robot.video_stream,
    skills=manipulation_skills,
    input_query_stream=web_interface.query_stream,
    # input_data_stream=enhanced_data_stream,
)

# # Subscribe to VLM responses to process manipulation points
# manipulation_vlm.get_response_observable().subscribe(
#     on_next=lambda response: process_manipulation_point(response, current_frame_subject.value),
#     on_error=lambda error: logger.error(f"Error in VLM response stream: {error}"),
# )


# Create a ClaudeAgent instance
# manipulation_agent = ClaudeAgent(
#     dev_name="test_agent",
#     # input_query_stream=stt_node.emit_text(),
#     input_query_stream=manipulation_vlm.get_response_observable(),
#     input_data_stream=enhanced_data_stream,  # Add the enhanced data stream
#     skills=robot.get_skills(),
#     system_query="system_query",
#     model_name="claude-3-7-sonnet-latest",
#     thinking_budget_tokens=0
# )

# tts_node = tts()
# tts_node.consume_text(agent.get_response_observable())

# robot_skills = robot.get_skills()
# robot_skills.add(ObserveStream)
# robot_skills.add(KillSkill)
# robot_skills.add(NavigateWithText)
# robot_skills.add(FollowHuman)
# robot_skills.add(GetPose)
# # robot_skills.add(Speak)
# robot_skills.add(NavigateToGoal)
# robot_skills.create_instance("ObserveStream", robot=robot, agent=manipulation_agent)
# robot_skills.create_instance("KillSkill", robot=robot, skill_library=robot_skills)
# robot_skills.create_instance("NavigateWithText", robot=robot)
# robot_skills.create_instance("FollowHuman", robot=robot)
# robot_skills.create_instance("GetPose", robot=robot)
# robot_skills.create_instance("NavigateToGoal", robot=robot)
# robot_skills.create_instance("Speak", tts_node=tts_node)

# Subscribe to agent responses and send them to the subject
# manipulation_agent.get_response_observable().subscribe(
#     lambda x: agent_response_subject.on_next(x)
# )


web_interface.run()

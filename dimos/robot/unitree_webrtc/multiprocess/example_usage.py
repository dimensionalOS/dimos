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

"""Example usage of UnitreeGo2Light and UnitreeGo2Heavy classes."""

import asyncio
import os
import threading

import reactivex as rx
import reactivex.operators as ops

from dimos.agents.claude_agent import ClaudeAgent
from dimos.perception.object_detection_stream import ObjectDetectionStream
from dimos.robot.unitree_webrtc.multiprocess.unitree_go2 import UnitreeGo2Light
from dimos.robot.unitree_webrtc.multiprocess.unitree_go2_heavy import UnitreeGo2Heavy
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import AbstractRobotSkill, SkillLibrary
from dimos.stream.audio.pipelines import stt, tts
from dimos.utils.reactive import backpressure
from dimos.web.robot_web_interface import RobotWebInterface


async def run_light_robot():
    """Example of running the lightweight robot without GPU modules."""
    ip = os.getenv("ROBOT_IP")

    robot = UnitreeGo2Light(ip)

    await robot.start()

    pose = robot.get_pose()
    print(f"Robot position: {pose['position']}")
    print(f"Robot rotation: {pose['rotation']}")

    from dimos.msgs.geometry_msgs import Vector3

    # robot.move(Vector3(0.5, 0, 0), duration=2.0)

    robot.explore()

    while True:
        await asyncio.sleep(1)


async def run_heavy_robot():
    """Example of running the heavy robot with full GPU modules."""
    ip = os.getenv("ROBOT_IP")

    # Create heavy robot instance with all features
    robot = UnitreeGo2Heavy(ip=ip, new_memory=True, enable_perception=True)

    # Start the robot
    await robot.start()

    if robot.spatial_memory:
        print("Spatial memory initialized")

    skills = robot.get_skills()
    print(f"Available skills: {[skill.__class__.__name__ for skill in skills]}")

    from dimos.types.robot_capabilities import RobotCapability

    if robot.has_capability(RobotCapability.VISION):
        print("Robot has vision capability")

    if robot.person_tracking_stream:
        print("Person tracking enabled")

    # Start exploration with spatial memory recording
    print(robot.spatial_memory.query_by_text("kitchen"))

    # robot.frontier_explorer.explore()

    # Create a subject for agent responses
    agent_response_subject = rx.subject.Subject()
    agent_response_stream = agent_response_subject.pipe(ops.share())
    audio_subject = rx.subject.Subject()

    video_stream = robot.get_video_stream()  # WebRTC doesn't use ROS video stream

    # # Initialize ObjectDetectionStream with robot
    # object_detector = ObjectDetectionStream(
    #     camera_intrinsics=robot.camera_intrinsics,
    #     get_pose=robot.get_pose,
    #     video_stream=video_stream,
    #     draw_masks=True,
    # )

    # # Create visualization stream for web interface
    # viz_stream = backpressure(object_detector.get_stream()).pipe(
    #     ops.share(),
    #     ops.map(lambda x: x["viz_frame"] if x is not None else None),
    #     ops.filter(lambda x: x is not None),
    # )

    streams = {
        "unitree_video": robot.get_video_stream(),  # Changed from get_ros_video_stream to get_video_stream for WebRTC
        # "object_detection": viz_stream,  # Uncommented object detection
    }
    text_streams = {
        "agent_responses": agent_response_stream,
    }

    web_interface = RobotWebInterface(
        port=5555, text_streams=text_streams, audio_subject=audio_subject, **streams
    )

    stt_node = stt()
    stt_node.consume_audio(audio_subject.pipe(ops.share()))

    agent = ClaudeAgent(
        dev_name="test_agent",
        # input_query_stream=stt_node.emit_text(),
        input_query_stream=web_interface.query_stream,
        skills=robot.get_skills(),
        system_query="You are a helpful robot.",
        model_name="claude-3-5-haiku-latest",
        thinking_budget_tokens=0,
        max_output_tokens_per_request=8192,
        # model_name="llama-4-scout-17b-16e-instruct",
    )
    agent.get_response_observable().subscribe(lambda x: agent_response_subject.on_next(x))

    # Start web interface in a separate thread to avoid blocking
    web_thread = threading.Thread(target=web_interface.run)
    web_thread.daemon = True
    web_thread.start()

    # Keep running
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    use_heavy = False

    if use_heavy:
        print("Running UnitreeGo2Heavy with GPU modules...")
        asyncio.run(run_heavy_robot())
    else:
        print("Running UnitreeGo2Light without GPU modules...")
        asyncio.run(run_light_robot())

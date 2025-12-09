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

"""Run GPU-enabled Unitree Go2 robot with full perception capabilities."""

import argparse
import asyncio
import os
import threading

import reactivex as rx
import reactivex.operators as ops

from dimos.agents.claude_agent import ClaudeAgent
from dimos.perception.object_detection_stream import ObjectDetectionStream
from dimos.protocol import pubsub
from dimos.robot.unitree_webrtc.multiprocess.unitree_go2_heavy import UnitreeGo2Heavy
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.navigation import Explore
from dimos.stream.audio.pipelines import stt
from dimos.utils.reactive import backpressure
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.skills.navigation import NavigateWithText


async def main():
    """Run the GPU-enabled robot with full perception capabilities."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Run GPU-enabled Unitree Go2 robot")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--simulated", action="store_true", help="Use simulated robot with FakeRTC")
    group.add_argument(
        "--webrtc", action="store_true", help="Use real robot with WebRTC connection"
    )
    args = parser.parse_args()

    # Configure LCM
    pubsub.lcm.autoconf()

    # Get robot IP
    ip = os.getenv("ROBOT_IP", "127.0.0.1" if args.simulated else None)
    if not ip and args.webrtc:
        raise ValueError("ROBOT_IP environment variable must be set for WebRTC connection")

    print(f"Starting UnitreeGo2Heavy robot with GPU modules...")
    print(f"Mode: {'Simulated (FakeRTC)' if args.simulated else 'Real (WebRTC)'}")
    print(f"IP: {ip}")

    # Create heavy robot instance with all features
    robot = UnitreeGo2Heavy(
        ip=ip,
        simulated=args.simulated,
        skill_library=MyUnitreeSkills(),
        new_memory=True,
        enable_perception=True,
    )

    # Start the robot
    await robot.start()
    robot.standup()

    # Get robot pose
    pose = robot.get_pose()
    print(f"Robot position: {pose['position']}")
    print(f"Robot rotation: {pose['rotation']}")

    # Check spatial memory
    if robot.spatial_memory:
        print("Spatial memory initialized")

    skills = robot.get_skills()

    skills.add(Explore)
    skills.add(NavigateWithText)

    skills.create_instance("Explore", robot=robot)
    skills.create_instance("NavigateWithText", robot=robot)

    print(f"Available skills: {[skill.__class__.__name__ for skill in skills]}")

    # Check capabilities
    from dimos.types.robot_capabilities import RobotCapability

    if robot.has_capability(RobotCapability.VISION):
        print("Robot has vision capability")

    # Create subjects for streams
    agent_response_subject = rx.subject.Subject()
    agent_response_stream = agent_response_subject.pipe(ops.share())
    audio_subject = rx.subject.Subject()

    # Get video stream
    video_stream = robot.get_video_stream(fps=10)

    # Initialize ObjectDetectionStream with robot
    object_detector = ObjectDetectionStream(
        camera_intrinsics=robot.camera_intrinsics,
        get_pose=robot.get_pose,
        video_stream=video_stream,
        draw_masks=True,
    )

    # Create visualization stream for web interface
    viz_stream = backpressure(object_detector.get_stream()).pipe(
        ops.share(),
        ops.map(lambda x: x["viz_frame"] if x is not None else None),
        ops.filter(lambda x: x is not None),
    )

    # Get tracking visualization streams if available
    tracking_streams = {}
    if robot.person_tracking_stream:
        tracking_streams["person_tracking"] = robot.person_tracking_stream.pipe(
            ops.map(lambda x: x.get("viz_frame") if x else None),
            ops.filter(lambda x: x is not None),
        )
    if robot.object_tracking_stream:
        tracking_streams["object_tracking"] = robot.object_tracking_stream.pipe(
            ops.map(lambda x: x.get("viz_frame") if x else None),
            ops.filter(lambda x: x is not None),
        )

    # Set up streams for web interface
    streams = {
        "unitree_video": video_stream,
        "object_detection": viz_stream,
    }
    text_streams = {
        "agent_responses": agent_response_stream,
    }

    # Create web interface
    web_interface = RobotWebInterface(
        port=5555, text_streams=text_streams, audio_subject=audio_subject, **streams
    )

    # Set up speech-to-text
    stt_node = stt()
    stt_node.consume_audio(audio_subject.pipe(ops.share()))

    # Create agent
    agent = ClaudeAgent(
        dev_name="gpu_robot_agent",
        input_query_stream=web_interface.query_stream,
        # input_query_stream=stt_node.emit_text(), # USE FOR VOICE CONTROL
        skills=skills,
        system_query="You are a helpful robot assistant. You can control a Unitree Go2 robot with full perception capabilities including object detection, person tracking, and spatial memory.",
        model_name="claude-3-5-haiku-latest",
        thinking_budget_tokens=0,
        max_output_tokens_per_request=8192,
    )

    # Subscribe agent responses to web interface
    agent.get_response_observable().subscribe(lambda x: agent_response_subject.on_next(x))

    # Start web interface in a separate thread
    web_thread = threading.Thread(target=web_interface.run)
    web_thread.daemon = True
    web_thread.start()

    print("\n" + "=" * 60)
    print("UnitreeGo2Heavy robot is running with GPU acceleration!")
    print(f"Web interface available at: http://localhost:5555")
    print("Available features:")
    print("  - Robot exploration and navigation")
    print("  - Video streaming with Detic-powered object detection")
    print("  - Person and object tracking")
    print("  - Spatial memory and semantic mapping with CLIP-based embeddings")
    print("=" * 60 + "\n")

    # Example spatial memory query
    # results = robot.spatial_memory.query_by_text("kitchen")
    # print(f"Spatial memory query results: {results}")

    # Optionally start exploration
    # robot.explore()

    # Keep running
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        robot.liedown()
        robot.cleanup()


if __name__ == "__main__":
    asyncio.run(main())

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

"""
Run script for Piper Arm robot with Claude agent integration.
Provides manipulation capabilities with natural language interface.
"""

import asyncio
import os
import sys

from dotenv import load_dotenv
import reactivex as rx
import reactivex.operators as ops

from dimos.agents_deprecated.claude_agent import ClaudeAgent
from dimos.robot.agilex.piper_arm import PiperArmRobot
from dimos.skills.kill_skill import KillSkill
from dimos.skills.manipulation.pick_and_place import PickAndPlace
from dimos.stream.audio.pipelines import stt, tts
from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()

# Load environment variables
load_dotenv()

# System prompt for the Piper Arm manipulation agent
SYSTEM_PROMPT = """You are an intelligent robotic assistant controlling a Piper Arm robot with advanced manipulation capabilities. Your primary role is to help users with pick and place tasks using natural language understanding.

## Your Capabilities:
1. **Visual Perception**: You have access to a ZED stereo camera that provides RGB and depth information
2. **Object Manipulation**: You can pick up and place objects using a 6-DOF robotic arm with a gripper
3. **Language Understanding**: You use the Qwen vision-language model to identify objects and locations from natural language descriptions

## Available Skills:
- **PickAndPlace**: Execute pick and place operations based on object and location descriptions
  - Pick only: "Pick up the red mug"
  - Pick and place: "Move the book to the shelf"
- **KillSkill**: Stop any currently running skill

## Guidelines:
1. **Safety First**: Always ensure safe operation. If unsure about an object's graspability or a placement location's stability, ask for clarification
2. **Clear Communication**: Explain what you're doing and ask for confirmation when needed
3. **Error Handling**: If a task fails, explain why and suggest alternatives
4. **Precision**: When users give specific object descriptions, use them exactly as provided to the vision model

## Interaction Examples:
- User: "Pick up the coffee mug"
  You: "I'll pick up the coffee mug for you." [Execute PickAndPlace with object_query="coffee mug"]

- User: "Put the toy on the table"
  You: "I'll place the toy on the table." [Execute PickAndPlace with object_query="toy", target_query="on the table"]

- User: "What do you see?"

Remember: You're here to assist with manipulation tasks. Be helpful, precise, and always prioritize safe operation of the robot."""


def main():  # type: ignore[no-untyped-def]
    """Main entry point."""
    print("\n" + "=" * 60)
    print("Piper Arm Robot with Claude Agent")
    print("=" * 60)
    print("\nThis system integrates:")
    print("  - Piper Arm 6-DOF robot")
    print("  - ZED stereo camera")
    print("  - Claude AI for natural language understanding")
    print("  - Qwen VLM for visual object detection")
    print("  - Web interface with text and voice input")
    print("  - Foxglove visualization via LCM")
    print("\nStarting system...\n")

    # Check for API key
    if not os.getenv("ANTHROPIC_API_KEY"):
        print("WARNING: ANTHROPIC_API_KEY not found in environment")
        print("Please set your API key in .env file or environment")
        sys.exit(1)

    logger.info("Starting Piper Arm Robot with Agent")

    # Create robot instance
    robot = PiperArmRobot()  # type: ignore[abstract]

    try:
        # Start the robot (this is async, so we need asyncio.run)
        logger.info("Initializing robot...")
        asyncio.run(robot.start())
        logger.info("Robot initialized successfully")

        # Set up skill library
        skills = robot.get_skills()  # type: ignore[no-untyped-call]
        skills.add(PickAndPlace)
        skills.add(KillSkill)

        # Create skill instances
        skills.create_instance("PickAndPlace", robot=robot)
        skills.create_instance("KillSkill", robot=robot, skill_library=skills)

        logger.info(f"Skills registered: {[skill.__name__ for skill in skills.get_class_skills()]}")

        # Set up streams for agent and web interface
        agent_response_subject = rx.subject.Subject()  # type: ignore[var-annotated]
        agent_response_stream = agent_response_subject.pipe(ops.share())
        audio_subject = rx.subject.Subject()  # type: ignore[var-annotated]

        # Set up streams for web interface
        streams = {}  # type: ignore[var-annotated]

        text_streams = {
            "agent_responses": agent_response_stream,
        }

        # Create web interface first (needed for agent)
        try:
            web_interface = RobotWebInterface(
                port=5555, text_streams=text_streams, audio_subject=audio_subject, **streams
            )
            logger.info("Web interface created successfully")
        except Exception as e:
            logger.error(f"Failed to create web interface: {e}")
            raise

        # Set up speech-to-text
        stt_node = stt()  # type: ignore[no-untyped-call]
        stt_node.consume_audio(audio_subject.pipe(ops.share()))

        # Create Claude agent
        agent = ClaudeAgent(
            dev_name="piper_arm_agent",
            input_query_stream=web_interface.query_stream,  # Use text input from web interface
            # input_query_stream=stt_node.emit_text(),  # Uncomment to use voice input
            skills=skills,
            system_query=SYSTEM_PROMPT,
            model_name="claude-3-5-haiku-latest",
            thinking_budget_tokens=0,
            max_output_tokens_per_request=4096,
        )

        # Subscribe to agent responses
        agent.get_response_observable().subscribe(lambda x: agent_response_subject.on_next(x))

        # Set up text-to-speech for agent responses
        tts_node = tts()  # type: ignore[no-untyped-call]
        tts_node.consume_text(agent.get_response_observable())

        logger.info("=" * 60)
        logger.info("Piper Arm Agent Ready!")
        logger.info("Web interface available at: http://localhost:5555")
        logger.info("Foxglove visualization available at: ws://localhost:8765")
        logger.info("You can:")
        logger.info("  - Type commands in the web interface")
        logger.info("  - Use voice commands")
        logger.info("  - Ask the robot to pick up objects")
        logger.info("  - Ask the robot to move objects to locations")
        logger.info("=" * 60)

        # Run web interface (this blocks)
        web_interface.run()

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Error running robot: {e}")
        import traceback

        traceback.print_exc()
    finally:
        logger.info("Shutting down...")
        # Stop the robot (this is also async)
        robot.stop()
        logger.info("Robot stopped")


if __name__ == "__main__":
    main()  # type: ignore[no-untyped-call]

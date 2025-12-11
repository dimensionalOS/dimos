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
Run script for Unitree Go2 robot with Claude agent integration.
Provides navigation and interaction capabilities with natural language interface.
"""

import os
import sys
import time
from dotenv import load_dotenv

import reactivex as rx
import reactivex.operators as ops

from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.agents.claude_agent import ClaudeAgent
from dimos.skills.kill_skill import KillSkill
from dimos.skills.navigation import NavigateWithText, GetPose, NavigateToGoal, Explore
from dimos.skills.unitree.unitree_speak import UnitreeSpeak
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.stream.audio.pipelines import stt, tts
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.run")

# Load environment variables
load_dotenv()

# System prompt - loaded from prompt.txt
SYSTEM_PROMPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
    "assets/agent/prompt.txt",
)


def main():
    """Main entry point."""
    print("\n" + "=" * 60)
    print("Unitree Go2 Robot with Claude Agent")
    print("=" * 60)
    print("\nThis system integrates:")
    print("  - Unitree Go2 quadruped robot")
    print("  - WebRTC communication interface")
    print("  - Claude AI for natural language understanding")
    print("  - Spatial memory and navigation")
    print("  - Web interface with text and voice input")
    print("\nStarting system...\n")

    # Check for API key
    if not os.getenv("ANTHROPIC_API_KEY"):
        print("WARNING: ANTHROPIC_API_KEY not found in environment")
        print("Please set your API key in .env file or environment")
        sys.exit(1)

    # Load system prompt
    try:
        with open(SYSTEM_PROMPT_PATH, "r") as f:
            system_prompt = f.read()
    except FileNotFoundError:
        logger.error(f"System prompt file not found at {SYSTEM_PROMPT_PATH}")
        sys.exit(1)

    logger.info("Starting Unitree Go2 Robot with Agent")

    # Create robot instance
    robot = UnitreeGo2(
        ip=os.getenv("ROBOT_IP"),
    )

    robot.start()
    time.sleep(3)

    try:
        logger.info("Robot initialized successfully")

        # Set up skill library
        skills = robot.get_skills()
        skills.add(KillSkill)
        skills.add(NavigateWithText)
        skills.add(GetPose)
        skills.add(NavigateToGoal)
        skills.add(Explore)

        # Create skill instances
        skills.create_instance("KillSkill", robot=robot, skill_library=skills)
        skills.create_instance("NavigateWithText", robot=robot)
        skills.create_instance("GetPose", robot=robot)
        skills.create_instance("NavigateToGoal", robot=robot)
        skills.create_instance("Explore", robot=robot)

        logger.info(f"Skills registered: {[skill.__name__ for skill in skills.get_class_skills()]}")

        # Set up streams for agent and web interface
        agent_response_subject = rx.subject.Subject()
        agent_response_stream = agent_response_subject.pipe(ops.share())
        audio_subject = rx.subject.Subject()

        # Set up streams for web interface
        streams = {}

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
        stt_node = stt()
        stt_node.consume_audio(audio_subject.pipe(ops.share()))

        # Create Claude agent
        agent = ClaudeAgent(
            dev_name="unitree_go2_agent",
            input_query_stream=web_interface.query_stream,  # Use text input from web interface
            # input_query_stream=stt_node.emit_text(),  # Uncomment to use voice input
            skills=skills,
            system_query=system_prompt,
            model_name="claude-3-5-haiku-latest",
            thinking_budget_tokens=0,
            max_output_tokens_per_request=8192,
        )

        # Subscribe to agent responses
        agent.get_response_observable().subscribe(lambda x: agent_response_subject.on_next(x))

        # Set up text-to-speech for agent responses
        tts_node = tts()
        tts_node.consume_text(agent.get_response_observable())

        # Create skill instances that need agent reference

        logger.info("=" * 60)
        logger.info("Unitree Go2 Agent Ready!")
        logger.info(f"Web interface available at: http://localhost:5555")
        logger.info("You can:")
        logger.info("  - Type commands in the web interface")
        logger.info("  - Use voice commands")
        logger.info("  - Ask the robot to navigate to locations")
        logger.info("  - Ask the robot to observe and describe its surroundings")
        logger.info("  - Ask the robot to follow people or explore areas")
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
        # WebRTC robot doesn't have a stop method, just log shutdown
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()

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
Run script for Unitree G1 humanoid robot with Claude agent integration.
Provides interaction capabilities with natural language interface and ZED vision.
"""

import os
import sys
import time
from dotenv import load_dotenv

import reactivex as rx
import reactivex.operators as ops

from dimos.robot.unitree_webrtc.unitree_g1 import UnitreeG1
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.agents.claude_agent import ClaudeAgent
from dimos.skills.kill_skill import KillSkill
from dimos.skills.navigation import GetPose
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.g1_run")

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
    print("Unitree G1 Humanoid Robot with Claude Agent")
    print("=" * 60)
    print("\nThis system integrates:")
    print("  - Unitree G1 humanoid robot")
    print("  - ZED camera for stereo vision and depth")
    print("  - WebRTC communication for robot control")
    print("  - Claude AI for natural language understanding")
    print("  - Web interface with text and voice input")
    print("\nStarting system...\n")

    # Load system prompt
    try:
        with open(SYSTEM_PROMPT_PATH, "r") as f:
            system_prompt = f.read()
    except FileNotFoundError:
        logger.error(f"System prompt file not found at {SYSTEM_PROMPT_PATH}")
        sys.exit(1)

    logger.info("Starting Unitree G1 Robot with Agent")

    # Create robot instance
    robot = UnitreeG1(ip=os.getenv("ROBOT_IP"))
    robot.start()
    time.sleep(3)

    try:
        logger.info("Robot initialized successfully")

        # Set up minimal skill library for G1 with robot_type="g1"
        skills = MyUnitreeSkills(robot=robot, robot_type="g1")
        skills.add(KillSkill)
        skills.add(GetPose)

        # Create skill instances
        skills.create_instance("KillSkill", robot=robot, skill_library=skills)
        skills.create_instance("GetPose", robot=robot)

        logger.info(f"Skills registered: {[skill.__name__ for skill in skills.get_class_skills()]}")

        # Set up streams for agent and web interface
        agent_response_subject = rx.subject.Subject()
        agent_response_stream = agent_response_subject.pipe(ops.share())
        audio_subject = rx.subject.Subject()

        # Set up streams for web interface
        text_streams = {
            "agent_responses": agent_response_stream,
        }

        # Create web interface
        try:
            web_interface = RobotWebInterface(
                port=5555, text_streams=text_streams, audio_subject=audio_subject
            )
            logger.info("Web interface created successfully")
        except Exception as e:
            logger.error(f"Failed to create web interface: {e}")
            raise

        # Create Claude agent with minimal configuration
        agent = ClaudeAgent(
            dev_name="unitree_g1_agent",
            input_query_stream=web_interface.query_stream,  # Text input from web
            skills=skills,
            system_query=system_prompt,
            model_name="claude-3-5-haiku-latest",
            thinking_budget_tokens=0,
            max_output_tokens_per_request=8192,
        )

        # Subscribe to agent responses
        agent.get_response_observable().subscribe(lambda x: agent_response_subject.on_next(x))

        logger.info("=" * 60)
        logger.info("Unitree G1 Agent Ready!")
        logger.info(f"Web interface available at: http://localhost:5555")
        logger.info("You can:")
        logger.info("  - Type commands in the web interface")
        logger.info("  - Use voice commands")
        logger.info("  - Ask the robot to move or perform actions")
        logger.info("  - Ask the robot to describe what it sees")
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
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()

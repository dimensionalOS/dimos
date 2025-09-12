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
Run script for Unitree Go2 robot with agents2 framework.
This is the migrated version using the new LangChain-based agent system.
"""

import asyncio  # Needed for event loop management in setup_agent
import os
import sys
import time
from pathlib import Path

from dotenv import load_dotenv

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from threading import Thread

import reactivex as rx
import reactivex.operators as ops

from dimos.agents2 import Agent, Output, Reducer, Stream, skill
from dimos.agents2.spec import Model, Provider
from dimos.core import Module
from dimos.hardware.webcam import ColorCameraModule, Webcam
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.utils.logging_config import setup_logger

# For web interface (simplified for now)
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger("dimos.agents2.run_unitree")

# Load environment variables
load_dotenv()

# System prompt path
SYSTEM_PROMPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
    "assets/agent/prompt.txt",
)


class WebModule(Module):
    web_interface: RobotWebInterface = None
    human_query: rx.subject.Subject = None
    agent_response: rx.subject.Subject = None

    thread: Thread = None

    _human_messages_running = False

    def __init__(self):
        super().__init__()
        self.agent_response = rx.subject.Subject()
        self.human_query = rx.subject.Subject()

    def start(self):
        text_streams = {
            "agent_responses": self.agent_response,
        }

        self.web_interface = RobotWebInterface(
            port=5555,
            text_streams=text_streams,
            audio_subject=rx.subject.Subject(),
        )

        self.web_interface.query_stream.subscribe(self.human_query.on_next)

        self.thread = Thread(target=self.web_interface.run, daemon=True)
        self.thread.start()

    def stop(self):
        if self.web_interface:
            self.web_interface.stop()
        if self.thread:
            self.thread.join(timeout=1.0)

        super().stop()

    @skill(stream=Stream.call_agent, reducer=Reducer.all, output=Output.human)
    def human_messages(self):
        """Provide human messages from web interface. Don't use this tool, it's running implicitly already"""
        if self._human_messages_running:
            return "already running"
        self._human_messages_running = True
        while True:
            message = self.human_query.pipe(ops.first()).run()
            yield message


class UnitreeAgentRunner:
    """Manages the Unitree robot with the new agents2 framework."""

    def __init__(self):
        self.robot = None
        self.agent = None
        self.web_interface = None
        self.agent_thread = None
        self.running = False

    def setup_robot(self) -> UnitreeGo2:
        """Initialize the robot connection."""
        logger.info("Initializing Unitree Go2 robot...")

        robot = UnitreeGo2(
            ip=os.getenv("ROBOT_IP"),
            connection_type=os.getenv("CONNECTION_TYPE", "webrtc"),
        )

        robot.start()
        time.sleep(3)

        logger.info("Robot initialized successfully")
        return robot

    def setup_agent(self, skillcontainers, system_prompt: str) -> Agent:
        """Create and configure the agent with skills."""
        logger.info("Setting up agent with skills...")

        # Create agent
        agent = Agent(
            system_prompt=system_prompt,
            model=Model.GPT_4O,  # Could add CLAUDE models to enum
            provider=Provider.OPENAI,  # Would need ANTHROPIC provider
        )

        for container in skillcontainers:
            print("REGISTERING SKILLS FROM CONTAINER:", container)
            agent.register_skills(container)

        # Start agent
        agent.start()

        # Log available skills
        tools = agent.get_tools()
        logger.info(f"Agent configured with {len(tools)} skills:")
        for tool in tools:  # Show first 5
            logger.info(f"  - {tool.name}")

        agent.run_implicit_skill("human_messages")
        # agent.run_implicit_skill("current_time")

        print("STARTING AGENT LOOP")
        print(agent.loop_thread())
        print("AGENT LOOP STARTED")
        return agent

    def setup_web(self) -> WebModule:
        logger.info("Setting up web interface...")
        web_module = WebModule()
        web_module.start()

        return web_module

    def run(self):
        """Main run loop."""
        print("\n" + "=" * 60)
        print("Unitree Go2 Robot with agents2 Framework")
        print("=" * 60)
        print("\nThis system integrates:")
        print("  - Unitree Go2 quadruped robot")
        print("  - WebRTC communication interface")
        print("  - LangChain-based agent system (agents2)")
        print("  - Converted skill system with @skill decorators")
        print("  - Web interface for text input")
        print("\nStarting system...\n")

        # Check for API key (would need ANTHROPIC_API_KEY for Claude)
        if not os.getenv("OPENAI_API_KEY"):
            print("WARNING: OPENAI_API_KEY not found in environment")
            print("Please set your API key in .env file or environment")
            print("(Note: Full Claude support would require ANTHROPIC_API_KEY)")
            sys.exit(1)

        # Load system prompt
        # try:
        #    with open(SYSTEM_PROMPT_PATH, "r") as f:
        #        system_prompt = f.read()
        # except FileNotFoundError:
        #   logger.warning(f"System prompt file not found at {SYSTEM_PROMPT_PATH}")

        # above made agent perform terrible
        system_prompt = """You are a helpful robot assistant controlling a Unitree Go2 quadruped robot.
You can move, navigate, speak, and perform various actions. Be helpful and friendly."""

        try:
            # Setup components
            self.robot = self.setup_robot()
            self.web_interface = self.setup_web()

            from dimos.protocol.skill.test_coordinator import SkillContainerTest

            webcam = ColorCameraModule()
            webcam.start()
            self.agent = self.setup_agent(
                [
                    # SkillContainerTest(),
                    webcam,
                    UnitreeSkillContainer(self.robot),
                    self.web_interface,
                ],
                system_prompt,
            )

            # Start handling queries
            self.running = True

            logger.info("=" * 60)
            logger.info("Unitree Go2 Agent Ready (agents2 framework)!")
            logger.info("Web interface available at: http://localhost:5555")
            logger.info("You can:")
            logger.info("  - Type commands in the web interface")
            logger.info("  - Ask the robot to move or navigate")
            logger.info("  - Ask the robot to perform actions (sit, stand, dance, etc.)")
            logger.info("  - Ask the robot to speak text")
            logger.info("=" * 60)

            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received")
        except Exception as e:
            logger.error(f"Error running robot: {e}")
            import traceback

            traceback.print_exc()
        # finally:
        # self.shutdown()

    def shutdown(self):
        """Clean shutdown of all components."""
        logger.info("Shutting down...")
        self.running = False

        if self.agent:
            try:
                self.agent.stop()
                logger.info("Agent stopped")
            except Exception as e:
                logger.error(f"Error stopping agent: {e}")

        if self.robot:
            try:
                # WebRTC robot doesn't have a stop method
                logger.info("Robot connection closed")
            except Exception as e:
                logger.error(f"Error stopping robot: {e}")

        logger.info("Shutdown complete")


def main():
    """Entry point for the application."""
    runner = UnitreeAgentRunner()
    runner.run()


if __name__ == "__main__":
    main()

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

"""DimOS skill tutorial: module definitions.

This file contains the module classes for the skill basics tutorial.

Note that these classes cannot be defined in the __main__ of the script
that's used to orchestrate the DimOS run,
because DimOS uses Dask with process-based workers,
and classes defined in __main__
(notebooks, scripts) cannot be pickled to other processes.

See: https://distributed.dask.org/en/stable/api.html
"""

import time

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

logger = setup_logger("tutorial_greeter")


# --8<-- [start:RobotCapabilities]
class RobotCapabilities(Module):
    """Low-level capabilities that our (mock) robot possesses.

    In a real setting, there would be a ConnectionModule for the robot platform you are using,
    as well as a wrapper module over that that hides platform-specific details.
    But to keep things simple here, we won't have anything like a ConnectionModule.
    """

    # In a real setting, you would see dependencies on methods of a 'ConnectionModule' here.
    # See, e.g., dimos/robot/unitree_webrtc/unitree_g1_skill_container.py
    rpc_calls = []

    @rpc
    def speak(self, text: str) -> str:
        """Speak text out loud through the robot's speakers.

        Args:
            text: The text to speak.

        Returns:
            Status message.
        """
        time.sleep(0.1)  # Simulate execution time
        logger.info(f"[Skill] RobotCapabilities.speak called: {text}")
        return f"SPEAK: {text}"

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    # The following dunder methods are for Dask serialization:
    # Module instances are serialized when deployed to worker processes.
    # We return {} in __getstate__ since this class has no custom state to preserve.
    def __getstate__(self):
        return {}

    def __setstate__(self, state):
        pass


# --8<-- [end:RobotCapabilities]


# --8<-- [start:Greeter]
class Greeter(Module):
    """High-level Greeter skill built on lower-level RobotCapabilities.

    Does *not* include LLM agent auto-registration -- see the skill_with_agent tutorial.
    """

    # Declares what this module needs from other modules
    rpc_calls = [
        "RobotCapabilities.speak",  # For speaking greetings
    ]

    @skill()
    # Note: Can't combine @skill and @rpc on one method (see multi_agent tutorial for details)
    def greet(self, name: str = "friend") -> str:
        """Greet someone by name.

        Args:
            name: Name of person to greet (default: "friend").

        Returns:
            Status message with greeting details.
        """
        # Skills need to have descriptive docstrings
        # when working with llm agents -- more on this in the skill_with_agent tutorial

        # Get the RPC method reference we need
        speak = self.get_rpc_calls("RobotCapabilities.speak")

        # Create and deliver the greeting
        greeting_text = f"Hello, {name}! Nice to meet you!"
        logger.info(f"[Skill] Greeter.greet executing for: {name}")
        speak(greeting_text)

        return f"Successfully greeted {name}"

    def __getstate__(self):
        return {}

    def __setstate__(self, state):
        pass


# --8<-- [end:Greeter]

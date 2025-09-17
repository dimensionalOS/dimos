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
Basic skill wrappers for agents2 framework.
These wrap the existing basic skills using the new @skill decorator.
"""

from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.skills.kill_skill import KillSkill as OldKillSkill
from dimos.skills.unitree.unitree_speak import UnitreeSpeak as OldUnitreeSpeak
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.agents2.skills.unitree_wrappers.basic")


class BasicSkillContainer(SkillContainer):
    """Container for basic skills using the new framework."""

    def __init__(self, robot=None, skill_library=None):
        """Initialize the basic skill container.

        Args:
            robot: The robot instance
            skill_library: The skill library for kill skill
        """
        super().__init__()
        self._robot = robot
        self._skill_library = skill_library

    @skill()
    def kill(self, skill_name: str) -> str:
        """Kill/stop a running skill by name.

        Args:
            skill_name: Name of the skill to kill
        """
        try:
            old_skill = OldKillSkill(
                robot=self._robot, skill_library=self._skill_library, skill_name=skill_name
            )
            result = old_skill()
            return str(result)
        except Exception as e:
            logger.error(f"Error in kill skill: {e}")
            return f"Error killing skill: {str(e)}"

    @skill()
    def speak(
        self, text: str, voice: str = "echo", speed: float = 1.2, use_megaphone: bool = False
    ) -> str:
        """Speak text out loud through the robot's speakers.

        Args:
            text: Text to speak
            voice: Voice to use (alloy, echo, fable, onyx, nova, shimmer)
            speed: Speech speed (0.25 to 4.0)
            use_megaphone: Use megaphone mode for lower latency
        """
        try:
            old_skill = OldUnitreeSpeak(
                robot=self._robot, text=text, voice=voice, speed=speed, use_megaphone=use_megaphone
            )
            result = old_skill()
            return str(result)
        except Exception as e:
            logger.error(f"Error in speak skill: {e}")
            return f"Error speaking: {str(e)}"

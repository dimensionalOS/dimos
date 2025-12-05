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
Kill skill for terminating running skills.

This module provides a skill that can terminate other running skills,
particularly those running in separate threads like the monitor skill.
"""

from pydantic import Field

from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class KillSkill(AbstractSkill):
    """
    A skill that terminates other running skills.

    This skill can be used to stop long-running or background skills
    like the monitor skill. It uses the centralized process management
    in the SkillLibrary to track and terminate skills.
    """

    skill_name: str = Field(..., description="Name of the skill to terminate")

    def __init__(self, skill_library: SkillLibrary | None = None, **data) -> None:  # type: ignore[no-untyped-def]
        """
        Initialize the kill skill.

        Args:
            skill_library: The skill library instance
            **data: Additional data for configuration
        """
        super().__init__(**data)
        self._skill_library = skill_library

    def __call__(self):  # type: ignore[no-untyped-def]
        """
        Terminate the specified skill.

        Returns:
            A message indicating whether the skill was successfully terminated
        """
        print("running skills", self._skill_library.get_running_skills())  # type: ignore[union-attr]
        # Terminate the skill using the skill library
        return self._skill_library.terminate_skill(self.skill_name)  # type: ignore[union-attr]

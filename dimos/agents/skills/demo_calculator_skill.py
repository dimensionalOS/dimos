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

from dimos.core.skill_module import SkillModule
from dimos.protocol.skill.skill import skill


class DemoCalculatorSkill(SkillModule):
    def start(self) -> None:
        super().start()

    def stop(self) -> None:
        super().stop()

    @skill()
    def sum_numbers(self, n1: int, n2: int, *args: int, **kwargs: int) -> str:
        """This skill adds two numbers. Always use this tool. Never add up numbers yourself.

        Example:

            sum_numbers(100, 20)

        Args:
            sum (str): The sum, as a string. E.g., "120"
        """

        return f"{int(n1) + int(n2)}"


demo_calculator_skill = DemoCalculatorSkill.blueprint

__all__ = ["DemoCalculatorSkill", "demo_calculator_skill"]

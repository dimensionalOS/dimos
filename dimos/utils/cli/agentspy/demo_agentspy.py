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

"""Demo script that runs skills in the background while agentspy monitors them."""

import time
import threading
from dimos.protocol.skill.agent_interface import AgentInterface
from dimos.protocol.skill.skill import SkillContainer, skill


class DemoSkills(SkillContainer):
    @skill()
    def count_to(self, n: int) -> str:
        """Count to n with delays."""
        for i in range(n):
            time.sleep(0.5)
        return f"Counted to {n}"

    @skill()
    def compute_fibonacci(self, n: int) -> int:
        """Compute nth fibonacci number."""
        if n <= 1:
            return n
        a, b = 0, 1
        for _ in range(2, n + 1):
            time.sleep(0.1)  # Simulate computation
            a, b = b, a + b
        return b

    @skill()
    def simulate_error(self) -> None:
        """Skill that always errors."""
        time.sleep(0.3)
        raise RuntimeError("Simulated error for testing")

    @skill()
    def quick_task(self, name: str) -> str:
        """Quick task that completes fast."""
        time.sleep(0.1)
        return f"Quick task '{name}' done!"


def run_demo_skills():
    """Run demo skills in background."""
    # Create and start agent interface
    agent_interface = AgentInterface()
    agent_interface.start()

    # Register skills
    demo_skills = DemoSkills()
    agent_interface.register_skills(demo_skills)

    # Run various skills periodically
    def skill_runner():
        counter = 0
        while True:
            time.sleep(2)

            # Run different skills based on counter
            if counter % 4 == 0:
                demo_skills.count_to(3, skillcall=True)
            elif counter % 4 == 1:
                demo_skills.compute_fibonacci(10, skillcall=True)
            elif counter % 4 == 2:
                demo_skills.quick_task(f"task-{counter}", skillcall=True)
            else:
                try:
                    demo_skills.simulate_error(skillcall=True)
                except:
                    pass  # Expected to fail

            counter += 1

    # Start skill runner in background
    thread = threading.Thread(target=skill_runner, daemon=True)
    thread.start()

    print("Demo skills running in background. Start agentspy in another terminal to monitor.")
    print("Run: agentspy")

    # Keep running
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nDemo stopped.")


if __name__ == "__main__":
    run_demo_skills()

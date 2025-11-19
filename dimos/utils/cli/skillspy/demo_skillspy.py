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

import threading
import time

from dimos.protocol.skill.coordinator import SkillCoordinator
from dimos.protocol.skill.skill import SkillContainer, skill


class DemoSkills(SkillContainer):
    @skill()
    def count_to(self, n: int) -> str:
        """Count to n with delays."""
        for _i in range(n):
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


def run_demo_skills() -> None:
    """Run demo skills in background."""
    # Create and start agent interface
    agent_interface = SkillCoordinator()
    agent_interface.start()

    # Register skills
    demo_skills = DemoSkills()
    agent_interface.register_skills(demo_skills)

    # Run various skills periodically
    def skill_runner() -> None:
        counter = 0
        while True:
            time.sleep(2)

            # Generate unique call_id for each invocation
            call_id = f"demo-{counter}"

            # Run different skills based on counter
            if counter % 4 == 0:
                # Run multiple count_to in parallel to show parallel execution
                agent_interface.call_skill(f"{call_id}-count-1", "count_to", {"args": [3]})
                agent_interface.call_skill(f"{call_id}-count-2", "count_to", {"args": [5]})
                agent_interface.call_skill(f"{call_id}-count-3", "count_to", {"args": [2]})
            elif counter % 4 == 1:
                agent_interface.call_skill(f"{call_id}-fib", "compute_fibonacci", {"args": [10]})
            elif counter % 4 == 2:
                agent_interface.call_skill(
                    f"{call_id}-quick", "quick_task", {"args": [f"task-{counter}"]}
                )
            else:
                agent_interface.call_skill(f"{call_id}-error", "simulate_error", {})

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

    agent_interface.stop()


if __name__ == "__main__":
    run_demo_skills()

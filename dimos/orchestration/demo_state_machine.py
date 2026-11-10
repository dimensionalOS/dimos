# Copyright 2026 Dimensional Inc.
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

"""A toy state machine and the fake worker it drives, with no robot."""

import asyncio
from concurrent.futures import Future
from pathlib import Path
from typing import Literal

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.orchestration.module import check_state_machines, state_machine


class DemoWorkerConfig(ModuleConfig):
    count_seconds: float = 2.0
    count_outcome: Literal["succeed", "fail", "hang"] = "succeed"
    tick_seconds: float = 1.0


class DemoWorker(Module):
    """A fake provider that serves the demo's call and action and feeds its count input."""

    config: DemoWorkerConfig

    count: Out[Int32]
    action_result: Out[GoalStatus]

    _counts: dict[str, Future[None]]

    @rpc
    def start(self) -> None:
        super().start()
        self._counts = {}
        self.spawn(self._tick())

    @rpc
    def add(self, a: int, b: int) -> int:
        """Add two numbers."""
        return a + b

    @rpc
    def count_to(self, goal_id: str, target: int) -> None:
        """Start counting to target, reporting the outcome on action_result."""
        if self.config.count_outcome != "hang":
            self._counts[goal_id] = self.spawn(self._finish(goal_id, target))

    @rpc
    def cancel_action(self, goal_id: str) -> None:
        """Stop a count and report it as preempted."""
        count = self._counts.pop(goal_id, None)
        if count is not None:
            count.cancel()
        self._report(goal_id, GoalStatus.PREEMPTED, "cancelled")

    async def _tick(self) -> None:
        value = 0
        while True:
            self.count.publish(Int32(value))
            value = (value + 1) % 10
            await asyncio.sleep(self.config.tick_seconds)

    async def _finish(self, goal_id: str, target: int) -> None:
        await asyncio.sleep(self.config.count_seconds)
        self._counts.pop(goal_id, None)
        if self.config.count_outcome == "fail":
            self._report(goal_id, GoalStatus.ABORTED, f"lost count before {target}")
        else:
            self._report(goal_id, GoalStatus.SUCCEEDED, f"counted to {target}")

    def _report(self, goal_id: str, status: int, text: str) -> None:
        self.action_result.publish(GoalStatus(goal_id=GoalID(id=goal_id), status=status, text=text))


demo_state_machine = autoconnect(
    state_machine(Path(__file__).with_suffix(".toml")),
    DemoWorker.blueprint(),
)
check_state_machines(demo_state_machine)

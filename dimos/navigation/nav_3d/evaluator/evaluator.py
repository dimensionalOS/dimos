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

"""Evaluate path planner modules with a set of loaded scenes.

Sends out global map, start pose, goal pose, and listens for
paths. Then evaluate each path for various metrics.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncGenerator
from dataclasses import dataclass
import time
from typing import Any

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_3d.evaluator.scenarios import (
    PlannerScenario,
    default_scenarios,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class ScenarioResult:
    name: str
    expected_path: bool
    got_path: bool
    passed: bool


class EvaluatorConfig(ModuleConfig):
    input_publish_delay: float = 0.2
    # Max seconds to wait for the planner's path reply per scenario.
    path_timeout: float = 2.0
    # Pause between scenes
    scenario_dwell: float = 2.0


class Evaluator(Module):
    """Drives a fixed scenario sequence through a black-box planner."""

    config: EvaluatorConfig

    global_map: Out[PointCloud2]
    start_pose: Out[PoseStamped]
    goal_pose: Out[PoseStamped]
    path: In[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_path: Path | None = None
        self._path_received: asyncio.Event | None = None
        self._eval_task: asyncio.Task[None] | None = None

    async def main(self) -> AsyncGenerator[None, None]:
        self._path_received = asyncio.Event()
        self._eval_task = asyncio.create_task(self._run_eval())
        yield
        if self._eval_task is not None and not self._eval_task.done():
            self._eval_task.cancel()
            try:
                await self._eval_task
            except asyncio.CancelledError:
                pass

    async def handle_path(self, msg: Path) -> None:
        self._latest_path = msg
        if self._path_received is not None:
            self._path_received.set()

    async def _run_eval(self) -> None:
        scenarios = default_scenarios()
        results: list[ScenarioResult] = []
        logger.info("Evaluator starting", scenarios=len(scenarios))
        await asyncio.sleep(1.0)
        for scenario in scenarios:
            result = await self._run_one(scenario)
            results.append(result)
            await asyncio.sleep(self.config.scenario_dwell)
        self._log_summary(results)

    async def _run_one(self, scenario: PlannerScenario) -> ScenarioResult:
        logger.info("Scenario start", name=scenario.name, expect_path=scenario.expect_path)
        assert self._path_received is not None

        now = time.time()
        scenario.global_map.ts = now
        scenario.start_pose.ts = now
        scenario.goal_pose.ts = now

        self.global_map.publish(scenario.global_map)
        await asyncio.sleep(self.config.input_publish_delay)
        self.start_pose.publish(scenario.start_pose)
        await asyncio.sleep(self.config.input_publish_delay)
        self.goal_pose.publish(scenario.goal_pose)

        self._latest_path = None
        self._path_received.clear()

        try:
            await asyncio.wait_for(self._path_received.wait(), timeout=self.config.path_timeout)
            got_path = self._latest_path is not None and len(self._latest_path) > 0
        except asyncio.TimeoutError:
            got_path = False

        passed = got_path == scenario.expect_path
        logger.info(
            "Scenario result",
            name=scenario.name,
            expected=scenario.expect_path,
            got=got_path,
            passed=passed,
        )
        return ScenarioResult(
            name=scenario.name,
            expected_path=scenario.expect_path,
            got_path=got_path,
            passed=passed,
        )

    def _log_summary(self, results: list[ScenarioResult]) -> None:
        n_pass = sum(1 for r in results if r.passed)
        logger.info("Evaluation complete", passed=n_pass, total=len(results))
        for r in results:
            logger.info(
                "  " + ("PASS" if r.passed else "FAIL"),
                scenario=r.name,
                expected=r.expected_path,
                got=r.got_path,
            )

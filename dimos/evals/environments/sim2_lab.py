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

"""A deterministic physical task using the existing EvalRunner lifecycle.

The environment advances simulation time during actions, so regression tests
need no wall-clock sleeps or LLM credentials. The private oracle is written on
stop; agent_artifacts stays empty. Interactive use is the sim2-lab blueprint.
"""

from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np

from dimos.evals.environments.base import Environment, EnvironmentConfig
from dimos.evals.types import RunningEnvironment
from dimos.robot.sim2_lab.model import lab_world
from dimos.sim2.client import SimRobotClient
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.runtime import Runtime


class LabEvalConfig(EnvironmentConfig):
    oracle_path: Path


class LabEnvironment(Environment):
    config: LabEvalConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._runtime: Runtime | None = None
        self._rows: list[list[float]] = []

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        if modules:
            raise ValueError("offline lab evaluation accepts a scripted subject, no added modules")
        self._runtime = Runtime(MujocoBackend(lab_world()))
        self._client = SimRobotClient(("left/slide_x",), self._runtime.submit)
        self._box = self._runtime.backend.layout.bodies.index("box")
        self._rows.clear()
        self._last_x = 0.0
        self._resources.callback(self._record)
        return RunningEnvironment(
            mcp_url="", streams=(), artifacts={"oracle": self.config.oracle_path}
        )

    def act(self, x: float) -> None:
        """Public robot action, then advance 100 ms. Does not return private scene truth."""
        if self._runtime is None:
            raise RuntimeError("environment not started")
        self._last_x = x
        self._client.observe(self._runtime.snapshot())
        self._client.write({"left/slide_x": x})
        self._runtime.advance(50)
        state = self._runtime.snapshot()
        self._rows.append([state.sim_time, *state.poses[self._box, :3]])

    def settle(self, budget_s: float) -> None:
        if budget_s <= 0:
            return
        for _ in range(10):
            self.act(self._last_x)

    def _record(self) -> None:
        self.config.oracle_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(self.config.oracle_path, box=np.asarray(self._rows))
        self._runtime = None

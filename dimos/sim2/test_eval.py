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

"""The existing EvalRunner grades real physical outcomes on a private test world."""

from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from dimos.evals.agents.base import EvalSubject
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.base import Environment, EnvironmentConfig
from dimos.evals.runner import EvalRunner
from dimos.evals.types import EvalCase, Outcome, RunningEnvironment, Trajectory
from dimos.sim2.client import SimRobotClient
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.runtime import Runtime
from dimos.sim2.types import World


class PushEvalConfig(EnvironmentConfig):
    oracle_path: Path


class PushEnvironment(Environment):
    config: PushEvalConfig

    def __init__(self, world: World, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._world = world
        self._runtime: Runtime | None = None
        self._rows: list[list[float]] = []

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        if modules:
            raise ValueError(
                "offline fixture evaluation accepts a scripted subject, no added modules"
            )
        self._runtime = Runtime(MujocoBackend(self._world))
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


def box_in_goal(outcome: Outcome) -> float:
    """Require the box to stay on the floor inside the target for the last .5 seconds."""
    with np.load(outcome.artifacts["oracle"]) as data:
        tail = data["box"][-5:]
        if len(tail) < 5:
            return 0.0
        return float(
            np.all(
                (tail[:, 1] > 0.15)
                & (tail[:, 1] < 0.8)
                & (np.abs(tail[:, 2]) < 0.2)
                & (tail[:, 3] > 0.03)
                & (tail[:, 3] < 0.08)
            )
        )


class ScriptedPusher(EvalSubject):
    def __init__(self, action: Callable[[float], None], push: bool) -> None:
        super().__init__()
        self._action = action
        self._push = push

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        for step in range(40):
            self._action(min(0.65, step * 0.02) if self._push else 0.0)
        trajectory = TrajectoryBuilder(inputs, name="scripted-pusher", model="none")
        return trajectory.build("answer")


def test_actual_physical_outcome_grades_push_and_noop_differently(tmp_path, world):
    scores = []
    for name, push in (("push", True), ("do_nothing", False)):
        environment = PushEnvironment(world, oracle_path=tmp_path / name / "oracle.npz")
        case = EvalCase(
            id=name,
            inputs="Push the orange box into the +X goal region.",
            environment=environment,
            grade=box_in_goal,
        )
        runner = EvalRunner(out_dir=tmp_path / name)
        result = runner.run([case], ScriptedPusher(environment.act, push))[0]
        assert not result.error
        scores.append(result.score)
    assert scores == [1.0, 0.0]

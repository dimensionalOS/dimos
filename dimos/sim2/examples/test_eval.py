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

from dimos_lcm.control_msgs import JointCommand, JointTarget
import numpy as np

from dimos.evals.agents.base import EvalSubject
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.base import Environment, EnvironmentConfig
from dimos.evals.runner import EvalRunner
from dimos.evals.types import EvalCase, Outcome, RunningEnvironment, Trajectory
from dimos.sim2.description import SceneConfig
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.physics.simulation import Simulation, SimulationConfig


class PushEvalConfig(EnvironmentConfig):
    oracle_path: Path


class PushEnvironment(Environment):
    config: PushEvalConfig

    def __init__(self, world: SceneConfig, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._world = world
        self._runtime: Simulation | None = None
        self._rows: list[list[float]] = []

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        if modules:
            raise ValueError(
                "offline fixture evaluation accepts a scripted subject, no added modules"
            )
        self._runtime = Simulation(MujocoBackend(), self._world, SimulationConfig())
        self._runtime.load()
        self._resources.callback(self._runtime.backend.close)
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
        sim = self._runtime
        sim.submit(
            JointCommand(
                header=sim.header("eval"),
                based_on_time_ns=sim.time_ns,
                apply_at_ns=sim.time_ns,
                valid_until_ns=sim.time_ns + 200_000_000,
                targets_length=1,
                targets=[JointTarget(joint_id="left/slide_x", position=x, kp=200, kd=20)],
            )
        )
        self._runtime.advance(50)
        pose = poses_by_id(sim.inspect()[0])["box"].position
        self._rows.append([sim.time_ns / 1e9, pose.x, pose.y, pose.z])

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

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

"""Physical pass/fail baseline; uses EvalRunner, not a second simulation eval runner.

Run: python -m dimos.evals.suites.sim2_lab --out /tmp/sim2-lab-eval
"""

import argparse
from collections.abc import Callable
from pathlib import Path

import numpy as np

from dimos.evals.agents.base import EvalSubject
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.sim2_lab import LabEnvironment
from dimos.evals.runner import EvalRunner
from dimos.evals.types import EvalCase, Outcome, RunningEnvironment, Trajectory


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


def run_baselines(out: Path) -> list[float]:
    scores = []
    for name, push in (("push", True), ("do_nothing", False)):
        environment = LabEnvironment(oracle_path=out / name / "oracle.npz")
        case = EvalCase(
            id=name,
            inputs="Push the orange box into the +X goal region.",
            environment=environment,
            grade=box_in_goal,
        )
        runner = EvalRunner(out_dir=out / name)
        result = runner.run([case], ScriptedPusher(environment.act, push))[0]
        if result.error:
            raise RuntimeError(result.error)
        print(f"{name}: score={result.score:g}, passed={result.passed}, artifacts={runner.run_dir}")
        scores.append(result.score)
    return scores


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args()
    if run_baselines(arguments.out) != [1.0, 0.0]:
        raise SystemExit("unexpected physical baseline outcome")

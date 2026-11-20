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

"""RoboLab-120 (NVIDIA RoboLab v0.3.3), the DROID robot driven by dimos.

    dimos evals run dimos.evals.suites.robolab --agent dimos.evals.agents.dimos_agent --tags simple

One case per RoboLab benchmark task, ``robolab_<task>``, tagged with RoboLab's difficulty
(simple, moderate, complex), its attributes (semantics, spatial, ...) and its scene. The
prompt is the task's default instruction. RoboLab runs the task in Isaac Sim and judges it
by its own predicates (see dimos.evals.environments.robolab); a case passes when RoboLab
records success.

``robolab_tasks.json`` lists the tasks, from RoboLab's
``robolab/tasks/_metadata/task_metadata.json``: name, default instruction, episode length
(sim seconds), scene, attributes and difficulty.

RoboLab's time limit is in sim time and the sim pauses while dimos is idle, so it is the
leaderboard's. The wall-clock limit per case covers the agent (20 min), the episode run
~3.5x slower than real time on an L4, and starting Isaac.
"""

from __future__ import annotations

import json
from pathlib import Path
import re

from dimos.evals.environments.robolab import RoboLabEnvironment
from dimos.evals.types import EvalCase, Outcome, Suite

TASKS = Path(__file__).with_name("robolab_tasks.json")
BLUEPRINT = ["droid-robolab-sim", "droid-robolab-graspgenx"]
GUIDANCE = (
    "You control a Franka Panda arm with a Robotiq parallel gripper, and a fixed RGB-D camera "
    "over the robot's left shoulder. Find objects with the localize module, get grasps for an "
    "object's point cloud from a grasp module, and move the arm and gripper through the "
    "manipulation module. If localize finds nothing, name the object by colour and shape, "
    "e.g. 'green cylinder'. Use only the camera and these modules. Do not read RoboLab's files "
    "or results, the recordings (memory.db), or any files under data/."
)
AGENT_S = 1200.0
STARTUP_S = 300.0
SIM_SLOWDOWN = 3.5


def succeeded(outcome: Outcome) -> float:
    """1.0 when RoboLab recorded success for the episode."""
    rows = outcome.artifacts["robolab_results"].read_text().splitlines()
    return 1.0 if rows and json.loads(rows[-1]).get("success") else 0.0


def case_id(task: str) -> str:
    """``BananaInBowlTask`` -> ``robolab_banana_in_bowl``."""
    name = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", task.removesuffix("Task")).lower()
    return f"robolab_{name}"


def _case(task: dict[str, object]) -> EvalCase:
    name = str(task["task"])
    episode_s = float(task["episode_s"])  # type: ignore[arg-type]
    attributes = [str(a) for a in task["attributes"]]  # type: ignore[attr-defined]
    return EvalCase(
        id=case_id(name),
        inputs=f"{str(task['instruction']).rstrip('. ')}. {GUIDANCE}",
        environment=RoboLabEnvironment(blueprint=BLUEPRINT, task=name),
        grade=succeeded,
        timeout_s=AGENT_S + SIM_SLOWDOWN * episode_s + STARTUP_S,
        # Its own id too, so --tags can pick out one case.
        tags=frozenset(
            {
                "robolab",
                "droid",
                "manipulation",
                case_id(name),
                str(task["difficulty"]),
                f"scene_{task['scene']}",
                *attributes,
            }
        ),
    )


SUITE: Suite = [_case(task) for task in json.loads(TASKS.read_text())]

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

"""LIBERO / LIBERO-PRO tasks as live evals: LIBERO runs the task, dimos's arm stack works it.

``panda-libero-sim`` launches ``LiberoSim``, which has LIBERO build the BDDL task with its
own Panda and publishes LIBERO's own goal check on ``task_status``. That topic is recorded
with the rest, and ``libero_success`` grades from it.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.evals.environments.mujoco_sim import MujocoEnvironment, MujocoEnvironmentConfig
from dimos.evals.types import recording

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.evals.types import Outcome


class LiberoEnvironmentConfig(MujocoEnvironmentConfig):
    blueprint: list[str] = ["panda-libero-sim", "mcp-server", "observe-skill"]
    bddl: Path
    seed: int = 0
    recorded_topics: tuple[str, ...] = (
        "color_image",
        "camera_info",
        "coordinator_joint_state",
        "tf",
        "task_status",
    )


class LiberoEnvironment(MujocoEnvironment):
    """One LIBERO task, run live by LIBERO with dimos's manipulation stack driving the Panda."""

    config: LiberoEnvironmentConfig

    def configure_launch(self, proc: DimosCliCall) -> None:
        proc.simulator = None  # LiberoSim is the simulator; there is no --simulation backend
        proc.global_args = ["--record-topics", ",".join(self.config.recorded_topics)]
        proc.extra_env.update(self.config.module_env)
        proc.extra_env["LIBEROSIM__BDDL"] = str(self.config.bddl.resolve())
        proc.extra_env["LIBEROSIM__SEED"] = str(self.config.seed)


def task_statuses(outcome: Outcome) -> list[dict[str, Any]]:
    """Every ``task_status`` LIBERO published during the run, oldest first."""
    with recording(outcome) as store:
        if "task_status" not in store.streams:
            return []
        return [json.loads(record.data.data) for record in store.streams.task_status.order_by("ts")]


def libero_success(outcome: Outcome) -> float:
    """1.0 once LIBERO judged the goal met, as LIBERO's own episodes end on first success;
    otherwise the fraction of goal predicates that held at the end."""
    statuses = task_statuses(outcome)
    if not statuses:
        return 0.0
    if any(status["success"] for status in statuses):
        return 1.0
    predicates = statuses[-1]["predicates"]
    return sum(bool(p[-1]) for p in predicates) / len(predicates) if predicates else 0.0

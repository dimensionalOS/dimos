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

"""TypeSafe navigation cases in a Habitat house: the DimSim suite's rubric on a
Habitat scene file.

A scene file is a ground-truth snapshot of the house in the layout
``dimos.evals.agents.typesafe_policy.load_scene`` reads (labelled
``center_xyz`` / ``size_xyz`` boxes in the ROS world frame), the same one the
Habitat navigation benchmark ships. The agent needs it as
``--set scene_json=<the same file>``.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.suites.typesafe_nav import reached
from dimos.evals.types import EvalCase, Suite

SCENES = Path(__file__).parent.parent / "scenes" / "habitat"
HABITAT_DATA = DIMOS_PROJECT_ROOT / "target" / "habitat" / "data"

# Habitat's twist integration is slower than DimSim's, and the houses are bigger.
TIMEOUT_S = 240.0


def habitat_suite(
    prefix: str,
    scene_json: Path,
    scene_id: str,
    spawn_xyz: tuple[float, float, float],
    spawn_yaw_deg: float,
    cases: Sequence[tuple[str, str]],
    *,
    scene_dataset_config: str | None = None,
) -> Suite:
    """One case per ``(goal word, instruction)``; the goal word must name exactly
    one object of the scene file, as the agent resolves it from the instruction.

    ``scene_dataset_config`` ``None`` is the HM3D example house that
    ``dimos run habitat-teleop`` installs; HSSD scenes name theirs.
    """

    def environment() -> HabitatEnvironment:
        # A fresh environment per case: each owns its own launched sim.
        return HabitatEnvironment(
            # No skill container: MCP comes up with zero tools exposed.
            blueprint=["habitat-teleop", "mcp-server"],
            scene_dataset_config=scene_dataset_config,
            scene_id=scene_id,
            start_position_ros_override=spawn_xyz,
            start_yaw_deg=spawn_yaw_deg,
        )

    return [
        EvalCase(
            id=f"{prefix}_{goal}",
            inputs=instruction,
            environment=environment(),
            grade=reached(goal, scene_json),
            timeout_s=TIMEOUT_S,
            threshold=0.6,
            tags=frozenset({"typesafe", "navigation", "habitat", scene_id, goal}),
        )
        for goal, instruction in cases
    ]

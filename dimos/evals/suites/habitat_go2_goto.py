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

"""Point-goal navigation for a Go2 in one HSSD house, driven through the raw robot bridge.

The environment hands the agent the scene's ground-truth navmesh map, with the goal on it,
as its ``nav_map`` artifact (made by ``misc/habitat/navmesh_map.py``). The robot spawns in
the open living area facing along the drivable route: 7.8 m nearly straight along open
floor, then one turn through the gap beside a wall, 10.5 m in all.

HSSD_DATASET_CONFIG=~/Documents/habitat-sim/data/hssd-hab/hssd-hab.scene_dataset_config.json \\
    dimos evals run dimos.evals.suites.habitat_go2_goto --agent dimos.evals.agents.decisions_nav
"""

import json
import math
import os
from pathlib import Path
from typing import Any

from dimos.evals.constants import RAW_README
from dimos.evals.environments.habitat import HabitatEnvironment, HabitatEnvironmentConfig
from dimos.evals.types import EvalCase, Outcome, Suite, recording
from dimos.utils.data import get_data_dir

MAPS = Path(__file__).parent / "maps"
SCENE = "106366410_174226806"
ARRIVE_M = 0.75


class GotoHabitatEnvironmentConfig(HabitatEnvironmentConfig):
    nav_map: Path
    agent_artifacts: tuple[str, ...] = ("nav_map",)  # the map, never the recording


class GotoHabitatEnvironment(HabitatEnvironment):
    """Habitat with a ground-truth map and goal handed to the agent as ``nav_map``."""

    config: GotoHabitatEnvironmentConfig

    def prepare_recording(self, recording: Any, path: Path, deadline: float) -> dict[str, Path]:
        return {
            **super().prepare_recording(recording, path, deadline),
            "nav_map": self.config.nav_map,
        }


def _spec(scene: str) -> dict[str, Any]:
    spec: dict[str, Any] = json.loads((MAPS / f"{scene}.navmesh.json").read_text())
    return spec


def _environment(scene: str) -> GotoHabitatEnvironment:
    spec = _spec(scene)
    (sx, sy, sz), (nx, ny, _) = spec["spawn_ros"], spec["reference_path_ros"][1]
    along_route = math.degrees(math.atan2(ny - sy, nx - sx))  # no turn needed to start
    return GotoHabitatEnvironment(
        scene_dataset_config=os.environ.get(
            "HSSD_DATASET_CONFIG", str(get_data_dir("hssd-hab/hssd-hab.scene_dataset_config.json"))
        ),
        scene_id=scene,
        start_position_ros_override=(sx, sy, sz),
        start_yaw_deg=along_route,
        blueprint=["habitat-teleop", "mcp-server"],
        raw_bridge=True,
        raw_guide=RAW_README,
        nav_map=MAPS / f"{scene}.navmesh.json",
    )


def reached(scene: str) -> Any:
    """1.0 within ARRIVE_M of the goal; otherwise the fraction of the straight gap closed."""
    spec = _spec(scene)
    gx, gy = spec["goal_ros"][:2]
    start = math.dist(spec["spawn_ros"][:2], (gx, gy))

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            if "odometry" not in store.streams:
                return 0.0
            end = store.streams.odometry.last().data
        remaining = math.dist((end.x, end.y), (gx, gy))
        if remaining <= ARRIVE_M:
            return 1.0
        return max(0.0, min(0.99, 1.0 - remaining / start))

    return grade


SUITE: Suite = [
    EvalCase(
        id=f"hssd_{SCENE}_go2_goto",
        inputs="Drive the robot to the red goal on the map.",
        environment=_environment(SCENE),
        grade=reached(SCENE),
        threshold=1.0,
        timeout_s=900.0,
        tags=frozenset({"habitat", "hssd", "navigation", "point_goal", "raw"}),
    ),
]

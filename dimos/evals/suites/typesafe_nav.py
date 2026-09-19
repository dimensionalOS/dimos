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

"""Point-goal navigation driven by a TypeSafe (Jev) policy in DimSim.

The blueprint carries no skill container, so MCP exposes zero tools: the only
way the agent affects the world is the Twist it publishes. The goal is named by
a word of each case's instruction that occurs in the scene label; the agent and
the grader resolve it the same way.

    dimos evals run dimos.evals.suites.typesafe_nav \
        --agent dimos.evals.agents.typesafe_policy \
        --set scene_json=dimos/evals/suites/scenes/apartment_detections.json \
        --tags bathtub          # or: couch; omit to run every case

``apartment_detections.json`` is DimSim's ground-truth snapshot of the
apartment: objects and walls with real extents, in the ROS world frame odometry
uses. Copied verbatim from PR #4208 (``misc/DimSim/scenes/apartment/
object_detections.json`` at 7fd0e2f17); regenerate with
``SceneClient.export_object_detections`` once that lands.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

from dimos.evals.agents.typesafe_policy import Box2D, load_scene
from dimos.evals.environments.dimsim import DimSimEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording

SCENE = Path(__file__).parent / "scenes" / "apartment_detections.json"
# Matches the DimSim-native go-to-couch rubric (objectDistance thresholdM: 2.0).
ARRIVAL_BAND_M = 2.0
MIN_TRAVEL_M = 0.1

# (case id, instruction, the word of the instruction that names the goal)
CASES: tuple[tuple[str, str, str], ...] = (
    # Same room as the spawn; one detour around the coffee table.
    ("typesafe_nav_couch", "navigate to the sectional couch", "sectional"),
    # In the bathroom: out through the main doorway at (0, 0), then in through
    # the bathroom door near (-2.5, 1). Twice the robot must move away from the
    # goal to reach it, which greedy bearing cannot do.
    ("typesafe_nav_bathtub", "go to the bathtub", "bathtub"),
)


def goal_box(goal_label: str, scene: Path = SCENE) -> Box2D:
    """The goal's real footprint (minx, miny, maxx, maxy) from the scene file."""
    return load_scene(scene, goal_label).goal_box


def distance_to_box(x: float, y: float, box: Box2D) -> float:
    """Euclidean distance from a point to an axis-aligned box; 0 inside."""
    dx = max(box[0] - x, 0.0, x - box[2])
    dy = max(box[1] - y, 0.0, y - box[3])
    return (dx * dx + dy * dy) ** 0.5


def positions(store: Any) -> list[Any]:
    """The recorded robot positions: DimSim records PoseStamped on ``odom``,
    Habitat nav_msgs Odometry on ``odometry``; both carry ``position``."""
    for name in ("odom", "odometry"):
        if name in store.streams:
            return [entry.data.position for entry in getattr(store.streams, name)]
    return []


def reached(goal_label: str, scene: Path = SCENE) -> Callable[[Outcome], float]:
    """Where it stopped (70%) and how directly it got there (30%).

    Arrival is measured to the goal's box edge: the centre is inside the object
    and unreachable.
    """

    def grade(outcome: Outcome) -> float:
        box = goal_box(goal_label, scene)
        with recording(outcome) as store:
            poses = positions(store)
        if not poses:
            raise LookupError("no odometry recorded")
        start, end = poses[0], poses[-1]
        arrival = ramp(distance_to_box(end.x, end.y, box), band=ARRIVAL_BAND_M)
        travelled = sum((poses[i + 1] - poses[i]).length() for i in range(len(poses) - 1))
        ideal = distance_to_box(start.x, start.y, box)
        # A robot that never moved has no path to be direct about; odom jitter
        # must not turn 1e-9 m of travel into full directness credit.
        directness = min(1.0, ideal / travelled) if travelled >= MIN_TRAVEL_M else 0.0
        return 0.7 * arrival + 0.3 * directness

    return grade


def _apartment() -> DimSimEnvironment:
    # A fresh environment per case: each owns its own launched sim and resources.
    return DimSimEnvironment(
        # No skill container: MCP comes up with zero tools exposed.
        blueprint=["unitree-go2", "mcp-server"],
        scene="apartment",
    )


SUITE: Suite = [
    EvalCase(
        id=case_id,
        inputs=instruction,
        environment=_apartment(),
        grade=reached(goal),
        timeout_s=180.0,
        threshold=0.6,
        tags=frozenset({"typesafe", "navigation", "dimsim", case_id.removeprefix("typesafe_nav_")}),
    )
    for case_id, instruction, goal in CASES
]

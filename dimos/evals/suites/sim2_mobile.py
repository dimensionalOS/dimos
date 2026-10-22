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

"""G1/Go2 navigation cases. One environment and rubric for scripted and model agents."""

from __future__ import annotations

from functools import partial
import math
from pathlib import Path
from typing import Any, Literal

from dimos.evals.environments.sim2 import Sim2Environment, Sim2EnvironmentConfig, scene_evidence
from dimos.evals.types import EvalCase, Outcome
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.sim2.world.scene_types import SceneState, SceneUpdate

Kind = Literal["reach", "replan", "search"]
SCENE = Path(__file__).parent / "scenes/mobile.xml"


class MobileConfig(Sim2EnvironmentConfig):
    robot_id: str
    task: Kind


class MobileEnvironment(Sim2Environment):
    config: MobileConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._changed = False
        self._near_since: float | None = None

    def setup_scene(self) -> None:
        self._changed = False
        self._near_since = None
        super().setup_scene()

    def on_state(self, state: SceneState) -> None:
        if self._changed:
            return
        robot = state.robots[self.config.robot_id].position
        if self.config.task == "replan" and robot.x > -1.6:
            self.intervene(
                SceneUpdate(poses={"obstacle": Pose((0, 0, 0.65))}), reason="block route"
            )
            self._changed = True
        elif self.config.task == "search":
            target = state.entities["target"].pose.position
            if math.hypot(robot.x - target.x, robot.y - target.y) > 1.25:
                self._near_since = None
            elif self._near_since is None:
                self._near_since = state.sim_time
            elif state.sim_time - self._near_since >= 0.5:
                self.intervene(
                    SceneUpdate(poses={"target": Pose((1.5, 1.2, 0.35))}), reason="relocate target"
                )
                self._changed = True


def grade_mobile(outcome: Outcome, *, robot_id: str, task: Kind) -> float:
    with SqliteStore(path=str(outcome.artifacts["truth"]), must_exist=True) as store:
        states, edits = scene_evidence(store)
    if task == "reach" and edits:
        raise ValueError("reach case must not mutate the scene")
    if task != "reach":
        if len(edits) != 1:
            return 0.0
        expected = "block route" if task == "replan" else "relocate target"
        if edits[0].reason != expected:
            raise ValueError("unexpected task intervention")
    since: float | None = None
    minimum_height = 0.5 if robot_id == "g1connection" else 0.18
    for state in states:
        pose = state.robots[robot_id]
        if pose.position.z < minimum_height:
            return 0.0
        # Sampled contact evidence, not a claim to detect every transient impact.
        if any(
            (a.startswith(robot_id + "/") and b in {"walls", "obstacle", "target"})
            or (b.startswith(robot_id + "/") and a in {"walls", "obstacle", "target"})
            for a, b in state.contacts
        ):
            return 0.0
        if edits and state.generation != edits[0].after.generation:
            continue
        goal = (
            state.entities["target"].pose.position
            if task == "search"
            else state.regions["goal"].pose.position
        )
        tolerance = 1.25 if task == "search" else 0.35
        near = math.hypot(pose.position.x - goal.x, pose.position.y - goal.y) <= tolerance
        since = (state.sim_time if since is None else since) if near else None
    return float(since is not None and states[-1].sim_time - since >= 1.0)


def mobile_case(robot: Literal["g1", "go2"], task: Kind) -> EvalCase:
    robot_id = "g1connection" if robot == "g1" else "go2connection"
    blueprint = "unitree-g1-groot-wbc" if robot == "g1" else "unitree-go2"
    instruction = (
        "Find the green box using the camera and approach to about one metre without touching it. "
        "It will be relocated after your first approach. Find it again and stay nearby for two seconds."
        if task == "search"
        else "Navigate to world goal (1.7, 0.0) and remain within 0.35 metres for two seconds. "
        + (
            "An obstacle will enter your route; avoid it and continue to the same goal."
            if task == "replan"
            else "Avoid obstacles."
        )
    )
    return EvalCase(
        id=f"sim2_{robot}_{task}",
        inputs=instruction,
        environment=MobileEnvironment(
            blueprint=[blueprint, "mcp-server"],
            scene=SCENE,
            robot_id=robot_id,
            task=task,
            launch_timeout_s=120,
            at_rest_s=2,
        ),
        grade=partial(grade_mobile, robot_id=robot_id, task=task),
        timeout_s=120,
        tags=frozenset({"sim2", robot, task}),
    )


ROBOTS: tuple[Literal["g1", "go2"], ...] = ("g1", "go2")
TASKS: tuple[Kind, ...] = ("reach", "replan", "search")
SUITE = [mobile_case(robot, task) for robot in ROBOTS for task in TASKS]

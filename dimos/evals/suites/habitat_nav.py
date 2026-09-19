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

"""Go to a named object in a Habitat scene; graded on arrival, time, facing, bumps, path.

One scene file per scene under ``scenes/habitat/``: the ground-truth boxes, either
inline as ``detections`` (the ``detection3d_array_to_dict`` layout, ROS world frame)
or by path as ``ground_truth`` (``misc/habitat/ground_truth/...``), the optional
``scene_dataset_config`` and the ``cases`` (label, spawn, end point on the object,
geodesic distance, difficulty); ``misc/habitat/nav_cases.py`` writes one from a
ground-truth file. The planner gets the navmesh as its map (``habitat-nav-gt``), so
nothing drives before the task clock starts. Every arm gets ``go to the <label> at (x, y)``; the
boxes are published by ``demo-objects`` so text-only agents see them in ``world_state``.

    # the planner alone, the end point straight to /goal
    dimos evals run dimos.evals.suites.habitat_nav --agent dimos.evals.agents.topic \\
        --set send=goal --set send_type=point --set done=goal_reached --set done_type=Bool
    # the TypeSafe reactive agent
    dimos evals run dimos.evals.suites.habitat_nav --agent dimos.evals.agents.topic \\
        --set 'modules=["type-safe-agent"]' --set trace=TypeSafeAgent
    # a coding agent with dimOS (go_to / stop / finish tools) or without it (raw topics)
    dimos evals run dimos.evals.suites.habitat_nav --agent dimos.evals.agents.dimcode --set model=gpt-6-astra
    dimos evals run dimos.evals.suites.habitat_nav --agent dimos.evals.agents.pi --set no_dimos=true --set model=gpt-6-astra
"""

from __future__ import annotations

from collections.abc import Callable
import json
import os
from pathlib import Path
import re

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.nav_metrics import (
    box_of,
    read_cmds,
    read_declared,
    read_poses,
    score_navigation,
    write_metrics,
)
from dimos.evals.types import EvalCase, Outcome, Suite, recording

TASK_BRIEF = (
    "You control a mobile robot in a furnished indoor scene. You know the robot's pose and the "
    "world-frame positions of the objects in the scene. Navigate the robot to the target object "
    "named below, driving around obstacles, and declare finished as soon as the robot is beside "
    "it. The last line is the goal; its coordinates are the target's centre in the world frame."
)
SCENES = Path(__file__).parent / "scenes" / "habitat"
BLUEPRINT = ["habitat-nav-gt", "mcp-server", "demo-objects", "nav-skills"]
TIMEOUT_S = float(os.environ.get("DIMOS_EVAL_TIMEOUT_S", 1800))
# What grading and replay need; images and clouds stay out (1 GB per few minutes otherwise).
RECORD_TOPICS = ("odom", "cmd_vel", "goal", "path", "goal_reached", "stop_movement", "finished")
# The depth scan is in base_link with the floor at z = 0; keep the floor out of the sectors.
MODULE_ENV = {
    "TYPESAFEAGENT__LIDAR_BAND": "[0.1, 0.8, 5.0]",
    "RAWROBOTBRIDGE__LIDAR_Z_MIN": "0.1",
}


def grade_nav(
    end_xy: tuple[float, float], box: tuple[float, float, float, float]
) -> Callable[[Outcome], float]:
    def grade(o: Outcome) -> float:
        start = json.loads(o.artifacts["episode"].read_text()).get("task_start_ts", 0.0)
        with recording(o) as store:
            poses = [p for p in read_poses(store) if p[0] >= start]
            cmds = [c for c in read_cmds(store) if c[0] >= start]
            m = score_navigation(poses, cmds, end_xy, box, declared_at=read_declared(store))
        write_metrics(
            m, o.artifacts["recording"].parent / "nav_metrics.json", end_xy=end_xy, box=box
        )
        traced = [s.extra.request for s in o.trajectory.steps if s.extra]
        if traced:  # beside the trajectory too: <run>/<case>/raw/N-request.json
            write_metrics(
                m,
                traced[-1].parent.parent / "nav_metrics.json",
                end_xy=end_xy,
                box=box,
                recording=str(o.artifacts["recording"].parent),
            )
        return m.score()

    return grade


def cases_for(scene_file: Path) -> list[EvalCase]:
    scene = json.loads(scene_file.read_text())
    objects = scene_file
    if "ground_truth" in scene:
        objects = DIMOS_PROJECT_ROOT / scene["ground_truth"]
        if not objects.is_file():  # ground truth ships separately (misc/habitat, PR #4211)
            return []
    detections = json.loads(objects.read_text())["detections"]
    boxes = {d["id"]: box_of(d["center_xyz"], d["size_xyz"]) for d in detections}
    dataset = scene.get("scene_dataset_config")
    out = []
    seen: dict[str, int] = {}
    for c in scene["cases"]:
        x, y = c["end_xy"]
        label = c["label"]
        slug = re.sub(r"[^A-Za-z0-9]+", "_", label).strip("_").lower()
        seen[slug] = seen.get(slug, 0) + 1
        case_id = f"{scene['scene_id']}_{slug}" + (f"_{seen[slug]}" if seen[slug] > 1 else "")
        out.append(
            EvalCase(
                id=case_id,
                inputs=f"{TASK_BRIEF}\n\ngo to the {label} at ({x:.2f}, {y:.2f})",
                environment=HabitatEnvironment(
                    blueprint=BLUEPRINT,
                    scene_id=scene["scene_id"],
                    scene_dataset_config=str(DIMOS_PROJECT_ROOT / dataset) if dataset else None,
                    start_position_ros_override=tuple(c["spawn_xyz"]),
                    start_yaw_deg=c["spawn_yaw_deg"],
                    raw_bridge=True,  # inert unless an agent connects; identical launches per arm
                    raw_topics=("world_state", "cmd_vel", "finished"),
                    record_topics=RECORD_TOPICS,
                    extra_env={"DEMOOBJECTS__SCENE_JSON": str(objects), **MODULE_ENV},
                ),
                grade=grade_nav((x, y), boxes[c["object_id"]]),
                timeout_s=TIMEOUT_S,
                threshold=0.5,  # passed == reached
                tags=frozenset(
                    {"habitat", "nav", scene["scene_id"], label, c.get("difficulty", "")} - {""}
                ),
            )
        )
    return out


SUITE: Suite = [case for f in sorted(SCENES.glob("*.json")) for case in cases_for(f)]

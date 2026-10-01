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
        --set 'modules=["type-safe-navigation-agent"]' --set trace=TypeSafeNavigationAgent
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
import tempfile
from typing import Any

from dimos.agents.typesafe.demo_objects import load_scene_objects
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
from dimos.robot.raw_robot_bridge import dry_run_world_state
from dimos.utils.logging_config import setup_logger

TASK_BRIEF = (
    "You control a mobile robot in a furnished indoor scene. You know the robot's pose and the "
    "world-frame positions of the objects in the scene. Navigate the robot to the target object "
    "named below, driving around obstacles, and declare finished as soon as the robot is beside "
    "it. The last line is the goal; its coordinates are the target's centre in the world frame."
)
logger = setup_logger()

SCENES = Path(__file__).parent / "scenes" / "habitat"
BLUEPRINT = ["habitat-nav-gt", "mcp-server", "demo-objects", "nav-skills"]
TIMEOUT_S = float(os.environ.get("DIMOS_EVAL_TIMEOUT_S", 1800))
# What grading and replay need; images and clouds stay out (1 GB per few minutes otherwise).
RECORD_TOPICS = ("odom", "cmd_vel", "goal", "path", "goal_reached", "stop_movement", "finished")
# The depth scan is in base_link with the floor at z = 0; keep the floor out of the sectors.
MODULE_ENV = {
    # Keys are <module class name>__<field>; test_module_env_names_the_navigation_agent pins the class.
    "TYPESAFENAVIGATIONAGENT__LIDAR_BAND": "[0.1, 0.8, 5.0]",
    "RAWROBOTBRIDGE__LIDAR_Z_MIN": "0.1",
}


STATS_DIR = (
    Path(tempfile.gettempdir()) / "world_state_stats"
)  # the bridge's tick counters, per case


class HabitatNavEnvironment(HabitatEnvironment):
    """Habitat plus a dry run of the world-state builder at the spawn before anything launches:
    a builder that fails there fails the case in a second, not after a blind ten minutes."""

    def preflight(self, agent: Any) -> None:
        super().preflight(agent)
        env = self.config.extra_env
        objects = load_scene_objects(Path(env["DEMOOBJECTS__SCENE_JSON"]))
        spawn = self.config.start_position_ros_override or (0.0, 0.0, 0.0)
        try:
            dry_run_world_state(
                env["RAWROBOTBRIDGE__GOAL"], spawn, self.config.start_yaw_deg, objects
            )
        except Exception as e:
            raise RuntimeError(f"world state builder fails at the spawn: {e!r}") from e


def world_state_check(stats: dict[str, Any], min_ticks: int = 1) -> None:
    """Raise when the bridge produced no world state (a builder fault, or no odometry at all):
    the text-only arms ran blind and the case is an infrastructure error, not a miss."""
    if stats and stats.get("ticks", 0) < min_ticks:
        raise RuntimeError(
            f"no world state was published: {stats.get('ticks', 0)} ticks, "
            f"{stats.get('errors', 0)} builder errors, last {stats.get('last_error', '')!r}"
        )


def grade_nav(
    end_xy: tuple[float, float],
    box: tuple[float, float, float, float],
    stats_path: Path | None = None,
    *,
    geodesic_m: float | None = None,
    reference: list[tuple[float, float]] | None = None,
    walls: list[tuple[float, float, float, float]] | None = None,
) -> Callable[[Outcome], float]:
    def grade(o: Outcome) -> float:
        start = json.loads(o.artifacts["episode"].read_text()).get("task_start_ts", 0.0)
        with recording(o) as store:
            poses = [p for p in read_poses(store) if p[0] >= start]
            cmds = [c for c in read_cmds(store) if c[0] >= start]
            m = score_navigation(
                poses,
                cmds,
                end_xy,
                box,
                declared_at=read_declared(store),
                geodesic_m=geodesic_m,
                reference=reference,
                walls=walls or (),
            )
        if stats_path is None:
            stats: dict[str, Any] = {}
        elif stats_path.exists():
            stats = json.loads(stats_path.read_text())
        else:  # the bridge never wrote its counters: no world state was published at all
            stats = {"ticks": 0, "errors": 0, "last_error": f"no stats file at {stats_path}"}
        write_metrics(
            m,
            o.artifacts["recording"].parent / "nav_metrics.json",
            end_xy=end_xy,
            box=box,
            world_state=stats,
        )
        traced = [s.extra.request for s in o.trajectory.steps if s.extra]
        if traced:  # beside the trajectory too: <run>/<case>/raw/N-request.json
            write_metrics(
                m,
                traced[-1].parent.parent / "nav_metrics.json",
                end_xy=end_xy,
                box=box,
                recording=str(o.artifacts["recording"].parent),
                world_state=stats,
            )
        world_state_check(stats)
        return m.score()

    return grade


def cases_for(scene_file: Path, goal_key: str = "end_xy") -> list[EvalCase]:
    """``goal_key`` names the case field the instruction's coordinates come from: the object's
    centre (``end_xy``) or the navigable point beside it (``end_nav_xy``, the planner arm)."""
    scene = json.loads(scene_file.read_text())
    objects = scene_file
    if "ground_truth" in scene:
        objects = DIMOS_PROJECT_ROOT / scene["ground_truth"]
        if not objects.is_file():  # ground truth ships separately (misc/habitat, PR #4211)
            logger.warning(
                "scene skipped: ground truth missing", scene=scene_file.name, path=str(objects)
            )
            return []
    detections = json.loads(objects.read_text())["detections"]
    boxes = {d["id"]: box_of(d["center_xyz"], d["size_xyz"]) for d in detections}
    walls = [boxes[d["id"]] for d in detections if d["label"] == "wall"]
    dataset = scene.get("scene_dataset_config")
    out = []
    seen: dict[str, int] = {}
    for c in scene["cases"]:
        x, y = c["end_xy"]
        gx, gy = c.get(goal_key, c["end_xy"])
        label = c["label"]
        slug = re.sub(r"[^A-Za-z0-9]+", "_", label).strip("_").lower()
        seen[slug] = seen.get(slug, 0) + 1
        case_id = f"{scene['scene_id']}_{slug}" + (f"_{seen[slug]}" if seen[slug] > 1 else "")
        inputs = f"{TASK_BRIEF}\n\ngo to the {label} at ({gx:.2f}, {gy:.2f})"
        out.append(
            EvalCase(
                id=case_id,
                inputs=inputs,
                environment=HabitatNavEnvironment(
                    blueprint=BLUEPRINT,
                    scene_id=scene["scene_id"],
                    scene_dataset_config=str(DIMOS_PROJECT_ROOT / dataset) if dataset else None,
                    start_position_ros_override=tuple(c["spawn_xyz"]),
                    start_yaw_deg=c["spawn_yaw_deg"],
                    raw_bridge=True,  # inert unless an agent connects; identical launches per arm
                    raw_topics=("world_state", "cmd_vel", "finished"),
                    record_topics=RECORD_TOPICS,
                    # The raw bridge gets the instruction too: text-only arms read one world state.
                    extra_env={
                        "DEMOOBJECTS__SCENE_JSON": str(objects),
                        "RAWROBOTBRIDGE__GOAL": inputs,
                        "RAWROBOTBRIDGE__STATS_PATH": str(STATS_DIR / f"{case_id}.json"),
                        **MODULE_ENV,
                    },
                ),
                grade=grade_nav(
                    (x, y),
                    boxes[c["object_id"]],
                    STATS_DIR / f"{case_id}.json",
                    geodesic_m=c.get("geodesic_m"),
                    reference=[(p[0], p[1]) for p in c.get("reference_path") or []] or None,
                    walls=walls,
                ),
                timeout_s=TIMEOUT_S,
                threshold=0.5,  # passed == reached
                tags=frozenset(
                    {"habitat", "nav", scene["scene_id"], label, c.get("difficulty", "")} - {""}
                ),
            )
        )
    return out


SUITE: Suite = [case for f in sorted(SCENES.glob("*.json")) for case in cases_for(f)]

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
"""The planner suite's goal is the navigable point beside the object; the grade stays on the box."""

import json
from pathlib import Path

import pytest

from dimos.evals.suites.habitat_nav import cases_for

SCENE = {
    "scene_id": "s1",
    "detections": [
        {
            "id": "couch@1",
            "label": "couch",
            "center_xyz": [2.0, 3.0, 0.4],
            "size_xyz": [2.0, 0.9, 0.8],
        }
    ],
    "cases": [
        {
            "label": "couch",
            "object_id": "couch@1",
            "spawn_xyz": [0.0, 0.0, 0.1],
            "spawn_yaw_deg": 0.0,
            "end_xy": [2.0, 3.0],
            "end_nav_xy": [2.0, 4.2],
            "difficulty": "easy",
        }
    ],
}


def test_goal_key_changes_the_instruction_only(tmp_path: Path) -> None:
    f = tmp_path / "s1.json"
    f.write_text(json.dumps(SCENE))
    centre = cases_for(f)[0]
    beside = cases_for(f, goal_key="end_nav_xy")[0]
    assert centre.id == beside.id == "s1_couch"
    assert centre.inputs.endswith("go to the couch at (2.00, 3.00)")
    assert beside.inputs.endswith("go to the couch at (2.00, 4.20)")
    # The bridge gets each suite's own instruction; everything else about the launch is shared.
    assert centre.environment.config.extra_env["RAWROBOTBRIDGE__GOAL"] == centre.inputs
    assert beside.environment.config.extra_env["RAWROBOTBRIDGE__GOAL"] == beside.inputs
    assert (
        centre.environment.config.start_position_ros_override
        == beside.environment.config.start_position_ros_override
    )


def test_goal_key_falls_back_to_the_centre(tmp_path: Path) -> None:
    scene = json.loads(json.dumps(SCENE))
    del scene["cases"][0]["end_nav_xy"]
    f = tmp_path / "s1.json"
    f.write_text(json.dumps(scene))
    assert cases_for(f, goal_key="end_nav_xy")[0].inputs.endswith("(2.00, 3.00)")


def test_missing_stats_read_as_no_world_state() -> None:
    from dimos.evals.suites.habitat_nav import world_state_check

    with pytest.raises(RuntimeError, match="no world state"):
        world_state_check({"ticks": 0, "errors": 0, "last_error": "no stats file"})
    world_state_check({})  # arms without a bridge have no counters to check


def test_module_env_names_the_navigation_agent() -> None:
    """The lidar band reaches the agent only through a key spelled from its class name; the
    default band includes the floor and leaves the robot nothing to steer by."""
    from dimos.agents.typesafe.navigation import TypeSafeNavigationAgent
    from dimos.evals.suites.habitat_nav import MODULE_ENV
    from dimos.robot.raw_robot_bridge import RawRobotBridge

    assert f"{TypeSafeNavigationAgent.__name__.upper()}__LIDAR_BAND" in MODULE_ENV
    assert f"{RawRobotBridge.__name__.upper()}__LIDAR_Z_MIN" in MODULE_ENV

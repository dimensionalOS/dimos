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

"""ABC Sim tasks for one OpenYAM arm through dimOS's MCP skills.

    dimos evals run dimos.evals.suites.abc_sim --agent <agent>

Scenes come from amazon-far/abc (data/abc_*, see data/abc_yam/SOURCE.md). One YAM arm
does the task and the other stays parked; graders follow abc_sim/task_eval.
"""

import json

from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import count_inside, placed_beside
from dimos.evals.suites.mujoco_xarm import PERCEPTION_MODULES
from dimos.evals.types import EvalCase, Suite
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_SIM_LEFT_BASE,
    OPENYAM_SIM_RIGHT_BASE,
)
from dimos.utils.data import LfsPath

GUIDANCE = (
    "You control one YAM arm with a parallel gripper (about 9.5 cm wide when open). "
    "Locate objects with the perception and camera skills, and use the manipulation skills. "
    "Release the object and move the gripper away when done."
)


def environment(
    scene: str, bodies: tuple[str, ...], base: tuple[float, float, float]
) -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=[
            "openyam-manipulation-sim",
            "perception-localize",
            "mcp-server",
            "observe-skill",
        ],
        disable=PERCEPTION_MODULES,
        scene=LfsPath(f"{scene}/scene.xml"),
        tracked_bodies=bodies,
        module_env={
            "MANIPULATIONMODULE__MODEL__BASE_POSE": json.dumps(
                {"frame_id": "world", "position": list(base)}
            )
        },
    )


COUNT_ITEMS = tuple(f"count_item_{i}" for i in range(1, 7))
# Interior of the lidded box, below the lid (abc_sim/task_eval/count_box.py).
COUNT_BOX = ((0.0, 0.0, 0.104), (0.106, 0.138, 0.088))
# grab_clutter_target_box_goal_region site.
CLUTTER_BOX = ((0.0, 0.0, 0.110), (0.060, 0.055, 0.180))

SUITE: Suite = [
    EvalCase(
        id="abc_count_two",
        inputs=(
            "Put exactly two objects into the dark box through the slot in its lid. "
            "Leave every other object outside the box. " + GUIDANCE
        ),
        environment=environment(
            "abc_count", (*COUNT_ITEMS, "opaque_count_box"), OPENYAM_SIM_RIGHT_BASE
        ),
        grade=count_inside(COUNT_ITEMS, "opaque_count_box", COUNT_BOX, want=2),
        timeout_s=1200.0,
        tags=frozenset({"mujoco", "abc", "openyam", "manipulation", "count"}),
    ),
    EvalCase(
        id="abc_grab_clutter",
        inputs="Pick up the red cube and place it in the box. " + GUIDANCE,
        environment=environment(
            "abc_clutter", ("target_object", "grab_clutter_target_box"), OPENYAM_SIM_LEFT_BASE
        ),
        grade=count_inside(("target_object",), "grab_clutter_target_box", CLUTTER_BOX, want=1),
        timeout_s=900.0,
        tags=frozenset({"mujoco", "abc", "openyam", "manipulation", "clutter"}),
    ),
    EvalCase(
        id="abc_put_relative",
        inputs=(
            "Put the orange cube to the left of the green cube, from the robot's point of "
            "view (world +y is left). " + GUIDANCE
        ),
        environment=environment(
            "abc_relative", ("relative_obj_06", "relative_obj_04"), OPENYAM_SIM_RIGHT_BASE
        ),
        grade=placed_beside("relative_obj_06", "relative_obj_04", (0.0, 1.0)),
        timeout_s=900.0,
        tags=frozenset({"mujoco", "abc", "openyam", "manipulation", "relative"}),
    ),
]

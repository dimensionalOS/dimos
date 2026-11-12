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

"""xArm7 on a table with gripper-sized shapes and bowls: a pick and a pick-and-place,
driven through the Python API with the planner and the wrist camera only, graded on the
bodies' recorded poses.

    dimos evals run dimos.evals.suites.mujoco_xarm --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import lifted, stacked_on
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

TRACKED = ("orange_cylinder", "red_cube", "blue_bowl")

PERCEPTION_MODULES = (
    "object-scene-registration-module",
    "pick-and-place-module",
    "heuristic-grasp-module",
)

SCENE = (
    "The arm stands on the table, whose top is at z=0.12 m in the world frame. About "
    "x=0.35 to 0.55 m ahead of the arm base are gripper-sized shapes: a 4 cm red cube, a "
    "3 cm blue cube, a standing green block, a yellow bar, an orange cylinder 4 cm wide and "
    "6 cm tall and two small balls. A basket is on the +y side; a round red bowl and a "
    "rectangular blue bowl are on the -y side. Find where things are by looking through the "
    "wrist camera. The gripper starts pointing straight down: for top-down moves omit the "
    "orientation in arm.move_pose to keep it."
)


def environment() -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=["xarm-perception-sim", "observe-skill"],
        disable=PERCEPTION_MODULES,
        scene=LfsPath("xarm7/scene.xml"),
        tracked_bodies=TRACKED,
    )


SUITE: Suite = [
    EvalCase(
        id="xarm_pick_cylinder",
        inputs=f"Pick up the orange cylinder from the table and hold it in the air. {SCENE}",
        environment=environment(),
        grade=lifted("orange_cylinder", by_m=0.05),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "manipulation", "pick"}),
    ),
    EvalCase(
        id="xarm_cube_in_bowl",
        inputs=f"Pick up the red cube and put it in the blue bowl. {SCENE}",
        environment=environment(),
        # Resting on the bowl's floor puts the cube's centre 2.8 cm above the bowl's origin.
        grade=stacked_on("red_cube", "blue_bowl", rise_m=(0.02, 0.04), band_m=0.05),
        timeout_s=900.0,
        threshold=0.5,
        tags=frozenset({"mujoco", "manipulation", "pick", "place"}),
    ),
]

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

"""xArm7 at a table with a red ball (``apple``) and a cylinder (``cup``): two picks,
planner skills and the wrist camera only, graded on the bodies' recorded poses.

    dimos evals run dimos.evals.suites.mujoco_xarm --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import lifted, stacked_on
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

TRACKED = ("apple", "cup")

PERCEPTION_MODULES = (
    "object-scene-registration-module",
    "pick-and-place-module",
    "heuristic-grasp-module",
)

SCENE = (
    "The table top is at z=0.13 m in the world frame and spans roughly x=0.30 to 0.60 m "
    "ahead of the arm base. On it are a red ball about 8 cm wide, an orange ball about 9 cm "
    "wide and a cylinder about 7 cm wide and 12 cm tall; find where they are by looking "
    "through the wrist camera. The gripper starts pointing straight down: for top-down moves "
    "omit roll/pitch/yaw in move_to_pose to keep the current orientation, or pass "
    "roll=3.1416, pitch=0. roll=pitch=yaw=0 points the gripper up."
)


def environment() -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=PERCEPTION_MODULES,
        scene=LfsPath("xarm7/scene.xml"),
        tracked_bodies=TRACKED,
    )


SUITE: Suite = [
    EvalCase(
        id="xarm_pick_cylinder",
        inputs=f"Pick up the cylinder from the table and hold it in the air. {SCENE}",
        environment=environment(),
        grade=lifted("cup", by_m=0.05),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "manipulation", "pick"}),
    ),
    EvalCase(
        id="xarm_ball_on_cylinder",
        inputs=f"Pick up the red ball and place it on top of the cylinder. {SCENE}",
        environment=environment(),
        grade=stacked_on("apple", "cup", rise_m=(0.08, 0.12), band_m=0.07),
        timeout_s=900.0,
        threshold=0.5,
        tags=frozenset({"mujoco", "manipulation", "pick", "place"}),
    ),
]

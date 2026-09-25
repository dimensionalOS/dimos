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

"""One question for the xArm7 table scene, no perception stack: pick up the cylinder.

The agent gets the planner skills (``move_to_pose``, ``set_gripper``, ...) and
``observe`` for wrist-camera frames, nothing that detects or localises objects.
The cylinder is the ``cup`` body in ``data/xarm7/scene.xml`` (radius 3.5 cm,
12 cm tall, standing at x=0.50 m in front of the arm). Full credit once it ends
the episode 5 cm or more above where it started.

    dimos evals run dimos.evals.suites.mujoco_xarm_pick_cylinder --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from dimos.evals.suites.mujoco_xarm import arm_only_environment, lifted
from dimos.evals.types import EvalCase, Suite

SUITE: Suite = [
    EvalCase(
        id="xarm_pick_cylinder",
        inputs=(
            "There is a cylinder standing on the table in front of you. Pick it up and "
            "hold it in the air. The table top is at z=0.12 m in the world frame and "
            "spans roughly x=0.30 to 0.60 m ahead of the arm base; the cylinder is about "
            "12 cm tall and 7 cm wide, standing near x=0.5 m, y=0. Look through the wrist "
            "camera whenever you need to check where things are."
        ),
        # MuJoCo window on Linux; on macOS the sim is headless and Rerun is the view.
        environment=arm_only_environment(headless=False, rerun=True),
        grade=lifted("cup", by_m=0.05),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "manipulation", "pick"}),
    )
]

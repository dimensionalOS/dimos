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

"""Default xArm cylinder lift through the raw robot interface.

dimos evals run dimos.evals.suites.mujoco_xarm_pick --agent dimos.evals.agents.pi \
    --set no_dimos=true --set max_steps=120

The agent learns the interface from ROBOT.md (RAW_ARM_README + XARM7_NOTES). Pi's default 40-step
budget is mostly spent on client setup and observation, so give it room to re-observe.
"""

from dimos.evals.constants import RAW_ARM_README
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.suites.mujoco_xarm import lifted
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

XARM7_NOTES = """
This robot is an xArm7. Its base is 0.12 m above world with the same axes. At the start pose
the wrist camera looks straight down: image right = world -Y, image up = world +X.
Gripper: the TCP is 0.172 m along the gripper axis from its root; the finger pads sit
11-48 mm behind the TCP. The jaw gap runs from about 1.6 mm (closed) to 88.9 mm (open).
"""

SUITE: Suite = [
    EvalCase(
        id="xarm_pick_cylinder_twist",
        inputs=(
            "Pick up the cylinder from the table and hold it in the air, at least 5 cm above "
            "its initial position. Use the robot interface described in ROBOT.md."
        ),
        environment=MujocoEnvironment(
            blueprint=["xarm-sim", "mcp-server"],
            raw_bridge=True,
            raw_guide=RAW_ARM_README + XARM7_NOTES,
            ready_streams=("color_image", "coordinator_joint_state"),
            scene=LfsPath("xarm7/scene.xml"),
            tracked_bodies=("cup",),
        ),
        grade=lifted("cup", by_m=0.05),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "manipulation", "raw", "pick"}),
    ),
]

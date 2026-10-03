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

dimos evals run dimos.evals.suites.mujoco_xarm_raw --agent dimos.evals.agents.pi --set no_dimos=true
"""

from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.robot_context import local_robot_context
from dimos.evals.suites.mujoco_xarm import lifted
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

SUITE: Suite = [
    EvalCase(
        id="xarm_raw_pick_cylinder",
        inputs=(
            "Pick up the cylinder from the table and hold it in the air, at least "
            "5 cm above its initial position. Use the robot interface described in ROBOT.md."
        ),
        environment=MujocoEnvironment(
            blueprint=["xarm-sim", "mcp-server"],
            raw_bridge=True,
            raw_interface="manipulation",
            robot_context=local_robot_context("xarm7"),
            ready_streams=("color_image", "overview_image", "coordinator_joint_state"),
            scene=LfsPath("xarm7/scene.xml"),
            tracked_bodies=("cup",),
        ),
        grade=lifted("cup", by_m=0.05),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "manipulation", "raw", "pick"}),
    ),
]

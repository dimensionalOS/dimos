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

"""The robosuite cases through the raw robot interface, for agents without dimOS.

Same scenes, tasks and graders as ``dimos.evals.suites.robosuite``; the robot is reached
over plain Zenoh topics described in ROBOT.md instead of dimOS skills.

dimos evals run dimos.evals.suites.robosuite_raw --agent dimos.evals.agents.pi \\
    --set no_dimos=true --set max_steps=150
"""

from dataclasses import replace

from dimos.evals.constants import RAW_ARM_README, XARM7_GRIPPER_NOTES
from dimos.evals.environments.mujoco_sim import MujocoEnvironment, MujocoEnvironmentConfig
from dimos.evals.suites.robosuite import GUIDANCE, SUITE as SKILL_SUITE
from dimos.evals.types import EvalCase, Suite

XARM7_ROBOSUITE_NOTES = f"""
This robot is an xArm7 on a pedestal at the edge of the table. Its base is 0.912 m above world
with the same axes: +x out across the table, +y to the robot's left, +z up. At the start pose
the wrist camera looks straight down: image right = world -Y, image up = world +X.
{XARM7_GRIPPER_NOTES}
"""


def _raw(case: EvalCase) -> EvalCase:
    skills = case.environment.config
    assert isinstance(skills, MujocoEnvironmentConfig)
    task = case.inputs.replace(GUIDANCE, "").strip()
    return replace(
        case,
        id=case.id.replace("robosuite_", "robosuite_raw_", 1),
        inputs=(
            f"{task} Keep the final result steady for at least two seconds before finishing. "
            "Use the robot interface described in ROBOT.md."
        ),
        environment=MujocoEnvironment(
            blueprint=["xarm-sim-robosuite", "mcp-server"],
            raw_bridge=True,
            raw_guide=RAW_ARM_README + XARM7_ROBOSUITE_NOTES,
            ready_streams=("color_image", "coordinator_joint_state"),
            scene=skills.scene,
            tracked_bodies=skills.tracked_bodies,
            agent_artifacts=(),  # ROBOT.md and the robot only; the recording holds ground truth
        ),
        tags=case.tags | {"raw"},
    )


SUITE: Suite = [_raw(case) for case in SKILL_SUITE]

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

"""The robosuite_2 cases through the raw robot interface, for agents without dimOS.

Same scenes, tasks and graders as ``dimos.evals.suites.robosuite_2``; the robot is reached
over plain Zenoh topics described in ROBOT.md instead of dimOS skills.

dimos evals run dimos.evals.suites.robosuite_2_raw --agent dimos.evals.agents.pi \\
    --set no_dimos=true --set max_steps=150
"""

from dataclasses import replace

from dimos.evals.constants import RAW_ARM_README
from dimos.evals.environments.mujoco_sim import MujocoEnvironment, MujocoEnvironmentConfig
from dimos.evals.suites.robosuite import GUIDANCE
from dimos.evals.suites.robosuite_2 import SUITE as SKILL_SUITE, sim_env
from dimos.evals.suites.robosuite_raw import XARM7_ROBOSUITE_NOTES
from dimos.evals.types import EvalCase, Suite


def _raw(case: EvalCase) -> EvalCase:
    skills = case.environment.config
    assert isinstance(skills, MujocoEnvironmentConfig)
    task = case.inputs.replace(GUIDANCE, "").strip()
    scene = next(s for s in ("bin", "bottle", "blocks") if s in case.tags)
    return replace(
        case,
        id=case.id.replace("robosuite_2_", "robosuite_2_raw_", 1),
        inputs=(
            f"{task} Keep the final result steady for at least two seconds before finishing. "
            "Use the robot interface described in ROBOT.md. Do not read the simulator's scene, "
            "asset or recording files or the source of robosuite or MuJoCo, and do not install "
            "them; learn about the scene only through the robot interface."
        ),
        environment=MujocoEnvironment(
            blueprint=["xarm-sim-robosuite", "mcp-server"],
            raw_bridge=True,
            raw_guide=RAW_ARM_README + XARM7_ROBOSUITE_NOTES,
            ready_streams=("color_image", "coordinator_joint_state"),
            scene=skills.scene,
            tracked_bodies=skills.tracked_bodies,
            agent_artifacts=(),  # ROBOT.md and the robot only; the recording holds ground truth
            module_env=sim_env(scene),
        ),
        tags=case.tags | {"raw"},
    )


SUITE: Suite = [_raw(case) for case in SKILL_SUITE]

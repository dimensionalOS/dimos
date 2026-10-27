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

"""The BinBottle cases through the raw robot interface, for agents without dimOS.

Same scenes, tasks, simulator settings and graders as ``dimos.evals.suites.binbottle_py``; the
robot is reached over plain Zenoh topics described in ROBOT.md instead of dimOS.

dimos evals run dimos.evals.suites.binbottle_raw --agent dimos.evals.agents.pi \\
    --set no_dimos=true --set max_steps=150
"""

from dataclasses import replace

from dimos.evals.constants import RAW_ARM_README
from dimos.evals.environments.mujoco_sim import MujocoEnvironment, MujocoEnvironmentConfig
from dimos.evals.suites.binbottle_py import GUIDANCE, SUITE as DIMOS_SUITE, sim_env
from dimos.evals.suites.robosuite_raw import XARM7_ROBOSUITE_NOTES
from dimos.evals.types import EvalCase, Suite


def raw(case: EvalCase) -> EvalCase:
    """``case`` through the raw robot interface: the same scene and grader, no dimOS."""
    dimos = case.environment.config
    assert isinstance(dimos, MujocoEnvironmentConfig)
    task = case.inputs.replace(GUIDANCE, "").strip()
    scene = next(s for s in ("bin", "bottle", "blocks") if s in case.tags)
    return replace(
        case,
        id=case.id.replace("_py_", "_raw_", 1),  # binbottle_py_bin_lift -> binbottle_raw_bin_lift
        inputs=(
            f"{task} "
            "Use the robot interface described in ROBOT.md. Do not read the simulator's scene, "
            "asset or recording files or the source of robosuite or MuJoCo, and do not install "
            "them; learn about the scene only through the robot interface."
        ),
        environment=MujocoEnvironment(
            blueprint=["xarm-raw-sim"],
            raw_bridge=True,
            raw_guide=RAW_ARM_README + XARM7_ROBOSUITE_NOTES,
            ready_streams=("color_image", "coordinator_joint_state"),
            scene=dimos.scene,
            tracked_bodies=dimos.tracked_bodies,
            agent_artifacts=(),  # ROBOT.md and the robot only; the recording holds ground truth
            module_env=sim_env(scene),
        ),
        tags=(case.tags - {"python"}) | {"raw"},
    )


SUITE: Suite = [raw(case) for case in DIMOS_SUITE]

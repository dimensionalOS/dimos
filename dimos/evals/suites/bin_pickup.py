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

"""Bin pickup: an empty shelf bin on the robosuite table.

This scene (``data/bin_pickup``) and those of ``dimosblocks`` and ``pour`` reuse the robosuite
lift table, xArm7, wrist camera and start pose, with robosuite's contact solver settings in the XML.
"""

from dimos.evals.constants import SIM_LOCALIZE_ENV
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import GRIPPER_BODIES, lift_partial
from dimos.evals.suites.mujoco_xarm import PERCEPTION_MODULES
from dimos.evals.suites.robosuite import GUIDANCE
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

# Gravity compensation so the arm holds its commanded pose as the real xArm does (MujocoSimModule).
SIM_ENV = {"MUJOCOSIMMODULE__GRAVITY_COMPENSATION": "true"}


def environment(scene: str, bodies: tuple[str, ...]) -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=[
            "xarm-perception-sim",
            "mcp-server",
            "observe-skill",
            "perception-localize",
        ],
        disable=PERCEPTION_MODULES,
        scene=LfsPath(f"{scene}/scene.xml"),
        base_height=0.912,
        tracked_bodies=(*bodies, *GRIPPER_BODIES),
        agent_artifacts=(),  # sensors and skills only; the recording holds ground-truth poses
        # Same localize params as the robosuite suite: one view, robosuite-style flat shading.
        module_env={**SIM_LOCALIZE_ENV, **SIM_ENV},
    )


SUITE: Suite = [
    EvalCase(
        id="bin_pickup",
        inputs="Pick up the yellow bin and hold it at least 5 cm above the table. " + GUIDANCE,
        environment=environment("bin_pickup", ("bin_main",)),
        # 0.2 touched + 0.8 x height; 1.0 only at 5 cm. half_size_m sets the touch radius.
        grade=lift_partial("bin_main", by_m=0.05, half_size_m=0.15),
        threshold=1.0,
        timeout_s=1200.0,
        tags=frozenset({"mujoco", "manipulation", "bin"}),
    ),
]

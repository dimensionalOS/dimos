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

"""BinBottle: household objects on the robosuite table, an empty shelf bin and a capped water
bottle.

The scenes (``data/robosuite_2``, shared with ``dimosblocks``) reuse the robosuite lift table, xArm7, wrist camera and start
pose, with robosuite's contact solver settings in the XML. Each also has an ``overview`` camera
that is recorded for viewing runs afterwards; no agent tool or bridge reads it.
"""

import json

from dimos.evals.constants import SIM_LOCALIZE_ENV
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import GRIPPER_BODIES, cap_partial, lift_partial
from dimos.evals.suites.mujoco_xarm import PERCEPTION_MODULES
from dimos.evals.suites.robosuite import GUIDANCE
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

# Recording-only scene camera, the bottle cap's thread release, and gravity compensation so the
# arm holds its commanded pose as the real xArm does (MujocoSimModule).
SIM_ENV = {
    "MUJOCOSIMMODULE__OVERVIEW_CAMERA_NAME": "overview",
    "MUJOCOSIMMODULE__OVERVIEW_FPS": "8",
    "MUJOCOSIMMODULE__GRAVITY_COMPENSATION": "true",
}
THREADS = {"bottle": {"cap_thread": "cap_hold"}}


def sim_env(scene: str) -> dict[str, str]:
    env = dict(SIM_ENV)
    if scene in THREADS:
        env["MUJOCOSIMMODULE__THREAD_RELEASES"] = json.dumps(THREADS[scene])
    return env


def environment(scene: str, bodies: tuple[str, ...]) -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=[
            "xarm-perception-sim",
            "mcp-server",
            "observe-skill",
            "perception-localize",
        ],
        disable=PERCEPTION_MODULES,
        scene=LfsPath(f"robosuite_2/{scene}/scene.xml"),
        base_height=0.912,
        tracked_bodies=(*bodies, *GRIPPER_BODIES),
        agent_artifacts=(),  # sensors and skills only; the recording holds ground-truth poses
        # Same localize params as the robosuite suite: one view, robosuite-style flat shading.
        module_env={
            **SIM_LOCALIZE_ENV,
            **sim_env(scene),
        },
    )


SUITE: Suite = [
    EvalCase(
        id="binbottle_bin_lift",
        inputs="Pick up the yellow bin and hold it at least 5 cm above the table. " + GUIDANCE,
        environment=environment("bin", ("bin_main",)),
        # 0.2 touched + 0.8 x height; 1.0 only at 5 cm. half_size_m sets the touch radius.
        grade=lift_partial("bin_main", by_m=0.05, half_size_m=0.15),
        threshold=1.0,
        timeout_s=1200.0,
        tags=frozenset({"mujoco", "binbottle", "manipulation", "bin"}),
    ),
    EvalCase(
        id="binbottle_open_bottle",
        inputs="Unscrew the cap of the water bottle and lift it off the bottle. " + GUIDANCE,
        environment=environment("bottle", ("cap_main", "cap_carrier", "bottle_main")),
        # 0.1 touched the cap + 0.5 x unscrewed + 0.4 taken off the bottle.
        grade=cap_partial,
        threshold=1.0,
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "binbottle", "manipulation", "bottle"}),
    ),
]

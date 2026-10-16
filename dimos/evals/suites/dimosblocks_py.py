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

"""DimosBlocks through dimOS's Python API: five lettered blocks lie at random angles on the
robosuite table; arrange them to spell DIMOS, reached with ``Dimos.connect()`` and the ``Arm``
SDK instead of MCP tool calls.

The agent gets DIMOS.md and no tool listing. The scene (``data/robosuite_2/blocks``) reuses the
robosuite lift table, xArm7, wrist camera and start pose; its ``overview`` camera is recorded
for viewing runs afterwards and no agent tool or bridge reads it.

dimos evals run dimos.evals.suites.dimosblocks_py --agent dimos.evals.agents.pi
"""

from dimos.evals.constants import DIMOS_PYTHON_README, XARM7_GRIPPER_NOTES
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import GRIPPER_BODIES, WORD, word_partial
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

GUIDANCE = (
    "The robot is an xArm7 arm with a two-finger parallel gripper, mounted at the edge of the "
    "table. Positions are in the world frame, in metres: the arm's base is at (0, 0, 0.912), "
    "+x points from the base out across the table, +y to the robot's left and +z up. "
    "One camera is connected: the wrist camera, mounted on the gripper and moving with it. "
    "Look and move through the robot's Python interface, described in DIMOS.md. "
    f"{XARM7_GRIPPER_NOTES} "
    "Do not read the simulator's scene, asset or recording files or the source of robosuite or "
    "MuJoCo; learn about the scene only through the robot's camera and tools."
)

# One view, and robosuite's flat-shaded objects score under OWLv2's real-world defaults.
LOCALIZE_POLICY = '{"min_views": 1, "candidate_floor": 0.2, "accept_score": 0.25}'
# Recording-only scene camera, and gravity compensation so the arm holds its commanded pose as
# the real xArm does (MujocoSimModule).
SIM_ENV = {
    "MUJOCOSIMMODULE__OVERVIEW_CAMERA_NAME": "overview",
    "MUJOCOSIMMODULE__OVERVIEW_FPS": "8",
    "MUJOCOSIMMODULE__GRAVITY_COMPENSATION": "true",
}

SUITE: Suite = [
    EvalCase(
        id="dimosblocks_py_spell_dimos",
        inputs="Arrange the letter blocks so they spell DIMOS. " + GUIDANCE,
        environment=MujocoEnvironment(
            # The Python-API stack (no MCP): the agent reaches modules and streams over the API.
            blueprint=["xarm-agent-sim"],
            scene=LfsPath("robosuite_2/blocks/scene.xml"),
            base_height=0.912,
            tracked_bodies=(*(f"block_{c}_main" for c in WORD), *GRIPPER_BODIES),
            agent_artifacts=(),  # sensors and the API only; the recording holds ground truth
            python_guide=DIMOS_PYTHON_README,
            module_env={"PERCEPTIONLOCALIZE__POLICY": LOCALIZE_POLICY, **SIM_ENV},
        ),
        # Any placement on the table: 0.7 x pairs in reading order + 0.3 letters upright for one
        # reader (O at any angle).
        grade=word_partial,
        threshold=1.0,
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "dimosblocks", "manipulation", "blocks", "python"}),
    ),
]

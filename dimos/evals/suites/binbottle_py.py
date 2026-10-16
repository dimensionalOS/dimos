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

"""BinBottle through dimOS's Python API: an empty shelf bin and a capped water bottle on the
robosuite table, reached with ``Dimos.connect()`` and the ``Arm`` SDK instead of MCP tool calls.

The agent gets DIMOS.md and no tool listing. The scenes (``data/robosuite_2``) reuse the
robosuite lift table, xArm7, wrist camera and start pose; their ``overview`` camera is recorded
for viewing runs afterwards and no agent tool or bridge reads it.

dimos evals run dimos.evals.suites.binbottle_py --agent dimos.evals.agents.pi
"""

import json

from dimos.evals.constants import DIMOS_PYTHON_README, XARM7_GRIPPER_NOTES
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import GRIPPER_BODIES, cap_partial, lift_partial
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
# Recording-only scene camera, the bottle cap's thread release, and gravity compensation so the
# arm holds its commanded pose as the real xArm does (MujocoSimModule).
SIM_ENV = {
    "MUJOCOSIMMODULE__OVERVIEW_CAMERA_NAME": "overview",
    "MUJOCOSIMMODULE__OVERVIEW_FPS": "8",
    "MUJOCOSIMMODULE__GRAVITY_COMPENSATION": "true",
}
THREADS = {"bottle": {"cap_thread": "cap_hold"}}


def sim_env(scene: str) -> dict[str, str]:
    """The simulator settings for ``scene``, shared with the raw suites."""
    env = dict(SIM_ENV)
    if scene in THREADS:
        env["MUJOCOSIMMODULE__THREAD_RELEASES"] = json.dumps(THREADS[scene])
    return env


def environment(scene: str, bodies: tuple[str, ...]) -> MujocoEnvironment:
    env = {"PERCEPTIONLOCALIZE__POLICY": LOCALIZE_POLICY, **sim_env(scene)}
    return MujocoEnvironment(
        # The Python-API stack (no MCP): the agent reaches modules and streams over the API.
        blueprint=["xarm-agent-sim"],
        scene=LfsPath(f"robosuite_2/{scene}/scene.xml"),
        base_height=0.912,
        tracked_bodies=(*bodies, *GRIPPER_BODIES),
        agent_artifacts=(),  # sensors and the API only; the recording holds ground-truth poses
        python_guide=DIMOS_PYTHON_README,
        module_env=env,
    )


SUITE: Suite = [
    EvalCase(
        id="binbottle_py_bin_lift",
        inputs="Pick up the yellow bin and hold it at least 5 cm above the table. " + GUIDANCE,
        environment=environment("bin", ("bin_main",)),
        # 0.2 touched + 0.8 x height; 1.0 only at 5 cm. half_size_m sets the touch radius.
        grade=lift_partial("bin_main", by_m=0.05, half_size_m=0.15),
        threshold=1.0,
        timeout_s=1200.0,
        tags=frozenset({"mujoco", "binbottle", "manipulation", "bin", "python"}),
    ),
    EvalCase(
        id="binbottle_py_open_bottle",
        inputs=(
            "Open the water bottle: unscrew the cap and take it off the bottle, then finish. "
            + GUIDANCE
        ),
        environment=environment("bottle", ("cap_main", "cap_carrier", "bottle_main")),
        # 0.1 touched the cap + 0.5 x unscrewed + 0.4 taken off the bottle.
        grade=cap_partial,
        threshold=1.0,
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "binbottle", "manipulation", "bottle", "python"}),
    ),
]

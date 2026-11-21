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

"""A real dual OpenYAM workcell: two arms, the overhead D405 and localize, graded by the operator.

The CAN ports and the overhead camera are module settings, given as env overrides.
A prebuilt camera binary also needs its source build turned off:

    DUALOPENYAMCONNECTION__LEFT_CAN_PORT=follower_l \\
    DUALOPENYAMCONNECTION__RIGHT_CAN_PORT=follower_r \\
    REALSENSECAMERA__SERIAL_NUMBER=230322272156 \\
    REALSENSECAMERA__EXECUTABLE=/path/to/realsense_native \\
    REALSENSECAMERA__SOURCE_DIR=null REALSENSECAMERA__BUILD_COMMAND=null \\
        dimos evals run dimos.evals.suites.hardware_dual_openyam --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from dimos.evals.environments.hardware import HardwareEnvironment
from dimos.evals.scorers import human
from dimos.evals.types import EvalCase, Suite

# Planner, both arms, the overhead D405 at its calibrated pose, localize and observe.
BLUEPRINT = ["dual-openyam-localize"]

# The motors go limp when dimos stops, so each arm returns to the folded pose it
# started in (init) before the disconnect; a raised arm would drop.
PARK_BOTH_ARMS = tuple(
    ("go_init", {"planning_group": f"{side}_manipulator"}) for side in ("left", "right")
)
# Checked once parked: if an arm is not back, the operator holds it before the motors stop.
PARKED = {f"{side}_manipulator": "init" for side in ("left", "right")}

BLUE_BLOCK_IN_BIN = (
    "Use the left arm to pick up the blue block and place it in the green container "
    "with the protein bars."
)
BLOCK_IN_CUP = "Pick up the red block and drop it into the red cup."
SOLVE_RUBIKS_CUBE = "Solve the Rubik's cube on the table, so each face shows one colour."


def _workcell(task: str) -> HardwareEnvironment:
    return HardwareEnvironment(
        blueprint=BLUEPRINT,
        task=task,
        stop_tool="cancel",
        stop_calls=PARK_BOTH_ARMS,
        parked_presets=PARKED,
        settle_on_joints=True,
        module_env={"MANIPULATIONMODULE__DEFAULT_SPEED_SCALE": "0.3"},
    )


SUITE: Suite = [
    EvalCase(
        id="dual_openyam_left_blue_block_in_bin",
        inputs=BLUE_BLOCK_IN_BIN,
        environment=_workcell(BLUE_BLOCK_IN_BIN),
        grade=human(BLUE_BLOCK_IN_BIN),
        timeout_s=600.0,
        tags=frozenset(
            {"hardware", "dual_openyam", "manipulation", "pick", "place", "blue_block_bin"}
        ),
    ),
    EvalCase(
        id="dual_openyam_block_in_cup",
        inputs=BLOCK_IN_CUP,
        environment=_workcell(BLOCK_IN_CUP),
        grade=human(BLOCK_IN_CUP),
        timeout_s=600.0,
        tags=frozenset({"hardware", "dual_openyam", "manipulation", "pick", "place"}),
    ),
    EvalCase(
        id="dual_openyam_solve_rubiks_cube",
        inputs=SOLVE_RUBIKS_CUBE,
        environment=_workcell(SOLVE_RUBIKS_CUBE),
        grade=human(SOLVE_RUBIKS_CUBE),
        timeout_s=900.0,
        tags=frozenset({"hardware", "dual_openyam", "manipulation", "bimanual", "rubiks"}),
    ),
]

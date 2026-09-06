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

"""Physics-backed Dual OpenYAM simulation blueprints.

The coordinator picks the MuJoCo whole-body adapter from
``global_config.simulation``, so these run under ``dimos --simulation mujoco``.
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_GRIPPER_JOINTS,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.sim import dual_openyam_sim_module


def dual_openyam_gripper_task(side: str, *, priority: int = 20) -> TaskConfig:
    index = 0 if side == "left" else 1
    return TaskConfig(
        name=f"{side}_arm_gripper",
        type="gripper",
        joint_names=[DUAL_OPENYAM_GRIPPER_JOINTS[index]],
        priority=priority,
    )


_dual_openyam_sim_tasks = [
    dual_openyam_trajectory_task(),
    dual_openyam_gripper_task("left"),
    dual_openyam_gripper_task("right"),
]

dual_openyam_sim = autoconnect(
    dual_openyam_sim_module(),
    planner(model=dual_openyam_model_config(), visualization={"backend": "viser"}),
    DualOpenYamCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=_dual_openyam_sim_tasks,
    ),
)

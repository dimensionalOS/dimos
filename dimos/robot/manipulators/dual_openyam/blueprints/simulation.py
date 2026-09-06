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

import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
    dual_openyam_gripper_task,
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.config import dual_openyam_model_config
from dimos.robot.manipulators.dual_openyam.sim import (
    DUAL_OPENYAM_SIM_CAMERAS,
    DualOpenYamSimModule,
    dual_openyam_sim_model_config,
    dual_openyam_sim_module,
)
from dimos.simulation.perception.blueprints import sim_scene_registration

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

# Bodies that make up the robot, so the obstacle cloud is not the arm itself.
DUAL_OPENYAM_ROBOT_BODIES = ("_arm", "_link", "_finger", "_lf_", "_rf_", "camera")
# Table, bin and gate are scenery: obstacles, never pick targets.
DUAL_OPENYAM_SCENERY = ("world", "table", "gate", "bar", "camera", "bin")
# Pink's defaults never converge on this model; these are the gains the Quest
# teleop blueprint already tunes for the same arm.
DUAL_OPENYAM_PINK = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)

dual_openyam_sim_pick_place = autoconnect(
    dual_openyam_sim_module(cls=DualOpenYamSimModule, extra_cameras=list(DUAL_OPENYAM_SIM_CAMERAS)),
    planner(
        model=dual_openyam_sim_model_config(),
        kinematics=DUAL_OPENYAM_PINK,
        visualization={"backend": "none"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world"),
    # The OpenYAM grasp frame does not point Z along the approach, so the
    # canonical top-down proposal needs a half turn about Y to be reachable.
    HeuristicGraspModule.blueprint(tool_rotation_rpy=(0.0, math.pi, 0.0)),
    sim_scene_registration(
        target_frame="world",
        robot_body_substrings=DUAL_OPENYAM_ROBOT_BODIES,
        ignore_substrings=list(DUAL_OPENYAM_SCENERY),
    ),
    DualOpenYamCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=_dual_openyam_sim_tasks,
    ),
)

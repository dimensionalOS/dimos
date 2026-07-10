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

"""Basic xArm coordinator and planner blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_PATH,
    XARM7_SIM_PATH,
    make_xarm6_model_config,
    make_xarm7_model_config,
    make_xarm_hardware,
    xarm6_hardware,
    xarm7_hardware,
)

xarm6_planner_only = ManipulationModule.blueprint(
    robots=[make_xarm6_model_config(name="arm")],
    planning_timeout=10.0,
)

dual_xarm6_planner = ManipulationModule.blueprint(
    robots=[
        make_xarm6_model_config(name="left_arm", y_offset=0.5),
        make_xarm6_model_config(name="right_arm", y_offset=-0.5),
    ],
    planning_timeout=10.0,
)

_mock_left_xarm6_hw = make_xarm_hardware("left_arm", 6)
_mock_right_xarm6_hw = make_xarm_hardware("right_arm", 6)

dual_xarm6_planner_coordinator = autoconnect(
    planner(
        robots=[
            make_xarm6_model_config(name="left_arm", y_offset=0.5),
            make_xarm6_model_config(name="right_arm", y_offset=-0.5),
        ],
        visualization={"backend": "viser"},
    ),
    coordinator(
        hardware=[_mock_left_xarm6_hw, _mock_right_xarm6_hw],
        tasks=[
            trajectory_task(_mock_left_xarm6_hw),
            trajectory_task(_mock_right_xarm6_hw),
        ],
    ),
)

dual_xarm6_planner_coordinator_mock_meshcat = autoconnect(
    planner(
        robots=[
            make_xarm6_model_config(name="left_arm", y_offset=0.5),
            make_xarm6_model_config(name="right_arm", y_offset=-0.5),
        ],
        visualization={"backend": "meshcat"},
    ),
    coordinator(
        hardware=[_mock_left_xarm6_hw, _mock_right_xarm6_hw],
        tasks=[
            trajectory_task(_mock_left_xarm6_hw),
            trajectory_task(_mock_right_xarm6_hw),
        ],
    ),
)

_xarm7_hw = xarm7_hardware("arm", gripper=True, mock_without_address=True)

xarm7_planner_coordinator = autoconnect(
    planner(robots=[make_xarm7_model_config(name="arm", add_gripper=True)]),
    coordinator(
        hardware=[_xarm7_hw],
        tasks=[trajectory_task(_xarm7_hw)],
    ),
)

_coordinator_xarm7_hw = xarm7_hardware("arm")

coordinator_xarm7 = autoconnect(
    coordinator(
        hardware=[_coordinator_xarm7_hw],
        tasks=[trajectory_task(_coordinator_xarm7_hw)],
    ),
    *mujoco_if_sim(XARM7_SIM_PATH, len(_coordinator_xarm7_hw.joints)),
)

_coordinator_xarm6_hw = xarm6_hardware("arm", gripper=True)

coordinator_xarm6 = autoconnect(
    coordinator(
        hardware=[_coordinator_xarm6_hw],
        tasks=[trajectory_task(_coordinator_xarm6_hw)],
    ),
    *mujoco_if_sim(XARM6_SIM_PATH, len(_coordinator_xarm6_hw.joints)),
)

_xarm7_left = xarm7_hardware("left_arm")
_xarm6_right = xarm6_hardware("right_arm")

coordinator_dual_xarm = ControlCoordinator.blueprint(
    hardware=[_xarm7_left, _xarm6_right],
    tasks=[
        TaskConfig(
            name="traj_left", type="trajectory", joint_names=_xarm7_left.joints, priority=10
        ),
        TaskConfig(
            name="traj_right", type="trajectory", joint_names=_xarm6_right.joints, priority=10
        ),
    ],
)

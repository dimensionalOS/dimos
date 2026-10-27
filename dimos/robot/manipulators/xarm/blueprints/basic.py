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

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.manipulators.common.blueprints import coordinator, planner, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    lite6_hardware,
    make_dual_xarm6_model_config,
    make_lite6_model_config,
    make_xarm7_model_config,
    make_xarm_hardware,
    xarm6_hardware,
    xarm7_hardware,
)

_dual_xarm6_model = make_dual_xarm6_model_config()
_mock_left_xarm6_hw = make_xarm_hardware(
    "left_arm", 6, canonical_joint_names=list(_dual_xarm6_model.planning_groups[0].joint_names)
)
_mock_right_xarm6_hw = make_xarm_hardware(
    "right_arm", 6, canonical_joint_names=list(_dual_xarm6_model.planning_groups[1].joint_names)
)

dual_xarm6_planner_coordinator = autoconnect(
    planner(
        model=_dual_xarm6_model,
        visualization={"backend": "viser"},
    ),
    coordinator(
        hardware=[_mock_left_xarm6_hw, _mock_right_xarm6_hw],
        tasks=[trajectory_task(_mock_left_xarm6_hw, _mock_right_xarm6_hw)],
    ),
)

_xarm7_hw = xarm7_hardware("arm", gripper=True, mock_without_address=True)


def _gripper_task() -> TaskConfig:
    return TaskConfig(
        name="arm_gripper",
        type="gripper",
        joint_names=["arm/gripper"],
        priority=20,
    )


xarm7_planner_coordinator = autoconnect(
    planner(
        model=make_xarm7_model_config(
            add_gripper=True,
            gripper_hardware_id="arm",
        )
    ),
    coordinator(
        hardware=[_xarm7_hw],
        tasks=[trajectory_task(_xarm7_hw), _gripper_task()],
    ),
)

_coordinator_xarm7_hw = xarm7_hardware("arm")

coordinator_xarm7 = autoconnect(
    coordinator(
        hardware=[_coordinator_xarm7_hw],
        tasks=[trajectory_task(_coordinator_xarm7_hw)],
    ),
)

_coordinator_xarm6_hw = xarm6_hardware("arm", gripper=True)

coordinator_xarm6 = autoconnect(
    coordinator(
        hardware=[_coordinator_xarm6_hw],
        tasks=[trajectory_task(_coordinator_xarm6_hw), _gripper_task()],
    ),
)

_coordinator_lite6_hw = lite6_hardware("arm", gripper=True)

coordinator_lite6 = coordinator(
    hardware=[_coordinator_lite6_hw],
    tasks=[trajectory_task(_coordinator_lite6_hw), _gripper_task()],
)

_lite6_hw = lite6_hardware("arm", gripper=True, mock_without_address=True)

lite6_planner_coordinator = autoconnect(
    planner(
        model=make_lite6_model_config(
            add_gripper=True,
            gripper_hardware_id="arm",
        ),
        visualization={"backend": "viser"},
    ),
    coordinator(
        hardware=[_lite6_hw],
        tasks=[trajectory_task(_lite6_hw), _gripper_task()],
    ),
)

_xarm7_left = xarm7_hardware(
    "left_arm", canonical_joint_names=[f"left_arm/joint{i}" for i in range(1, 8)]
)
_xarm6_right = xarm6_hardware(
    "right_arm", canonical_joint_names=[f"right_arm/joint{i}" for i in range(1, 7)]
)

coordinator_dual_xarm = coordinator(
    hardware=[_xarm7_left, _xarm6_right],
    tasks=[trajectory_task(_xarm7_left, _xarm6_right)],
    base_poses={
        "left_arm": PoseStamped(frame_id="world", position=Vector3(0, 0.4, 0.12)),
        "right_arm": PoseStamped(frame_id="world", position=Vector3(0, -0.4, 0.12)),
    },
)

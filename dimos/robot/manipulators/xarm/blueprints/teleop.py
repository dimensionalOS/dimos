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

"""Keyboard teleop blueprints for xArm6 and xArm7."""

from __future__ import annotations

import math
from typing import cast

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.blueprints import (
    GripperTaskOverrides,
    eef_twist_task,
    teleop_ik_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_PATH,
    XARM7_SIM_PATH,
    XARM_GRIPPER_PARAMS,
    make_xarm6_model_config,
    make_xarm7_model_config,
    make_xarm_hardware,
    xarm6_hardware,
    xarm7_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_xarm6_hw = xarm6_hardware("arm", gripper=True, mock_without_address=True)
_xarm7_hw = xarm7_hardware("arm", gripper=True, mock_without_address=True)
_xarm6_control_model = make_xarm6_model_config(add_gripper=False)
_xarm7_control_model = make_xarm7_model_config(add_gripper=False)
_xarm_gripper_params = cast("GripperTaskOverrides", XARM_GRIPPER_PARAMS)

keyboard_teleop_xarm6 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm6_hw],
        tasks=[
            eef_twist_task(
                _xarm6_hw,
                robot_model=_xarm6_control_model,
                timeout=0.0,
                params=_xarm_gripper_params,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[make_xarm6_model_config(add_gripper=True)],
        visualization={"backend": "viser"},
    ),
)

keyboard_teleop_xarm7 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_hw],
        tasks=[
            eef_twist_task(
                _xarm7_hw,
                robot_model=_xarm7_control_model,
                timeout=0.0,
                params=_xarm_gripper_params,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[make_xarm7_model_config(add_gripper=True)],
        visualization={"backend": "viser"},
    ),
)

_xarm6_control_hw = make_xarm_hardware(
    "arm",
    6,
    adapter_type="xarm",
    address=global_config.xarm6_ip,
    gripper=True,
)

coordinator_servo_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_control_hw],
    tasks=[
        TaskConfig(
            name="servo_arm",
            type="servo",
            joint_names=_xarm6_control_hw.joints,
            priority=10,
        ),
    ],
)

coordinator_velocity_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_control_hw],
    tasks=[
        TaskConfig(
            name="velocity_arm",
            type="velocity",
            joint_names=_xarm6_control_hw.joints,
            priority=10,
        ),
    ],
)

coordinator_combined_xarm6 = ControlCoordinator.blueprint(
    hardware=[_xarm6_control_hw],
    tasks=[
        TaskConfig(
            name="servo_arm",
            type="servo",
            joint_names=_xarm6_control_hw.joints,
            priority=10,
        ),
        TaskConfig(
            name="velocity_arm",
            type="velocity",
            joint_names=_xarm6_control_hw.joints,
            priority=10,
        ),
    ],
)

_xarm7_teleop_hw = xarm7_hardware(
    "arm",
    gripper=True,
    gripper_open_position=0.85,
    gripper_closed_position=0.0,
    mock_without_address=True,
)
_xarm6_teleop_hw = xarm6_hardware(
    "arm",
    gripper=True,
    gripper_open_position=0.85,
    gripper_closed_position=0.0,
    mock_without_address=True,
)
_xarm7_teleop_model = make_xarm7_model_config(add_gripper=True)
_xarm6_teleop_model = make_xarm6_model_config(add_gripper=True)

# Conservative end-effector envelopes in the arm base frame. Adjust these to
# the installed table and tooling before operating on hardware.
_xarm6_teleop_workspace_min = (0.10, -0.40, 0.12)
_xarm6_teleop_workspace_max = (0.58, 0.40, 0.55)
_xarm6_teleop_max_reach_m = 0.62
_xarm7_teleop_workspace_min = (0.10, -0.45, 0.12)
_xarm7_teleop_workspace_max = (0.63, 0.45, 0.60)
_xarm7_teleop_max_reach_m = 0.68
_xarm_home_orientation_rpy = (math.pi, 0.0, 0.0)
_xarm_max_home_orientation_delta_rpy = (math.pi / 2.0, math.pi / 2.0, math.pi / 2.0)

# Dual-input arm: VR (teleop_ik) preempts browser keyboard (eef_twist) via
# higher priority; when VR is idle the always-active eef_twist holds/drives.
# While engaged, VR also owns the gripper joint (trigger), so the browser
# gripper toggle only takes effect when VR is disengaged.


class _XArm7TeleopCoordinator(ControlCoordinator):
    arm_joints: Out[JointState]


coordinator_teleop_xarm7 = autoconnect(
    _XArm7TeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        publish_robot_joint_states=True,
        hardware=[_xarm7_teleop_hw],
        tasks=[
            teleop_ik_task(
                _xarm7_teleop_hw,
                hand="right",
                name="teleop_xarm",
                robot_model=_xarm7_control_model,
                priority=20,
                params=_xarm_gripper_params,
                workspace_min=_xarm7_teleop_workspace_min,
                workspace_max=_xarm7_teleop_workspace_max,
                max_reach_m=_xarm7_teleop_max_reach_m,
                home_orientation_rpy=_xarm_home_orientation_rpy,
                max_home_orientation_delta_rpy=_xarm_max_home_orientation_delta_rpy,
            ),
            eef_twist_task(
                _xarm7_teleop_hw,
                robot_model=_xarm7_control_model,
                priority=10,
                timeout=0.0,
                params=_xarm_gripper_params,
                workspace_min=_xarm7_teleop_workspace_min,
                workspace_max=_xarm7_teleop_workspace_max,
                max_reach_m=_xarm7_teleop_max_reach_m,
                home_orientation_rpy=_xarm_home_orientation_rpy,
                max_home_orientation_delta_rpy=_xarm_max_home_orientation_delta_rpy,
            ),
            trajectory_task(_xarm7_teleop_hw, priority=30),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_xarm7_teleop_model],
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(XARM7_SIM_PATH, len(_xarm7_teleop_hw.joints)),
)

coordinator_teleop_xarm6 = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm6_teleop_hw],
        tasks=[
            teleop_ik_task(
                _xarm6_teleop_hw,
                hand="right",
                name="teleop_xarm",
                robot_model=_xarm6_control_model,
                priority=20,
                params=_xarm_gripper_params,
                workspace_min=_xarm6_teleop_workspace_min,
                workspace_max=_xarm6_teleop_workspace_max,
                max_reach_m=_xarm6_teleop_max_reach_m,
                home_orientation_rpy=_xarm_home_orientation_rpy,
                max_home_orientation_delta_rpy=_xarm_max_home_orientation_delta_rpy,
            ),
            eef_twist_task(
                _xarm6_teleop_hw,
                robot_model=_xarm6_control_model,
                priority=10,
                timeout=0.0,
                params=_xarm_gripper_params,
                workspace_min=_xarm6_teleop_workspace_min,
                workspace_max=_xarm6_teleop_workspace_max,
                max_reach_m=_xarm6_teleop_max_reach_m,
                home_orientation_rpy=_xarm_home_orientation_rpy,
                max_home_orientation_delta_rpy=_xarm_max_home_orientation_delta_rpy,
            ),
            trajectory_task(_xarm6_teleop_hw, priority=30),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_xarm6_teleop_model],
        visualization={"backend": "viser"},
    ),
    *mujoco_if_sim(XARM6_SIM_PATH, len(_xarm6_teleop_hw.joints)),
)

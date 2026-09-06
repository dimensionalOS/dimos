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

"""MuJoCo binding for the arm-only Dual OpenYAM."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec
from dimos.utils.data import LfsPath

DUAL_OPENYAM_SCENE_PATH = LfsPath("dual_openyam_sim/put_bottle.xml")

# The MJCF drives one finger per gripper and mirrors the other through an
# equality constraint, so a gripper is a single actuated slide joint whose
# value is the half-opening in metres.
DUAL_OPENYAM_MJCF_JOINTS = (
    *(f"left_joint{index}" for index in range(1, 7)),
    *(f"right_joint{index}" for index in range(1, 7)),
    "left_left_finger",
    "right_left_finger",
)
DUAL_OPENYAM_MJCF_ACTUATORS = (
    *(f"left_joint{index}" for index in range(1, 7)),
    *(f"right_joint{index}" for index in range(1, 7)),
    "left_gripper",
    "right_gripper",
)
DUAL_OPENYAM_SIM_GRIPPER_RANGE = (0.0, 0.0475)
DUAL_OPENYAM_SIM_CAMERAS = ("top", "left", "right")


def dual_openyam_sim_spec(hardware_joints: tuple[str, ...]) -> RobotSimSpec:
    """Bind the dual-arm hardware joints to their MJCF joints and actuators."""
    if len(hardware_joints) != len(DUAL_OPENYAM_MJCF_JOINTS):
        raise ValueError(
            f"Dual OpenYAM sim spec expects {len(DUAL_OPENYAM_MJCF_JOINTS)} hardware "
            f"joints, got {len(hardware_joints)}"
        )
    return RobotSimSpec(
        robot_id="dual_openyam",
        hardware_joints=hardware_joints,
        model_joint_names=DUAL_OPENYAM_MJCF_JOINTS,
        model_actuator_names=DUAL_OPENYAM_MJCF_ACTUATORS,
    )


def dual_openyam_sim_module_kwargs(
    scene_path: str | Path = DUAL_OPENYAM_SCENE_PATH,
    *,
    headless: bool = True,
    camera_name: str = "top",
) -> dict[str, Any]:
    from dimos.robot.manipulators.dual_openyam.config import (
        DUAL_OPENYAM_HOME_JOINTS,
        DUAL_OPENYAM_JOINTS,
    )

    return {
        "address": scene_path,
        "headless": headless,
        "dof": len(DUAL_OPENYAM_JOINTS),
        "camera_name": camera_name,
        "base_frame_id": "world",
        # MuJoCo starts at qpos zero, which sits exactly on joint2/joint3's
        # lower limit and makes every plan fail on an invalid start state.
        "reset_joint_positions": [*DUAL_OPENYAM_HOME_JOINTS, 0.0, 0.0],
        "robot_sim_spec": dual_openyam_sim_spec(tuple(DUAL_OPENYAM_JOINTS)),
    }


def dual_openyam_sim_module(
    scene_path: str | Path = DUAL_OPENYAM_SCENE_PATH,
    *,
    headless: bool = True,
    camera_name: str = "top",
    **kwargs: Any,
) -> Blueprint:
    return MujocoSimModule.blueprint(
        **dual_openyam_sim_module_kwargs(scene_path, headless=headless, camera_name=camera_name),
        **kwargs,
    )

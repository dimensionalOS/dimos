#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""R1 Lite quest teleop: bimanual arm tracking, grippers and chassis.

Runs the full control stack plus two TeleopIK tasks that track the Quest
controllers while the primary buttons are held. Replaces
r1lite-coordinator while it runs; never run both at once. Motion needs
the connection armed through the preflight tool and RC mode 5.

The sim variant swaps the connection for mock adapters and renders the
arms in viser; drive it with a real headset or
scripts/r1lite_test/fake_quest_stream.py.

    dimos run r1lite-quest-teleop
    dimos run r1lite-quest-teleop-sim
"""

from __future__ import annotations

import os

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.galaxea.r1lite import config as cfg
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import (
    r1lite_control_base,
    r1lite_standard_tasks,
)
from dimos.robot.galaxea.r1lite.quest_module import R1LiteQuestTeleopModule
from dimos.robot.manipulators._modeling import base_pose, joint_names
from dimos.robot.manipulators.a1z.config import (
    A1Z_COLLISION_EXCLUSIONS,
    A1Z_PACKAGE_PATHS,
)

_TASK_NAMES = {"left": "teleop_left_arm", "right": "teleop_right_arm"}
_TELEOP_PRIORITY = 20  # preempts the servo holder (10) on the arm joints while engaged

# Merged Pink teleop architecture (PR #3237): the solver's velocity
# clamp and damping task own smoothness, configuration limits own
# safety, and the old chase-window/step-gate machinery is gone. Values
# mirror the A1Z quest config that hardware-validated as smooth, with
# two field-informed R1 Lite deviations: the timeout rides through
# vendor pose-stream gaps (2026-07-28), and seed_limit_tolerance covers
# the folded boot pose, measured up to 1.74 deg below the vendor URDF
# lower bound — the merged default of ~0.57 deg would still hard-fail
# there. The controlled point is the grasp-center URDF frame, 0.17 m
# past joint 6 (formerly the task-level tool_offset_m).
_ARM_IK_PARAMS = {
    "timeout": 1.5,
    # Ride through pose-stream gaps instead of cycling engage state.
}
_ARM_CONTROL_IK = {
    "max_velocity": 2.0,
    # Softened from 1.0 per the upstream Pink tuning guidance: a stiff
    # orientation objective makes translation stiff or unreachable near
    # awkward poses. Kept above the guidance floor because the 0.17 m
    # grasp-center lever otherwise lets the wrist swing the tool.
    "orientation_cost": 0.5,
    # The joint-centering task targets joint-range centers — for these
    # arms that is the raised-elbow guard pose — so a stronger weight is
    # a standing null-space bias away from wraps and limit corners.
    "joint_centering_cost": 1e-2,
    # Uniform joint treatment: these arms are 6-DOF — no null space —
    # so per-joint centering weights and manipulability objectives trade
    # directly against tracking (VR session 2026-08-10: shoulder motion
    # went stiff while the wrist overcompensated). The redundancy-shaping
    # tricks stay reserved for redundant arms.
    # Frame-task damping near singularities, raised from the 1e-4
    # default per the upstream tuning guidance: measured to cut
    # velocity jitter at the straight-arm pose by ~40% on its own,
    # with no effect on reach.
    "lm_damping": 1e-2,
    "seed_limit_tolerance": 0.05,
}


def _teleop_control_model(side: str) -> RobotModelConfig:
    coordinator_joints = cfg.LEFT_ARM_JOINTS if side == "left" else cfg.RIGHT_ARM_JOINTS
    model_joints = cfg.LEFT_ARM_URDF_JOINTS if side == "left" else cfg.RIGHT_ARM_URDF_JOINTS
    model_path = cfg.R1LITE_LEFT_ARM_MODEL if side == "left" else cfg.R1LITE_RIGHT_ARM_MODEL
    return RobotModelConfig(
        name=f"r1lite_{side}_teleop",
        model_path=model_path,
        joint_names=list(model_joints),
        base_link=f"{side}_arm_base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joints),
                base_link=f"{side}_arm_base_link",
                tip_link=f"{side}_arm_grasp_center",
            )
        ],
        joint_name_mapping=dict(zip(coordinator_joints, model_joints, strict=True)),
    )


def _teleop_tasks() -> list[TaskConfig]:
    left_model = _teleop_control_model("left")
    right_model = _teleop_control_model("right")
    return [
        TaskConfig(
            name=_TASK_NAMES["left"],
            type="teleop_ik",
            joint_names=list(cfg.LEFT_ARM_JOINTS),
            priority=_TELEOP_PRIORITY,
            params={
                "control_ik": {"robot_model": left_model, **_ARM_CONTROL_IK},
                "hand": "left",
                **_ARM_IK_PARAMS,
            },
        ),
        TaskConfig(
            name=_TASK_NAMES["right"],
            type="teleop_ik",
            joint_names=list(cfg.RIGHT_ARM_JOINTS),
            priority=_TELEOP_PRIORITY,
            params={
                "control_ik": {"robot_model": right_model, **_ARM_CONTROL_IK},
                "hand": "right",
                **_ARM_IK_PARAMS,
            },
        ),
    ]


r1lite_quest_teleop = autoconnect(
    # Headset video off: the JPEG encode starves the module loop and the
    # stalls surface as arm twitch and chassis dead-man dropouts.
    R1LiteQuestTeleopModule.blueprint(
        task_names=_TASK_NAMES,
        # Session recording stays opt-in: QUEST_RECORD=1 writes every raw
        # headset frame to a timestamped file under the host-visible logs
        # directory for offline replay; any other value is used verbatim
        # as the path.
        record_path=(
            "/app/logs/quest_record_%Y%m%d_%H%M%S.jsonl"
            if os.environ.get("QUEST_RECORD", "") == "1"
            else os.environ.get("QUEST_RECORD", "")
        ),
        motion_gain=1.3,
        # World-frame rotation deltas: the merged teleop task applies
        # delta rotations world-frame (the old task-side rotation_frame
        # pairing is gone), so hand-local deltas here would land in the
        # wrong frame and drift with wrist excursion. A1Z parity.
        local_rotation=False,
        position_deadband_m=0.02,
    ),
    # tracking_speed is the actual arm speed (the vendor tracker follows
    # each target at this rate); cameras off keeps decode off the control
    # path during teleop.
    r1lite_control_base(
        extra_tasks=_teleop_tasks(),
        connection_kwargs={"tracking_speed": 2.5, "enable_cameras": False},
    ),
).remappings(
    [
        (R1LiteQuestTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
        (R1LiteQuestTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
        (R1LiteQuestTeleopModule, "color_image", "head_left_color"),
    ]
)


def _sim_hardware() -> list[HardwareComponent]:
    return [
        HardwareComponent(
            hardware_id="r1lite",
            hardware_type=HardwareType.MANIPULATOR,
            joints=list(cfg.R1LITE_ARM_JOINTS),
            adapter_type="mock",
        ),
        HardwareComponent(
            hardware_id="chassis",
            hardware_type=HardwareType.BASE,
            joints=make_twist_base_joints("chassis"),
            adapter_type="mock_twist_base",
        ),
    ]


def _sim_arm_model(side: str, y_offset: float) -> RobotModelConfig:
    # Viser needs meshes; the exact R1 Lite models are geometry-stripped, so
    # the viewer renders the closely related A1Z arm and gripper as a visual
    # stand-in, with the visualization TCP 5 cm past its stock EEF. The
    # control tasks still use the exact R1 Lite kinematics and 0.17 m TCP.
    local_joints = joint_names(cfg.ARM_DOF, prefix="arm_joint")
    return RobotModelConfig(
        name=f"{side}_arm",
        model_path=cfg.R1LITE_VISER_ARM_MODEL,
        base_pose=base_pose(y=y_offset),
        joint_names=local_joints,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(local_joints),
                base_link="base_link",
                tip_link="r1lite_gripper_tip",
            )
        ],
        package_paths=A1Z_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=A1Z_COLLISION_EXCLUSIONS,
        joint_name_mapping={
            f"r1lite/{side}_arm_joint{i}": f"arm_joint{i}" for i in range(1, cfg.ARM_DOF + 1)
        },
        coordinator_task_name=f"traj_{side}_arm",
    )


def _sim_trajectory_tasks() -> list[TaskConfig]:
    return [
        TaskConfig(
            name=f"traj_{side}_arm",
            type="trajectory",
            joint_names=list(cfg.LEFT_ARM_JOINTS if side == "left" else cfg.RIGHT_ARM_JOINTS),
            priority=10,
        )
        for side in ("left", "right")
    ]


r1lite_quest_teleop_sim = autoconnect(
    R1LiteQuestTeleopModule.blueprint(task_names=_TASK_NAMES, local_rotation=False),
    ControlCoordinator.blueprint(
        hardware=_sim_hardware(),
        tasks=[*r1lite_standard_tasks(), *_teleop_tasks(), *_sim_trajectory_tasks()],
    ),
    ManipulationModule.blueprint(
        robots=[_sim_arm_model("left", 0.25), _sim_arm_model("right", -0.25)],
        visualization={"backend": "viser"},
    ),
).remappings(
    [
        (R1LiteQuestTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
        (R1LiteQuestTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
        (R1LiteQuestTeleopModule, "cmd_vel", "twist_command"),
    ]
)

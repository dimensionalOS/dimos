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

"""OpenArm Mini leader teleop for the bimanual OpenArm follower."""

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_connection, arm_remaps
from dimos.robot.manipulators.openarm.config import (
    openarm_bimanual_model_config,
    openarm_urdf_joints,
)
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.teleop.joycon.joycon_module import JoyConTeleopModule
from dimos.teleop.leader.calibration import (
    DEFAULT_LEADER_MOTOR_NAMES,
    LEADER_GRIPPER_MOTOR_NAME,
    LEADER_SIDES,
    LeaderSide,
)
from dimos.teleop.leader.coordinator import (
    LeaderCoordinator,
    leader_gripper_tasks,
    leader_task,
)
from dimos.teleop.leader.mapping import GripperMapping, JointMapping, SideMapping
from dimos.teleop.leader.module import LeaderTeleopModule

# Leader-follower needs human arm speeds, well above the WebXR IK profile.
_ARM_VELOCITY_PROFILE_RAD_S = (3.0, 3.0, 3.0, 3.0, 6.0, 6.0, 6.0)
# Inside the OpenArm URDF limits, which the connection enforces by refusing
# the whole command; clamping keeps a leader resting on a stop from stalling
# every joint. Joint 4 (elbow) has no negative range.
_JOINT_LIMITS: dict[LeaderSide, tuple[tuple[float, float], ...]] = {
    "left": (
        (-3.45, 1.35),
        (-3.30, 0.15),
        (-1.50, 1.50),
        (0.0, 2.40),
        (-1.50, 1.50),
        (-0.75, 0.75),
        (-1.50, 1.50),
    ),
    "right": (
        (-1.35, 3.45),
        (-0.15, 3.30),
        (-1.50, 1.50),
        (0.0, 2.40),
        (-1.50, 1.50),
        (-0.75, 0.75),
        (-1.50, 1.50),
    ),
}
# Squeezing the trigger this far closes the jaw fully.
_GRIPPER_TRAVEL_RADIANS = 1.0
# Low-pass on the leader targets: halves wrist roughness on the OpenArm for
# about 10 ms of lag; without it the wrists step at the 50 Hz target rate.
_COMMAND_FILTER_CUTOFF_HZ = 6.0
# A target every coordinator tick: each ramp the trajectory task runs is half
# the size of the 50 Hz one, and the feel on the arm is the best measured.
_LEADER_TICK_PERIOD_S = 0.01

OPENARM_ROUTER = os.environ.get("DIMOS_OPENARM_ROUTER", "tcp/127.0.0.1:7447")

_MODEL = openarm_bimanual_model_config(robot="arm")


def _side_mapping(side: LeaderSide, *, trigger: bool = True) -> SideMapping:
    # Leader joint N drives the connection's joint N of the same side.
    return SideMapping(
        joints=tuple(
            JointMapping(motor=motor, joint=f"arm/{joint}", lower=lower, upper=upper)
            for motor, joint, (lower, upper) in zip(
                DEFAULT_LEADER_MOTOR_NAMES,
                openarm_urdf_joints(side),
                _JOINT_LIMITS[side],
                strict=True,
            )
        ),
        gripper=GripperMapping(
            motor=LEADER_GRIPPER_MOTOR_NAME, travel_radians=_GRIPPER_TRAVEL_RADIANS
        )
        if trigger
        else None,
    )


MAPPINGS: dict[LeaderSide, SideMapping] = {side: _side_mapping(side) for side in LEADER_SIDES}
# Leaders whose trigger servo gave way to a Joy-Con: arm joints only.
JOYCON_MAPPINGS: dict[LeaderSide, SideMapping] = {
    side: _side_mapping(side, trigger=False) for side in LEADER_SIDES
}

_VELOCITY_LIMITS = {
    mapping.joint: limit
    for side in LEADER_SIDES
    for mapping, limit in zip(MAPPINGS[side].joints, _ARM_VELOCITY_PROFILE_RAD_S, strict=True)
}


def _leader(mappings: dict[LeaderSide, SideMapping] = MAPPINGS) -> Blueprint:
    return LeaderTeleopModule.blueprint(
        mappings=mappings,
        command_filter_cutoff_hz=_COMMAND_FILTER_CUTOFF_HZ,
        tick_period_s=_LEADER_TICK_PERIOD_S,
    )


def _follower(visualization: ViserVisualizationConfig | None = None) -> Blueprint:
    """The leader's targets over the OpenArm connection; --simulation swaps in a mock."""
    return autoconnect(
        LeaderCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=_MODEL.model,
            tasks=[
                leader_task(_MODEL.joint_names, velocity_limits=_VELOCITY_LIMITS),
                *leader_gripper_tasks(OpenArmConnection.robot.grippers),
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenArmConnection),
        ManipulationModule.blueprint(
            model=_MODEL,
            visualization=visualization if visualization is not None else {"backend": "viser"},
            gripper_tasks={f"{side}_arm": f"{side}_arm_gripper" for side in LEADER_SIDES},
        ),
    ).remappings(arm_remaps(OpenArmConnection, LeaderCoordinator))


teleop_openarm_mini = autoconnect(_leader(), _follower())

# Split deployment: leaders on an operator machine, follower on the robot.
# The robot stack owns the Coordinator name, so the leader half does not claim it.
teleop_openarm_mini_leader = _leader().global_config(
    zenoh_mode="client", serve_coordinator_rpc=False
)

teleop_openarm_mini_follower = _follower(ViserVisualizationConfig(host="0.0.0.0")).global_config(
    zenoh_connect=OPENARM_ROUTER
)

# Joy-Cons on the leaders instead of trigger servos: ZL/ZR drive the jaws,
# B/Y mark episodes, plus pauses and resumes following.
teleop_openarm_mini_joycon = autoconnect(
    _leader(JOYCON_MAPPINGS), JoyConTeleopModule.blueprint(), _follower()
)

teleop_openarm_mini_joycon_leader = autoconnect(
    _leader(JOYCON_MAPPINGS), JoyConTeleopModule.blueprint()
).global_config(zenoh_mode="client", serve_coordinator_rpc=False)

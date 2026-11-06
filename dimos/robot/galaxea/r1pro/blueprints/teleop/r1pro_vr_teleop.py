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

"""R1 Pro Quest WebXR teleop: both arms and the torso through one IK model.

The Quest's controllers drive the whole 18-DOF upper body (torso 4 + arms
7+7) through a single ``teleop_ik`` task on the standard control contract —
the same input, task, IK and model path regardless of backend — and each
trigger drives its own jaw through a dedicated feedback-gated gripper task.

The same controllers drive the holonomic chassis: the thumbsticks are
mapped to a body-frame Twist exactly as the Go2's are (one shared
``thumbstick_twist``, left stick translate, right stick yaw, with the
deadzone), published on the coordinator's ``twist_command`` by the
``vel_chassis`` velocity task's arbitration and routed to the connection's
``cmd_vel``. The teleop module publishes the twist every control-loop tick
from the latest stick state, so sticks centred, controllers absent or a
stale session send zero speed — never the previous one. Speeds are capped at
the R1 Pro ``base_range`` (±1/±1 m/s, ±2 rad/s) by the module's own scaling
and again by the connection's limits table; nothing bypasses the
coordinator's arbitration, its timeout (a silent twist stream zeroes the
chassis), its holds, or its E-STOP.

Backends (``make_r1pro_vr_teleop(backend=...)``):

- ``"fake"`` (default, and what ``dimos run r1pro-vr-teleop`` starts):
  ``FakeR1ProConnection``, an in-memory closed-loop stand-in on the same
  contract. No ROS 2, vendor stack, cameras, lidar, CAN or robot; the
  coordinator_joint_state bus stream and the rerun viewer show the Quest
  input moving the simulated 18 DOF and both jaws, and the chassis is
  integrated in memory (``base_state()``) so a thumbstick command is
  observable as pose and speed.
- ``"real"``: ``R1ProConnection`` (ROS 2 + vendor stack). Assembly is
  identical otherwise; selecting it is explicit. Bring-up on hardware is a
  separate, deliberate operation — see the R1 Pro README.

Safety semantics are the framework's, unchanged: middle-finger grips engage
(hands only, for the arms), releasing either grip, input going stale, or a
coordinator E-STOP ends the arm session and clears the captured references,
so no stale intent replays on the next engagement. The chassis follows the
sticks whenever the module runs (as on the Go2) and is zeroed by the module
on no input, by the velocity task's timeout when twist frames stop, and by
the connection's deadman when commands stop. A jaw whose feedback drops out
loses its emitted target; a target latched *during* an outage has never been
sent and goes out on the first tick whose feedback covers the jaw again —
that replay-on-recovery is the framework's current, deliberate-but-unreviewed
semantic, exercised and documented in the tests, and it is not a vendor
cancel or a physical E-STOP.

Usage:
    dimos run r1pro-vr-teleop            # fake backend, safe anywhere
    # real robot (explicit):
    #   python -c "from ...r1pro_vr_teleop import make_r1pro_vr_teleop; \\
    #       make_r1pro_vr_teleop(backend='real').build().loop()"
"""

from __future__ import annotations

from typing import Any, Literal

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.operator_hold_task.operator_hold_task import operator_hold_task
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import In, Out
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_visualization
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_COORDINATOR_MODEL,
    R1PRO_PLANAR_BASE,
    make_r1pro_model_config,
)
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_GRIPPER_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnection,
)
from dimos.robot.galaxea.r1pro.fake_connection import FakeR1ProConnection
from dimos.teleop.webxr.controller_types import Buttons
from dimos.teleop.webxr.extensions import ArmThumbstickTeleopModule

R1PRO_VR_TASK_NAME = "teleop_r1pro_vr"
R1PRO_VR_CHASSIS_TASK_NAME = "vel_chassis"

#: End-effector frames the Quest controllers steer, in the R1 Pro URDF.
R1PRO_VR_TARGET_FRAMES = {"left": "left_gripper_link", "right": "right_gripper_link"}

_r1pro_vr_pink = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=0.25,
)


def _teleop_task() -> TaskConfig:
    """Both hands on one 18-DOF model: torso and both arms solve together.

    ``max_joint_velocity_rad_s`` matches the vendor tracker's speed cap, so
    teleop never asks the real tracker for more than it follows.
    """
    return TaskConfig(
        name=R1PRO_VR_TASK_NAME,
        type="teleop_ik",
        priority=20,
        joint_names=list(R1PRO_UPPER_BODY_JOINTS),
        params={
            "robot_model": make_r1pro_model_config(),
            "bindings": [
                {"hand": hand, "target_frame": frame}
                for hand, frame in R1PRO_VR_TARGET_FRAMES.items()
            ],
            "pink": _r1pro_vr_pink,
            "timeout": 0.5,
            # The vendor tracker follows at config.tracking_speed (0.5 rad/s).
            "max_joint_velocity_rad_s": 0.5,
            "max_command_tracking_error_deg": 10.0,
            "joint_command_filter_cutoff_hz": 5.0,
        },
    )


def _gripper_task(name: str, joint: str, port: str) -> TaskConfig:
    """One jaw per task: feedback-gated, and scoped by port, not broadcast.

    ``stream_bind`` moves the task's ``gripper_command`` input off the shared
    broadcast port onto this hand's own trigger stream, so one message can
    never move both jaws and the unscoped ``gripper_command`` port has no
    route to either task. (``accept_broadcast=False`` would refuse scoped
    stream input too — it exists for invoke-driven manipulation stacks.)
    """
    return TaskConfig(
        name=name,
        type="gripper",
        joint_names=[joint],
        priority=20,
        params={"require_feedback": True},
        stream_bind={"gripper_command": port},
    )


def _chassis_task() -> TaskConfig:
    """The chassis' three axes, as speeds, through the shared velocity task.

    ``R1PRO_BASE_JOINTS`` are the axes the coordinator's base port exposes
    (``r1pro/base/vx|vy|wz``), claimed in ``VELOCITY`` mode and arbitrated
    like every other joint. The task's timeout is the second stop path: if
    twist frames stop arriving, it outputs zeros rather than holding the
    last speed, and the connection's own deadman halts after that.
    """
    return TaskConfig(
        name=R1PRO_VR_CHASSIS_TASK_NAME,
        type="velocity",
        joint_names=list(R1PRO_BASE_JOINTS),
        priority=10,
        params={"timeout": 0.2, "zero_on_timeout": True},
    )


class R1ProVrCoordinator(ControlCoordinator):
    """R1 Pro VR teleop coordinator: positions, jaw commands and base speeds.

    ``r1pro_cmd_vel`` is the base port the twist chain lands on: the
    coordinator maps ``twist_command`` onto the ``r1pro/base/*`` axes, the
    ``vel_chassis`` velocity task claims them, arbitration settles them, and
    the connection receives a clamped ``Twist`` on its ``cmd_vel``. The jaws
    are command-only (never held, never zero-filled, feedback never gates
    readiness).
    """

    r1pro_position_command: Out[JointState]
    r1pro_cmd_vel: Out[Twist]
    r1pro_joint_state: In[JointState]

    # Quest input streams (fed by the teleop module).
    left_cartesian_command: In[PoseStamped]
    right_cartesian_command: In[PoseStamped]
    teleop_buttons: In[Buttons]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]


def _vr_tasks() -> list[TaskConfig]:
    return [
        _teleop_task(),
        _gripper_task("r1pro_left_gripper", "r1pro/left_gripper", "left_gripper_command"),
        _gripper_task("r1pro_right_gripper", "r1pro/right_gripper", "right_gripper_command"),
        joint_trajectory_task(R1PRO_UPPER_BODY_JOINTS),
        _chassis_task(),
        operator_hold_task(),
    ]


def make_r1pro_vr_teleop(
    *,
    backend: Literal["fake", "real"] = "fake",
    **connection: Any,
) -> Blueprint:
    """Assemble the R1 Pro Quest VR teleop stack on the chosen backend.

    Both backends share the teleop module (poses, buttons, jaws and the
    thumbstick twist), the task set, the IK model and the coordinator; only
    the connection module differs. ``fake`` (the default) instantiates no
    ROS, vendor or sensor code at all. ``real`` selects ``R1ProConnection``;
    other keyword args are its config fields. ``backend="real"`` only
    assembles the blueprint — starting it against hardware is a separate,
    deliberate operation.
    """
    if backend == "fake":
        connection_bp = FakeR1ProConnection.blueprint(**connection)
        connection_cls: type = FakeR1ProConnection
    elif backend == "real":
        connection_bp = R1ProConnection.blueprint(**connection)
        connection_cls = R1ProConnection
    else:
        raise ValueError(f"unknown backend {backend!r}, expected 'fake' or 'real'")

    # Full stick deflection is the chassis' own limit on each axis, so the
    # teleop scaling never asks for more than base_range allows; the same
    # range is enforced again by the connection's limits table.
    vx, vy, wz = R1PRO_PLANAR_BASE.velocity_limits
    teleop_bp = ArmThumbstickTeleopModule.blueprint(
        linear_speed=min(vx, vy),
        angular_speed=wz,
    )

    return (
        autoconnect(
            r1pro_visualization(),
            teleop_bp,
            connection_bp,
            R1ProVrCoordinator.blueprint(
                instance_name="ControlCoordinator",
                tick_rate=100,
                model=R1PRO_COORDINATOR_MODEL,
                command_only_joints=list(R1PRO_GRIPPER_JOINTS),
                tasks=_vr_tasks(),
            ),
        )
        .remappings(
            [
                (connection_cls, "position_command", "r1pro_position_command"),
                (connection_cls, "cmd_vel", "r1pro_cmd_vel"),
                (R1ProVrCoordinator, "r1pro_joint_state", "joint_state"),
                # One teleop module feeds both halves of the robot: poses and
                # jaw openings to the coordinator's task inputs, and the
                # thumbstick twist onto the coordinator's twist bus, which
                # its vel_chassis task arbitrates onto r1pro_cmd_vel.
                (ArmThumbstickTeleopModule, "left_controller_output", "left_cartesian_command"),
                (ArmThumbstickTeleopModule, "right_controller_output", "right_cartesian_command"),
                (ArmThumbstickTeleopModule, "cmd_vel", "twist_command"),
            ]
        )
        .global_config(transport="zenoh")
    )


# Registered blueprint: the safe fake backend, which is what
# `dimos run r1pro-vr-teleop` starts. The trailing `.global_config(...)` is
# also the terminal the registry generator's AST scan recognises as a
# runnable blueprint, so keep it on this assignment.
r1pro_vr_teleop = make_r1pro_vr_teleop(backend="fake").global_config(transport="zenoh")

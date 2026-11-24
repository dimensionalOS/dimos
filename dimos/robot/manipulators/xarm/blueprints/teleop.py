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

"""Teleop blueprints for xArm6 and xArm7."""

from __future__ import annotations

from typing import Any

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators._modeling import joint_names
from dimos.robot.manipulators.common.coordinators import (
    ArmTwistCoordinator,
)
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.xarm.blueprints.basic import (
    SIMULATED,
    XArmCoordinator,
    xarm_coordinator,
    xarm_gripper_task,
)
from dimos.robot.manipulators.xarm.config import (
    XARM_ARMS,
    XArmType,
    make_xarm_model_config,
    xarm_link_frames,
    xarm_model,
    xarm_wrist_camera_transform,
)
from dimos.robot.manipulators.xarm.connection import XArmConnection
from dimos.robot.urdf_manager import UrdfManager
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


class XArmJogCoordinator(ArmTwistCoordinator, XArmCoordinator):
    """End-effector twists in on ``ee_twist_command``; one xArm named "arm" out."""


class XArmTeleopCoordinator(TeleopControlCoordinator, XArmCoordinator):
    """VR poses, twists and gripper openings in; one xArm named "arm" out."""


def xarm_jog_task(
    model: RobotModelConfig,
    target_frame: str,
    *,
    priority: int = 10,
    timeout: float = 0.3,
    max_joint_velocity_rad_s: float = 5.0,
) -> TaskConfig:
    """Move the arm's ``target_frame`` at the twist on ``ee_twist_command``.

    Args:
        model: The IK model; its joints are the ones moved.
        target_frame: The link the twist moves.
        priority: Higher wins a joint shared with another task.
        timeout: Stop this many seconds after the last twist; 0 never stops.
        max_joint_velocity_rad_s: The fastest any joint is moved, rad/s.
    """
    return TaskConfig(
        name=EEF_TWIST_TASK_NAME,
        type="eef_twist",
        joint_names=list(model.joint_names),
        priority=priority,
        params={
            "robot_model": model,
            "target_frame": target_frame,
            "timeout": timeout,
            "max_joint_velocity_rad_s": max_joint_velocity_rad_s,
            "max_command_tracking_error_deg": 10.0,
        },
    )


def _arm_model(arm: XArmType, **kwargs: Any) -> RobotModelConfig:
    """The arm's planning or IK model, its joints named ``arm/joint1`` ..."""
    dof, robot_type = XARM_ARMS[arm]
    return make_xarm_model_config(dof, robot_type=robot_type, robot="arm", **kwargs)


def _keyboard_teleop(arm: XArmType, *, gripper: bool = True) -> MetaBlueprint:
    """Jog an xArm's end from the keyboard and plan its motions; in MuJoCo under
    ``--simulation``. The Lite 6's gripper has no joint, so it is not driven."""
    dof = XARM_ARMS[arm][0]
    jog = xarm_jog_task(_arm_model(arm, add_gripper=False), f"link{dof}", timeout=0.0)
    return autoconnect(
        KeyboardTeleopModule.blueprint(),
        xarm_coordinator(
            arm,
            cls=XArmJogCoordinator,
            tasks=[jog, xarm_gripper_task()] if gripper else [jog],
            gripper=gripper,
            simulated=SIMULATED and arm != "lite6",
        ),
        ManipulationModule.blueprint(
            gripper_tasks={"manipulator": "arm_gripper"} if gripper else {},
            model=_arm_model(arm, add_gripper=True),
            visualization={"backend": "viser"},
        ),
    )


keyboard_teleop_xarm6 = autoconnect(_keyboard_teleop("xarm6"))
keyboard_teleop_xarm7 = autoconnect(_keyboard_teleop("xarm7"))
keyboard_teleop_lite6 = autoconnect(_keyboard_teleop("lite6", gripper=False))

coordinator_velocity_xarm6 = autoconnect(
    xarm_coordinator(
        "xarm6",
        tasks=[
            TaskConfig(
                name="velocity_arm",
                type="velocity",
                joint_names=joint_names(6, prefix="arm/joint"),
                priority=10,
            ),
        ],
        velocity=True,
    )
)

# The xArm7's ready pose for teleop (the planner's "home"), joint angles in radians:
# the gripper tip 0.40 m in front of the base, 0.23 m above it, pointing down.
XARM7_TELEOP_READY_JOINTS = [0.0, -0.247, 0.0, 0.909, 0.0, 1.15644, 0.0]


def _vr_teleop(
    arm: XArmType, model: RobotModelConfig, *, jog_timeout: float, trajectory_priority: int
) -> MetaBlueprint:
    """Drive an xArm from a VR controller (the right hand), a browser jog and the
    planner; in MuJoCo under ``--simulation``.

    VR (teleop_ik) preempts the jog (eef_twist) by priority; when VR is idle the
    always-active jog holds or drives the arm. The gripper task takes the right
    trigger.

    Args:
        arm: Which arm, e.g. "xarm7".
        model: The planning and VR IK model, joints named ``arm/joint1`` ...
        jog_timeout: Stop the jog this many seconds after its last twist; 0 never stops.
        trajectory_priority: The planned motions' priority; above the jog's 10
            lets a planned motion run while the jog holds the arm.
    """
    dof = XARM_ARMS[arm][0]
    return autoconnect(
        xarm_coordinator(
            arm,
            cls=XArmTeleopCoordinator,
            tasks=[
                TaskConfig(
                    name="teleop_xarm",
                    type="teleop_ik",
                    joint_names=list(model.joint_names),
                    priority=20,
                    params={
                        "robot_model": model,
                        "bindings": [{"hand": "right", "target_frame": "link_tcp"}],
                        "solver_type": PinkPoseTargetSolver,
                    },
                ),
                xarm_jog_task(
                    _arm_model(arm, add_gripper=False), f"link{dof}", timeout=jog_timeout
                ),
                xarm_gripper_task(stream_bind={"gripper_command": "right_gripper_command"}),
                joint_trajectory_task(list(model.joint_names), priority=trajectory_priority),
            ],
            simulated=SIMULATED,
        ),
        ManipulationModule.blueprint(
            gripper_tasks={"manipulator": "arm_gripper"},
            model=model,
            visualization={"backend": "viser"},
            static_transforms=[]
            if global_config.simulation
            else [xarm_wrist_camera_transform(dof)],
        ),
        # Serves the URDF and its meshes to viewers; the planner already puts the links on tf.
        UrdfManager.blueprint(model=model.model, publish_joint_tf=False),
    )


# A browser jog streams twists while a button is held; one that stops arriving
# for 0.3 s is dropped, so a lost page cannot keep the arm moving.
coordinator_teleop_xarm7 = autoconnect(
    _vr_teleop(
        "xarm7",
        _arm_model(
            "xarm7",
            add_gripper=True,
            tf_extra_links=xarm_link_frames(7),
            home_joints=XARM7_TELEOP_READY_JOINTS,
        ),
        jog_timeout=0.3,
        # Below VR, which takes over and aborts a planned motion.
        trajectory_priority=15,
    )
)
coordinator_teleop_xarm6 = autoconnect(
    _vr_teleop(
        "xarm6", _arm_model("xarm6", add_gripper=True), jog_timeout=0.0, trajectory_priority=10
    )
)


class XArmTwistCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper openings in, from the Desktop Manipulation app;
    one xArm named "arm" out, driven through ``XArmConnection``."""

    right_gripper_command: In[Float32]
    arm_position_command: Out[JointState]
    arm_velocity_command: Out[JointState]
    arm_joint_state: In[JointState]


# The connection module's ports, onto the coordinator's ports for the robot named "arm".
_ARM_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (XArmConnection, "position_command", "arm_position_command"),
    (XArmConnection, "velocity_command", "arm_velocity_command"),
    (XArmTwistCoordinator, "arm_joint_state", "joint_state"),
]


def _teleop(arm: XArmType) -> MetaBlueprint:
    """Jog an xArm's end and open and close its xArm Gripper from the Desktop
    Manipulation app, through ``XArmConnection``. ``UrdfManager`` serves the
    arm's URDF and puts its joints on tf, so the app can draw it.

    Wire it up with ``.remappings(_ARM_REMAPS)``. The arm's address is given at
    run time, after the blueprint name (``--xarmconnection.ip 192.168.1.185``)
    or as ``XARMCONNECTION__IP``.

    Args:
        arm: Which arm, e.g. "xarm7". It picks the coordinator's model, the IK
            model, the connection's URDF and the URDF the app draws, so they agree.
    """
    dof, robot_type = XARM_ARMS[arm]
    ik_model = make_xarm_model_config(dof, robot_type=robot_type, robot="arm", add_gripper=False)
    model = xarm_model(arm, gripper="xarm", robot="arm")
    declaration = ConnectionConfig(
        source="arm",
        model=model,
        joint_commands={joint.name: (Interface.POSITION,) for joint in movable_joints(model)},
    )
    return autoconnect(
        XArmTwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[declaration],
            tasks=[
                TaskConfig(
                    name=EEF_TWIST_TASK_NAME,
                    type="eef_twist",
                    joint_names=list(ik_model.joint_names),
                    priority=10,
                    params={
                        "robot_model": ik_model,
                        "target_frame": f"link{dof}",
                        # The app sends twists only while a jog is held: the
                        # end stops 0.3 s after the last one.
                        "timeout": 0.3,
                    },
                ),
                # drive_joint is open at 0 rad.
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/drive_joint"],
                    priority=20,
                    params={"open_at": "lower"},
                    stream_bind={"gripper_command": "right_gripper_command"},
                ),
            ],
        ),
        XArmConnection.blueprint(arm=arm, gripper="xarm", **declaration.model_dump()),
        UrdfManager.blueprint(model=model),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
xarm7_teleop = _teleop("xarm7").remappings(_ARM_REMAPS)
xarm6_teleop = _teleop("xarm6").remappings(_ARM_REMAPS)

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


"""Mixed-manipulator coordinator blueprints: an xArm6 and a Piper on one coordinator.

The xArm6's address is given at run time (``--xarm_arm.ip 192.168.1.185``);
the Piper is on CAN port can0.
"""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import In, Out
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators._modeling import joint_names, named_for_robot
from dimos.robot.manipulators.piper.config import make_piper_model_config, piper_model
from dimos.robot.manipulators.piper.connection import PiperConnection
from dimos.robot.manipulators.xarm.blueprints.basic import xarm_connection
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config


class _PiperXArmCoordinator(ControlCoordinator):
    xarm_arm_position_command: Out[JointState]
    xarm_arm_velocity_command: Out[JointState]
    xarm_arm_joint_state: In[JointState]
    piper_arm_position_command: Out[JointState]
    piper_arm_joint_state: In[JointState]


class _PiperXArmTeleopCoordinator(TeleopControlCoordinator, _PiperXArmCoordinator):
    pass


def _piper() -> tuple[ConnectionConfig, Blueprint]:
    """The Piper named "piper_arm", with its gripper, on can0."""
    model = piper_model(gripper=True, robot="piper_arm")
    declaration = ConnectionConfig(
        source="piper_arm",
        model=model,
        joint_commands={j.name: (Interface.POSITION,) for j in movable_joints(model)},
    )
    connection = PiperConnection.blueprint(
        instance_name="piper_arm", source="piper_arm", can_port="can0", gripper=True
    ).remappings(
        [
            ("piper_arm", "position_command", "piper_arm_position_command"),
            ("piper_arm", "joint_state", "piper_arm_joint_state"),
        ]
    )
    return declaration, connection


_xarm6_dual, _xarm6_dual_connection = xarm_connection(
    "xarm6", "xarm_arm", gripper=False, instance_name="xarm_arm"
)
_piper_dual, _piper_dual_connection = _piper()

coordinator_piper_xarm = autoconnect(
    _PiperXArmCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[_xarm6_dual, _piper_dual],
        tasks=[
            joint_trajectory_task(
                [
                    *joint_names(6, prefix="xarm_arm/joint"),
                    *joint_names(6, prefix="piper_arm/joint"),
                    # This coordinator has no gripper task; trajectories move the Piper's gripper.
                    "piper_arm/gripper",
                ]
            )
        ],
    ),
    _xarm6_dual_connection,
    _piper_dual_connection,
)

_xarm6_teleop, _xarm6_teleop_connection = xarm_connection(
    "xarm6", "xarm_arm", instance_name="xarm_arm"
)
_piper_teleop, _piper_teleop_connection = _piper()
_xarm6_teleop_model = make_xarm6_model_config(add_gripper=False, robot="xarm_arm")
_piper_teleop_model: RobotModelConfig = named_for_robot(make_piper_model_config(), "piper_arm")


def _teleop_ik(name: str, model: RobotModelConfig, hand: str, target_frame: str) -> TaskConfig:
    """Follow the VR controller in ``hand`` with ``target_frame`` of ``model``'s arm."""
    return TaskConfig(
        name=name,
        type="teleop_ik",
        joint_names=list(model.joint_names),
        priority=10,
        params={
            "robot_model": model,
            "bindings": [{"hand": hand, "target_frame": target_frame}],
            "solver_type": PinkPoseTargetSolver,
        },
    )


coordinator_teleop_dual = autoconnect(
    _PiperXArmTeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[_xarm6_teleop, _piper_teleop],
        tasks=[
            _teleop_ik("teleop_xarm", _xarm6_teleop_model, "left", "link6"),
            _teleop_ik("teleop_piper", _piper_teleop_model, "right", "gripper_base"),
            # drive_joint is open at 0 rad.
            TaskConfig(
                name="xarm_arm_gripper",
                type="gripper",
                joint_names=["xarm_arm/drive_joint"],
                priority=20,
                params={"open_at": "lower"},
                stream_bind={"gripper_command": "left_gripper_command"},
            ),
            TaskConfig(
                name="piper_arm_gripper",
                type="gripper",
                joint_names=["piper_arm/gripper"],
                priority=20,
                stream_bind={"gripper_command": "right_gripper_command"},
            ),
        ],
    ),
    _xarm6_teleop_connection,
    _piper_teleop_connection,
)

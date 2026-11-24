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

"""Basic Piper coordinator blueprints, and the Piper parts the teleop blueprints share."""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.piper.config import PIPER_DOF, PIPER_SIM_PATH, piper_model
from dimos.robot.manipulators.piper.connection import PiperConnection
from dimos.simulation.engines.mujoco_arm_connection import MujocoArmConnection

SIMULATED = bool(global_config.simulation)
# The Piper named "arm": its joints and its gripper, as the coordinator names them.
PIPER_ARM_JOINTS = [f"arm/joint{i}" for i in range(1, PIPER_DOF + 1)]
PIPER_JOINTS = [*PIPER_ARM_JOINTS, "arm/gripper"]
# Where the simulated arm starts, in rad: the scene's home keyframe. At 0, joint2
# and joint3 sit on a limit, and the sim's readings stray just past it.
_SIM_START_JOINTS = [0.0, 1.57, -1.3485, 0.0, 0.0, 0.0]


def piper_declaration() -> ConnectionConfig:
    """The coordinator's side of the Piper named "arm": joints ``arm/joint1`` to
    ``arm/joint6`` in rad and ``arm/gripper``, the finger gap in m (0 closed,
    0.07 open), all driven by position."""
    model = piper_model(gripper=True, robot="arm")
    return ConnectionConfig(
        source="arm",
        model=model,
        joint_commands={joint.name: (Interface.POSITION,) for joint in movable_joints(model)},
    )


def piper_arm(*, simulated: bool) -> Blueprint | MetaBlueprint:
    """The Piper named "arm" with its gripper, through ``PiperConnection``, or in
    MuJoCo behind the same ports, joint names and units when ``simulated``.

    The CAN port is given at run time (``--piperconnection.can-port can0``).
    """
    if not simulated:
        return PiperConnection.blueprint(gripper=True)
    # mujoco is optional; import it only when a blueprint runs the sim.
    from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

    return autoconnect(
        # The scene's gripper joint moves one finger; the model's is the gap between both.
        MujocoArmConnection.blueprint(
            model=piper_model(gripper=True), gripper_joint="gripper", gripper_scale=2.0
        ),
        MujocoSimModule.blueprint(
            address=str(PIPER_SIM_PATH),
            headless=False,
            dof=PIPER_DOF,
            reset_joint_positions=_SIM_START_JOINTS,
        ),
    )


def piper_remaps(
    coordinator: type[ModuleBase], *, simulated: bool
) -> list[tuple[type[ModuleBase], str, str]]:
    """The ports of ``piper_arm(simulated=...)``, onto ``coordinator``'s ports for
    the robot named "arm": ``arm_position_command`` and ``arm_joint_state``."""
    connection = MujocoArmConnection if simulated else PiperConnection
    return [
        (connection, "position_command", "arm_position_command"),
        (coordinator, "arm_joint_state", "joint_state"),
    ]


class PiperCoordinator(ControlCoordinator):
    """Joint targets out to one Piper named "arm", and its joints in."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
coordinator_piper = autoconnect(
    PiperCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[piper_declaration()],
        tasks=[joint_trajectory_task(PIPER_JOINTS)],
    ),
    piper_arm(simulated=SIMULATED),
).remappings(piper_remaps(PiperCoordinator, simulated=SIMULATED))

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

"""Dedicated R1 Pro SysID graph: no teleop, camera modules, or planner."""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In, Out
from dimos.core.transport import ZenohTransport
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_GRIPPER_JOINTS,
    R1ProConnection,
    R1ProConnectionConfig,
)
from dimos.robot.galaxea.r1pro.sysid import BASE, HZ, UPPER, R1ProSysIdTask


class R1ProSysIdCoordinator(ControlCoordinator):
    r1pro_position_command: Out[JointState]
    r1pro_velocity_command: Out[JointState]
    r1pro_joint_state: In[JointState]
    r1pro_diagnostics: In[String]

    def _publish_command(self, robot: str, values: dict[str, float]) -> None:
        task = self._tasks.get("sysid")
        if isinstance(task, R1ProSysIdTask):
            # Serialize publication admission with abort. The inherited writer
            # still owns ROS-independent command routing and the E-stop gate.
            publish = super()._publish_command
            task.admit_write(lambda: publish(robot, values))


RECORD_STREAMS = {
    "r1pro_diagnostics": String,
    "r1pro_position_command": JointState,
    "r1pro_velocity_command": JointState,
    "joint_state": JointState,
    "odometry": Odometry,
    "imu_chassis": Imu,
    "imu_torso": Imu,
}

_declaration = R1ProConnectionConfig(
    boot_vendor_stack=False,
    diagnostic_tap=True,
    enable_head_depth=False,
    publish_odom_tf=False,
)
_declaration.joint_commands = {
    joint: interfaces
    for joint, interfaces in _declaration.joint_commands.items()
    if joint not in R1PRO_GRIPPER_JOINTS
}

r1pro_sysid = (
    autoconnect(
        R1ProConnection.blueprint(**_declaration.model_dump()),
        R1ProSysIdCoordinator.blueprint(
            instance_name="R1ProSysIdCoordinator",
            tick_rate=HZ,
            reading_timeout_s=0.15,
            publish_joint_state=False,
            connections=[
                ConnectionConfig.model_validate(
                    _declaration.model_dump(include=set(ConnectionConfig.model_fields))
                )
            ],
            tasks=[TaskConfig(name="sysid", type="r1pro_sysid", joint_names=[*UPPER, *BASE])],
        ),
    )
    .remappings(
        [
            (R1ProConnection, "position_command", "r1pro_position_command"),
            (R1ProConnection, "velocity_command", "r1pro_velocity_command"),
            (R1ProSysIdCoordinator, "r1pro_joint_state", "joint_state"),
        ]
    )
    .transports(
        {
            (name, msg_type): ZenohTransport.spec(f"/{name}", msg_type)
            for name, msg_type in RECORD_STREAMS.items()
        }
    )
    .global_config(transport="zenoh", n_workers=2, viewer="none")
)

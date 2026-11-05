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

"""G1 GR00T control, initially unarmed with policy output in dry-run."""

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.requirements import g1_hardware
from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection
from dimos.utils.data import LfsPath

unitree_g1_groot_wbc = (
    autoconnect(
        G1WholeBodyConnection.blueprint(),
        ControlCoordinator.blueprint(
            hardware=[
                HardwareComponent(
                    hardware_id="g1",
                    hardware_type=HardwareType.WHOLE_BODY,
                    joints=g1_joints,
                    adapter_type="transport_lcm",
                    wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
                ),
            ],
            tasks=[
                TaskConfig(
                    name="groot_wbc",
                    type="g1_groot_wbc",
                    joint_names=g1_legs_waist,
                    priority=50,
                    auto_start=True,
                    params={
                        "model_path": LfsPath("groot"),
                        "hardware_id": "g1",
                        "auto_arm": False,
                        "auto_dry_run": True,
                        "default_ramp_seconds": 10.0,
                        # The 100 Hz coordinator runs the trained policy at 50 Hz.
                        "decimation": 2,
                    },
                ),
                joint_trajectory_task(
                    g1_arms,
                    priority=10,
                    velocity_limits={name: 1.0 for name in g1_arms},
                ),
            ],
        ),
    )
    .remappings([(ControlCoordinator, "twist_command", "cmd_vel")])
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/g1/cmd_vel", Twist),
            ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
            ("motor_states", JointState): LCMTransport("/g1/motor_states", JointState),
            ("imu", Imu): LCMTransport("/g1/imu", Imu),
            ("motor_command", MotorCommandArray): LCMTransport(
                "/g1/motor_command", MotorCommandArray
            ),
        }
    )
    .global_config(robot_model="unitree_g1")
    .requirements(g1_hardware)
)

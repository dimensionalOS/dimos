#!/usr/bin/env python3
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

"""Go2 tripod RL policy on REAL hardware via Go2WholeBodyConnection.

Real-hardware counterpart to [go2_tripod_sim]. Same RLPolicyTask, same
joint conventions, swap the sim MuJoCo adapter for the DDS-backed Unitree
connection.

Usage:
    ROBOT_INTERFACE=<nic> dimos run go2-tripod-real
"""

from __future__ import annotations

import os

from dimos.control.components import HardwareComponent, HardwareType, make_quadruped_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.go2.wholebody_connection import Go2WholeBodyConnection

_HW = "go2"
_DEFAULT_POLICY = "data/2026-05-28_11-42-04/model_1200.pt"

# Training env PD gains per leg (hip, thigh, calf).
_KP = (20.0, 20.0, 40.0) * 4
_KD = (1.0, 1.0, 2.0) * 4

_joints = make_quadruped_joints(_HW)


go2_tripod_real = autoconnect(
    Go2WholeBodyConnection.blueprint(
        release_sport_mode=True,
        network_interface=os.getenv("ROBOT_INTERFACE", ""),
    ),
    ControlCoordinator.blueprint(
        # Bring-up: start at 5Hz, climb to 50Hz once stable.
        tick_rate=5,
        hardware=[
            HardwareComponent(
                hardware_id=_HW,
                hardware_type=HardwareType.WHOLE_BODY,
                joints=_joints,
                adapter_type="transport_lcm",
                wb_config=WholeBodyConfig(kp=_KP, kd=_KD),
            ),
        ],
        tasks=[
            TaskConfig(
                name="rl_walk_go2",
                type="rl_policy_go2",
                joint_names=_joints,
                priority=10,
                auto_start=True,
                params={
                    "policy_path": _DEFAULT_POLICY,
                    "hardware_id": _HW,
                    "inference_period": 0.02,
                    "mask_fr": False,
                    "device": "cpu",
                    "pre_ramp_hold_seconds": 2.0,
                    "activation_ramp_seconds": 3.0,
                    "post_ramp_hold_seconds": 2.0,
                },
            ),
        ],
    ),
).transports(
    {
        ("motor_states", JointState): LCMTransport("/go2/motor_states", JointState),
        ("imu", Imu): LCMTransport("/go2/imu", Imu),
        ("motor_command", MotorCommandArray): LCMTransport("/go2/motor_command", MotorCommandArray),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/go2/joint_command", JointState),
    }
)


__all__ = ["go2_tripod_real"]

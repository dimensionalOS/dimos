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

"""Go2 tripod RL policy in MuJoCo + Quest teleop (joysticks + FR arm).

Loads the trained 3-leg velocity policy ([data/2026-05-28_11-42-04/model_1200.pt])
and walks the Go2 in MuJoCo. The FR ("right arm") is owned by a
TeleopIKTask (translation-only) gated by the right primary (A) button via
the upstream Quest module; the policy controls the other 9 leg joints.

Quest controller mapping:
    left thumbstick   -> twist linear (drive)
    right thumbstick X -> twist angular.z (yaw)
    right A (hold)    -> engage FR-leg pose tracking
    right hand pose   -> FR_hip / FR_thigh / FR_calf deltas

Connect the Quest at https://<host>:8443/teleop (cert auth).

Usage:
    dimos run go2-tripod-sim
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.learning.inference.obs_builder import GO2_JOINT_ORDER
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_extensions import JoystickTwistTeleopModule
from dimos.teleop.quest.quest_types import Buttons

_DEFAULT_POLICY = "data/2026-05-28_11-42-04/model_1200.pt"
_DEFAULT_MJCF = "data/go2_mjlab/xmls/scene_go2.xml"

# Training env PD gains per leg (hip, thigh, calf).
_KP = (20.0, 20.0, 40.0) * 4
_KD = (1.0, 1.0, 2.0) * 4

_HW = "go2"
_joints = [f"{_HW}/{j}" for j in GO2_JOINT_ORDER]
_FR_JOINTS = [f"{_HW}/FR_hip", f"{_HW}/FR_thigh", f"{_HW}/FR_calf"]

# Quest stamps this name into right-controller PoseStamped frame_id for routing.
_FR_TASK_NAME = "fr_teleop"


go2_tripod_sim = autoconnect(
    ControlCoordinator.blueprint(
        tick_rate=100,
        hardware=[
            HardwareComponent(
                hardware_id=_HW,
                hardware_type=HardwareType.WHOLE_BODY,
                joints=_joints,
                adapter_type="sim_mujoco_go2",
                adapter_kwargs={
                    "mjcf_path": _DEFAULT_MJCF,
                    "render": True,
                },
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
                    "mask_fr": True,
                    "device": "cpu",
                    # Ramps from MJCF "lie" keyframe to standing.
                    "pre_ramp_hold_seconds": 0.0,
                    "activation_ramp_seconds": 2.5,
                    "post_ramp_hold_seconds": 0.0,
                },
            ),
            TaskConfig(
                name=_FR_TASK_NAME,
                type="teleop_ik",
                joint_names=_FR_JOINTS,
                priority=20,
                auto_start=True,
                params={
                    "model_path": "data/go2_mjlab/go2_fr_leg.urdf",
                    "ee_joint_id": 3,
                    "hand": None,
                    "translation_only": True,
                    "timeout": 0.3,
                    "max_joint_delta_deg": 30.0,
                },
            ),
        ],
    ),
    JoystickTwistTeleopModule.blueprint(
        linear_speed=1.0,
        angular_speed=0.8,
        task_names={"right": _FR_TASK_NAME},
    ),
).transports(
    {
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("cartesian_command", PoseStamped): LCMTransport("/cartesian_command", PoseStamped),
        ("right_controller_output", PoseStamped): LCMTransport("/cartesian_command", PoseStamped),
        ("buttons", Buttons): LCMTransport("/buttons", Buttons),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


__all__ = ["go2_tripod_sim"]

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

"""MicroDuck policy and simulator constants.

The order and home pose below are the deployed policy ABI. Keep them in one
place: the task, whole-body component, and MuJoCo binding all consume these
same tuples.
"""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.hardware.spec import JointLimits
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec
from dimos.utils.data import LfsPath

MICRODUCK_HARDWARE_ID = "microduck"

MICRODUCK_JOINT_SUFFIXES: tuple[str, ...] = (
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
)

MICRODUCK_JOINTS: tuple[str, ...] = tuple(
    f"{MICRODUCK_HARDWARE_ID}/{name}" for name in MICRODUCK_JOINT_SUFFIXES
)

MICRODUCK_HOME: tuple[float, ...] = (
    0.0,
    -0.08726646259971647,
    -0.457924,
    -0.004940,
    0.452984,
    0.3490658503988659,
    0.3490658503988659,
    0.0,
    0.0,
    0.0,
    0.08726646259971647,
    0.457924,
    0.004940,
    -0.452984,
)

MICRODUCK_POSITION_LOWER: tuple[float, ...] = (
    -0.4363323129985824,
    -0.3839724354386992,
    -1.570796326794949,
    -1.5707963267948983,
    -1.5707963267949063,
    -1.5707963267948974,
    -1.5707963267948966,
    -2.9670597283903613,
    -0.43633231299858327,
    -0.523598775598297,
    -0.3839724354387507,
    -1.57079632679494,
    -1.5707963267949339,
    -1.5707963267949028,
)

MICRODUCK_POSITION_UPPER: tuple[float, ...] = (
    0.5235987755982988,
    0.38397243543880577,
    1.5707963267948442,
    1.5707963267948948,
    1.5707963267948868,
    1.0471975511965967,
    1.5707963267948966,
    2.9670597283903595,
    0.4363323129985815,
    0.43633231299858416,
    0.38397243543875426,
    1.570796326794853,
    1.5707963267948593,
    1.5707963267948903,
)

MICRODUCK_ASSET = LfsPath("microduck")
MICRODUCK_SCENE = MICRODUCK_ASSET / "scene.xml"
MICRODUCK_ROBOT_MJCF = MICRODUCK_ASSET / "robot_groundcontact.xml"
MICRODUCK_MESHDIR = MICRODUCK_ASSET / "assets"
MICRODUCK_POLICY_DIR = MICRODUCK_ASSET / "policies"

MICRODUCK_SIM_SPEC = RobotSimSpec(
    robot_id=MICRODUCK_HARDWARE_ID,
    hardware_joints=MICRODUCK_JOINTS,
    root_body_names=("trunk_base",),
    root_joint_names=("trunk_base_freejoint",),
    require_floating_base=True,
    model_joint_names=MICRODUCK_JOINT_SUFFIXES,
    model_actuator_names=MICRODUCK_JOINT_SUFFIXES,
    imu_quat_names=("orientation",),
    imu_gyro_names=("imu_ang_vel", "angular-velocity"),
    imu_accel_names=("imu_accel",),
    imu_linvel_names=("imu_lin_vel",),
    require_imu=True,
)


def make_microduck_sim_hardware(
    address: str | Path = MICRODUCK_SCENE,
) -> HardwareComponent:
    """Build the coordinator component for the native-actuator MuJoCo sim."""

    return HardwareComponent(
        hardware_id=MICRODUCK_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(MICRODUCK_JOINTS),
        adapter_type="sim_mujoco_microduck",
        address=address,
        limits=JointLimits(
            position_lower=MICRODUCK_POSITION_LOWER,
            position_upper=MICRODUCK_POSITION_UPPER,
            velocity_max=(None,) * len(MICRODUCK_JOINTS),
        ),
    )

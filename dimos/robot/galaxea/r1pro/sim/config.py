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

"""Mobile R1 Pro model and demonstration servo tuning."""

from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_BODY_AXES,
    R1PRO_COLLISION_EXCLUSIONS,
    R1PRO_PLANAR_BASE,
)
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS
from dimos.robot.galaxea.r1pro.sim.model import (
    FINGER_JOINTS,
    MODEL_PATH,
    STEER_JOINTS,
    WHEEL_JOINTS,
)
from dimos.sim2.description import Drive, Instance, SceneConfig
from dimos.sim2.module import ControllerConfig

UPPER_JOINTS = tuple(f"r1pro/{name}" for name in UPPER_BODY_JOINTS)
SERVO_JOINTS = (*UPPER_JOINTS, *FINGER_JOINTS)
JOINTS = (*SERVO_JOINTS, *STEER_JOINTS, *WHEEL_JOINTS)
HOME = (0.0,) * 18
# SI torque PD gains for MuJoCo, not WIM's raw firmware kp=2/kd=25 fields.
# Gravity feedforward supplies the static load; gains remain uncalibrated.
KP = (800.0, 800.0, 800.0, 800.0) + (180.0,) * 14 + (1000.0,) * 4 + (120.0,) * 3 + (0.0,) * 3
KD = (60.0, 60.0, 60.0, 60.0) + (12.0,) * 14 + (10.0,) * 4 + (12.0,) * 3 + (3.0,) * 3
CONTROL = ControllerConfig("r1pro/servo", 4_000_000, JOINTS)
# Simulation tracking profile. WIM's 1.5 rad/s demo setting is not a measured
# hardware maximum; its per-joint ceilings are 3/3/3/5/5/5/5 rad/s.
ARM_RATE = 1.5
TORSO_RATE = 0.5
UPPER_RATES = (TORSO_RATE,) * 4 + (ARM_RATE,) * 14
MODEL = RobotModel.from_file(MODEL_PATH)
CONTROL_MODEL = MODEL.with_default_joint_acceleration_limit(2.0).with_fixed_joints(
    *STEER_JOINTS, *WHEEL_JOINTS, FINGER_JOINTS[1], FINGER_JOINTS[3]
)
for _side in ("left", "right"):
    _finger = f"r1pro/{_side}_gripper_finger_joint1"
    CONTROL_MODEL = CONTROL_MODEL.with_joint_position_limits(
        _finger, lower=0.0, upper=0.1
    ).with_renamed_joints({_finger: f"r1pro/{_side}_gripper"})
CONTROL_MODEL = CONTROL_MODEL.with_added_joints(
    *(
        JointDescription(
            name=name,
            type="continuous" if i == 2 else "prismatic",
            parent_link="r1pro/base_link",
            child_link=f"r1pro/body_control_{i}",
            axis=axis,
            velocity=velocity,
            acceleration=acceleration,
        )
        for i, (name, axis, velocity, acceleration) in enumerate(
            zip(
                R1PRO_BODY_AXES,
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                R1PRO_PLANAR_BASE.velocity_limits,
                R1PRO_PLANAR_BASE.acceleration_limits,
                strict=True,
            )
        )
    )
)
SCENE = SceneConfig(
    instances=(
        Instance(
            "r1pro",
            xyz=(0, 0, 0.005),
            floating=True,
            drives=tuple(
                Drive(name, max(kp, 1.0), kd, armature=0.02)
                for name, kp, kd in zip(JOINTS, KP, KD, strict=True)
            ),
            collision_exclusions=tuple(
                (f"r1pro/{a}", f"r1pro/{b}")
                for a, b in R1PRO_COLLISION_EXCLUSIONS
                if "gmsl" not in a + b and "d405" not in a + b
            ),
        ),
    )
)

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

"""OpenArm hardware and planning model configuration."""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType
from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import base_pose

OPENARM_DESCRIPTION_URL = "https://github.com/enactic/openarm_description"
OPENARM_DESCRIPTION_REF = "1fba2cbc05001f05b4514120b70130b4ac06f409"

OPENARM_DESCRIPTION_SOURCE = RobotDescriptionSource(
    url=OPENARM_DESCRIPTION_URL,
    ref=OPENARM_DESCRIPTION_REF,
)
OPENARM_DESCRIPTION_ROOT = OPENARM_DESCRIPTION_SOURCE / "."
OPENARM_BIMANUAL_XACRO = (
    OPENARM_DESCRIPTION_SOURCE
    / "assets"
    / "robot"
    / "openarm_v2.0"
    / "urdf"
    / "openarm_v20.urdf.xacro"
)

OPENARM_BIMANUAL_MODEL = RobotModel.from_file(
    OPENARM_BIMANUAL_XACRO,
    package_paths={"openarm_description": OPENARM_DESCRIPTION_ROOT},
    xacro_args={
        "robot_preset": "default_bimanual",
        "emit_grasp_frame": "true",
    },
).with_fixed_joints(
    "openarm_left_finger_joint1",
    "openarm_left_finger_joint2",
    "openarm_right_finger_joint1",
    "openarm_right_finger_joint2",
)

OPENARM_DOF = 7
OPENARM_HARDWARE_ID = "openarm"
OPENARM_SIDES = ("left", "right")
# Order must match OpenArmDamiaoAdapter.joint_names: all arm groups in
# declaration order (left then right), then all grippers.
OPENARM_LEFT_ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, OPENARM_DOF + 1)]
OPENARM_RIGHT_ARM_JOINTS = [f"openarm_right_joint{i}" for i in range(1, OPENARM_DOF + 1)]
OPENARM_ARM_JOINTS = [*OPENARM_LEFT_ARM_JOINTS, *OPENARM_RIGHT_ARM_JOINTS]
OPENARM_GRIPPER_JOINTS = ["left_arm/gripper", "right_arm/gripper"]
OPENARM_JOINTS = [*OPENARM_ARM_JOINTS, *OPENARM_GRIPPER_JOINTS]
OPENARM_HOME_JOINTS = [0.0] * len(OPENARM_ARM_JOINTS)
OPENARM_GRIPPER_COLLISION_EXCLUSIONS = [
    (f"openarm_{side}_ee_link1", f"openarm_{side}_ee_link2") for side in OPENARM_SIDES
]

# MIT gains measured on v1.0 hardware, carried over as the v2.0 starting
# point: with gravity compensation active the PD terms only handle transient
# tracking, and high kd excites gearbox buzz. Gripper slots bypass MIT
# control, so their gains are 0.
_ARM_KP = (100.0, 100.0, 80.0, 80.0, 60.0, 60.0, 60.0)
_ARM_KD = (1.5, 1.5, 1.0, 1.0, 0.8, 0.8, 0.8)


def validate_side(side: str) -> None:
    if side not in OPENARM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def openarm_arm_joints(side: str) -> list[str]:
    validate_side(side)
    return [f"{side}_arm/joint{i}" for i in range(1, OPENARM_DOF + 1)]


def openarm_urdf_joints(side: str) -> list[str]:
    validate_side(side)
    return [f"openarm_{side}_joint{i}" for i in range(1, OPENARM_DOF + 1)]


def openarm_hardware(
    *,
    left_can_port: str | None = None,
    right_can_port: str | None = None,
) -> HardwareComponent:
    """Use mock hardware unless both physical CAN interfaces are explicit."""
    if (left_can_port is None) != (right_can_port is None):
        raise ValueError("OpenArm hardware requires both left and right CAN ports")

    adapter_type = "openarm_damiao" if left_can_port is not None else "mock_whole_body"
    adapter_kwargs: dict[str, object] = {}
    limits: JointLimits | None = None
    if left_can_port is not None and right_can_port is not None:
        adapter_kwargs["runtime_config"] = DamiaoRuntimeConfig(
            bus_addresses={"left": left_can_port, "right": right_can_port},
            gravity_comp=True,
        )
    else:
        limits = JointLimits(
            position_lower=[*([None] * len(OPENARM_ARM_JOINTS)), 0.0, 0.0],
            position_upper=[*([None] * len(OPENARM_ARM_JOINTS)), 1.0, 1.0],
            velocity_max=[None] * len(OPENARM_JOINTS),
        )
    return HardwareComponent(
        hardware_id=OPENARM_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(OPENARM_JOINTS),
        adapter_type=adapter_type,
        auto_enable=True,
        limits=limits,
        adapter_kwargs=adapter_kwargs,
        wb_config=WholeBodyConfig(
            kp=(*_ARM_KP, *_ARM_KP, 0.0, 0.0),
            kd=(*_ARM_KD, *_ARM_KD, 0.0, 0.0),
        ),
    )


def openarm_bimanual_model_config() -> RobotModelConfig:
    """Build the single fourteen-joint planning model with one group per arm.

    Collision exclusions cannot span robots, so both arms plan as one robot.
    """
    canonical_joint_names = [*openarm_urdf_joints("left"), *openarm_urdf_joints("right")]
    return RobotModelConfig(
        model=OPENARM_BIMANUAL_MODEL,
        base_pose=base_pose(),
        joint_names=canonical_joint_names,
        base_link="openarm_body_link0",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=tuple(openarm_urdf_joints("left")),
                base_link="openarm_body_link0",
                tip_link="openarm_left_grasp_frame",
            ),
            PlanningGroupDefinition(
                name="right_arm",
                joint_names=tuple(openarm_urdf_joints("right")),
                base_link="openarm_body_link0",
                tip_link="openarm_right_grasp_frame",
            ),
            PlanningGroupDefinition(
                name="both_arms",
                joint_names=tuple(canonical_joint_names),
                base_link="openarm_body_link0",
            ),
        ],
        collision_exclusion_pairs=OPENARM_GRIPPER_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        max_velocity=0.5,
        max_acceleration=1.0,
        home_joints=list(OPENARM_HOME_JOINTS),
    )

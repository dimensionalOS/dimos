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

"""Pinned MakerMods Metal geometry, arm planning coordinates and robot data."""

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.gripper import GripperConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import named_for_robot
from dimos.robot.manipulators.damiao.robot import (
    DamiaoBus,
    DamiaoGripper,
    DamiaoMotor,
    DamiaoProfile,
    DamiaoRobot,
)

METAL_SOURCE = RobotDescriptionSource(
    url="https://github.com/makermods-robotics/metal-python-ros",
    ref="bccf3c82a21453a7b1784a94966456e16ff0f008",
)
METAL_URDF = METAL_SOURCE / "metal_ros2/src/metal_description/urdf/metal_with_gripper.urdf"
METAL_MESHDIR = METAL_SOURCE / "metal_ros2/src/metal_description/meshes"
# The vendor URDF puts joint3's lower stop at exactly 0 rad, while the encoder
# reports -0.05050 rad when the arm rests on that stop. Widen the stop so the
# measured pose is inside the model, and a hold there does not push to 0 rad.
METAL_JOINT3_LIMITS = (-0.06, 3.14)
METAL_MODEL = RobotModel.from_file(
    METAL_URDF,
    package_paths={"metal_description": METAL_SOURCE / "metal_ros2/src/metal_description"},
).with_joint_position_limits("joint3", lower=METAL_JOINT3_LIMITS[0], upper=METAL_JOINT3_LIMITS[1])
METAL_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
METAL_HARDWARE_ID = "metal"
METAL_GRIPPER_NAME = "gripper"
#: The jaw's finger joints, in m: joint7 is driven and the URDF has joint8 copy
#: it. joint7 is fully open at its lower end and closed at 0.
METAL_FINGER_JOINTS = ("joint7", "joint8")
#: joint7 fully open, then closed, metres.
METAL_GRIPPER_RANGE = (-0.05, 0.0)
#: The arm links, for a viewer that draws each URDF link at its tf frame.
#: gripper_base is the planning tip and is already on tf. The fingers stay
#: off: the planner's state has no finger joint, so it would put them at the
#: closed pose whatever the jaw does; a viewer places them from the joint
#: state instead.
METAL_TF_LINKS = ["base_link", *(f"link{index}" for index in range(1, 7))]
#: MIT-mode position and velocity gains of the six arm motors, N m/rad and N m s/rad.
METAL_KP = [100.0, 100.0, 100.0, 50.0, 20.0, 20.0]
METAL_KD = [5.0, 5.0, 5.0, 5.0, 0.6, 0.6]
# Where the jaw's parts sit along its approach, measured from the planning tip
# (gripper_base, the mount), metres, for heuristic grasps. The pads are the last
# 2.7 cm of the fingers, the faces that meet when the jaw closes; they and the
# gripper body's front face are ahead of the tip, so "behind" is negative.
METAL_HEURISTIC_AIM = {
    "pad_behind_tcp": -0.078,
    "fingertip_reach": 0.092,
    "palm_behind_tcp": -0.009,
}

METAL_MJCF_PATH = Path(__file__).parent / "assets" / "metal.xml"
METAL_BENCH_PATH = Path(__file__).parent / "assets" / "bench.xml"
#: Free bodies on the bench, by MJCF body name; the sim reports their poses.
METAL_BENCH_OBJECTS = ("cube", "cylinder", "bowl")
#: Arm joints, rad, as commanded: the jaw points straight down with its mount
#: about 0.33 m above the bench and 0.28 m out from the base, where the wrist
#: camera sees all three objects. Without gravity torque the arm settles a
#: little below this.
METAL_SIM_HOME = [0.0, -1.87, 2.15, -1.85, 0.0, 0.0]
#: Edge of one cell of the simulated bench's voxel map, metres; the mapper and
#: the planner's octree share it.
METAL_SIM_VOXEL_SIZE = 0.025
#: How close to the bench, metres along a camera ray, the map never clears a
#: voxel: just over one cell, so a cube that moved off a spot clears from it.
METAL_SIM_MAP_GRACE = 0.03
# Custom firmware profiles: the stock DM scaling does not match these motors.
METAL_PROFILE_LOW = DamiaoProfile(0x8000, p_max=6.28, v_max=10.0, t_max=30.0)
METAL_PROFILE_SHOULDER = DamiaoProfile(0x8001, p_max=6.28, v_max=10.0, t_max=120.0)
METAL_PROFILE_HIGH = DamiaoProfile(0x8002, p_max=6.28, v_max=30.0, t_max=20.0)
METAL_PROFILES = (
    METAL_PROFILE_LOW,
    METAL_PROFILE_SHOULDER,
    METAL_PROFILE_LOW,
    METAL_PROFILE_LOW,
    METAL_PROFILE_HIGH,
    METAL_PROFILE_HIGH,
)


def make_metal_model_config(
    *,
    robot: str | None = None,
    home_joints: list[float] | None = None,
    tf_extra_links: list[str] | None = None,
) -> RobotModelConfig:
    """Plan the arm to the gripper mount; the jaw moves on its own finger joint.

    Args:
        robot: Name the moving joints under this robot, e.g. "arm" gives
            "arm/joint1" and "arm/joint7".
        home_joints: The six arm joints of the "home" preset, rad; none by default.
        tf_extra_links: URDF links the planner also publishes on tf, besides
            the gripper mount; ``METAL_TF_LINKS`` for the arm links.
    """
    driven = f"{robot}/{METAL_FINGER_JOINTS[0]}" if robot else METAL_FINGER_JOINTS[0]
    return named_for_robot(
        RobotModelConfig(
            model=METAL_MODEL.with_subtree_rooted_at("base_link")
            .with_renamed_joints({"gripper_base": "gripper_mount_joint"})
            .with_default_joint_acceleration_limit(1.0),
            joint_names=list(METAL_ARM_JOINTS),
            base_link="base_link",
            planning_groups=[
                PlanningGroupDefinition(
                    name="arm",
                    joint_names=METAL_ARM_JOINTS,
                    base_link="base_link",
                    tip_link="gripper_base",
                )
            ],
            # The closed fingers touch each other.
            collision_exclusion_pairs=[("link7", "link8")],
            auto_convert_meshes=True,
            gripper=GripperConfig(
                pad_links=("link7", "link8"), finger_joints={driven: METAL_GRIPPER_RANGE}
            ),
            home_joints=home_joints,
            tf_extra_links=list(tf_extra_links or []),
        ),
        robot,
        fingers=METAL_FINGER_JOINTS,
    )


def metal_sim_joints(robot: str = "arm") -> list[str]:
    """The joints the simulated Metal reports, in order: six arm joints, then joint7."""
    return [f"{robot}/{joint}" for joint in (*METAL_ARM_JOINTS, METAL_FINGER_JOINTS[0])]


def make_metal_sim_hardware() -> HardwareComponent:
    """The simulated Metal, attached to the blueprint's MujocoSimModule through shared memory."""
    return HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=metal_sim_joints(),
        adapter_type="sim_mujoco",
    )


def metal_motors(prefix: str = "") -> list[DamiaoMotor]:
    """The six arm motors with the profiles the firmware expects."""
    return [
        DamiaoMotor(f"{prefix}{joint}", "profile", index, index + 0x10, profile)
        for index, (joint, profile) in enumerate(
            zip(METAL_ARM_JOINTS, METAL_PROFILES, strict=True), start=1
        )
    ]


def metal_gripper(name: str, joint: str) -> DamiaoGripper:
    """A Metal jaw on the high-torque profile, opening toward more motor position.

    Args:
        name: The native gripper and motor name, e.g. "gripper".
        joint: Its URDF finger joint, e.g. "joint7"; open at its lower end.
    """
    return DamiaoGripper(
        name=name,
        joint=joint,
        open_at="lower",
        motor=DamiaoMotor(name, "profile", 0x07, 0x17, METAL_PROFILE_HIGH),
        opening="increasing_position",
        current=0.1,
    )


#: One Metal arm and its jaw on one Damiao bus.
METAL = DamiaoRobot(
    buses=[
        DamiaoBus(
            arm=metal_motors(),
            kp=METAL_KP,
            kd=METAL_KD,
            gripper=metal_gripper(METAL_GRIPPER_NAME, METAL_FINGER_JOINTS[0]),
            name="arm",
        )
    ],
    model=make_metal_model_config().model,
    gravity_model=METAL_MODEL,
    base_link="base_link",
    joint_names=[*METAL_ARM_JOINTS, METAL_FINGER_JOINTS[0]],
    gravity=False,
)

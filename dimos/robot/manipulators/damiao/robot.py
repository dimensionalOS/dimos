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

"""Plain Damiao robot data: motors, buses, grippers and the two models.

Nothing here imports the DimOS control stack. A robot is a description of
what is on the wire (motor type, address and firmware profile), how it should
be driven (gains, gripper settings) and which URDFs describe it, so a
deployment can be reviewed without reading any motor code.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from dimos.robot.assets.model import RobotModel

GripperOpening = Literal["increasing_position", "decreasing_position"]
OpenAt = Literal["lower", "upper"]


@dataclass(frozen=True, slots=True)
class DamiaoProfile:
    """A custom Damiao firmware profile, for motors whose stock scaling is wrong."""

    code: int
    p_max: float
    v_max: float
    t_max: float


@dataclass(frozen=True, slots=True)
class DamiaoMotor:
    """One motor on a bus.

    Attributes:
        name: The native motor name, e.g. "yam_joint1" or "gripper_motor".
        type: The native Damiao motor type, e.g. "DM4310"; "profile" when the
            motor runs a custom :class:`DamiaoProfile`.
        send_id: The CAN id the motor listens on.
        reply_id: The CAN id the motor answers on.
        profile: A custom firmware profile, or ``None`` for the stock one.
    """

    name: str
    type: str
    send_id: int
    reply_id: int
    profile: DamiaoProfile | None = None


@dataclass(frozen=True, slots=True)
class DamiaoGripper:
    """One jaw, reported and commanded as a finger joint of the robot's URDF.

    The jaw's travel is that joint's URDF position range.

    Attributes:
        name: The native gripper name, e.g. "gripper" or "left_gripper".
        joint: The URDF finger joint the jaw moves, e.g. "joint7".
        open_at: Which end of the joint's range is fully open, "lower" or
            "upper"; the other end is fully closed.
        motor: The motor that drives the jaw.
        opening: Which motor direction opens the jaw.
        current: The current limit used while gripping, in amperes.
        calibration_travel: How far to drive past the stroke when calibrating,
            in radians; ``None`` leaves calibration to the native default.
    """

    name: str
    joint: str
    open_at: OpenAt
    motor: DamiaoMotor
    opening: GripperOpening
    current: float
    calibration_travel: float | None = None


@dataclass(frozen=True, slots=True)
class DamiaoBus:
    """One CAN bus and the arm and gripper it carries.

    Attributes:
        arm: The arm motors, in joint order.
        kp: Position gains, one per arm motor.
        kd: Velocity gains, one per arm motor.
        gripper: The gripper on this bus, or ``None``.
        name: The native bus and arm-group name, e.g. "arm" or "left_arm".
    """

    arm: list[DamiaoMotor]
    kp: list[float]
    kd: list[float]
    gripper: DamiaoGripper | None = None
    name: str = "arm"


@dataclass(frozen=True, slots=True)
class DamiaoRobot:
    """A whole Damiao robot: its buses in order, models, gains and teach setup.

    Attributes:
        buses: The buses in deployment order; each has its own CAN selector.
        model: The URDF the connection drives and reports: the arm joints and
            each gripper's finger joint, with any other finger copying it.
        gravity_model: The mass-bearing URDF, whose joint order must match the
            arm motors exactly.
        base_link: The link a planning base pose is attached to.
        joint_names: Every commanded joint: all arm joints in bus order, then
            each gripper's finger joint.
        gravity: Whether to compensate gravity in the MIT torque slot.
        teach_kd: Damping used instead of ``kd`` with zero stiffness when an
            operator guides the arm by hand; ``None`` means no teaching.
        passive_grippers: Jaws that follow the hand, enabled with zero
            stiffness while teaching, by native gripper name.
        home_joints: The canonical arm posture a simulator starts from; the
            grippers start closed.
    """

    buses: list[DamiaoBus]
    model: RobotModel
    gravity_model: RobotModel
    base_link: str
    joint_names: list[str]
    gravity: bool = True
    teach_kd: list[float] | None = None
    passive_grippers: frozenset[str] = frozenset()
    home_joints: list[float] | None = None

    def __post_init__(self) -> None:
        if not self.buses:
            raise ValueError("a Damiao robot needs at least one bus")
        names = [bus.name for bus in self.buses]
        if any(not name for name in names) or len(set(names)) != len(names):
            raise ValueError("bus names must be non-empty and distinct")
        for bus in self.buses:
            if not bus.arm:
                raise ValueError(f"bus {bus.name!r} has no arm motors")
            if len(bus.kp) != len(bus.arm) or len(bus.kd) != len(bus.arm):
                raise ValueError(f"bus {bus.name!r} needs one gain per arm motor")
        grippers = [bus.gripper.name for bus in self.buses if bus.gripper is not None]
        if len(set(grippers)) != len(grippers):
            raise ValueError("gripper names must be distinct")
        arm_names = [motor.name for bus in self.buses for motor in bus.arm]
        motor_names = [
            *arm_names,
            *[bus.gripper.motor.name for bus in self.buses if bus.gripper is not None],
        ]
        if len(set(motor_names)) != len(motor_names):
            raise ValueError("motor names must be distinct")
        if self.joint_names != [*arm_names, *self.gripper_joint_names]:
            raise ValueError("joint_names must list the arm joints then the gripper joints")
        unknown = self.passive_grippers - set(grippers)
        if unknown:
            raise ValueError(f"unknown passive grippers: {sorted(unknown)}")
        if self.teach_kd is not None and len(self.teach_kd) != len(arm_names):
            raise ValueError("teach_kd needs one value per arm joint")
        if self.home_joints is not None and len(self.home_joints) != len(arm_names):
            raise ValueError("home_joints needs one value per arm joint")

    @property
    def arm_joint_names(self) -> list[str]:
        """Every arm joint, in bus then motor order."""
        return [motor.name for bus in self.buses for motor in bus.arm]

    @property
    def grippers(self) -> list[DamiaoGripper]:
        """Every gripper, in bus order."""
        return [bus.gripper for bus in self.buses if bus.gripper is not None]

    @property
    def gripper_joint_names(self) -> list[str]:
        """Every gripper's finger joint, in bus order."""
        return [gripper.joint for gripper in self.grippers]

    @property
    def arm_joints(self) -> dict[str, list[str]]:
        """Arm joint names grouped by native bus name."""
        return {bus.name: [motor.name for motor in bus.arm] for bus in self.buses}

    @property
    def gripper_joints(self) -> dict[str, str]:
        """Gripper joint names keyed by their native group name."""
        return {gripper.name: gripper.joint for gripper in self.grippers}

    def gripper_travel(self) -> dict[str, tuple[float, float]]:
        """Each gripper joint's (closed, open) positions, from its URDF range in ``model``.

        Loads the model. Positions are in the joint's unit: m for a sliding
        finger, rad for a turning one.
        """
        joints = {joint.name: joint for joint in self.model.load().joints}
        travel: dict[str, tuple[float, float]] = {}
        for gripper in self.grippers:
            joint = joints.get(gripper.joint)
            if joint is None or joint.type == "fixed" or joint.mimic is not None:
                raise ValueError(f"{gripper.joint!r} is not a driven joint of the robot model")
            if joint.lower is None or joint.upper is None or not joint.lower < joint.upper:
                raise ValueError(f"{gripper.joint!r} has no position range in the robot model")
            if gripper.open_at == "lower":
                travel[gripper.joint] = (joint.upper, joint.lower)
            else:
                travel[gripper.joint] = (joint.lower, joint.upper)
        return travel

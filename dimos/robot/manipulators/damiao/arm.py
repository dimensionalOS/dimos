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

"""The Damiao motor layer: native buses, MIT commands, gravity and feedback.

``DamiaoArm`` owns everything that touches the vendor SDK. It takes plain
:class:`~dimos.robot.manipulators.damiao.robot.DamiaoRobot` data and CAN
selectors, and exposes four operations: :meth:`DamiaoArm.enable`,
:meth:`DamiaoArm.command`, :meth:`DamiaoArm.read` and :meth:`DamiaoArm.disable`.
The DimOS connection module only translates ports and time into those calls.

Calibration changes how each joint is held, never where it is told to go:
:meth:`DamiaoArm.apply_calibration` puts a calibration file's gains and link
masses in place, and :meth:`DamiaoArm.set_gains` and :meth:`DamiaoArm.set_float`
change one joint live until the next :meth:`DamiaoArm.hold`. The torque of every
MIT row is clipped to the motor's MIT range in ``DamiaoArm._send``.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
import sys
from typing import Any

import can_motor_control
from can_motor_control import damiao
import numpy as np
from numpy.typing import NDArray
import pinocchio

from dimos.control.calibration._inertial import (
    LinkInertial,
    apply_inertial_overrides,
)
from dimos.control.calibration.spec import (
    DAMIAO_STOCK_TORQUE_LIMIT_NM,
    motor_rated_torque_nm,
    motor_torque_limit_nm,
)
from dimos.control.task import MotorCommand
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.damiao.robot import (
    DamiaoBus,
    DamiaoMotor,
    DamiaoProfile,
    DamiaoRobot,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

#: Native Damiao motor types a robot may name in its data.
MOTOR_TYPES: dict[str, Any] = {
    "DM4310": damiao.MotorType.DM4310,
    "DM4340": damiao.MotorType.DM4340,
    "DM8009": damiao.MotorType.DM8009,
}

#: The kp and kd ranges the Damiao MIT frame encodes (firmware fact, every motor type).
MIT_KP_MAX = 500.0
MIT_KD_MAX = 5.0


class PhysicalModel:
    """Numerical arm model: exact joint coordinates and gravity.

    Args:
        robot_model: The mass-bearing URDF.
        joint_names: The arm joints in motor order; every other joint is
            locked at its neutral position.
        gravity_comp: Whether :meth:`gravity` computes torques or returns zeros.
        inertial_overrides: Fitted mass and centre of mass by link body, e.g.
            from ``calibration.schema.to_inertial_overrides``. They replace the
            URDF values after the other joints are locked, so a body is the
            link plus the links merged into it.
    """

    def __init__(
        self,
        robot_model: RobotModel,
        joint_names: list[str],
        *,
        gravity_comp: bool,
        inertial_overrides: Mapping[str, LinkInertial] | None = None,
    ) -> None:
        self._gravity_comp = gravity_comp
        self._names = tuple(joint_names)
        model = pinocchio.buildModelFromXML(robot_model.load().xml)
        locked = [
            index
            for index, name in enumerate(model.names)
            if index and str(name) not in self._names
        ]
        if locked:
            model = pinocchio.buildReducedModel(model, locked, pinocchio.neutral(model))
        count = len(self._names)
        if model.nq != count or model.nv != count:
            raise ValueError("physical model dimensions do not match angular joints")
        if tuple(str(name) for name in model.names[1:]) != self._names:
            raise ValueError("physical model joint order does not match angular joints")
        overrides = dict(inertial_overrides or {})
        if overrides:
            unknown = sorted(
                name for name in overrides if not model.existFrame(name, pinocchio.FrameType.BODY)
            )
            if unknown:
                raise ValueError(f"inertial overrides name links the arm model lacks: {unknown}")
            for name, inertial in overrides.items():
                if not math.isfinite(inertial.mass_kg) or not np.isfinite(inertial.com_m).all():
                    raise ValueError(f"inertial override for {name!r} is not finite")
            apply_inertial_overrides(model, overrides)
        self._overrides = overrides
        self._model = model
        self._data = model.createData()

    @property
    def inertial_overrides(self) -> dict[str, LinkInertial]:
        """The link bodies whose URDF mass and centre of mass were replaced."""
        return dict(self._overrides)

    def gravity(self, positions: NDArray[np.float64]) -> NDArray[np.float64]:
        """Gravity torques for a measured arm pose, or zeros without compensation."""
        if positions.shape != (len(self._names),) or not np.isfinite(positions).all():
            raise ValueError("arm feedback contains non-finite or invalid positions")
        gravity = (
            np.asarray(
                pinocchio.computeGeneralizedGravity(self._model, self._data, positions),
                dtype=np.float64,
            )
            if self._gravity_comp
            else np.zeros(len(self._names))
        )
        if gravity.shape != positions.shape or not np.isfinite(gravity).all():
            raise ValueError("gravity compensation produced invalid torques")
        return gravity


@dataclass(frozen=True, slots=True)
class ArmReading:
    """One synchronous sample of every commanded joint, in ``joint_names`` order.

    Attributes:
        position: Measured positions; a jaw's comes from its opening.
        velocity: Measured velocities; 0 for a jaw.
        effort: The torque each arm motor reports it is producing, N m,
            never the commanded one; 0 for a jaw.
    """

    position: list[float]
    velocity: list[float]
    effort: list[float]


class DamiaoArm:
    """One Damiao robot on its buses: enable, command, read and disable it.

    Each jaw is read and commanded as its URDF finger joint, in that joint's
    unit (m or rad). The SDK's opening, 0 closed to 1 open, maps linearly
    onto the joint's URDF range, with the gripper's ``open_at`` end as open.

    Args:
        robot: The motors, gains, gripper settings and models to use.
        can_ports: One explicitly selected CAN interface or bridge serial
            number per bus, in the robot's bus order.
        teach: Command zero stiffness with the robot's ``teach_kd`` damping,
            expose its passive grippers, and keep re-sending gravity torques
            from the reader so they never go stale.
    """

    def __init__(
        self,
        robot: DamiaoRobot,
        can_ports: Sequence[str],
        *,
        teach: bool = False,
    ) -> None:
        self._robot = robot
        self._ports = list(can_ports)
        if len(self._ports) != len(robot.buses) or any(not port.strip() for port in self._ports):
            raise ValueError("CAN selectors must cover exactly the robot buses")
        if len(set(self._ports)) != len(self._ports):
            raise ValueError("CAN selectors must be distinct")
        self._teach = teach
        if teach and robot.teach_kd is None:
            raise ValueError("teaching needs teach_kd in the robot data")
        # Jaws follow the hand only while teaching; otherwise they hold their
        # commanded opening.
        self._passive = robot.passive_grippers if teach else frozenset()
        self._native: can_motor_control.Robot | None = None
        self._arms: dict[str, can_motor_control.Arm] = {}
        self._grippers: dict[str, can_motor_control.Gripper] = {}
        self._physical: PhysicalModel | None = None
        # Each gripper joint's (closed, open) positions.
        self._travel: dict[str, tuple[float, float]] = {}
        self._targets: dict[str, float] = {}
        # Arm joint -> its motor command from the last command; absent means the defaults.
        self._motor_commands: dict[str, MotorCommand] = {}
        self._active = False
        # How each arm joint is held, from the bottom layer up: the bus
        # defaults, a calibration's gains, then live overrides while tuning.
        self._calibration_gains: dict[str, tuple[float, float]] = {}
        self._gain_overrides: dict[str, tuple[float, float]] = {}
        # Joints floating with zero stiffness, by joint, with their damping.
        self._floating: dict[str, float] = {}
        self._inertial_overrides: dict[str, LinkInertial] = {}
        # Each arm motor's MIT torque range and rated torque, N m, joint order.
        self._torque_limits: NDArray[np.float64] | None = None
        self._rated_torques: NDArray[np.float64] | None = None
        # Joints whose torque is clipped now; logged as each one enters.
        self._saturated: set[str] = set()

    @property
    def robot(self) -> DamiaoRobot:
        return self._robot

    @property
    def joint_names(self) -> list[str]:
        return list(self._robot.joint_names)

    @property
    def active(self) -> bool:
        """Whether the motors are enabled and accepting commands."""
        return self._active

    @property
    def floating(self) -> dict[str, float]:
        """The joints floating with zero stiffness, with the damping each floats with."""
        return dict(self._floating)

    @property
    def inertial_overrides(self) -> dict[str, LinkInertial]:
        """The link masses the gravity model uses instead of the URDF's."""
        return dict(self._inertial_overrides)

    @property
    def rated_torques(self) -> dict[str, float]:
        """Each arm motor's rated peak torque by joint, N m, never above its MIT range.

        From the motor type (``motor_rated_torque_nm``); a motor of unknown
        type, such as one on a custom firmware profile, gets the smallest
        rating known.
        """
        _, rated = self._torque_tables()
        return dict(zip(self._robot.arm_joint_names, rated.tolist(), strict=True))

    def connect(self) -> None:
        """Build the native robot, open its buses and take a first reading."""
        self._physical = PhysicalModel(
            self._robot.gravity_model,
            self._robot.arm_joint_names,
            gravity_comp=self._robot.gravity,
            inertial_overrides=self._inertial_overrides,
        )
        # A motor without a known torque range fails here, before any bus opens.
        self._torque_tables()
        self._travel = self._robot.gripper_travel()
        native = self._build()
        self._native = native
        native.connect()
        for bus in self._robot.buses:
            group = native[bus.name]
            if not isinstance(group, can_motor_control.Arm):
                raise TypeError(f"native group {bus.name!r} is not an Arm")
            self._arms[bus.name] = group
        for bus in self._robot.buses:
            if bus.gripper is None:
                continue
            group = native[bus.gripper.name]
            if not isinstance(group, can_motor_control.Gripper):
                raise TypeError(f"native group {bus.gripper.name!r} is not a Gripper")
            self._grippers[bus.gripper.name] = group

    def enable(self) -> None:
        """Check limits and gravity, then enable the arm motors and jaws."""
        native = self._native
        if native is None:
            raise RuntimeError("Damiao robot is disconnected")
        # Fail before the motors move: a bad pose or model must not energize.
        assert self._physical is not None
        self._physical.gravity(self._arm_positions())
        for arm in self._arms.values():
            arm.set_mode("mit")
        native.enable()
        self._enable_passive_grippers()
        self._active = True
        self._targets = dict(zip(self.joint_names, self._sample().position, strict=True))

    def command(
        self,
        targets: Mapping[str, float],
        motor_commands: Mapping[str, MotorCommand] | None = None,
    ) -> None:
        """Merge position targets by joint name and send one MIT tick.

        Args:
            targets: Joint name to target position, in rad, or m for a sliding
                jaw. A joint left out keeps its last target.
            motor_commands: Arm joint name to all five numbers it is told, for
                joints not in ``targets``. Replaces what the previous command
                set: a joint left out, or every joint when None, gets its usual
                kp and kd (see :meth:`get_gains`), no target speed and no extra
                torque. An unset position aims the joint where it is measured
                now, so its spring adds no force.
        """
        motor_commands = dict(motor_commands or {})
        unknown = set(targets) - set(self.joint_names)
        if unknown:
            raise ValueError(f"unknown Damiao joints: {sorted(unknown)}")
        not_arm = set(motor_commands) - set(self._robot.arm_joint_names)
        if not_arm:
            raise ValueError(f"only arm joints take a motor command: {sorted(not_arm)}")
        both = set(motor_commands) & set(targets)
        if both:
            raise ValueError(f"joints given both a target and a motor command: {sorted(both)}")
        if self._native is None or not self._active:
            raise RuntimeError("Damiao robot is inactive")
        measured = dict(zip(self._robot.arm_joint_names, self._arm_positions(), strict=True))
        self._targets.update(targets)
        self._targets.update(
            {
                name: measured[name] if each.position is None else each.position
                for name, each in motor_commands.items()
            }
        )
        self._motor_commands = motor_commands
        self._send()

    def hold(self) -> None:
        """Undo every live change and hold every joint where it is now; the calibration stays.

        Floating joints, gain overrides and the last motor commands are
        dropped, so each joint is held with the calibration's gains, else the
        bus defaults, and gravity torque alone. The next :meth:`command` moves
        the arm again.
        """
        undone = sorted({*self._floating, *self._gain_overrides})
        if undone:
            logger.info(f"Damiao live gains and floating of {undone} undone")
        self._floating.clear()
        self._gain_overrides.clear()
        self._motor_commands = {}
        if self._native is None or not self._active:
            return
        self._targets = dict(zip(self.joint_names, self._sample().position, strict=True))
        self._send()

    def read(self) -> ArmReading:
        """Sample every joint; while teaching or floating, also refresh the gravity torques."""
        reading = self._sample()
        if (self._teach or self._floating) and self._active:
            # The coordinator may go quiet while an operator moves the arm. The
            # MIT torque slot must still follow the new pose, so re-send it.
            self._send()
        return reading

    def get_gains(self) -> dict[str, tuple[float, float]]:
        """Each arm joint's holding gains, (kp, kd), in joint order.

        A live override from :meth:`set_gains`, else the calibration's gains,
        else the bus defaults. Teaching and floating hold a joint with zero
        stiffness instead without changing these, so restoring what this
        returns never leaves a joint limp.
        """
        return {
            motor.name: self._hold_gains(motor.name, (float(kp), float(kd)))
            for bus in self._robot.buses
            for motor, kp, kd in zip(bus.arm, bus.kp, bus.kd, strict=True)
        }

    def set_gains(self, joint: str, kp: float, kd: float) -> None:
        """Hold ``joint`` with ``kp`` and ``kd`` until the next :meth:`hold`, sent at once.

        A floating joint is held again where it is now. Gains outside the MIT
        frame's range raise ``ValueError`` (:func:`check_mit_gains`).
        """
        index = self._arm_index(joint)
        check_mit_gains(joint, kp, kd)
        if self._floating.pop(joint, None) is not None and self._arms:
            self._targets[joint] = float(self._arm_positions()[index])
        self._gain_overrides[joint] = (float(kp), float(kd))
        logger.info(f"Damiao {joint} gains kp {kp:g}, kd {kd:g}")
        self._resend()

    def set_float(self, joint: str, kd: float) -> None:
        """Let ``joint`` float with zero stiffness and ``kd`` damping until the next :meth:`hold`.

        Its gravity torque is still applied, so it stays where a hand leaves
        it. A robot that does not compensate gravity raises ``ValueError``:
        nothing would hold the joint up.
        """
        self._arm_index(joint)
        check_mit_gains(joint, 0.0, kd)
        if not self._robot.gravity:
            raise ValueError(f"Damiao {joint} cannot float: this robot does not compensate gravity")
        self._floating[joint] = float(kd)
        logger.info(f"Damiao {joint} floating with kd {kd:g}")
        self._resend()

    def gravity(
        self,
        inertial_overrides: Mapping[str, LinkInertial] | None = None,
        positions: Sequence[float] | None = None,
    ) -> dict[str, float]:
        """Each arm joint's gravity torque, N m, as :meth:`command` would send it.

        Args:
            inertial_overrides: Link masses to use instead of the current
                ones, e.g. a calibration's; checked as
                :meth:`apply_calibration` checks them.
            positions: An arm pose in joint order, rad; defaults to the
                measured one, which needs a connected robot.

        Zeros on a robot that does not compensate gravity.
        """
        physical = self._physical
        if inertial_overrides is not None or physical is None:
            physical = PhysicalModel(
                self._robot.gravity_model,
                self._robot.arm_joint_names,
                gravity_comp=self._robot.gravity,
                inertial_overrides=(
                    self._inertial_overrides if inertial_overrides is None else inertial_overrides
                ),
            )
        if positions is None:
            if not self._arms:
                raise RuntimeError("Damiao robot is disconnected")
            pose = self._arm_positions()
        else:
            pose = np.asarray(positions, dtype=np.float64)
        torques = physical.gravity(pose)
        return dict(zip(self._robot.arm_joint_names, torques.tolist(), strict=True))

    def check_calibration(
        self,
        gains: Mapping[str, tuple[float, float]],
        inertial_overrides: Mapping[str, LinkInertial],
    ) -> None:
        """Raise ``ValueError`` if :meth:`apply_calibration` would refuse these."""
        self._prepare_calibration(gains, inertial_overrides)

    def apply_calibration(
        self,
        gains: Mapping[str, tuple[float, float]],
        inertial_overrides: Mapping[str, LinkInertial],
    ) -> None:
        """Hold the named joints with these gains and use these link masses for gravity.

        Replaces the previous calibration as a whole and drops every live
        change, so the arm runs exactly this one; a floating joint is held
        where it is. Everything is checked first: an unknown joint or link, a
        non-finite value or a gain outside the MIT range raises ``ValueError``
        and leaves the arm as it was. Sent at once when enabled.
        """
        physical = self._prepare_calibration(gains, inertial_overrides)
        if self._arms:
            measured = self._arm_positions()
            for joint in self._floating:
                self._targets[joint] = float(measured[self._arm_index(joint)])
        self._calibration_gains = dict(gains)
        self._gain_overrides.clear()
        self._floating.clear()
        self._inertial_overrides = dict(inertial_overrides)
        if self._physical is not None:
            # Before connect, connect builds the model with these overrides.
            self._physical = physical
        self._resend()

    def disable(self) -> None:
        """De-energize the motors. Safe before enable and after a fault."""
        self._active = False
        if self._native is not None:
            self._native.disable()

    def _make_can_bus(self, port: str) -> Any:
        if port.startswith("/dev/"):
            return can_motor_control.DmSerialBus(port)
        if sys.platform == "linux":
            return can_motor_control.SocketCanBus(port)
        if sys.platform == "darwin":
            return can_motor_control.GsUsbBus(
                vendor_id=0x1D50, product_id=0x606F, serial_number=port
            )
        raise RuntimeError(f"Damiao CAN transport is unsupported on {sys.platform}")

    def _make_profiles(self) -> dict[DamiaoProfile, Any]:
        """Build each custom firmware profile once, in motor declaration order."""
        profiles: dict[DamiaoProfile, Any] = {}
        motors = [
            motor
            for bus in self._robot.buses
            for motor in [
                *bus.arm,
                *([bus.gripper.motor] if bus.gripper is not None else []),
            ]
        ]
        for motor in motors:
            profile = motor.profile
            if profile is None or profile in profiles:
                continue
            profile_type = getattr(damiao, "MotorProfile", None)
            if profile_type is None:
                raise RuntimeError(
                    "MetalConnection requires can-motor-control>=0.0.11 with custom "
                    "MotorProfile support; stock DM profiles use incompatible firmware scaling"
                )
            profiles[profile] = profile_type(
                profile.code, p_max=profile.p_max, v_max=profile.v_max, t_max=profile.t_max
            )
        return profiles

    def _motor_spec(self, motor: DamiaoMotor, profiles: dict[DamiaoProfile, Any]) -> Any:
        if motor.profile is not None:
            return can_motor_control.MotorSpec(
                motor.name, profiles[motor.profile], motor.send_id, motor.reply_id
            )
        motor_type = MOTOR_TYPES.get(motor.type)
        if motor_type is None:
            raise ValueError(f"unknown Damiao motor type {motor.type!r} for {motor.name!r}")
        return can_motor_control.MotorSpec(motor.name, motor_type, motor.send_id, motor.reply_id)

    def _build(self) -> can_motor_control.Robot:
        profiles = self._make_profiles()
        builder = can_motor_control.Robot.builder()
        for bus, port in zip(self._robot.buses, self._ports, strict=True):
            builder.add_bus(bus.name, self._make_can_bus(port), self._codec(bus, profiles))
            builder.add_arm(
                bus.name,
                bus=bus.name,
                motors=[self._motor_spec(motor, profiles) for motor in bus.arm],
            )
            if bus.gripper is not None:
                builder.add_gripper(
                    bus.gripper.name,
                    bus=bus.name,
                    motor=self._motor_spec(bus.gripper.motor, profiles),
                    opening_direction=bus.gripper.opening,
                    default_current=bus.gripper.current,
                    **(
                        {"calibration_travel": bus.gripper.calibration_travel}
                        if bus.gripper.calibration_travel is not None
                        else {}
                    ),
                )
        return builder.build()

    def _codec(self, bus: DamiaoBus, profiles: dict[DamiaoProfile, Any]) -> Any:
        used = list(
            dict.fromkeys(profiles[motor.profile] for motor in bus.arm if motor.profile is not None)
        )
        if bus.gripper is not None and bus.gripper.motor.profile is not None:
            profile = profiles[bus.gripper.motor.profile]
            if profile not in used:
                used.append(profile)
        return damiao.DamiaoCodec(profiles=used) if used else damiao.DamiaoCodec()

    def _arm_index(self, joint: str) -> int:
        names = self._robot.arm_joint_names
        if joint not in names:
            raise ValueError(f"{joint!r} is not an arm joint of this Damiao robot; it has {names}")
        return names.index(joint)

    def _hold_gains(self, joint: str, default: tuple[float, float]) -> tuple[float, float]:
        """A live override, else the calibration's gains, else ``default`` (the bus's)."""
        for layer in (self._gain_overrides, self._calibration_gains):
            if joint in layer:
                return layer[joint]
        return default

    def _wire_gains(
        self, index: int, joint: str, default: tuple[float, float]
    ) -> tuple[float, float]:
        """The gains a joint is sent with: teaching, then floating, then holding."""
        if self._teach:
            assert self._robot.teach_kd is not None
            return 0.0, float(self._robot.teach_kd[index])
        if joint in self._floating:
            return 0.0, self._floating[joint]
        return self._hold_gains(joint, default)

    def _torque_tables(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Each arm motor's MIT torque range and rated torque, N m, in joint order.

        Built once, at connect at the latest.
        """
        if self._torque_limits is None or self._rated_torques is None:
            motors = [motor for bus in self._robot.buses for motor in bus.arm]
            limits = np.asarray([motor_torque_limit_nm(motor) for motor in motors])
            fallback = min(DAMIAO_STOCK_TORQUE_LIMIT_NM.values())
            ratings = [motor_rated_torque_nm(motor) for motor in motors]
            rated = np.asarray([fallback if rating is None else rating for rating in ratings])
            self._torque_limits = limits.astype(np.float64)
            self._rated_torques = np.minimum(rated, limits).astype(np.float64)
        return self._torque_limits, self._rated_torques

    def _prepare_calibration(
        self,
        gains: Mapping[str, tuple[float, float]],
        inertial_overrides: Mapping[str, LinkInertial],
    ) -> PhysicalModel:
        for joint, (kp, kd) in gains.items():
            self._arm_index(joint)
            check_mit_gains(joint, kp, kd)
        physical = PhysicalModel(
            self._robot.gravity_model,
            self._robot.arm_joint_names,
            gravity_comp=self._robot.gravity,
            inertial_overrides=inertial_overrides,
        )
        if self._arms:
            # The new masses must give finite torques at the measured pose.
            physical.gravity(self._arm_positions())
        return physical

    def _resend(self) -> None:
        """Send changed gains or torques at once; before enable they apply from enable."""
        if self._native is not None and self._active:
            self._send()

    def _passive_gripper_names(self) -> list[str]:
        return [gripper.name for gripper in self._robot.grippers if gripper.name in self._passive]

    def _enable_passive_grippers(self) -> None:
        """Enable passive jaws in MIT mode with zero stiffness and torque."""
        for name in self._passive_gripper_names():
            gripper = self._grippers[name]
            gripper.set_mode("mit")
            gripper.enable()
            gripper.mit_control(0.0, 0.0, float(gripper.motor.position), 0.0, 0.0)
        if self._passive:
            assert self._native is not None
            self._native.tick(1_000)

    def _arm_positions(self) -> NDArray[np.float64]:
        return np.concatenate([arm.positions().astype(np.float64) for arm in self._arms.values()])

    def _send(self) -> None:
        native = self._native
        if native is None or not self._active:
            raise RuntimeError("Damiao robot is inactive")
        missing = [name for name in self.joint_names if name not in self._targets]
        if missing:
            raise RuntimeError(f"no target for Damiao joints: {missing}")
        measured = self._arm_positions()
        arm_names = self._robot.arm_joint_names
        # A floating joint is wherever the hand left it; its target follows,
        # so holding it again never pulls it back.
        for joint in self._floating:
            self._targets[joint] = float(measured[arm_names.index(joint)])
        positions = [self._targets[name] for name in self.joint_names]
        if not np.isfinite(positions).all():
            raise ValueError("position command contains non-finite values")
        openings: dict[str, float] = {}
        for gripper in self._robot.grippers:
            if gripper.name in self._passive:
                continue
            closed, opened = self._travel[gripper.joint]
            target = self._targets[gripper.joint]
            opening = (target - closed) / (opened - closed)
            if not 0 <= opening <= 1:
                raise ValueError(f"{gripper.joint} target {target} is outside its URDF range")
            openings[gripper.name] = opening
        assert self._physical is not None
        gravity = self._physical.gravity(measured)
        torque_limits, _ = self._torque_tables()
        offset = 0
        arm_rows: list[tuple[str, NDArray[np.float64]]] = []
        for bus in self._robot.buses:
            values: list[list[float]] = []
            for index, motor in enumerate(bus.arm, start=offset):
                default = (bus.kp[index - offset], bus.kd[index - offset])
                kp, kd = self._wire_gains(index, motor.name, default)
                each = self._motor_commands.get(motor.name, MotorCommand())
                # The motor applies kp * (q - measured q) + kd * (dq - measured dq) + tau.
                values.append(
                    [
                        kp if each.stiffness is None else each.stiffness,
                        kd if each.damping is None else each.damping,
                        positions[index],
                        each.velocity or 0.0,
                        gravity[index] + (each.effort or 0.0),
                    ]
                )
            rows = np.asarray(values, dtype=np.float64)
            if not np.isfinite(rows).all():
                raise ValueError(f"arm {bus.name!r} command contains non-finite values")
            limits = torque_limits[offset : offset + len(bus.arm)]
            for motor, tau, limit in zip(bus.arm, rows[:, 4], limits, strict=True):
                if abs(tau) <= limit:
                    self._saturated.discard(motor.name)
                elif motor.name not in self._saturated:
                    self._saturated.add(motor.name)
                    logger.warning(
                        f"Damiao {motor.name} torque {tau:+.2f} N m saturated at the motor's "
                        f"{limit:g} N m MIT range"
                    )
            rows[:, 4] = np.clip(rows[:, 4], -limits, limits)
            arm_rows.append((bus.name, rows))
            offset += len(bus.arm)
        for name, rows in arm_rows:
            self._arms[name].mit_control(rows)
        for name, group in self._grippers.items():
            if name in self._passive:
                group.mit_control(0.0, 0.0, float(group.motor.position), 0.0, 0.0)
            else:
                group.set_opening(openings[name])
        native.tick(1_000)

    def _sample(self) -> ArmReading:
        native = self._native
        if native is None:
            raise RuntimeError("Damiao robot is disconnected")
        native.refresh()
        native.tick(1_000)
        q: list[float] = []
        dq: list[float] = []
        tau: list[float] = []
        for bus in self._robot.buses:
            group = self._arms[bus.name]
            arrays = [group.positions(), group.velocities(), group.torques()]
            if any(len(array) != len(bus.arm) for array in arrays):
                raise RuntimeError(f"native arm {bus.name!r} returned the wrong state length")
            for output, array in zip((q, dq, tau), arrays, strict=True):
                output.extend(array.astype(np.float64).tolist())
        for gripper in self._robot.grippers:
            opening = float(self._grippers[gripper.name].opening) if self._active else 0.0
            if not np.isfinite(opening) or not 0 <= opening <= 1:
                raise RuntimeError(f"gripper {gripper.name!r} returned invalid opening {opening}")
            closed, opened = self._travel[gripper.joint]
            q.append(closed * (1.0 - opening) + opened * opening)
            dq.append(0.0)
            tau.append(0.0)
        if not np.isfinite([q, dq, tau]).all():
            raise RuntimeError("Damiao feedback contains non-finite values")
        return ArmReading(position=q, velocity=dq, effort=tau)


def check_mit_gains(joint: str, kp: float, kd: float) -> None:
    """Raise ``ValueError`` unless 0 <= kp <= ``MIT_KP_MAX`` and 0 <= kd <= ``MIT_KD_MAX``."""
    if not (0.0 <= kp <= MIT_KP_MAX and 0.0 <= kd <= MIT_KD_MAX):
        raise ValueError(
            f"{joint} gains kp={kp}, kd={kd} are outside the Damiao MIT range "
            f"0 <= kp <= {MIT_KP_MAX:g}, 0 <= kd <= {MIT_KD_MAX:g}"
        )

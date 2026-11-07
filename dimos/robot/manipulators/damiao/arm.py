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
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import sys
from typing import Any

import can_motor_control
from can_motor_control import damiao
import numpy as np
from numpy.typing import NDArray
import pinocchio

from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.damiao.robot import (
    DamiaoBus,
    DamiaoMotor,
    DamiaoProfile,
    DamiaoRobot,
)

#: Native Damiao motor types a robot may name in its data.
MOTOR_TYPES: dict[str, Any] = {
    "DM4310": damiao.MotorType.DM4310,
    "DM4340": damiao.MotorType.DM4340,
    "DM8009": damiao.MotorType.DM8009,
}


class PhysicalModel:
    """Numerical arm model: exact joint coordinates and gravity."""

    def __init__(
        self,
        robot_model: RobotModel,
        joint_names: list[str],
        *,
        gravity_comp: bool,
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
        self._model = model
        self._data = model.createData()

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
    """One synchronous sample of every commanded joint."""

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
        self._active = False

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

    def connect(self) -> None:
        """Build the native robot, open its buses and take a first reading."""
        self._physical = PhysicalModel(
            self._robot.gravity_model,
            self._robot.arm_joint_names,
            gravity_comp=self._robot.gravity,
        )
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

    def command(self, targets: Mapping[str, float]) -> None:
        """Merge position targets by joint name and send one MIT tick."""
        unknown = set(targets) - set(self.joint_names)
        if unknown:
            raise ValueError(f"unknown Damiao joints: {sorted(unknown)}")
        if self._native is None or not self._active:
            raise RuntimeError("Damiao robot is inactive")
        self._targets.update(targets)
        self._send()

    def hold(self) -> None:
        """Hold every joint where it is now, e.g. when commands stop arriving."""
        if self._native is None or not self._active:
            return
        self._targets = dict(zip(self.joint_names, self._sample().position, strict=True))
        self._send()

    def read(self) -> ArmReading:
        """Sample every joint; while teaching, also refresh the gravity torques."""
        reading = self._sample()
        if self._teach and self._active:
            # The coordinator may go quiet while an operator moves the arm. The
            # MIT torque slot must still follow the new pose, so re-send it.
            self._send()
        return reading

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

    def _gains(self, bus: DamiaoBus, offset: int) -> tuple[list[float], list[float]]:
        if self._teach:
            assert self._robot.teach_kd is not None
            return [0.0] * len(bus.arm), list(self._robot.teach_kd[offset : offset + len(bus.arm)])
        return list(bus.kp), list(bus.kd)

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
        gravity = self._physical.gravity(self._arm_positions())
        offset = 0
        arm_rows: list[tuple[str, NDArray[np.float64]]] = []
        for bus in self._robot.buses:
            kp, kd = self._gains(bus, offset)
            rows = np.asarray(
                [
                    [
                        kp[index],
                        kd[index],
                        positions[offset + index],
                        0.0,
                        gravity[offset + index],
                    ]
                    for index in range(len(bus.arm))
                ],
                dtype=np.float64,
            )
            if not np.isfinite(rows).all():
                raise ValueError(f"arm {bus.name!r} command contains non-finite values")
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

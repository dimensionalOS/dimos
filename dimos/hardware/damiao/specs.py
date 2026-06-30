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

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class DamiaoMotorSpec:
    """Typed metadata for one Damiao motor in adapter joint order."""

    name: str
    type: object
    send_id: int
    recv_id: int | None = None

    @property
    def effective_recv_id(self) -> int:
        """Return the explicit receive CAN ID, or Damiao's default response ID."""

        return self.recv_id if self.recv_id is not None else (self.send_id | 0x10)


@dataclass(frozen=True)
class DamiaoBusSpec:
    """Named communication channel for Damiao motors."""

    address: str | Path = "can0"
    fd: bool = False


@dataclass(frozen=True)
class DamiaoJointGroupSpec:
    """Ordered Damiao joints forming a controllable physical group."""

    bus_name: str
    motors: tuple[DamiaoMotorSpec, ...]
    position_lower: tuple[float, ...]
    position_upper: tuple[float, ...]
    velocity_max: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    gravity_model_path: str | Path | None = None
    gravity_torque_limits: tuple[float, ...] | None = None
    supports_velocity: bool = False

    @property
    def dof(self) -> int:
        """Return the number of joints described by this group spec."""

        return len(self.motors)

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Return joint names in command-vector order."""

        return tuple(motor.name for motor in self.motors)

    def validate(self, *, group_name: str, bus_names: set[str] | None = None) -> None:
        """Validate per-group metadata and optional bus reference."""

        if not self.motors:
            raise ValueError(f"DamiaoJointGroupSpec {group_name!r} requires at least one motor")
        if bus_names is not None and self.bus_name not in bus_names:
            raise ValueError(f"group {group_name!r} references unknown bus {self.bus_name!r}")
        send_ids = [motor.send_id for motor in self.motors]
        if len(set(send_ids)) != len(send_ids):
            raise ValueError(f"duplicate send_id in group {group_name!r}: {send_ids}")
        recv_ids = [motor.effective_recv_id for motor in self.motors]
        if len(set(recv_ids)) != len(recv_ids):
            raise ValueError(f"duplicate recv_id in group {group_name!r}: {recv_ids}")
        joint_names = [motor.name for motor in self.motors]
        if len(set(joint_names)) != len(joint_names):
            raise ValueError(f"duplicate joint name in group {group_name!r}: {joint_names}")
        for name, values in {
            "position_lower": self.position_lower,
            "position_upper": self.position_upper,
            "velocity_max": self.velocity_max,
            "kp": self.kp,
            "kd": self.kd,
        }.items():
            if len(values) != self.dof:
                raise ValueError(
                    f"{name} length {len(values)} does not match dof {self.dof} "
                    f"for group {group_name!r}"
                )
        for index, (lower, upper) in enumerate(
            zip(self.position_lower, self.position_upper, strict=True),
        ):
            if lower > upper:
                raise ValueError(
                    f"position_lower[{index}] > position_upper[{index}] for group {group_name!r}"
                )
        if self.gravity_torque_limits is not None and len(self.gravity_torque_limits) != self.dof:
            raise ValueError(
                f"gravity_torque_limits length does not match dof for group {group_name!r}"
            )


@dataclass(frozen=True)
class DamiaoRobotSpec:
    """Python-native Damiao robot config with named buses and joint groups."""

    name: str
    vendor: str
    model: str
    buses: Mapping[str, DamiaoBusSpec]
    groups: Mapping[str, DamiaoJointGroupSpec]
    requires_binding: bool = False

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Return all group joint names in mapping iteration order."""

        return tuple(joint for group in self.groups.values() for joint in group.joint_names)

    def group_joint_names(self, group_names: Sequence[str]) -> tuple[str, ...]:
        """Return concatenated joint names for the requested groups."""

        return tuple(
            joint for group_name in group_names for joint in self.groups[group_name].joint_names
        )

    def validate(self) -> None:
        """Validate bus/group references and global joint-name uniqueness."""

        if not self.buses:
            raise ValueError("DamiaoRobotSpec requires at least one bus")
        if not self.groups:
            raise ValueError("DamiaoRobotSpec requires at least one joint group")
        bus_names = set(self.buses)
        all_joint_names: list[str] = []
        ids_by_bus: dict[str, set[int]] = {bus_name: set() for bus_name in bus_names}
        for group_name, group in self.groups.items():
            group.validate(group_name=group_name, bus_names=bus_names)
            all_joint_names.extend(group.joint_names)
            bus_ids = ids_by_bus[group.bus_name]
            for motor in group.motors:
                if motor.send_id in bus_ids:
                    raise ValueError(f"duplicate send_id {motor.send_id} on bus {group.bus_name!r}")
                bus_ids.add(motor.send_id)
        if len(set(all_joint_names)) != len(all_joint_names):
            raise ValueError(f"duplicate joint names across DamiaoRobotSpec: {all_joint_names}")

    @classmethod
    def from_arm_spec(
        cls,
        arm_spec: DamiaoArmSpec,
        *,
        address: str | Path = "can0",
    ) -> DamiaoRobotSpec:
        """Build a one-group robot spec from a compatibility arm spec."""

        return cls(
            name=arm_spec.name,
            vendor=arm_spec.vendor,
            model=arm_spec.model,
            buses={arm_spec.bus_name: DamiaoBusSpec(address=address, fd=arm_spec.fd)},
            groups={
                arm_spec.arm_name: DamiaoJointGroupSpec(
                    bus_name=arm_spec.bus_name,
                    motors=arm_spec.motors,
                    position_lower=arm_spec.position_lower,
                    position_upper=arm_spec.position_upper,
                    velocity_max=arm_spec.velocity_max,
                    kp=arm_spec.kp,
                    kd=arm_spec.kd,
                    gravity_model_path=arm_spec.gravity_model_path,
                    gravity_torque_limits=arm_spec.gravity_torque_limits,
                    supports_velocity=arm_spec.supports_velocity,
                )
            },
            requires_binding=arm_spec.requires_binding,
        )


@dataclass(frozen=True)
class DamiaoArmSpec:
    """Compatibility metadata for a single Damiao arm/group adapter."""

    name: str
    vendor: str
    model: str
    motors: tuple[DamiaoMotorSpec, ...]
    position_lower: tuple[float, ...]
    position_upper: tuple[float, ...]
    velocity_max: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    gravity_model_path: str | Path | None = None
    gravity_torque_limits: tuple[float, ...] | None = None
    requires_binding: bool = False
    bus_name: str = "can"
    arm_name: str = "arm"
    fd: bool = False
    supports_velocity: bool = False

    @property
    def dof(self) -> int:
        """Return the number of joints described by this arm spec."""

        return len(self.motors)

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Return joint names in adapter and command-vector order."""

        return tuple(motor.name for motor in self.motors)

    @classmethod
    def from_values(
        cls,
        *,
        name: str,
        vendor: str,
        model: str,
        motors: Sequence[Mapping[str, object] | DamiaoMotorSpec],
        position_lower: list[float] | tuple[float, ...],
        position_upper: list[float] | tuple[float, ...],
        velocity_max: list[float] | tuple[float, ...],
        kp: list[float] | tuple[float, ...],
        kd: list[float] | tuple[float, ...],
        gravity_model_path: str | Path | None = None,
        gravity_torque_limits: list[float] | tuple[float, ...] | None = None,
        requires_binding: bool = False,
        bus_name: str = "can",
        arm_name: str = "arm",
        fd: bool = False,
        supports_velocity: bool = False,
    ) -> DamiaoArmSpec:
        """Build a typed arm spec from list/tuple metadata values."""

        return cls(
            name=name,
            vendor=vendor,
            model=model,
            motors=coerce_motor_specs(motors, len(motors)),
            position_lower=tuple(float(value) for value in position_lower),
            position_upper=tuple(float(value) for value in position_upper),
            velocity_max=tuple(float(value) for value in velocity_max),
            kp=tuple(float(value) for value in kp),
            kd=tuple(float(value) for value in kd),
            gravity_model_path=gravity_model_path,
            gravity_torque_limits=(
                tuple(float(value) for value in gravity_torque_limits)
                if gravity_torque_limits is not None
                else None
            ),
            requires_binding=requires_binding,
            bus_name=bus_name,
            arm_name=arm_name,
            fd=fd,
            supports_velocity=supports_velocity,
        )

    def validate(self) -> None:
        """Validate CAN ID uniqueness and per-joint metadata lengths."""

        DamiaoRobotSpec.from_arm_spec(self).validate()


def coerce_motor_specs(
    motor_specs: Sequence[Mapping[str, object] | DamiaoMotorSpec],
    dof: int,
) -> tuple[DamiaoMotorSpec, ...]:
    """Normalize mapping or dataclass motor metadata into typed motor specs."""

    specs: list[DamiaoMotorSpec] = []
    for spec in motor_specs:
        if isinstance(spec, DamiaoMotorSpec):
            specs.append(spec)
        else:
            name = spec.get("name")
            send_id = spec.get("send_id")
            recv_id = spec.get("recv_id")
            if not isinstance(name, str):
                raise TypeError("motor spec name must be a string")
            if not isinstance(send_id, int):
                raise TypeError("motor spec send_id must be an integer")
            if recv_id is not None and not isinstance(recv_id, int):
                raise TypeError("motor spec recv_id must be an integer")
            specs.append(
                DamiaoMotorSpec(
                    name=name,
                    type=spec.get("type"),
                    send_id=send_id,
                    recv_id=recv_id,
                )
            )
    if len(specs) != dof:
        raise ValueError(f"motor_specs length {len(specs)} does not match dof {dof}")
    return tuple(specs)


__all__ = [
    "DamiaoArmSpec",
    "DamiaoBusSpec",
    "DamiaoJointGroupSpec",
    "DamiaoMotorSpec",
    "DamiaoRobotSpec",
    "coerce_motor_specs",
]

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

"""What a Feetech leader robot is made of: its buses, motors and triggers.

Plain data. Each motor names the follower joint it drives, so the
connection publishes follower joint targets directly. The motor layer is
:mod:`dimos.robot.manipulators.feetech.arm` and the DimOS module is
:mod:`dimos.robot.manipulators.feetech.connection`.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

FEETECH_DEFAULT_BAUDRATE = 1_000_000
# A trigger drives the jaw on its own side, through that side's gripper stream.
FEETECH_TRIGGER_SIDES = ("left", "right")


@dataclass(frozen=True)
class FeetechMotor:
    """One leader motor and the follower joint it drives, angle for angle.

    Attributes:
        name: The motor name in the calibration file, e.g. "joint_1". Its ID,
            zero and direction come from that file.
        joint: The follower joint it drives, e.g. "left_joint1".
        lower: The follower joint's lowest angle, in radians; lower readings
            are sent as this.
        upper: The follower joint's highest angle, in radians.
    """

    name: str
    joint: str
    lower: float
    upper: float

    def __post_init__(self) -> None:
        if not self.lower < self.upper:
            raise ValueError(f"{self.joint}: lower {self.lower} must be below upper {self.upper}")

    def target(self, reading: float) -> float:
        """The follower target for one reading, in radians, inside the joint's limits.

        Args:
            reading: The motor's angle in radians, within half a turn either
                side of its zero, as the encoder reports it.
        """
        # The encoder wraps half a turn from the motor's zero, which is inside
        # the range of a joint that reaches past +-pi. Move the reading by whole
        # turns into the turn centred on the joint's range, so the wrap falls
        # outside the range and the target stays continuous across +-pi.
        middle = (self.lower + self.upper) / 2
        angle = middle + math.remainder(reading - middle, math.tau)
        return max(self.lower, min(self.upper, angle))


@dataclass(frozen=True)
class FeetechTrigger:
    """A leader trigger that opens and closes the follower's jaw on the same side.

    Attributes:
        name: The motor name in the calibration file, e.g. "joint_7".
        travel_radians: How far the trigger turns from released to fully
            squeezed, in radians. Fully squeezed closes the jaw.
    """

    name: str
    travel_radians: float

    def __post_init__(self) -> None:
        if not self.travel_radians > 0.0:
            raise ValueError(f"{self.name}: travel_radians must be positive")

    def opening(self, reading: float) -> float:
        """The jaw opening for one reading: 1 when released, 0 when fully squeezed."""
        return 1.0 - max(0.0, min(1.0, abs(reading) / self.travel_radians))


@dataclass(frozen=True)
class FeetechBus:
    """One serial bus, e.g. one leader arm.

    Attributes:
        name: The bus name, e.g. "left". It names the bus's calibration
            file, and a bus with a trigger must be "left" or "right".
        motors: The motors that drive follower joints.
        trigger: The motor that drives the follower's jaw, if the arm has one.
    """

    name: str
    motors: tuple[FeetechMotor, ...]
    trigger: FeetechTrigger | None = None

    @property
    def motor_names(self) -> tuple[str, ...]:
        """Every motor the bus reads, joints first, then the trigger."""
        names = [motor.name for motor in self.motors]
        if self.trigger is not None:
            names.append(self.trigger.name)
        return tuple(names)


@dataclass(frozen=True)
class FeetechRobot:
    """Every Feetech bus one connection reads, e.g. a left and a right leader arm.

    Attributes:
        buses: The buses, in the order their joints are published.
        baudrate: Serial speed of every bus, in bits per second.
    """

    buses: tuple[FeetechBus, ...]
    baudrate: int = FEETECH_DEFAULT_BAUDRATE

    def __post_init__(self) -> None:
        names = [bus.name for bus in self.buses]
        if not names or len(set(names)) != len(names):
            raise ValueError(f"bus names must be given and distinct: {names}")
        joints = [motor.joint for bus in self.buses for motor in bus.motors]
        if len(set(joints)) != len(joints):
            raise ValueError(f"a follower joint is driven twice: {joints}")
        for bus in self.buses:
            if not bus.motors or len(set(bus.motor_names)) != len(bus.motor_names):
                raise ValueError(f"{bus.name}: motor names must be given and distinct")
            if bus.trigger is not None and bus.name not in FEETECH_TRIGGER_SIDES:
                raise ValueError(f"{bus.name}: a bus with a trigger must be left or right")

    def bus(self, name: str) -> FeetechBus:
        """The bus called ``name``."""
        for bus in self.buses:
            if bus.name == name:
                return bus
        raise KeyError(f"no Feetech bus named {name!r}")

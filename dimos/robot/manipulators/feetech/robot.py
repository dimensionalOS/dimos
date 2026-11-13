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

"""What a Feetech robot is made of: its serial buses and the motors on each.

Plain data. The motor layer is :mod:`dimos.robot.manipulators.feetech.arm` and
the DimOS module that publishes the readings is
:mod:`dimos.robot.manipulators.feetech.connection`.
"""

from __future__ import annotations

from dataclasses import dataclass

FEETECH_DEFAULT_BAUDRATE = 1_000_000


@dataclass(frozen=True)
class FeetechBus:
    """One serial bus and the motors on it, e.g. one leader arm.

    Attributes:
        name: The bus name, e.g. "left". It starts each of the bus's joint
            names ("left_joint_1") and names its calibration file.
        motors: Motor names as the calibration file names them, e.g.
            ("joint_1", ..., "joint_7"). Motor IDs come from that file.
    """

    name: str
    motors: tuple[str, ...]


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
        for bus in self.buses:
            if not bus.motors or len(set(bus.motors)) != len(bus.motors):
                raise ValueError(f"{bus.name}: motor names must be given and distinct")

    def bus(self, name: str) -> FeetechBus:
        """The bus called ``name``."""
        for bus in self.buses:
            if bus.name == name:
                return bus
        raise KeyError(f"no Feetech bus named {name!r}")


def joint_name(bus: str, motor: str) -> str:
    """The joint name a connection publishes for one motor, e.g. "left_joint_1"."""
    return f"{bus}_{motor}"

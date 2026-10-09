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

"""What a robot can do, written down as plain data.

Before anything will talk to a robot it has to know five things about it:
its name, which parts it has (and what each reports, accepts, and measures in),
how far each command may go, how often it reports, and how long it waits for
an instruction before stopping itself. ``ControlDescription`` holds those five.

It is only data. Nothing here talks to hardware or over the network, and
nothing checks itself as you build it. Build it however you like, then call
``validate_description`` once, which reports everything wrong with it at once.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum

from dimos.control.contract.keys import SEPARATOR, Unit


class ResourceKind(Enum):
    """What kind of part this is.

    A gripper counts as a JOINT: it has a position something drives, and what
    makes it a gripper is the unit it measures in. An IMU is a SENSOR.
    """

    JOINT = "joint"
    BASE = "base"
    SENSOR = "sensor"


@dataclass(frozen=True, slots=True, kw_only=True)
class Resource:
    """One part of a robot, and the numbers it accepts and reports.

    What it reports and what it accepts are listed separately, because they
    are usually different. A sensor only reports. Some joints accept a
    position but cannot measure how fast they are turning, and those leave
    velocity out of what they report rather than sending zeros that would look
    like real readings.

    Attributes:
        name: What this part is called, e.g. "joint1".
        kind: Whether it is a joint, a base, or a sensor.
        state_interfaces: What it can tell you about itself.
        command_interfaces: What it can be told to do.
        units: The unit of each of those.
    """

    name: str
    kind: ResourceKind
    state_interfaces: tuple[str, ...] = ()
    command_interfaces: tuple[str, ...] = ()
    units: Mapping[str, Unit] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # A plain dict pickles across processes, and a caller changing their
        # own mapping afterwards cannot reach in here.
        object.__setattr__(self, "units", dict(self.units))


@dataclass(frozen=True, slots=True)
class Limits:
    """How far one command is allowed to go.

    A command past a limit is refused. With ``clamp`` set it is pulled back to
    the limit and sent instead, so a teleop stick pushed too far still drives a
    base at its top speed.

    Attributes:
        lo: The lowest allowed value, in the key's unit. ``None`` for no limit.
        hi: The highest allowed value, in the key's unit. ``None`` for no limit.
        clamp: Pull the value back to the limit instead of refusing the
            command. Only allowed on a base's commands, and needs both ``lo``
            and ``hi``.
    """

    lo: float | None = None
    hi: float | None = None
    clamp: bool = False


@dataclass(frozen=True, slots=True, kw_only=True)
class ControlDescription:
    """Everything needed to drive one robot. Fixed for as long as it is connected.

    Attributes:
        source: The robot's name, and the first part of each of its keys,
            e.g. "arm".
        resources: Its parts, and what each reports and accepts.
        limits: How far each command may go, by key such as
            "arm/joint1/position". A command key with no entry is unlimited.
        state_rate_hz: How many times a second the robot reports its state.
        deadman_timeout_s: How long the robot goes without an instruction, in
            seconds, before it stops itself.
    """

    source: str
    resources: tuple[Resource, ...]
    limits: Mapping[str, Limits] = field(default_factory=dict)
    state_rate_hz: float
    deadman_timeout_s: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "limits", dict(self.limits))

    def resource(self, name: str) -> Resource | None:
        """The part called ``name``, or ``None``."""
        for res in self.resources:
            if res.name == name:
                return res
        return None

    def state_keys(self) -> tuple[str, ...]:
        """Every key the robot reports, parts in order, then interfaces in order."""
        return tuple(
            f"{self.source}{SEPARATOR}{res.name}{SEPARATOR}{iface}"
            for res in self.resources
            for iface in res.state_interfaces
        )

    def command_keys(self) -> tuple[str, ...]:
        """Every key the robot accepts, parts in order, then interfaces in order."""
        return tuple(
            f"{self.source}{SEPARATOR}{res.name}{SEPARATOR}{iface}"
            for res in self.resources
            for iface in res.command_interfaces
        )

    def joint_names(self) -> tuple[str, ...]:
        """Name every joint, in order, e.g. "arm/joint1".

        Only joints. A base is one thing that is told how fast to move, not a
        set of pretend joints, and a sensor is never driven at all.
        """
        return tuple(
            f"{self.source}{SEPARATOR}{res.name}"
            for res in self.resources
            if res.kind is ResourceKind.JOINT
        )

    def unit_of(self, key: str) -> Unit | None:
        """The unit of ``key``, or ``None`` if this robot does not declare it."""
        parts = key.split(SEPARATOR)
        if len(parts) != 3 or parts[0] != self.source:
            return None
        res = self.resource(parts[1])
        if res is None:
            return None
        return res.units.get(parts[2])

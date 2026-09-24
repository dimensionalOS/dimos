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

Before anything will talk to a robot it has to know what the robot is: which
parts it has, what each part can be told to do and report back, in what units,
within what limits, which things can be driven at the same time, and how it
stops. ``ControlDescription`` holds all of that.

It is only data. Nothing here talks to hardware or over the network, and
nothing checks itself as you build it. Build it however you like, then call
``validate_description``, which reports everything wrong with it at once.

A robot with two separate halves, such as a body and the wheels under it, has
one of these for each.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum

from dimos.control.contract.keys import KD, KP, POSITION, Unit

_MAPPING_FIELDS: tuple[str, ...] = ()
"""Overridden per class below. Frozen dataclasses normalize these to dict."""


def _normalize_mappings(instance: object, names: tuple[str, ...]) -> None:
    """Store a plain copy of each named mapping field.

    Callers may hand in any kind of mapping, but only a plain dict can be sent
    between processes. Copying also means a caller changing their own mapping
    afterwards cannot reach inside a description that is meant to be fixed.
    """
    for name in names:
        current = getattr(instance, name)
        if current is not None and not isinstance(current, dict):
            object.__setattr__(instance, name, dict(current))


class ResourceKind(Enum):
    """What kind of part this is.

    Only three kinds, because only three behave differently. A gripper counts
    as a JOINT: it has a position something drives, and what makes it a gripper
    is the unit it measures in, not its kind. An orientation sensor is a
    SENSOR.
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
        _normalize_mappings(self, ("units",))


class LimitPolicy(Enum):
    """What to do with a command that falls outside the limits.

    REJECT turns the whole command away. CLAMP pulls the value back to the
    nearest limit and carries on, which suits a robot balancing itself, where
    overshooting by a fraction should not stop it dead.
    """

    REJECT = "reject"
    CLAMP = "clamp"


@dataclass(frozen=True, slots=True)
class Limits:
    """How far one number is allowed to go.

    ``None`` on either side means no limit in that direction.
    """

    lo: float | None = None
    hi: float | None = None
    policy: LimitPolicy = LimitPolicy.REJECT


@dataclass(frozen=True, slots=True, kw_only=True)
class ModeGroup:
    """A set of things that can be driven at the same time.

    An arm can usually be told where to go, or how fast to move, but not both
    at once -- the two instructions would contradict each other. Each way of
    driving it is one of these groups, and marking them exclusive means only
    one can be in use at a time.

    Attributes:
        name: What this way of driving is called, e.g. "position".
        resources: The parts it covers.
        interfaces: What it drives on them.
        exclusive: Whether using this rules out the other groups covering the
            same parts. A gripper is not exclusive, so it stays usable
            whichever way the arm is being driven.
    """

    name: str
    resources: tuple[str, ...]
    interfaces: frozenset[str]
    exclusive: bool = True


class Omission(Enum):
    """What it means when an instruction does not mention something.

    RETAIN_LAST keeps whatever it was last told. ZERO sets it to zero. UNSET
    passes nothing through at all, for hardware that reads a commanded zero as
    a real instruction rather than as silence -- on some robots a commanded
    speed of zero means "use your own default speed", so sending zero would be
    saying something quite different from saying nothing.
    """

    RETAIN_LAST = "retain_last"
    ZERO = "zero"
    UNSET = "unset"


#: Omission for an interface no description overrides. Position and gains hold;
#: everything else falls to zero.
DEFAULT_OMISSION: Mapping[str, Omission] = {
    POSITION: Omission.RETAIN_LAST,
    KP: Omission.RETAIN_LAST,
    KD: Omission.RETAIN_LAST,
}

#: Omission for any interface not in ``DEFAULT_OMISSION``.
FALLBACK_OMISSION = Omission.ZERO


class SafeStopKind(Enum):
    """How a robot comes to rest when whatever was driving it lets go."""

    HOLD = "hold"
    ZERO = "zero"
    ZERO_RAMP = "zero_ramp"
    DAMP = "damp"
    VENDOR = "vendor"


@dataclass(frozen=True, slots=True, kw_only=True)
class SafeStop:
    """How the robot stops, and where that leaves it.

    Attributes:
        kind: The way it stops.
        kd: For DAMP, how strongly to resist movement as it goes slack.
        ramp_s: For ZERO_RAMP, how many seconds to slow down over.
        stable_state: Where this leaves it, in words, e.g. "rolls to a stop".
            For whoever has to decide whether it is safe to stand nearby.
    """

    kind: SafeStopKind
    kd: Mapping[str, float] | None = None
    ramp_s: float | None = None
    stable_state: str = ""

    def __post_init__(self) -> None:
        _normalize_mappings(self, ("kd",))


class EstopKind(Enum):
    """What the robot does when stopped in an emergency."""

    DISABLE = "disable"
    DAMP = "damp"
    ZERO = "zero"
    HOLD = "hold"
    VENDOR = "vendor"


class EstopRecovery(Enum):
    """What it takes to get going again after an emergency stop."""

    CLEAR = "clear"
    PREPARE_ARM_REQUIRED = "prepare_arm_required"


@dataclass(frozen=True, slots=True, kw_only=True)
class Estop:
    """What an emergency stop does to this robot, and how to recover.

    Recovery is stated because it is not free and it differs. Some robots start
    again at the touch of a button; others end up on the floor, or have to be
    recalibrated before they will move.

    Attributes:
        kind: What the emergency stop does.
        recovery: What it takes to get going again.
        stable_state: Where the stop leaves it, in words, e.g. "limp".
    """

    kind: EstopKind
    recovery: EstopRecovery
    stable_state: str = ""


class ActivationPolicy(Enum):
    """Whether a person has to confirm before the robot will move."""

    DIRECT = "direct"
    OPERATOR_CONFIRMED = "operator_confirmed"


class WriteMode(Enum):
    """The one non-numeric setting ``Timing.write_rate_hz`` accepts."""

    ON_RECEIPT = "on_receipt"


#: Send an instruction only when a new one arrives, rather than repeating the
#: last one on a clock. For robots that re-plan every time they are told
#: something, where repeating would keep restarting the plan.
WRITE_ON_RECEIPT = WriteMode.ON_RECEIPT


@dataclass(frozen=True, slots=True, kw_only=True)
class Timing:
    """How often the robot is expected to speak, and how long to wait for it.

    Attributes:
        state_rate_hz: How many times a second it reports.
        stale_timeout_s: How long without a report before it counts as gone
            quiet.
        watchdog_timeout_s: How long without an instruction before it stops
            itself.
        hook_timeout_s: How long any one call into the robot's own software
            may take before giving up.
        prepare_arm_timeout_s: How long its start-up may take.
        write_rate_hz: How often to repeat the last instruction. ``None``
            means the same rate it reports at. ``WRITE_ON_RECEIPT`` means only
            when a new instruction arrives, for robots that re-plan every time
            they are told something.
    """

    state_rate_hz: float
    stale_timeout_s: float
    watchdog_timeout_s: float
    hook_timeout_s: float = 1.0
    prepare_arm_timeout_s: float = 10.0
    write_rate_hz: float | WriteMode | None = None

    def effective_write_rate_hz(self) -> float | None:
        """The clock to re-emit on, or ``None`` when writing only on receipt."""
        if self.write_rate_hz is WRITE_ON_RECEIPT:
            return None
        if self.write_rate_hz is None:
            return self.state_rate_hz
        assert isinstance(self.write_rate_hz, float | int)
        return float(self.write_rate_hz)


class ProcessLoss(Enum):
    """What the hardware does by itself if the program driving it dies.

    UNKNOWN is the honest default. No robot should claim better until somebody
    has actually pulled the plug and watched what happens.
    """

    UNPROTECTED = "unprotected"
    NATIVE_WATCHDOG = "native_watchdog"
    EXTERNAL_SUPERVISOR = "external_supervisor"
    INTRINSICALLY_SAFE = "intrinsically_safe"
    SIMULATION = "simulation"
    UNKNOWN = "unknown"


class AvailableAfter(Enum):
    """When a part starts giving real readings: as soon as it is connected, or
    only after the robot has been started up."""

    CONNECT = "connect"
    PREPARE_ARM = "prepare_arm"


@dataclass(frozen=True, slots=True, kw_only=True)
class ShutdownMotion:
    """Where to put the robot before switching it off.

    For an arm with no brakes, which would otherwise drop under its own weight.

    Attributes:
        pose: Where each joint should be, in its own unit.
        tolerance: How close is close enough.
        timeout_s: How long to allow for getting there.
    """

    pose: Mapping[str, float]
    tolerance: float
    timeout_s: float

    def __post_init__(self) -> None:
        _normalize_mappings(self, ("pose",))


@dataclass(frozen=True, slots=True, kw_only=True)
class ControlDescription:
    """Everything anyone needs to know about one robot in order to drive it.

    This never changes while the robot is running. If something about it does
    change -- a different speed limit, say -- the robot is stopped, a fresh
    description is published, and ``epoch`` goes up to mark it as a new one.
    Nothing edits a description in place.
    """

    source: str
    resources: tuple[Resource, ...]
    limits: Mapping[str, Limits] = field(default_factory=dict)
    mode_groups: tuple[ModeGroup, ...] = ()
    omission: Mapping[str, Omission] = field(default_factory=dict)
    initial_values: Mapping[str, float] = field(default_factory=dict)
    safe_stop: SafeStop
    estop: Estop
    activation_policy: ActivationPolicy
    timing: Timing
    process_loss: ProcessLoss | Mapping[str, ProcessLoss]
    covered_resources: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    available_after: Mapping[str, AvailableAfter] = field(default_factory=dict)
    shutdown_motion: ShutdownMotion | None = None
    meta: Mapping[str, str] = field(default_factory=dict)
    epoch: int = 0

    def __post_init__(self) -> None:
        _normalize_mappings(
            self,
            (
                "limits",
                "omission",
                "initial_values",
                "covered_resources",
                "available_after",
                "meta",
            ),
        )
        # process_loss is a single value or a per-group mapping; only copy the
        # mapping form.
        if not isinstance(self.process_loss, ProcessLoss):
            object.__setattr__(self, "process_loss", dict(self.process_loss))

    def resource(self, name: str) -> Resource | None:
        """The declared resource called ``name``, or ``None``."""
        for res in self.resources:
            if res.name == name:
                return res
        return None

    def state_keys(self) -> tuple[str, ...]:
        """Every declared state key, resources in order, interfaces in order."""
        return tuple(
            f"{self.source}/{res.name}/{iface}"
            for res in self.resources
            for iface in res.state_interfaces
        )

    def command_keys(self) -> tuple[str, ...]:
        """Every declared command key, resources in order, interfaces in order."""
        return tuple(
            f"{self.source}/{res.name}/{iface}"
            for res in self.resources
            for iface in res.command_interfaces
        )

    def joint_names(self) -> tuple[str, ...]:
        """Name every joint, in order.

        Only joints. A base is one thing that is told how fast to move, not a
        set of pretend joints, and a sensor is never driven at all.
        """
        return tuple(
            f"{self.source}/{res.name}" for res in self.resources if res.kind is ResourceKind.JOINT
        )

    def is_state_key(self, key: str) -> bool:
        """True when ``key`` is a declared state key of this source."""
        return key in set(self.state_keys())

    def is_command_key(self, key: str) -> bool:
        """True when ``key`` is a declared command key of this source."""
        return key in set(self.command_keys())

    def unit_of(self, key: str) -> Unit | None:
        """The declared unit of ``key``, or ``None`` if undeclared."""
        parts = key.split("/")
        if len(parts) != 3 or parts[0] != self.source:
            return None
        res = self.resource(parts[1])
        if res is None:
            return None
        return res.units.get(parts[2])

    def omission_of(self, key: str) -> Omission:
        """How an omitted ``key`` is filled, applying the defaults."""
        explicit = self.omission.get(key)
        if explicit is not None:
            return explicit
        return DEFAULT_OMISSION.get(key.rsplit("/", 1)[-1], FALLBACK_OMISSION)

    def groups_for(self, resource: str) -> tuple[ModeGroup, ...]:
        """Every mode group whose resource set contains ``resource``, in order."""
        return tuple(g for g in self.mode_groups if resource in g.resources)

    def process_loss_of(self, group_name: str | None = None) -> ProcessLoss:
        """Process-loss class overall, or for one mode group when declared per group."""
        if isinstance(self.process_loss, ProcessLoss):
            return self.process_loss
        if group_name is None:
            return ProcessLoss.UNKNOWN
        return self.process_loss.get(group_name, ProcessLoss.UNKNOWN)

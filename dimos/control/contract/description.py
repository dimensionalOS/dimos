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

"""What one control source can do, as plain data.

``ControlDescription`` is the whole vendor-facing surface: the resources a
source owns, the interfaces on each, their units and limits, which interfaces
may be driven together, what happens to an omitted value, and how the thing
stops. The coordinator reads it to arbitrate; ``ConnectedHardware`` reads it to
assemble frames. Neither has vendor-specific code, because everything that used
to be a subclass is a field here.

A connection module's ``describe_control()`` RPC returns ``list[ControlDescription]``
-- one per ``ConnectedHardware`` it owns, so an R1Pro module returns two (upper
body and chassis) on one port pair (D15). That RPC lives on the module; this
package is pure data and knows nothing about transport.

Everything here is frozen, pickles, and maps onto a Rust struct or enum without
cleverness. Nothing validates itself on construction -- build freely, then call
``validate_description``, which reports every problem at once.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum

from dimos.control.contract.keys import KD, KP, POSITION, Unit


class ResourceKind(Enum):
    """What a resource is. Informational: behaviour comes from the other fields."""

    JOINT = "joint"
    GRIPPER = "gripper"
    BASE = "base"
    IMU = "imu"
    SENSOR = "sensor"


#: Resource kinds a task claims as a joint and that ``coordinator_joint_state``
#: reports. A base is claimed as a resource, not a joint; IMU and sensors are
#: state-only.
JOINT_LIKE_KINDS = frozenset({ResourceKind.JOINT, ResourceKind.GRIPPER})


@dataclass(frozen=True, slots=True, kw_only=True)
class Resource:
    """One addressable thing on a source, and the interfaces it exposes.

    ``state_interfaces`` and ``command_interfaces`` deliberately differ: an IMU
    is state-only, a gripper is often command-only ``position``, and an xArm
    reports no joint velocity at all rather than publishing fabricated zeros.
    """

    name: str
    kind: ResourceKind
    state_interfaces: tuple[str, ...] = ()
    command_interfaces: tuple[str, ...] = ()
    units: Mapping[str, Unit] = field(default_factory=dict)


class LimitPolicy(Enum):
    """What a batch outside a limit does. Per interface, default reject (D13)."""

    REJECT = "reject"
    CLAMP = "clamp"


@dataclass(frozen=True, slots=True)
class Limits:
    """Closed range for one key. ``None`` means unbounded on that side."""

    lo: float | None = None
    hi: float | None = None
    policy: LimitPolicy = LimitPolicy.REJECT


@dataclass(frozen=True, slots=True, kw_only=True)
class ModeGroup:
    """A set of interfaces over a set of resources that may be driven together.

    This replaces ``ControlMode``. Two exclusive groups over overlapping
    resources cannot be active at once, which is what stops a velocity task and
    a trajectory task from starving each other on one arm (D10).
    """

    name: str
    resources: tuple[str, ...]
    interfaces: frozenset[str]
    exclusive: bool = True


class Omission(Enum):
    """What a frame means when it does not mention a command key.

    ``UNSET`` is opt-in per key and is the only value that lets ``None`` reach a
    vendor's ``write()``. It exists because 0.0 is not "no command" everywhere:
    R1Pro reads ``dq=0`` as "use the tracking speed" and G1 encodes no-command as
    its ``VEL_STOP`` sentinel (D11).
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
    """How a source comes to rest when command authority is withdrawn."""

    HOLD = "hold"
    ZERO = "zero"
    ZERO_RAMP = "zero_ramp"
    DAMP = "damp"
    VENDOR = "vendor"


@dataclass(frozen=True, slots=True, kw_only=True)
class SafeStop:
    """Safe-stop policy. ``stable_state`` says in words where the robot ends up."""

    kind: SafeStopKind
    kd: Mapping[str, float] | None = None
    ramp_s: float | None = None
    stable_state: str = ""


class EstopKind(Enum):
    """How a source drops authority immediately."""

    DISABLE = "disable"
    DAMP = "damp"
    ZERO = "zero"
    HOLD = "hold"
    VENDOR = "vendor"


class EstopRecovery(Enum):
    """What it takes to leave a latched estop."""

    CLEAR = "clear"
    PREPARE_ARM_REQUIRED = "prepare_arm_required"


@dataclass(frozen=True, slots=True, kw_only=True)
class Estop:
    """Estop policy. Recovery is declared because it differs per vendor.

    A Go2 ``Damp`` drops the dog on the floor and a Damiao recovery recalibrates
    the gripper, so a caller cannot assume clearing is free.
    """

    kind: EstopKind
    recovery: EstopRecovery
    stable_state: str = ""


class ActivationPolicy(Enum):
    """Whether arming needs an operator step. Replaces ``auto_enable``."""

    DIRECT = "direct"
    OPERATOR_CONFIRMED = "operator_confirmed"


class WriteMode(Enum):
    """Non-numeric values for ``Timing.write_rate_hz``."""

    ON_RECEIPT = "on_receipt"


#: Write once per accepted command frame instead of on a clock. The D16 escape
#: hatch for SDKs that replan per message. Distinct from ``None``, which means
#: "re-emit the retained frame at ``state_rate_hz``", so neither value is
#: overloaded.
WRITE_ON_RECEIPT = WriteMode.ON_RECEIPT


@dataclass(frozen=True, slots=True, kw_only=True)
class Timing:
    """Rates and deadlines, all in seconds or hertz.

    ``write_rate_hz`` of ``None`` means "same as ``state_rate_hz``"; the sentinel
    ``WRITE_ON_RECEIPT`` means "only when a frame arrives".
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
    """What the hardware does if the commanding process dies.

    ``UNKNOWN`` is the honest default and is what every audited physical device
    is entitled to claim until someone runs the bench checklist (D23).
    """

    UNPROTECTED = "unprotected"
    NATIVE_WATCHDOG = "native_watchdog"
    EXTERNAL_SUPERVISOR = "external_supervisor"
    INTRINSICALLY_SAFE = "intrinsically_safe"
    SIMULATION = "simulation"
    UNKNOWN = "unknown"


class AvailableAfter(Enum):
    """The lifecycle point at which a resource starts reporting real state."""

    CONNECT = "connect"
    PREPARE_ARM = "prepare_arm"


@dataclass(frozen=True, slots=True, kw_only=True)
class ShutdownMotion:
    """A pose to park at before de-energizing, for arms without brakes."""

    pose: Mapping[str, float]
    tolerance: float
    timeout_s: float


@dataclass(frozen=True, slots=True, kw_only=True)
class ControlDescription:
    """Everything the coordinator and ConnectedHardware need about one source.

    Immutable per epoch. A disarmed re-describe bumps ``epoch``, which is how a
    Go2 switching into rage mode publishes a different velocity envelope without
    anyone mutating a live description.
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
        """``"<source>/<resource>"`` for every joint-like resource, in order.

        Joints and grippers, not bases or sensors -- a base is claimed as a
        resource with vx/vy/wz, never as a virtual joint (D3).
        """
        return tuple(
            f"{self.source}/{res.name}" for res in self.resources if res.kind in JOINT_LIKE_KINDS
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

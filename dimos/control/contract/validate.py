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

"""Checks that a robot's description, readings and commands all make sense.

Three checks, which fail in three different ways because they are used at
three different times:

``validate_description`` runs once when a robot connects, and reports
everything wrong at once rather than the first thing, so a bad description can
be fixed in one go instead of one typo per attempt.

``validate_state`` checks a reading from the robot, and raises. A reading that
does not match what the robot said it would send is a fault in the robot's
driver, not something to expect.

``validate_command`` checks an instruction being sent to the robot, and
returns a result instead of raising. It runs on every command, and a refused
command is an ordinary event to be counted, not an error to unwind.

An instruction is all or nothing. If any part of it is refused, none of it is
applied -- the robot is never left half-told.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from dimos.control.contract.description import ControlDescription, ResourceKind
from dimos.control.contract.keys import SEPARATOR, is_valid_segment
from dimos.control.contract.sequence import is_newer
from dimos.msgs.control_msgs.ControlValues import ControlValues


class DescriptionError(Exception):
    """A description that cannot be used, listing everything wrong with it.

    Every problem at once rather than the first one, so a bad description can
    be fixed in one pass.
    """

    def __init__(self, errors: list[str]) -> None:
        self.errors = list(errors)
        super().__init__(self.errors)

    def __str__(self) -> str:
        return f"{len(self.errors)} problem(s): " + "; ".join(self.errors)


class FrameRejectedError(Exception):
    """A reading from a robot that does not match what the robot said it sends."""

    def __init__(self, reason: str, detail: str) -> None:
        self.reason = reason
        self.detail = detail
        super().__init__(reason, detail)

    def __str__(self) -> str:
        return f"{self.reason}: {self.detail}"


@dataclass(frozen=True, slots=True)
class Rejected:
    """Why an instruction was refused. Returned rather than raised, because a
    refusal is an ordinary event to count, not an error to unwind."""

    reason: str
    detail: str


@dataclass(frozen=True, slots=True)
class CommandBatch:
    """An accepted instruction, ready to send to the robot.

    Attributes:
        values: The numbers to send, by key.
        clamped: Keys whose values were past a clamping limit and were pulled
            back to it, so the caller can count them without comparing
            against what it sent.
    """

    values: dict[str, float]
    clamped: tuple[str, ...] = ()


def validate_description(desc: ControlDescription) -> None:
    """Check a robot's description for anything that would not work.

    Run once when the robot connects.

    Args:
        desc: The description to check.

    Raises:
        DescriptionError: Listing every problem found, not just the first.
    """
    errors: list[str] = []
    command_keys = set(desc.command_keys())
    resource_names = [res.name for res in desc.resources]

    # 1. Names are legal segments, resources are unique, interfaces have units.
    if not is_valid_segment(desc.source):
        errors.append(f"source {desc.source!r} is not a valid segment")
    if len(set(resource_names)) != len(resource_names):
        dupes = sorted({n for n in resource_names if resource_names.count(n) > 1})
        errors.append(f"duplicate resource names: {dupes}")
    for res in desc.resources:
        if not is_valid_segment(res.name):
            errors.append(f"resource {res.name!r} is not a valid segment")
        for label, interfaces in (
            ("state", res.state_interfaces),
            ("command", res.command_interfaces),
        ):
            repeated = sorted({i for i in interfaces if interfaces.count(i) > 1})
            if repeated:
                errors.append(
                    f"resource {res.name!r} declares {label} interface(s) {repeated} "
                    f"more than once, which would make its keys ambiguous"
                )
        for iface in res.state_interfaces + res.command_interfaces:
            if not is_valid_segment(iface):
                errors.append(f"interface {iface!r} on {res.name!r} is not a valid segment")
            elif iface not in res.units:
                errors.append(f"interface {iface!r} on {res.name!r} has no declared unit")

    # 2. Limits sit on declared command keys, describe a real range, and only
    #    a base's commands may clamp.
    base_commands = {
        f"{desc.source}{SEPARATOR}{res.name}{SEPARATOR}{iface}"
        for res in desc.resources
        if res.kind is ResourceKind.BASE
        for iface in res.command_interfaces
    }
    for key, lim in desc.limits.items():
        if key not in command_keys:
            errors.append(f"limit key {key!r} is not a declared command key")
        for bound_name, bound in (("lo", lim.lo), ("hi", lim.hi)):
            if bound is not None and not math.isfinite(bound):
                errors.append(f"limit {key!r} has non-finite {bound_name}")
        if lim.lo is not None and lim.hi is not None and lim.lo > lim.hi:
            errors.append(f"limit {key!r} has lo {lim.lo} above hi {lim.hi}")
        if lim.clamp:
            if lim.lo is None or lim.hi is None:
                errors.append(f"limit {key!r} clamps but is not bounded on both sides")
            if key in command_keys and key not in base_commands:
                errors.append(
                    f"limit {key!r} clamps, but only a base's commands may clamp; "
                    f"anything else past its limit is refused"
                )

    # 3. Rates are positive and finite.
    for label, value in (
        ("state_rate_hz", desc.state_rate_hz),
        ("deadman_timeout_s", desc.deadman_timeout_s),
    ):
        if not (math.isfinite(value) and value > 0):
            errors.append(f"{label} must be positive, got {value}")

    if errors:
        raise DescriptionError(errors)


def validate_state(desc: ControlDescription, frame: ControlValues) -> dict[str, float]:
    """Check a reading from a robot, and return it as a lookup by name.

    A reading is a full report, not a partial one. Everything the robot said it
    would send must be there exactly once, and nothing else may be.

    Args:
        desc: What the robot said it would send.
        frame: The reading that arrived.

    Returns:
        The reading, as a value per name.

    Raises:
        FrameRejectedError: If the reading is from a different robot, or leaves
            something out, adds something unexpected, or repeats a name.
    """
    if len(frame.interface_names) != len(frame.values):
        raise FrameRejectedError(
            "shape",
            f"{len(frame.interface_names)} names against {len(frame.values)} values",
        )
    if frame.source != desc.source:
        raise FrameRejectedError(
            "source", f"frame is from {frame.source!r}, expected {desc.source!r}"
        )

    seen: dict[str, float] = {}
    duplicates: list[str] = []
    for name, value in zip(frame.interface_names, frame.values, strict=True):
        if name in seen:
            duplicates.append(name)
        seen[name] = value
    if duplicates:
        raise FrameRejectedError("duplicate_key", f"repeated keys {sorted(set(duplicates))}")

    expected = set(desc.state_keys())
    got = set(seen)
    missing = sorted(expected - got)
    unknown = sorted(got - expected)
    if missing:
        raise FrameRejectedError("missing_key", f"state frame omits {missing}")
    if unknown:
        raise FrameRejectedError("unknown_key", f"state frame declares undeclared {unknown}")
    return seen


def validate_command(
    desc: ControlDescription,
    frame: ControlValues,
    *,
    last_sequence: int | None,
) -> CommandBatch | Rejected:
    """Check an instruction for a robot, and return what to send, or why not.

    Only keys belonging to this robot are looked at. One instruction can carry
    commands for several robots at once, so another robot's keys are skipped
    rather than treated as a mistake.

    An instruction that carries nothing is valid but changes nothing. It does
    not keep a robot connection's deadman alive: that is fed only by
    instructions carrying the connection's own keys.

    Args:
        desc: What this robot said it accepts.
        frame: The instruction that arrived.
        last_sequence: The sequence number of the last instruction accepted,
            so anything older or repeated is refused. ``None`` for the first.

    Returns:
        The values to send, or the reason they were refused.
    """
    # ControlValues enforces equal lengths on construction, so this only fires
    # for a frame built by hand or mutated after the fact. It is still checked
    # here because this path is specified never to raise.
    if len(frame.interface_names) != len(frame.values):
        return Rejected(
            "shape", f"{len(frame.interface_names)} names against {len(frame.values)} values"
        )
    if not is_newer(frame.sequence, last_sequence):
        return Rejected(
            "sequence", f"frame sequence {frame.sequence} not newer than {last_sequence}"
        )

    prefix = f"{desc.source}{SEPARATOR}"
    considered: list[tuple[str, float]] = [
        (name, value)
        for name, value in zip(frame.interface_names, frame.values, strict=True)
        if name.startswith(prefix)
    ]

    command_keys = set(desc.command_keys())
    unknown = sorted({name for name, _ in considered if name not in command_keys})
    if unknown:
        return Rejected("unknown_key", f"not declared command keys: {unknown}")

    counts: dict[str, int] = {}
    for name, _ in considered:
        counts[name] = counts.get(name, 0) + 1
    duplicates = sorted(k for k, n in counts.items() if n > 1)
    if duplicates:
        return Rejected("duplicate_key", f"repeated keys {duplicates}")

    values: dict[str, float] = {}
    clamped: list[str] = []
    for name, value in considered:
        lim = desc.limits.get(name)
        if lim is None:
            values[name] = value
        elif lim.lo is not None and value < lim.lo:
            if not lim.clamp:
                return Rejected("limit", f"{name} = {value} is below its limit {lim.lo}")
            values[name] = float(lim.lo)
            clamped.append(name)
        elif lim.hi is not None and value > lim.hi:
            if not lim.clamp:
                return Rejected("limit", f"{name} = {value} is above its limit {lim.hi}")
            values[name] = float(lim.hi)
            clamped.append(name)
        else:
            values[name] = value

    return CommandBatch(values=values, clamped=tuple(clamped))

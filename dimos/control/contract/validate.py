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

"""Validators for descriptions, state frames and command batches.

Three entry points, with deliberately different failure styles:

``validate_description`` runs once at startup and reports *every* problem at
once, because a vendor fixing one typo at a time is a bad afternoon.

``validate_state`` raises, because an unusable state frame is a bug in the
source, not a routine event.

``validate_command`` returns ``CommandBatch | Rejected`` and never raises, because
it runs every cycle and a rejected batch is an ordinary thing that gets counted
and logged, not an exception to unwind.

A batch is all-or-nothing. On any rejection nothing is applied -- there is no
path that writes half a frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from dimos.control.contract.description import (
    ControlDescription,
    LimitPolicy,
    Omission,
    ProcessLoss,
    SafeStopKind,
    WriteMode,
)
from dimos.control.contract.keys import KD, POSITION, is_valid_segment
from dimos.control.contract.sequence import is_newer
from dimos.msgs.control_msgs.ControlValues import ControlValues


class DescriptionError(Exception):
    """A description that cannot be used, carrying every reason at once."""

    def __init__(self, errors: list[str]) -> None:
        self.errors = list(errors)
        super().__init__(f"{len(self.errors)} problem(s): " + "; ".join(self.errors))


class FrameRejectedError(Exception):
    """A state frame that does not match the description it claims to describe."""

    def __init__(self, reason: str, detail: str) -> None:
        self.reason = reason
        self.detail = detail
        super().__init__(f"{reason}: {detail}")


@dataclass(frozen=True, slots=True)
class Rejected:
    """Why a command batch was not applied. Returned, never raised."""

    reason: str
    detail: str


@dataclass(frozen=True, slots=True)
class CommandBatch:
    """An accepted command batch, ready to be merged into a retained frame.

    ``clamped`` names the keys a CLAMP-policy limit moved, so the caller can
    count them without diffing against what it sent.
    """

    values: dict[str, float]
    active_groups: frozenset[str]
    clamped: tuple[str, ...] = ()


def _finite(value: float) -> bool:
    return math.isfinite(value)


def validate_description(desc: ControlDescription) -> None:
    """Check a description against every contract rule.

    Raises:
        DescriptionError: Carrying one message per problem found.
    """
    errors: list[str] = []
    state_keys = set(desc.state_keys())
    command_keys = set(desc.command_keys())
    resource_names = [res.name for res in desc.resources]
    declared = set(resource_names)

    # 1. Names are legal segments, resources are unique, interfaces have units.
    if not is_valid_segment(desc.source):
        errors.append(f"source {desc.source!r} is not a valid segment")
    if len(declared) != len(resource_names):
        dupes = sorted({n for n in resource_names if resource_names.count(n) > 1})
        errors.append(f"duplicate resource names: {dupes}")
    for res in desc.resources:
        if not is_valid_segment(res.name):
            errors.append(f"resource {res.name!r} is not a valid segment")
        for iface in tuple(res.state_interfaces) + tuple(res.command_interfaces):
            if not is_valid_segment(iface):
                errors.append(f"interface {iface!r} on {res.name!r} is not a valid segment")
            elif iface not in res.units:
                errors.append(f"interface {iface!r} on {res.name!r} has no declared unit")

    # 2. Limits address declared keys and describe a real range.
    for key, lim in desc.limits.items():
        if key not in state_keys and key not in command_keys:
            errors.append(f"limit key {key!r} is not a declared state or command key")
        for bound_name, bound in (("lo", lim.lo), ("hi", lim.hi)):
            if bound is not None and not _finite(bound):
                errors.append(f"limit {key!r} has non-finite {bound_name}")
        if lim.lo is not None and lim.hi is not None and _finite(lim.lo) and _finite(lim.hi):
            if lim.lo > lim.hi:
                errors.append(f"limit {key!r} has lo {lim.lo} above hi {lim.hi}")
        if lim.policy is LimitPolicy.CLAMP and (lim.lo is None or lim.hi is None):
            errors.append(f"limit {key!r} is CLAMP but is not bounded on both sides")

    # 3. Mode groups: unique, declared, subset of command interfaces, and every
    #    commandable (resource, interface) covered exactly once.
    group_names = [g.name for g in desc.mode_groups]
    if len(set(group_names)) != len(group_names):
        dupes = sorted({n for n in group_names if group_names.count(n) > 1})
        errors.append(f"duplicate mode group names: {dupes}")
    for group in desc.mode_groups:
        if not group.resources:
            errors.append(f"mode group {group.name!r} covers no resources")
        # Union, not intersection: a group may span resources with different
        # interfaces, which is what lets one group drive an arm's position and
        # a base's vx/vy/wz together. Commanding an interface a resource does
        # not declare is still caught, as an unknown key.
        commandable: set[str] = set()
        for name in group.resources:
            target = desc.resource(name)
            if target is None:
                errors.append(f"mode group {group.name!r} names undeclared resource {name!r}")
                continue
            commandable |= set(target.command_interfaces)
        extra = group.interfaces - commandable
        if extra:
            errors.append(
                f"mode group {group.name!r} has interfaces {sorted(extra)} "
                f"not commandable on any of its resources"
            )
    for res in desc.resources:
        for iface in res.command_interfaces:
            covering = [g.name for g in desc.groups_for(res.name) if iface in g.interfaces]
            if not covering:
                errors.append(f"{res.name!r}.{iface} is in no mode group, so it can never be sent")
            elif len(covering) > 1:
                errors.append(f"{res.name!r}.{iface} is in several mode groups: {sorted(covering)}")

    # 4. Omission addresses command keys.
    for key in desc.omission:
        if key not in command_keys:
            errors.append(f"omission key {key!r} is not a declared command key")

    # 5. Initial values seed retain-last non-position interfaces, in range.
    for key, value in desc.initial_values.items():
        if key not in command_keys:
            errors.append(f"initial value key {key!r} is not a declared command key")
            continue
        if key.rsplit("/", 1)[-1] == POSITION:
            errors.append(f"initial value {key!r} is a position; positions seed from measured")
            continue
        if desc.omission_of(key) is not Omission.RETAIN_LAST:
            errors.append(f"initial value {key!r} does not resolve to RETAIN_LAST omission")
        if not _finite(value):
            errors.append(f"initial value {key!r} is not finite")
            continue
        bounds = desc.limits.get(key)
        if bounds is not None:
            if bounds.lo is not None and value < bounds.lo:
                errors.append(f"initial value {key!r} ({value}) is below its limit {bounds.lo}")
            if bounds.hi is not None and value > bounds.hi:
                errors.append(f"initial value {key!r} ({value}) is above its limit {bounds.hi}")

    # 6. Coverage names declared resources, never itself, with no cycles.
    for owner, covered in desc.covered_resources.items():
        if owner not in declared:
            errors.append(f"covered_resources key {owner!r} is not a declared resource")
        for name in covered:
            if name not in declared:
                errors.append(f"covered_resources value {name!r} is not a declared resource")
            if name == owner:
                errors.append(f"resource {owner!r} covers itself")
    cycle = _first_cycle(desc.covered_resources)
    if cycle:
        errors.append(f"covered_resources has a cycle: {' -> '.join(cycle)}")

    # 7. available_after names declared resources.
    for name in desc.available_after:
        if name not in declared:
            errors.append(f"available_after key {name!r} is not a declared resource")

    # 8. Timing is positive and staleness outlives one state period.
    t = desc.timing
    for label, value in (
        ("state_rate_hz", t.state_rate_hz),
        ("stale_timeout_s", t.stale_timeout_s),
        ("watchdog_timeout_s", t.watchdog_timeout_s),
        ("hook_timeout_s", t.hook_timeout_s),
        ("prepare_arm_timeout_s", t.prepare_arm_timeout_s),
    ):
        if not _finite(value) or value <= 0:
            errors.append(f"timing.{label} must be positive, got {value}")
    if isinstance(t.write_rate_hz, float | int) and not isinstance(t.write_rate_hz, WriteMode):
        if not _finite(float(t.write_rate_hz)) or float(t.write_rate_hz) <= 0:
            errors.append(f"timing.write_rate_hz must be positive, got {t.write_rate_hz}")
    if _finite(t.state_rate_hz) and t.state_rate_hz > 0 and _finite(t.stale_timeout_s):
        if t.stale_timeout_s < 1.0 / t.state_rate_hz:
            errors.append(
                f"timing.stale_timeout_s {t.stale_timeout_s} is shorter than one state "
                f"period {1.0 / t.state_rate_hz}, so a healthy source reads as stale"
            )

    # 9. Stop policies carry what their kind needs.
    if desc.safe_stop.kind is SafeStopKind.DAMP:
        if not desc.safe_stop.kd:
            errors.append("safe_stop DAMP needs a kd table")
        else:
            for key in desc.safe_stop.kd:
                if key not in command_keys or key.rsplit("/", 1)[-1] != KD:
                    errors.append(f"safe_stop kd key {key!r} is not a declared '<joint>/kd' key")
    if desc.safe_stop.kind is SafeStopKind.ZERO_RAMP:
        if desc.safe_stop.ramp_s is None or desc.safe_stop.ramp_s <= 0:
            errors.append("safe_stop ZERO_RAMP needs a positive ramp_s")

    # 10. Per-group process loss names mode groups.
    if not isinstance(desc.process_loss, ProcessLoss):
        for name in desc.process_loss:
            if name not in set(group_names):
                errors.append(f"process_loss key {name!r} is not a mode group name")

    # 11. Epoch is non-negative.
    if desc.epoch < 0:
        errors.append(f"epoch must be non-negative, got {desc.epoch}")

    if errors:
        raise DescriptionError(errors)


def _first_cycle(graph: object) -> list[str]:
    """The first cycle in a resource-coverage graph, as a path, or ``[]``."""
    assert isinstance(graph, dict)
    colour: dict[str, int] = {}
    path: list[str] = []

    def walk(node: str) -> list[str]:
        colour[node] = 1
        path.append(node)
        for nxt in graph.get(node, ()):
            if colour.get(nxt, 0) == 1:
                return [*path[path.index(nxt) :], nxt]
            if colour.get(nxt, 0) == 0 and nxt in graph:
                found = walk(nxt)
                if found:
                    return found
        path.pop()
        colour[node] = 2
        return []

    for node in graph:
        if colour.get(node, 0) == 0:
            found = walk(node)
            if found:
                return found
    return []


def validate_state(desc: ControlDescription, frame: ControlValues) -> dict[str, float]:
    """Check a state frame against ``desc`` and return it as a mapping.

    A state frame is a full report, not a sparse one: every declared state key
    must be present exactly once and nothing else may be.

    Raises:
        FrameRejectedError: On a foreign source, or any missing, unknown or duplicate key.
    """
    if frame.source != desc.source:
        raise FrameRejectedError(
            "source", f"frame is from {frame.source!r}, expected {desc.source!r}"
        )

    names = frame.interface_names
    seen: dict[str, float] = {}
    duplicates: list[str] = []
    for name, value in zip(names, frame.values, strict=True):
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
    current_epoch: int,
    last_sequence: int | None,
) -> CommandBatch | Rejected:
    """Check a command frame and return the batch to apply, or why not.

    Only keys prefixed with this source are considered. A coordinator frame
    carries every source's commands on the shared stream, so another source's
    keys are ignored rather than rejected (D15).

    An empty result is valid and is the heartbeat: it still advances the
    sequence and proves the coordinator is alive.
    """
    if frame.epoch != current_epoch:
        return Rejected("epoch", f"frame epoch {frame.epoch}, current {current_epoch}")

    last = None if last_sequence is None else (current_epoch, last_sequence)
    if not is_newer(frame.epoch, frame.sequence, last):
        return Rejected(
            "sequence", f"frame sequence {frame.sequence} not newer than {last_sequence}"
        )

    prefix = f"{desc.source}/"
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
            continue
        low_bad = lim.lo is not None and value < lim.lo
        high_bad = lim.hi is not None and value > lim.hi
        if not (low_bad or high_bad):
            values[name] = value
            continue
        if lim.policy is LimitPolicy.REJECT:
            bound = lim.lo if low_bad else lim.hi
            side = "below" if low_bad else "above"
            return Rejected("limit", f"{name} = {value} is {side} its limit {bound}")
        assert lim.lo is not None and lim.hi is not None  # CLAMP is bounded both sides
        values[name] = lim.lo if low_bad else lim.hi
        clamped.append(name)

    group_result = _resolve_groups(desc, values)
    if isinstance(group_result, Rejected):
        return group_result

    return CommandBatch(values=values, active_groups=group_result, clamped=tuple(clamped))


def _resolve_groups(
    desc: ControlDescription, values: dict[str, float]
) -> frozenset[str] | Rejected:
    """Which mode groups a set of commanded keys puts into effect (D10)."""
    by_resource: dict[str, set[str]] = {}
    for key in values:
        _, resource, iface = key.split("/")
        by_resource.setdefault(resource, set()).add(iface)

    resolved: dict[str, str] = {}
    for resource, ifaces in by_resource.items():
        candidates = [g for g in desc.groups_for(resource) if ifaces <= g.interfaces]
        if not candidates:
            return Rejected(
                "mode_group",
                f"no single group on {resource!r} covers interfaces {sorted(ifaces)}",
            )
        if len(candidates) > 1:
            return Rejected(
                "mode_group",
                f"interfaces {sorted(ifaces)} on {resource!r} fit several groups: "
                f"{sorted(g.name for g in candidates)}",
            )
        resolved[resource] = candidates[0].name

    by_name = {g.name: g for g in desc.mode_groups}
    for group_name in set(resolved.values()):
        group = by_name[group_name]
        if not group.exclusive:
            continue
        for resource in group.resources:
            other = resolved.get(resource)
            if other is not None and other != group_name:
                return Rejected(
                    "mode_group",
                    f"exclusive group {group_name!r} and {other!r} both claim "
                    f"resources including {resource!r}",
                )

    return frozenset(resolved.values())

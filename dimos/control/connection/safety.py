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

"""Work out what to send to a robot to bring it safely to rest.

Pure functions: given how the robot is described and what it was last told,
they return the values to send. Nothing here talks to hardware, so each way of
stopping can be checked exactly.

The ways of stopping, set by the robot's description:

  HOLD       stay where it is: keep the last targets, stop all motion
  ZERO       stop all motion and let go of any pushing force
  ZERO_RAMP  like ZERO, but slow down gradually instead of at once
  DAMP       go slack but resist movement, so it sinks rather than drops
  VENDOR     the robot's own driver handles it; nothing is sent from here

None of these checks the values against the robot's limits before sending. A
stop must never be refused.
"""

from __future__ import annotations

from collections.abc import Mapping

from dimos.control.contract.description import (
    ControlDescription,
    EstopKind,
    Omission,
    SafeStopKind,
)
from dimos.control.contract.keys import EFFORT, KD, KP, POSITION, VELOCITY, VX, VY, VZ, WX, WY, WZ

#: Interfaces that say how fast something should move. Stopping sets them to 0.
RATE_INTERFACES: frozenset[str] = frozenset({VELOCITY, VX, VY, VZ, WX, WY, WZ})

#: Interfaces that say how hard something should push.
FORCE_INTERFACES: frozenset[str] = frozenset({EFFORT})


def _interface(key: str) -> str:
    return key.rsplit("/", 1)[-1]


def hold_values(last: Mapping[str, float]) -> dict[str, float]:
    """Keep every target from the last command, but with all speeds at 0.

    An arm that was being told where to go keeps being told the same place. An
    arm that was being told how fast to move is told to move at 0, so it stops.
    Pushing forces such as gravity compensation are kept, so it does not sag.

    Args:
        last: The last command the robot obeyed, by name.
    """
    return {
        key: 0.0 if _interface(key) in RATE_INTERFACES else value for key, value in last.items()
    }


def zero_values(last: Mapping[str, float]) -> dict[str, float]:
    """Like ``hold_values``, but pushing forces go to 0 as well.

    Args:
        last: The last command the robot obeyed, by name.
    """
    stopped = RATE_INTERFACES | FORCE_INTERFACES
    return {key: 0.0 if _interface(key) in stopped else value for key, value in last.items()}


def ramp_values(last: Mapping[str, float], fraction: float) -> dict[str, float]:
    """Scale every speed down, part way to 0. Pushing forces go straight to 0.

    Args:
        last: The last command the robot obeyed, by name.
        fraction: How much of each speed to keep: 1.0 keeps it all, 0.0 keeps
            none. Anything outside 0 to 1 is treated as the nearest end.
    """
    keep = min(1.0, max(0.0, fraction))
    out: dict[str, float] = {}
    for key, value in last.items():
        interface = _interface(key)
        if interface in RATE_INTERFACES:
            out[key] = value * keep
        elif interface in FORCE_INTERFACES:
            out[key] = 0.0
        else:
            out[key] = value
    return out


def damp_values(
    desc: ControlDescription,
    last: Mapping[str, float],
    measured: Mapping[str, float],
) -> dict[str, float]:
    """Let the listed joints go slack while still resisting movement.

    Every joint named in the description's damping table gets no stiffness,
    that table's damping, no pushing force and a target speed of 0, so it
    resists being moved without holding any position. Every other part of the
    robot is held, as in ``hold_values``.

    Works even if the robot has never been commanded: a damped joint's target
    position then comes from its latest reading, or 0.0 if there is none. With
    no stiffness the target position has no effect, so either is safe.

    Args:
        desc: The robot's description. Its ``safe_stop.kd`` is the damping
            table, by name of each joint's ``kd``.
        last: The last command the robot obeyed, by name. May be empty.
        measured: The robot's latest readings, by name. May be empty.
    """
    out = hold_values(last)
    for kd_key, gain in (desc.safe_stop.kd or {}).items():
        source, resource_name, _ = kd_key.split("/")
        resource = desc.resource(resource_name)
        if resource is None:
            continue
        for interface in resource.command_interfaces:
            key = f"{source}/{resource_name}/{interface}"
            if interface == KD:
                out[key] = gain
            elif interface == KP:
                out[key] = 0.0
            elif interface == POSITION:
                if key not in out:
                    out[key] = measured.get(f"{source}/{resource_name}/{POSITION}", 0.0)
            elif key not in last and desc.omission_of(key) is Omission.UNSET:
                # This robot reads "nothing sent" differently from 0, and it
                # was not being sent anything here, so leave it that way.
                continue
            else:
                out[key] = 0.0
    return out


def damp_groups(desc: ControlDescription) -> frozenset[str]:
    """The ways of driving the robot that a damping stop uses.

    Needed when damping a robot that was never commanded, so there is no last
    command to take them from.
    """
    damped = {key.split("/")[1] for key in (desc.safe_stop.kd or {})}
    return frozenset(
        group.name
        for group in desc.mode_groups
        if KD in group.interfaces and damped.intersection(group.resources)
    )


def estop_as_stop_kind(kind: EstopKind) -> SafeStopKind:
    """The way of stopping that carries out an emergency stop of this kind.

    DISABLE and VENDOR can only be carried out by the robot's own driver, so
    they map to VENDOR.
    """
    return {
        EstopKind.HOLD: SafeStopKind.HOLD,
        EstopKind.ZERO: SafeStopKind.ZERO,
        EstopKind.DAMP: SafeStopKind.DAMP,
    }.get(kind, SafeStopKind.VENDOR)


def stop_values(
    desc: ControlDescription,
    kind: SafeStopKind,
    last: Mapping[str, float] | None,
    measured: Mapping[str, float],
    *,
    elapsed_s: float,
) -> dict[str, float] | None:
    """The values to send right now to stop the robot in the given way.

    Args:
        desc: The robot's description. A gradual stop reads ``safe_stop.ramp_s``
            from it and a damping stop reads ``safe_stop.kd``.
        kind: How to stop.
        last: The last command the robot obeyed, or ``None`` if it has not
            been commanded since it was last armed.
        measured: The robot's latest readings.
        elapsed_s: Seconds since the stop began. Only a gradual stop uses it.

    Returns:
        The values to send, or ``None`` when there is nothing to send: the
        robot's own driver handles the stop, or the robot was never commanded
        so there is no motion to stop.
    """
    if kind is SafeStopKind.VENDOR:
        return None
    if kind is SafeStopKind.DAMP:
        return damp_values(desc, last or {}, measured)
    if last is None:
        return None
    if kind is SafeStopKind.HOLD:
        return hold_values(last)
    if kind is SafeStopKind.ZERO:
        return zero_values(last)
    # ZERO_RAMP. The validator guarantees a positive ramp_s.
    ramp_s = desc.safe_stop.ramp_s or 0.0
    fraction = 1.0 - elapsed_s / ramp_s if ramp_s > 0 else 0.0
    return ramp_values(last, fraction)

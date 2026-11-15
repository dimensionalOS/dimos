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

"""Checks a command against a robot's limits table before it is sent.

``validate_command`` runs on every command and returns a result instead of
raising: a refused command is an ordinary event to count, not an error to
unwind. A command is all or nothing. If any value is refused, none is sent,
so the robot is never left half-told.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

from dimos.control.contract.limits import Limits


@dataclass(frozen=True, slots=True)
class Rejected:
    """Why a command was refused: a short ``reason`` and the ``detail`` behind it."""

    reason: str
    detail: str


@dataclass(frozen=True, slots=True)
class CommandBatch:
    """An accepted command, ready to send to the robot.

    Attributes:
        values: The numbers to send, by key.
        clamped: Keys whose values were past a clamping limit and were pulled
            back to it.
    """

    values: dict[str, float]
    clamped: tuple[str, ...] = ()


def validate_command(
    values: Mapping[str, float], limits: Mapping[str, Limits]
) -> CommandBatch | Rejected:
    """Check a command against a robot's limits table, and return what to send, or why not.

    Every key must be in the table, and every value finite and within its
    limit; a value past a limit that clamps is pulled back to it. One
    refused value refuses the whole command. An empty command is accepted
    and sends nothing.

    Args:
        values: Named values in each key's unit, e.g.
            ``{"arm/joint1/position": 0.3}``.
        limits: The robot's table, from ``dimos.control.contract.limits``.
    """
    unknown = sorted(key for key in values if key not in limits)
    if unknown:
        return Rejected("unknown_key", f"not keys this robot accepts: {unknown}")

    sent: dict[str, float] = {}
    clamped: list[str] = []
    for key, value in values.items():
        if not math.isfinite(value):
            return Rejected("non-finite", f"{key} = {value}")
        limit = limits[key]
        if limit.lo is not None and value < limit.lo:
            if not limit.clamp:
                return Rejected("limit", f"{key} = {value} is below its limit {limit.lo}")
            sent[key] = limit.lo
            clamped.append(key)
        elif limit.hi is not None and value > limit.hi:
            if not limit.clamp:
                return Rejected("limit", f"{key} = {value} is above its limit {limit.hi}")
            sent[key] = limit.hi
            clamped.append(key)
        else:
            sent[key] = value
    return CommandBatch(values=sent, clamped=tuple(clamped))

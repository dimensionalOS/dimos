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

"""What a robot connection reports about itself.

These are the answers that come back from a connection's lifecycle calls and
its ``status()`` call. They are sent between processes, so everything here is
plain data that pickles.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class LifecycleState(Enum):
    """Where a connection is in the process of being allowed to move.

    STANDBY       connected and reporting, but ignoring commands
    PREPARING     running its bring-up (e.g. standing a dog up)
    PREPARED      ready to go live, waiting for the go-ahead
    ARMED         obeying commands
    SAFE_STOPPED  stopped by itself or on request; stays stopped until cleared
    ESTOPPED      stopped in an emergency; stays stopped until cleared
    """

    STANDBY = "standby"
    PREPARING = "preparing"
    PREPARED = "prepared"
    ARMED = "armed"
    SAFE_STOPPED = "safe_stopped"
    ESTOPPED = "estopped"


@dataclass(frozen=True, slots=True)
class LifecycleAck:
    """The answer to one lifecycle request, such as "prepare" or "stop".

    A request that cannot be done right now comes back with ``ok`` False and
    the state the connection is actually in. It never raises.

    Attributes:
        operation_id: The number the caller gave the request, so a retried
            request can be matched to its answer. Must be different for every
            new request: reusing one returns the earlier answer instead of
            acting again.
        source: Which piece of hardware this answer is about, e.g. "arm".
        state: The state after the request was handled.
        ok: Whether the request did what it asked.
        reason: Why not, when ``ok`` is False. Empty otherwise.
    """

    operation_id: int
    source: str
    state: LifecycleState
    ok: bool
    reason: str = ""


@dataclass(frozen=True, slots=True)
class ConnectionStatus:
    """A snapshot of one piece of hardware, for display and for deciding what
    to do next.

    Attributes:
        source: Which piece of hardware, e.g. "arm".
        state: Where it is in the lifecycle.
        epoch: The number it was armed under, or ``None`` when not armed.
            Commands carrying any other number are refused.
        description_epoch: Which version of its description is current. Goes
            up whenever the description changes.
        last_accepted_sequence: The number of the last command it obeyed in
            this arming, or ``None`` if none yet.
        confirmed_groups: The ways of driving the hardware is currently
            switched into, e.g. ``{"position"}``.
        fault: Why it stopped itself, or ``None``.
        state_fresh: Whether readings from the hardware are arriving on time.
        command_fresh: Whether commands are arriving on time.
        rejections: How many commands or readings were refused, by reason, as
            ``(reason, count)`` pairs.
        extras: Anything else the hardware's driver wants to show. Values
            must pickle.
    """

    source: str
    state: LifecycleState
    epoch: int | None
    description_epoch: int
    last_accepted_sequence: int | None
    confirmed_groups: frozenset[str]
    fault: str | None
    state_fresh: bool
    command_fresh: bool
    rejections: tuple[tuple[str, int], ...] = ()
    extras: dict[str, Any] = field(default_factory=dict)

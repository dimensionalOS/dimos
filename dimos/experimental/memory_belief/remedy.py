# Copyright 2026 Dimensional Inc.
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

"""Turning "I do not know" into something to do about it.

A failed query names the capability that would fix it. Mapping that name onto a
skill is a deployment fact, not a property of the query: a legged robot resolves
``observe_place`` by walking there, an arm by turning its wrist, a fixed camera
not at all. Keeping the resolution here is what lets one belief layer run on all
three.

A capability with no provider is reported as such rather than silently doing
nothing -- "nobody can fix this" and "I forgot to try" lead an agent to opposite
next moves.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from dimos.experimental.memory_belief.answer import UnknownReason

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping


@dataclass(frozen=True, slots=True)
class Remedy:
    """What to do about one unknown, and whether it is worth doing."""

    reason: UnknownReason
    capability: str | None
    provider: str | None
    #: Whether acting could plausibly change the answer. False for a terminal
    #: reason, and for one nothing on this robot provides.
    actionable: bool
    #: Whether fixing it needs the robot to go and look again. False for
    #: OUT_OF_VOCABULARY, which is answered by recomputing over stored frames.
    needs_revisit: bool
    note: str


class RemedyResolver:
    """Maps a needed capability onto whatever provides it here.

    Providers are registered by capability name, not by reason, so one skill can
    answer several reasons and a reason can be answered differently on different
    robots.
    """

    def __init__(self, providers: Mapping[str, Callable[..., Any]] | None = None) -> None:
        self._providers: dict[str, Callable[..., Any]] = dict(providers or {})

    @property
    def capabilities(self) -> frozenset[str]:
        return frozenset(self._providers)

    def plan_for(self, reason: UnknownReason | None) -> Remedy | None:
        """What to do about one reason.

        A reason, not an answer object. An earlier design passed the whole
        answer and read one field off it, which coupled the remedy loop to a
        return type the query layer stopped producing -- and left the loop
        unreachable from the one surface an agent actually calls.
        """
        if reason is None:
            return None
        capability = reason.suggested_capability
        if capability is None:
            return Remedy(
                reason=reason,
                capability=None,
                provider=None,
                actionable=False,
                needs_revisit=False,
                note="the data source cannot answer this however long it looks",
            )
        provider = self._providers.get(capability)
        if provider is None:
            return Remedy(
                reason=reason,
                capability=capability,
                provider=None,
                actionable=False,
                needs_revisit=reason.needs_revisit,
                note=f"nothing here provides {capability!r}",
            )
        return Remedy(
            reason=reason,
            capability=capability,
            provider=getattr(provider, "__name__", repr(provider)),
            actionable=True,
            needs_revisit=reason.needs_revisit,
            note=f"{capability} is provided by {getattr(provider, '__name__', provider)}",
        )

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

"""Why a belief query could not be answered.

The shape of an answer belongs to ``api.py``, which returns an envelope. What
lives here is only the vocabulary of refusal, because that vocabulary is shared
by three layers -- the query engine that emits it, the skill that hands it to
an agent, and the resolver that turns it into a next action -- and a reason
spelled differently in any one of them breaks the loop silently.

Every member names the capability that would resolve it, or ``None`` when
nothing the robot can do will. A capability, not a skill: which skill provides
``sweep_place`` is a deployment question, and hardcoding one here would bind
the query layer to a particular robot's toolset.
"""

from __future__ import annotations

from enum import Enum


class UnknownReason(Enum):
    """Why a query could not be answered, and what to do about it.

    The value is the capability that would resolve it, or None when nothing the
    robot can do will, in which case no amount of looking would help.
    """

    #: No sensor has ever swept this place and time.
    NEVER_COVERED = "sweep_place"
    #: Sightings that do not plausibly describe one object. Reporting their
    #: centroid would invent a location no sensor saw, so the honest answer is
    #: that the grouping itself is in doubt. Resolving it needs the
    #: re-identification this layer does not yet have, which is why the remedy
    #: is a look rather than a recompute.
    INCOHERENT = "observe_place"
    #: The detector was never asked about this kind of thing. Cheapest remedy in
    #: the list: the frames are already stored, so re-running detection with a
    #: wider vocabulary answers it without moving the robot at all.
    OUT_OF_VOCABULARY = "redetect_with_vocabulary"
    #: The data source cannot support this question however long it looks.
    NO_CAPABILITY = None

    @property
    def suggested_capability(self) -> str | None:
        value: str | None = self.value
        return value

    @property
    def is_terminal(self) -> bool:
        """Whether acting on the suggestion is pointless."""
        return self.value is None

    @property
    def needs_revisit(self) -> bool:
        """Whether the remedy requires going and looking again.

        ``OUT_OF_VOCABULARY`` is the one that does not: it is answered by
        recomputation over stored frames.
        """
        return self.value is not None and self is not UnknownReason.OUT_OF_VOCABULARY

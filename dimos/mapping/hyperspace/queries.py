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

"""One query, three shapes, and a handle an agent can come back to.

An agent asking "where is the fire extinguisher" wants the first answer now and the rest
only if the first one does not work out. A detector frame costs the better part of a
second and a query runs a dozen of them, so a call that blocks until every place is found
spends ten seconds answering a question whose first line was ready in two.

So every query starts, returns what it has, and leaves the rest behind a handle:

    start = start_item_query("a fire extinguisher")      # first place, right away
    rest  = query_results(start.query_id)                # the others, when wanted

The three shapes differ in what an answer IS, not in how the search works -- all of them
run the same `run_query` underneath:

* **item** -- OWLv2 draws a box. An answer is a thing with a size, and the detector can
  refuse, which is what makes a box trustworthy.
* **heatmap** -- no detector at all. The hot patches go into voxels and an answer is a
  place with a score. Nothing refuses, so this answers where a box cannot: a query the
  detector has no word for, or one that is not an object.
* **area** -- a room rather than a thing, contrasted against objects AND surfaces.
  MEASURED on "kitchen" over sf_office_drive1 (2026-09-14), top six frames judged by eye:
  contrasting against "a close-up of an object / a single object / a piece of equipment"
  gave 6 of 6 and favoured the wide shots that show a space; the object contrast also
  gave 6 of 6 but prefers close ones; "an item" and naming other rooms gave 5 of 6; and
  **no contrast at all gave 2 of 6** -- mostly luggage and a pile of boxes. Dropping the
  contrast for an area query is the one thing that clearly does not work.

  Judging by the top frames hid a bug for a day, because the frames were right either
  way. **Counted at the CELL level against the kitchen's real rectangle (2026-09-15),
  cells with 3+ viewpoints, in/out:** object-ness alone 22 in / **268 out**; the surface
  set alone 20 / 40; **both together 20 / 29**; "a wall" and "a floor" by themselves
  37 / 235; no contrast 29 / 242. Subtracting object-ness alone -- which is what this
  was -- left nothing subtracting walls, and the false positives were walls. Both
  together costs two in-box cells and removes nine tenths of the rest. Note also that
  the two bare surface words do far worse than the full surface set: it is not "a wall"
  doing the work.

  How a cell is SCORED still matters more than which set is used. Asked for "kitchen"
  and measured against the same rectangle, four different contrasts all answered
  7.5-9.7 m outside it while cells were scored by summing the patches that landed in
  them, and all four answered inside it once they were scored by the mean over 25 cm
  cells seen three times. See `HyperspaceConfig.heat_cell_m`.

A caller that knows better than any of these can say so: every query skill takes
`negative_terms`, and what it names replaces the default rather than adding to it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import itertools
import threading
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Sequence

# What an area is contrasted against: objects AND surfaces, both. The object-ness terms
# are what favour a wide shot of a space over a close-up; the surface terms are what stop
# a wall from answering. Subtracting only the first three was a real bug -- see the
# module docstring for the counts.
AREA_PROMPTS = (
    "a close-up of an object",
    "a single object",
    "a piece of equipment",
    "a floor",
    "a wall",
    "a ceiling",
    "a shelf",
    "a photo of a room",
)


@dataclass
class Place:
    """One answer, in the shape the three kinds share.

    A box has an extent and a detector score; a heatmap voxel has neither, and pretending
    otherwise would have an agent trust a number that was never measured. So `extent` and
    `confidence` are optional, and `kind` says which sort of answer this is.
    """

    where: tuple[float, float, float]
    frame: str
    kind: str
    score: float
    # Metres from the camera that saw it, when something saw it from somewhere.
    distance_m: float = 0.0
    # None for a heatmap or an area: nothing measured a size.
    extent: tuple[float, float, float] | None = None
    # Distinct viewpoints behind this answer. One is not a lie, it is one look.
    views: int = 1
    # Seconds into the recording, or wall clock live.
    seen_at: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        found = {
            "where": [round(value, 3) for value in self.where],
            "frame": self.frame,
            "kind": self.kind,
            "score": round(self.score, 4),
            "views": self.views,
            "seen_at": round(self.seen_at, 3),
        }
        if self.distance_m:
            found["distance_m"] = round(self.distance_m, 2)
        if self.extent is not None:
            found["extent"] = [round(value, 3) for value in self.extent]
        return found


@dataclass
class Query:
    """A question that has been asked, and everything found for it so far."""

    query_id: str
    text: str
    kind: str
    places: list[Place] = field(default_factory=list)
    # How many of `places` a caller has already been handed.
    taken: int = 0
    # Episodes the detector refused. Only an item query can refuse.
    refused: int = 0
    ms: float = 0.0
    timings: dict[str, float] = field(default_factory=dict)
    asked_at: float = field(default_factory=time.time)
    # Set when the search found the thing but no answer could be made of it -- which is
    # a different outcome from finding nothing, and the one an agent most needs told.
    note: str = ""
    # What this query subtracted, so an answer can be read beside the words that shaped
    # it. Empty means the kind's own default was used.
    negatives: tuple[str, ...] = ()

    @property
    def remaining(self) -> int:
        return max(0, len(self.places) - self.taken)


class QueryBook:
    """The queries this module has been asked, keyed by id.

    Bounded, because a robot that has been asked ten thousand questions should not be
    holding ten thousand answers: the oldest go first. A caller that comes back for the
    rest of a query that has aged out is told so rather than handed an empty list, since
    "there were no more" and "I forgot" are different facts.
    """

    def __init__(self, keep: int = 32) -> None:
        self.keep = keep
        self._queries: dict[str, Query] = {}
        self._order: list[str] = []
        self._numbers = itertools.count(1)
        self._lock = threading.Lock()

    def next_id(self, kind: str) -> str:
        return f"{kind}-{next(self._numbers)}"

    def put(self, query: Query) -> Query:
        with self._lock:
            if query.query_id in self._queries:
                self._order.remove(query.query_id)
            self._queries[query.query_id] = query
            self._order.append(query.query_id)
            while len(self._order) > self.keep:
                del self._queries[self._order.pop(0)]
        return query

    def get(self, query_id: str) -> Query | None:
        with self._lock:
            return self._queries.get(query_id)

    def ids(self) -> list[str]:
        with self._lock:
            return list(self._order)


def negative_prompts(negative_terms: str | Sequence[str], kind: str) -> tuple[str, ...] | None:
    """What a query subtracts, from what the caller asked for and what kind it is.

    Every patch score is the query's own score minus the best of a handful of other
    prompts, which is what stops a wall from answering every question moderately well.
    Which prompts those are decides what the search is blind to, and the right set is not
    the same for a thing and for a room -- so a caller that knows what is in the way can
    say so, and gets exactly what it named rather than that plus the defaults.

    Nothing named falls back to the kind's own default: `AREA_PROMPTS` for an area, and
    None for the other two, which the search reads as "your generic surfaces".

    Comma separated, because these arguments are written by a language model and one
    string is a shape it gets right where a list of strings is one it does not.
    """
    if isinstance(negative_terms, str):
        wanted = negative_terms.split(",")
    else:
        wanted = [str(term) for term in negative_terms]
    named = tuple(term.strip() for term in wanted if term.strip())
    if named:
        return named
    return AREA_PROMPTS if kind == "area" else None


def near_enough(
    places: Sequence[Place], origin: Sequence[float] | None, within_m: float
) -> list[Place]:
    """Places within *within_m* of *origin*. No radius, or no origin, keeps everything.

    The radius is the point of a proximity query: "where is the nearest X" is a different
    question from "where is any X", and on a long recording the second one answers with
    something a kilometre away.
    """
    if not within_m or origin is None:
        return list(places)
    import numpy as np

    here = np.asarray(origin, dtype=float)
    return [
        place
        for place in places
        if float(np.linalg.norm(np.asarray(place.where, dtype=float) - here)) <= within_m
    ]

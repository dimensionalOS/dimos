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

"""Saying that two sightings are the same thing, and being able to take it back.

``target_ref`` is fixed at write time, when nothing yet knows two detections are
one chair. So identity is a claim, not a column: an ``entity_id`` field on the
observation would mean every correction rewrites history. Claims live in an
append-only stream and the mapping is the fold of it; retracting is appending.

A claim naming ``retracts`` supersedes an earlier one, which is what makes a bad
merge reversible: the wrong claim stays in the stream, and the fold stops
honouring it. A thing that merely moved needs no retraction -- both records were
true when written.

Tracker ids are not entity ids: a tracker reuses numbers after a reset, so claims
namespace them by session. A second retraction path, a `belief_retraction` stream
naming individual wrong sightings, was written and removed: nothing ever wrote to
it, and one live mechanism is easier to reason about than one live and one that
merely looks live.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from pydantic import ConfigDict
from pydantic.dataclasses import dataclass

from dimos.experimental.memory_belief.types import SCHEMA_VERSION

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

IDENTITY_STREAM_NAME = "belief_identity"


@dataclass(frozen=True)
class IdentityClaim:
    """A claim that some sightings are one entity."""

    __pydantic_config__ = ConfigDict(extra="forbid")

    schema_version: str
    claim_id: str
    entity_id: str
    target_refs: tuple[str, ...]
    basis: str
    """How the claim was reached, e.g. ``tracker:botsort``, ``reid:clip``,
    ``user``. Kept because a downstream reader weighs a tracker's word and a
    person's confirmation very differently."""
    confidence: float
    valid_ts: float
    retracts: str | None = None
    """``claim_id`` this supersedes. The retracted claim stays in the stream --
    it is what makes the correction auditable."""


def append_identity(stream: Any, claim: IdentityClaim) -> Any:
    """Append one identity claim, indexed by valid time."""
    return stream.append(claim, ts=claim.valid_ts)


def resolve_identity(claims: Iterable[Any]) -> dict[str, str]:
    """Fold the identity stream into ``target_ref -> entity_id``.

    Consumes an iterator and holds only the resulting mapping, so it runs
    against a live stream as well as a stored one. Later claims win, and a claim
    naming ``retracts`` removes the entity its predecessor asserted, which is
    what makes a bad merge reversible rather than permanent.
    """
    by_claim: dict[str, IdentityClaim] = {}
    order: list[str] = []
    for item in claims:
        claim = getattr(item, "data", item)
        if claim.retracts is not None:
            by_claim.pop(claim.retracts, None)
        by_claim[claim.claim_id] = claim
        order.append(claim.claim_id)

    mapping: dict[str, str] = {}
    for claim_id in order:
        claim = by_claim.get(claim_id)
        if claim is None:
            continue
        for ref in claim.target_refs:
            mapping[ref] = claim.entity_id
    return mapping


def claims_from_tracks(
    sightings: Iterable[tuple[str, Any, float]],
    *,
    session: str,
    basis: str = "tracker:botsort",
    confidence: float = 0.6,
    batch_size: int = 32,
) -> Iterator[IdentityClaim]:
    """Turn ``(target_ref, track_id, ts)`` triples into identity claims.

    Yields claims incrementally rather than accumulating every sighting, so a
    live run does not grow a dictionary for the length of its uptime. Each claim
    covers up to ``batch_size`` sightings of one track; a reader folding the
    stream sees them merge into one entity regardless of how they were batched.

    ``track_id`` of ``None`` yields nothing: an untracked detection is exactly
    the case this cannot speak to, and inventing a singleton entity for it would
    make "seen once" indistinguishable from "followed and lost".
    """
    pending: dict[str, list[tuple[str, float]]] = {}
    counter = 0

    def _emit(track_key: str, rows: list[tuple[str, float]]) -> IdentityClaim:
        nonlocal counter
        counter += 1
        return IdentityClaim(
            schema_version=SCHEMA_VERSION,
            claim_id=f"{session}:{track_key}:{counter}",
            entity_id=f"{session}:{track_key}",
            target_refs=tuple(ref for ref, _ in rows),
            basis=basis,
            confidence=confidence,
            valid_ts=rows[-1][1],
        )

    for target_ref, track_id, ts in sightings:
        if track_id is None:
            continue
        key = str(track_id)
        rows = pending.setdefault(key, [])
        rows.append((target_ref, ts))
        if len(rows) >= batch_size:
            yield _emit(key, rows)
            pending[key] = []
    for key, rows in pending.items():
        if rows:
            yield _emit(key, rows)

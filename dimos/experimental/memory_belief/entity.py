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

"""Entities as a stream, so the store's existing filters can query them.

Folding sightings and identity claims in Python produces a dictionary, which
cannot be filtered -- every "as of" would re-fold the whole history. As a stream,
``as_of`` is a timestamp filter, and snapshots accumulate rather than overwrite.

**The envelope pose is the entity's position, not the sensor's** -- the opposite
convention from :func:`~dimos.experimental.memory_belief.write.append_belief`. Backwards, it
would make ``near(x, y, r)`` quietly answer a different question.
"""

from __future__ import annotations

from collections import Counter
import math
import statistics
from typing import TYPE_CHECKING, Any

from pydantic import ConfigDict
from pydantic.dataclasses import dataclass

from dimos.experimental.memory_belief.identity import resolve_identity
from dimos.experimental.memory_belief.types import SCHEMA_VERSION

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

ENTITY_STREAM_NAME = "belief_entity"

#: Below this, a set of sightings is not describing one object well enough to
#: place it. Derived, not tuned: it is the ratio of sightings to the entities
#: they resolve into, discounted by how far apart they sit.
COHERENT = 0.5


@dataclass(frozen=True)
class Entity:
    """What is believed about one thing, as of one moment."""

    __pydantic_config__ = ConfigDict(extra="forbid")

    schema_version: str
    entity_id: str
    label: str | None
    """The label carried by most of this entity's sightings. Entities whose
    label is unstable are exactly the ones a reader should distrust, so the
    agreement ratio travels alongside it."""
    label_agreement: float
    support: int
    first_seen_ts: float
    last_seen_ts: float
    position: tuple[float, float, float] | None = None
    extent: tuple[float, float, float] | None = None
    dispersion_m: float = 0.0
    """Median distance from the sightings to their own centroid. A position with
    large dispersion is not a location, it is an average of several things."""
    coherence: float = 1.0
    """Whether these sightings plausibly describe one object. Low values are the
    honest reason to refuse a position rather than report the centroid."""
    displacement_m: float = 0.0
    motion: str = "unknown"
    """``static`` / ``moving`` / ``unknown`` -- behaviour, not category. A chair
    that never moves and a chair that does are different to a planner in a way
    that "chair" is not, and only the sightings can say which this one is."""
    place_ref: str | None = None
    identity_basis: str | None = None

    def to_rerun(self):  # type: ignore[no-untyped-def]
        """A box where the thing is, labelled with how much to trust it.

        Support shows as opacity and motion as colour, so a scatter of
        one-sighting boxes does not read as a crowd. Each entity gets its own
        path: logged to one shared path, every snapshot would overwrite the
        last and a viewer would show one box rather than the room.
        """
        import rerun as rr

        key = rr.escape_entity_path_part(self.entity_id)
        if self.position is None:
            return [(key, rr.Clear(recursive=False))]
        half = tuple(max(v, 0.05) / 2 for v in (self.extent or (0.2, 0.2, 0.2)))
        colour = {
            "static": (90, 170, 255),
            "moving": (255, 150, 60),
        }.get(self.motion, (150, 150, 150))
        # Saturating at ten sightings: past that the entity is well supported and
        # the difference between 10 and 300 is not what anyone is looking for.
        alpha = 55 + int(200 * min(self.support, 10) / 10)
        return [
            (
                key,
                rr.Boxes3D(
                    centers=[list(self.position)],
                    half_sizes=[list(half)],
                    colors=[(*colour, alpha)],
                    # The label as written. A glyph table stood here, mapping each term
                    # to an emoji; it cost 1,979 lines of lookup data inside dimos/ to
                    # say less than the word does, and an unmapped term read as a
                    # placeholder rather than as itself.
                    labels=[self.label or "?"],
                ),
            )
        ]


def append_entity(stream: Any, entity: Entity) -> Any:
    """Append one snapshot, indexed by valid time and the entity's own position.

    ``pose`` is the entity's position on purpose -- see the module docstring.
    ``tags`` carry the fields worth filtering on so that ``TagsFilter`` covers
    label, place and motion without any new filter type.
    """
    return stream.append(
        entity,
        # One row per entity, parked here, so an `as_of` between first and last
        # sighting matches nothing and the query answers NEVER_COVERED -- 19.8 s
        # of silence per entity on the reference recording, 238 s on its longest
        # track. Parking it at `first_seen_ts` would be worse, not better: the
        # aggregate would then answer from sightings that had not happened yet.
        # The fix is to fold at query time bounded by `as_of`, not to move this.
        ts=entity.last_seen_ts,
        pose=entity.position,
        tags={
            "entity_id": entity.entity_id,
            "label": entity.label,
            "place_ref": entity.place_ref,
            "motion": entity.motion,
            "support": entity.support,
        },
    )


class _Accumulator:
    """Running state for one entity. Bounded by entity count, not by sightings."""

    __slots__ = ("basis", "first", "labels", "last", "n", "places", "positions")

    def __init__(self) -> None:
        self.n = 0
        self.first = math.inf
        self.last = -math.inf
        self.labels: Counter[str] = Counter()
        self.places: Counter[str] = Counter()
        self.positions: list[tuple[float, float, float]] = []
        self.basis: str | None = None

    def add(self, record: Any) -> None:
        self.n += 1
        self.first = min(self.first, record.valid_ts)
        self.last = max(self.last, record.valid_ts)
        if record.label:
            self.labels[record.label] += 1
        if record.place_ref:
            self.places[record.place_ref] += 1
        if record.target_pose is not None:
            self.positions.append(tuple(record.target_pose))
        self.basis = self.basis or record.identity_basis


def _summarise(entity_id: str, acc: _Accumulator, *, static_threshold_m: float) -> Entity:
    label, label_n = acc.labels.most_common(1)[0] if acc.labels else (None, 0)
    place = acc.places.most_common(1)[0][0] if acc.places else None

    position = extent = None
    dispersion = displacement = 0.0
    coherence = 1.0
    if acc.positions:
        xs = [p[0] for p in acc.positions]
        ys = [p[1] for p in acc.positions]
        zs = [p[2] for p in acc.positions]
        centre = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))
        dispersion = statistics.median(math.dist(centre, p) for p in acc.positions)
        displacement = max((math.dist(acc.positions[0], p) for p in acc.positions), default=0.0)
        position = centre
        extent = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        # Sightings spread thinly over a wide area describe several things, or
        # nothing. One metre of spread halves the score.
        coherence = 1.0 / (1.0 + dispersion)

    motion = "unknown"
    if len(acc.positions) >= 3:
        motion = "moving" if displacement >= static_threshold_m else "static"

    return Entity(
        schema_version=SCHEMA_VERSION,
        entity_id=entity_id,
        label=label,
        label_agreement=(label_n / acc.n) if acc.n else 0.0,
        support=acc.n,
        first_seen_ts=acc.first,
        last_seen_ts=acc.last,
        position=position,
        extent=extent,
        dispersion_m=dispersion,
        coherence=coherence,
        displacement_m=displacement,
        motion=motion,
        place_ref=place,
        identity_basis=acc.basis,
    )


def fold_entities(
    observations: Iterable[Any],
    identities: Iterable[Any],
    *,
    static_threshold_m: float = 0.5,
) -> Iterator[Entity]:
    """Fold sightings and identity claims into entity snapshots.

    Consumes ``observations`` as an iterator and never materialises it, so this
    runs against a live stream. One snapshot per entity, emitted at the end.

    A sighting no identity claim covers is skipped rather than made its own
    entity -- "seen once" and "followed and lost" are different, and a singleton
    per unassociated detection would bury that under one-sighting entities.
    """
    mapping = resolve_identity(identities)
    accs: dict[str, _Accumulator] = {}

    for item in observations:
        record = getattr(item, "data", item)
        entity_id = mapping.get(record.target_ref)
        if entity_id is None:
            continue
        acc = accs.setdefault(entity_id, _Accumulator())
        acc.add(record)

    for entity_id, acc in accs.items():
        yield _summarise(entity_id, acc, static_threshold_m=static_threshold_m)

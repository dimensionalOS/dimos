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

"""The one place that says how sightings become the views questions are asked of.

Three folds over one input: identity claims tie sightings together, entities are
sightings grouped by identity, and annotations are sightings regrouped by the
frame they were drawn on.

**Why this is a module and not a script.** The same folds have to run twice: once
offline over a finished recording, once on a robot that is still driving. If each
caller carried its own copy of the order and the parameters, the two would drift
-- and drift here is invisible. Nothing raises when one store grouped entities at
a different threshold than the store it is compared against; the queries simply
stop agreeing, and the benchmark stops reproducing, with no error to follow back.
:class:`ViewParams` exists so that divergence has to be deliberate.

Today only the offline path exists: ``detect_recording -> build_views``. The
functions here take stores and iterators rather than paths and file handles so
that a live producer, when there is one, calls the same code with the same
:class:`ViewParams` instead of a second copy that drifts.

A fourth fold, places, lived here until regions were removed. A place is
entities binned by region and time, so it has no meaning without a region map --
and a geometric partition, which can split space but cannot name a room, left
every question that named one exactly as unanswerable as before.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from dimos.experimental.memory_belief.annotate import (
    annotation_stream,
    append_annotation,
    fold_annotations,
)
from dimos.experimental.memory_belief.entity import (
    ENTITY_STREAM_NAME,
    Entity,
    append_entity,
    fold_entities,
)
from dimos.experimental.memory_belief.identity import (
    IDENTITY_STREAM_NAME,
    IdentityClaim,
    append_identity,
    claims_from_tracks,
)
from dimos.experimental.memory_belief.write import belief_stream, derived_stream

if TYPE_CHECKING:
    from collections.abc import Callable


@dataclass(frozen=True, slots=True)
class ViewParams:
    """Everything that changes what the derived views mean.

    Frozen and passed by value rather than read from module constants, because
    the failure this guards against is two callers disagreeing silently. A
    parameter belongs here if changing it makes two stores incomparable; a
    parameter that only changes how long the build takes does not.
    """

    #: How far an entity may move and still count as the same static thing.
    #: One field, because one fold survives that changes meaning. The grid and
    #: region parameters that stood here went with the folds that read them.
    static_threshold_m: float = 0.5


def build_views(
    *,
    belief: Any,
    out: Any,
    session: str,
    params: ViewParams | None = None,
    report: Callable[[str], None] | None = None,
) -> dict[str, int]:
    """Write identity, entity and annotation streams into ``out``.

    ``belief`` and ``out`` are separate arguments because they are two different
    roles, not because they are two different files: a live robot passes the same
    store for both, and an offline build passes two. Nothing here assumes they
    differ.

    Progress goes to ``report`` rather than to ``print`` so a module deployed in
    a worker does not write to a stdout nobody is reading. Returns the counts so
    a caller that wants to assert on them does not have to parse text.
    """
    params = params or ViewParams()
    say = report or (lambda _message: None)
    counts: dict[str, int] = {}

    # Tracker claims are derived from what every sighting already stores in
    # `identity_basis`, rather than being a flag on the detector. That keeps
    # re-identification a rebuild of the views -- minutes -- instead of a second
    # pass over every frame with a segmentation model.
    identity_out = derived_stream(out, IDENTITY_STREAM_NAME, IdentityClaim)
    sightings = [
        (record.data.target_ref, record.data.identity_basis.split(":")[-1], record.data.valid_ts)
        for record in belief_stream(belief)
        if record.data.identity_basis
    ]
    claims = 0
    for claim in claims_from_tracks(sightings, session=session):
        append_identity(identity_out, claim)
        claims += 1
    counts["identity"] = claims
    say(f"identity: {claims} claims over {len({s[1] for s in sightings})} tracks")

    entity_out = derived_stream(out, ENTITY_STREAM_NAME, Entity)
    entities = 0
    for entity in fold_entities(
        belief_stream(belief),
        derived_stream(out, IDENTITY_STREAM_NAME, IdentityClaim),
        static_threshold_m=params.static_threshold_m,
    ):
        append_entity(entity_out, entity)
        entities += 1
    counts["entities"] = entities
    say(f"entities: {entities}")

    annotation_out = annotation_stream(out)
    annotations = boxes = 0
    for annotation in fold_annotations(belief_stream(belief)):
        append_annotation(annotation_out, annotation)
        annotations += 1
        boxes += len(annotation.boxes)
    counts["annotations"] = annotations
    counts["boxes"] = boxes
    say(f"annotations: {annotations} frames, {boxes} boxes")

    return counts

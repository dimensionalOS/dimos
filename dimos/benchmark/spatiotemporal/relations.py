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

"""Image-plane relation derivation."""

from dataclasses import dataclass
from math import isfinite
from typing import Literal

from dimos.benchmark.spatiotemporal.models import ObjectObservation, SpatialPredicate


@dataclass(frozen=True)
class SpatialRelationCandidate:
    """A temporary accepted relation backed by two observations."""

    subject: ObjectObservation
    predicate: SpatialPredicate
    object: ObjectObservation
    margin: float


def _derive_ordered(
    subject: ObjectObservation,
    object_: ObjectObservation,
    margin: float,
    predicate: SpatialPredicate,
    axis: Literal["x", "y"],
    *,
    reverse: bool = False,
) -> SpatialRelationCandidate | None:
    if not isfinite(margin) or not 0.0 <= margin <= 1.0:
        raise ValueError("margin must be finite and within [0, 1]")
    subject_sample = (subject.episode_id, subject.frame_id, subject.timestamp_s)
    object_sample = (object_.episode_id, object_.frame_id, object_.timestamp_s)
    if subject_sample != object_sample:
        raise ValueError("relations require observations from the same sample")
    if subject.object_id == object_.object_id:
        return None
    before, after = (object_, subject) if reverse else (subject, object_)
    before_max = getattr(before.box, f"{axis}_max")
    after_min = getattr(after.box, f"{axis}_min")
    separation = after_min - before_max
    if separation > margin:
        return SpatialRelationCandidate(
            subject=subject,
            predicate=predicate,
            object=object_,
            margin=margin,
        )
    return None


def derive_left_of(
    subject: ObjectObservation,
    object_: ObjectObservation,
    margin: float,
) -> SpatialRelationCandidate | None:
    """Derive an accepted left-of candidate when evidence is sufficient."""
    return _derive_ordered(subject, object_, margin, SpatialPredicate.LEFT_OF, "x")


def derive_right_of(
    subject: ObjectObservation,
    object_: ObjectObservation,
    margin: float,
) -> SpatialRelationCandidate | None:
    """Derive right-of by reusing left-of with swapped arguments."""
    return _derive_ordered(
        subject,
        object_,
        margin,
        SpatialPredicate.RIGHT_OF,
        "x",
        reverse=True,
    )


def derive_above(
    subject: ObjectObservation,
    object_: ObjectObservation,
    margin: float,
) -> SpatialRelationCandidate | None:
    """Derive an accepted above candidate when evidence is sufficient."""
    return _derive_ordered(subject, object_, margin, SpatialPredicate.ABOVE, "y")


def derive_below(
    subject: ObjectObservation,
    object_: ObjectObservation,
    margin: float,
) -> SpatialRelationCandidate | None:
    """Derive below by reusing above with swapped arguments."""
    return _derive_ordered(
        subject,
        object_,
        margin,
        SpatialPredicate.BELOW,
        "y",
        reverse=True,
    )

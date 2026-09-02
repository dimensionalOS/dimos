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

"""Turning a 2D detector's output into belief.

What a flat image cannot support is encoded in the records, not left to whoever
reads them later: no depth means ``target_pose`` and ``place_ref`` stay None and
only ``capture_place_ref`` is filled; no association means every record is its
own target; and a detector that does not fire is not evidence of absence, so no
``absent`` record is written here -- that comes from
:mod:`~dimos.experimental.memory_belief.grid`.

The vocabulary travels with the record, so a later "is there a mug here" answers
OUT_OF_VOCABULARY instead of a confident and wrong "no".
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.experimental.memory_belief.types import (
    SCHEMA_VERSION,
    BeliefObservation,
    Confidence,
    EvidenceRef,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Iterator, Sequence


def bright_enough(observation: Any, *, min_mean: float = 40.0) -> bool:
    """Whether a frame carries enough signal for detection to mean anything.

    Running a detector on a near-black frame does not produce absence, it
    produces silence -- and silence that gets recorded as "nothing detected" is
    indistinguishable from a real negative later. Filtering first keeps that
    ambiguity out of the store.
    """
    import cv2

    array = observation.data.to_opencv()
    grey = cv2.cvtColor(array, cv2.COLOR_RGB2GRAY) if array.ndim == 3 else array
    return bool(float(np.mean(grey)) >= min_mean)


def _cell_place(pose: Any, *, size_m: float) -> str | None:
    """A coarse place identifier from a pose, with no room list anywhere.

    Deliberately derived from coordinates rather than looked up in a
    configuration of named rooms: naming a place is a claim that needs its own
    evidence, and this producer has none.
    """
    if pose is None:
        return None
    return (
        f"cell({int(np.floor(pose.position.x / size_m))},{int(np.floor(pose.position.y / size_m))})"
    )


def _cell_place_xy(position: tuple[float, float, float], *, size_m: float) -> str:
    """Place identifier for a point in the world, using the same lattice as
    :func:`_cell_place` so entity places and capture places are comparable."""
    return f"cell({int(np.floor(position[0] / size_m))},{int(np.floor(position[1] / size_m))})"


def _bbox_of(detection: Any) -> tuple[float, float, float, float] | None:
    """A detection's box as four floats, or None when it carries no box.

    Spelled out rather than built with a generator so the four-element shape is
    a checked fact: `BeliefObservation.bbox` is consumed as x1/y1/x2/y2, and a
    detector handing over three or five numbers should fail here rather than
    silently produce a record nothing can crop.
    """
    box = getattr(detection, "bbox", None)
    if box is None:
        return None
    x1, y1, x2, y2 = (float(v) for v in box)
    return (x1, y1, x2, y2)


def detect_to_belief(
    observations: Iterable[Any],
    detector: Any,
    *,
    stream_name: str,
    vocabulary: tuple[str, ...] | None,
    source: str,
    min_confidence: float = 0.0,
    place_size_m: float = 5.0,
    frame_filter: Callable[[Any], bool] | None = None,
    locate: Callable[[Any, Any], Sequence[Any]] | None = None,
) -> Iterator[BeliefObservation]:
    """Run ``detector`` over observations, yielding one record per detection.

    ``detector`` needs only ``process_image``, so any 2D detector fits.

    ``vocabulary`` is recorded verbatim on every record -- it is what lets a
    later "is there a mug here" answer OUT_OF_VOCABULARY rather than "no" --
    and ``source`` stays separate from it so the two limits query separately.

    Detections below ``min_confidence`` are dropped rather than recorded with a
    low score, since a record that exists at all is evidence downstream.
    """
    keep_frame = frame_filter or (lambda _obs: True)

    for observation in observations:
        if not keep_frame(observation):
            continue
        result = detector.process_image(observation.data)
        evidence = (
            EvidenceRef(stream=stream_name, observation_id=observation.id, ts=observation.ts),
        )
        capture_place = _cell_place(getattr(observation, "pose", None), size_m=place_size_m)
        detections = tuple(getattr(result, "detections", ()))
        # A locator that cannot place a detection returns None for it, and that
        # None is carried through rather than smoothed over: a position inferred
        # from nothing reads downstream as evidence, which is worse than having
        # no position at all.
        placements = list(locate(observation, detections)) if locate else [None] * len(detections)
        for index, detection in enumerate(detections):
            if detection.confidence < min_confidence:
                continue
            placed = placements[index] if index < len(placements) else None
            # A tracker numbers what it is following now; that is a tentative
            # claim about identity, not a confirmed one, and it is recorded as
            # such so a later re-identification pass can overrule it.
            track_id = getattr(detection, "track_id", None)
            yield BeliefObservation(
                schema_version=SCHEMA_VERSION,
                # Unassociated: each sighting is its own target until something
                # matches it across frames. `identity.py` is where that match
                # gets asserted, revisably, without rewriting this reference.
                target_ref=f"{stream_name}#{observation.id}:{index}",
                label=detection.name,
                visibility="present",
                valid_ts=observation.ts,
                observed_ts=observation.ts,
                source=source,
                capture_place_ref=capture_place,
                # `place_ref` comes from where the *entity* is, never from where
                # the sensor was. It stays None without a placement, which is
                # what keeps "seen from the kitchen" out of "in the kitchen".
                # A lattice square. Named regions were derived from the map
                # geometrically and removed: geometry can split space but cannot
                # name a room, which is what the questions needed.
                place_ref=(
                    _cell_place_xy(placed.position, size_m=place_size_m) if placed else None
                ),
                target_pose=placed.position if placed else None,
                frame_id="world" if placed else None,
                identity_status="tentative" if track_id is not None else "none",
                identity_basis=(f"tracker:{track_id}" if track_id is not None else None),
                vocabulary=vocabulary,
                evidence=evidence,
                bbox=_bbox_of(detection),
                confidence=Confidence(detection=float(detection.confidence)),
            )

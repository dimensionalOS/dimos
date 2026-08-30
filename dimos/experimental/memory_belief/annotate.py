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

"""The boxes one frame's sightings were drawn from, so the picture can show them.

:class:`~dimos.experimental.memory_belief.types.BeliefObservation` carries its evidence one
detection at a time, and a viewer given one box at a time draws one box -- a
frame with six detections shows the sixth. Regrouping by frame is the whole of
this module.

**Placed and unplaced are drawn differently on purpose.** A detection with no
lidar returns in its box is still a real detection; colouring it like the ones
that survived would hide the largest source of loss between what the camera saw
and what the store believes.
"""

from __future__ import annotations

from itertools import groupby
from typing import TYPE_CHECKING, Any

from pydantic import ConfigDict
from pydantic.dataclasses import dataclass

from dimos.experimental.memory_belief.types import SCHEMA_VERSION

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

ANNOTATION_STREAM_NAME = "belief_frame_annotation"


@dataclass(frozen=True)
class FrameAnnotation:
    """Every detection made on one frame, in that frame's pixel coordinates."""

    __pydantic_config__ = ConfigDict(extra="forbid")

    schema_version: str
    frame_stream: str
    frame_observation_id: int
    ts: float
    boxes: tuple[tuple[float, float, float, float], ...]
    labels: tuple[str, ...]
    confidences: tuple[float, ...]
    placed: tuple[bool, ...]
    """Whether each detection got a 3D position. ``False`` means the detector saw
    it and the belief layer dropped it."""

    def to_rerun(self):  # type: ignore[no-untyped-def]
        """The frame's boxes, coloured by whether they survived into the world."""
        import rerun as rr

        if not self.boxes:
            return rr.Clear(recursive=False)
        return rr.Boxes2D(
            array=[list(b) for b in self.boxes],
            array_format=rr.Box2DFormat.XYXY,
            colors=[(80, 220, 120) if p else (160, 160, 160) for p in self.placed],
            labels=[
                f"{label} {conf:.2f}" + ("" if placed else " (unplaced)")
                for label, conf, placed in zip(
                    self.labels, self.confidences, self.placed, strict=True
                )
            ],
        )


def annotation_stream(store: Any, *, name: str = ANNOTATION_STREAM_NAME) -> Any:
    """Open (or create) the frame-annotation stream on ``store``."""
    return store.stream(name, FrameAnnotation, codec="json")


def append_annotation(stream: Any, annotation: FrameAnnotation) -> Any:
    """Append one frame's annotations, indexed at the frame's own timestamp."""
    return stream.append(
        annotation,
        ts=annotation.ts,
        tags={
            "frame_stream": annotation.frame_stream,
            "frame_observation_id": annotation.frame_observation_id,
            "detections": len(annotation.boxes),
        },
    )


def fold_annotations(observations: Iterable[Any]) -> Iterator[FrameAnnotation]:
    """Group sightings back into the frames they were drawn on.

    Grouped on consecutive runs, so memory is bounded by one frame's detections
    and the input has to arrive in timestamp order; out-of-order input degrades
    into duplicate frame records rather than lost boxes.

    Sightings without a ``bbox`` are skipped -- inventing a rectangle would put a
    fabricated overlay on a real photograph.
    """
    drawn = (
        record
        for item in observations
        if (record := getattr(item, "data", item)).bbox and record.evidence
    )
    for _, frame in groupby(
        drawn, key=lambda r: (r.evidence[0].stream, r.evidence[0].observation_id)
    ):
        records = list(frame)
        source = records[0].evidence[0]
        yield FrameAnnotation(
            schema_version=SCHEMA_VERSION,
            frame_stream=source.stream,
            frame_observation_id=source.observation_id,
            ts=source.ts,
            boxes=tuple(tuple(r.bbox) for r in records),
            labels=tuple(r.label or "?" for r in records),
            confidences=tuple(float(r.confidence.detection or 0.0) for r in records),
            placed=tuple(r.target_pose is not None for r in records),
        )

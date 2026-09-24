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

"""One place a hyperspace query found a thing, with the frame that found it.

A caller that asks "where is the traffic cone" wants three different things and they
are not interchangeable -- somewhere to drive to (the 3D box), a reason to believe it
(the picture the detector actually looked at, and how sure it was), and a way to put the
box back on that picture (the 2D box and the frame it was taken in). All three travel
together, because an answer without its evidence cannot be checked and a picture without
its frame id cannot be placed.

**The box is a `vision_msgs.Detection3D` and not a redeclaration of one**: frame,
centre, extent, score and the query that found it all mean here exactly what they mean
everywhere else in dimos, so anything that already draws a Detection3D can draw this.
What is added is only what that message has no home for -- the evidence picture and the
camera frame and stamp it was taken in (a Detection3D has ONE header, not a per-detection
source frame), the 2D box on it, how far away it was seen, how many views agreed, and
which checkpoints put the frame in front of the detector.

This lives beside the module rather than in `dimos/msgs` deliberately: it is one
module's answer shape, not a sensor type the rest of the system speaks.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image


def detection_of(
    frame: str,
    centre: tuple[float, float, float],
    extent: tuple[float, float, float],
    confidence: float,
    query: str,
    place_id: int,
    stamp: float,
) -> Detection3D:
    """A `Detection3D` for one placed answer.

    Every field is assigned rather than passed to the constructor because the generated
    LCM constructor hands out a SHARED default `bbox`, so messages built the short way
    end up writing over each other's geometry.
    """
    detection = Detection3D()
    detection.header = Header(stamp, frame)
    detection.id = str(place_id)
    detection.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(class_id=query, score=float(confidence))
        )
    ]
    detection.results_length = len(detection.results)
    detection.bbox = BoundingBox3D(
        center=Pose(position=Vector3(*(float(value) for value in centre))),
        size=Vector3(*(float(value) for value in extent)),
    )
    return detection


@dataclass(eq=False)
class FoundObject:
    """One place the thing was found, with the frame that found it."""

    # Where it is, and how sure the detector was. The centre and extent are metres in
    # `detection.header.frame_id` (the world frame the query was asked in), NOT in the
    # camera's frame: a caller navigating to this should not have to know which camera
    # saw it. The properties below read the same numbers back out.
    # (Built rather than defaulted to a bare `Detection3D()` for the same reason
    # `detection_of` assigns every field: the generated LCM default bbox is shared.)
    detection: Detection3D = field(
        default_factory=lambda: detection_of("", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, "", 0, 0.0)
    )

    # How far the object was from the camera THAT SAW IT, at the moment it was seen, in
    # metres. Kept because a box eight metres out is worth less than the same box at one
    # metre, and the centre alone does not say which this was.
    #
    # IT IS NOT THE DISTANCE FROM WHOEVER IS ASKING, and it reads exactly like one. An
    # agent handed this alongside the centre told a user a basket was "about 1 meter
    # away" while drawing them a 33.7 m route to it -- the recording's camera had passed
    # within a metre of it, and the person had not. Anything rendering this to a human
    # wants "seen from about a metre", or nothing; the distance from the asker is theirs
    # to compute from the centre and where they are.
    depth_m: float = 0.0

    # The evidence: the colour frame the detector looked at, that frame's camera frame
    # id, and the box it drew on it in pixels (x0, y0, x1, y1). The stamp lives on
    # `detection.header`, which is the moment that frame was taken.
    image: Image | None = field(default=None, repr=False)
    camera_frame: str = ""
    box2d: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    # How many views agreed, and which checkpoints put the frame in front of the
    # detector at all.
    views: int = 1
    models: list[str] = field(default_factory=list)

    @property
    def frame(self) -> str:
        """The world frame the box is in."""
        return str(self.detection.header.frame_id)

    @property
    def stamp(self) -> float:
        """When the frame that found it was taken."""
        return float(self.detection.ts)

    @property
    def centre(self) -> tuple[float, float, float]:
        position = self.detection.bbox.center.position
        return (float(position.x), float(position.y), float(position.z))

    @property
    def extent(self) -> tuple[float, float, float]:
        size = self.detection.bbox.size
        return (float(size.x), float(size.y), float(size.z))

    @property
    def confidence(self) -> float:
        """How sure the detector was, 0-1, on its own calibrated scale."""
        return float(self.detection.results[0].hypothesis.score) if self.detection.results else 0.0

    @property
    def query(self) -> str:
        """The text this was an answer to."""
        return str(self.detection.results[0].hypothesis.class_id) if self.detection.results else ""

    @property
    def place_id(self) -> int:
        """Which place this is. Two answers sharing a place_id are two looks at one
        thing, so a caller can tell "a second cone" from "the same cone again"."""
        return int(self.detection.id) if self.detection.id else 0

    def __eq__(self, other: object) -> bool:
        """Two answers are equal when they say the same thing.

        Written out because the geometry is an LCM `Detection3D`, which has no `__eq__`
        of its own: the generated dataclass equality compared it by IDENTITY and so
        called two identical answers different.
        """
        if not isinstance(other, FoundObject):
            return NotImplemented
        return self.as_dict() == other.as_dict() and self.image == other.image

    def as_dict(self) -> dict[str, Any]:
        """Everything but the picture, for logs and JSON callers."""
        return {
            "frame": self.frame,
            "centre": list(self.centre),
            "extent": list(self.extent),
            "depth_m": self.depth_m,
            "confidence": self.confidence,
            "query": self.query,
            "camera_frame": self.camera_frame,
            "stamp": self.stamp,
            "box2d": list(self.box2d),
            "place_id": self.place_id,
            "views": self.views,
            "models": list(self.models),
            "has_image": self.image is not None,
        }

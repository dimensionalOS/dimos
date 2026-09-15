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

"""What `Hyperspace.find_objects` hands back: where the thing is, and the proof.

A caller that asks "where is the traffic cone" wants three different things and they
are not interchangeable -- somewhere to drive to (the 3D box), a reason to believe it
(the picture the detector actually looked at, and how sure it was), and a way to put the
box back on that picture (the 2D box and the frame it was taken in). All three travel
together, because an answer without its evidence cannot be checked and a picture without
its frame id cannot be placed.

This lives beside the module rather than in `dimos/msgs` deliberately: it is one
module's answer shape, not a sensor type the rest of the system speaks. It has no
`lcm_encode`, so the transport factory gives it the pickled transport -- which is what
carries the colour frames without a second copy of the image codecs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import TYPE_CHECKING, Any, ClassVar

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image


@dataclass
class FoundObject:
    """One place the thing was found, with the frame that found it."""

    # Where it is. `centre` and `extent` are metres in `frame` (the world frame the
    # query was asked in), NOT in the camera's frame: a caller navigating to this
    # should not have to know which camera saw it.
    frame: str = ""
    centre: tuple[float, float, float] = (0.0, 0.0, 0.0)
    extent: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # How far the object was from the camera THAT SAW IT, at the moment it was seen, in
    # metres. Kept because a box eight metres out is worth less than the same box at one
    # metre, and the centre alone does not say which this was.
    #
    # IT IS NOT THE DISTANCE FROM WHOEVER IS ASKING, and it reads exactly like one. An
    # agent handed this alongside `centre` told a user a basket was "about 1 meter away"
    # while drawing them a 33.7 m route to it -- the recording's camera had passed within
    # a metre of it, and the person had not. Anything rendering this to a human wants
    # "seen from about a metre", or nothing; the distance from the asker is theirs to
    # compute from `centre` and where they are.
    depth_m: float = 0.0

    # How sure the detector was, 0-1, on its own calibrated scale.
    confidence: float = 0.0

    # The evidence: the colour frame the detector looked at, that frame's camera
    # frame id and stamp, and the box it drew on it in pixels (x0, y0, x1, y1).
    image: Image | None = field(default=None, repr=False)
    camera_frame: str = ""
    stamp: float = 0.0
    box2d: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    # Which place this is. Two answers sharing a place_id are two looks at one thing,
    # so a caller can tell "a second cone" from "the same cone again".
    place_id: int = 0
    # How many views agreed, and which checkpoints put the frame in front of the
    # detector at all.
    views: int = 1
    models: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        """Everything but the picture, for logs and JSON callers."""
        return {
            "frame": self.frame,
            "centre": list(self.centre),
            "extent": list(self.extent),
            "depth_m": self.depth_m,
            "confidence": self.confidence,
            "camera_frame": self.camera_frame,
            "stamp": self.stamp,
            "box2d": list(self.box2d),
            "place_id": self.place_id,
            "views": self.views,
            "models": list(self.models),
            "has_image": self.image is not None,
        }


@dataclass
class FoundObjects:
    """Every place one query found, strongest first."""

    msg_name: ClassVar[str] = "hyperspace.FoundObjects"

    query: str = ""
    objects: list[FoundObject] = field(default_factory=list)
    # Which sort of answer these are, and a subscriber has to read it before it reads
    # `confidence` or `extent`. "item" means the detector drew a box: `extent` is
    # measured and `confidence` is OWLv2's own calibrated score. "heatmap" and "area"
    # mean a scored CELL: `extent` is not measured, and `confidence` carries the cell's
    # score, which is a different quantity on a different scale and must not be compared
    # with a detector's. Without this a viewer reads a cell score as a detection and
    # believes a number nothing measured.
    kind: str = "item"
    # The world frame every box is in, repeated here so a caller reading only the
    # envelope does not have to open an object to find out.
    frame: str = "odom"
    # Episodes the detector refused. A query that found nothing and a query that was
    # never asked look identical without this.
    refused: int = 0
    # How long the whole thing took, and how that split. Live callers budget against
    # this, and the split is what says whether a slow answer was the search or OWLv2.
    ms: float = 0.0
    timings: dict[str, float] = field(default_factory=dict)
    ts: float = field(default_factory=time.time)

    def __len__(self) -> int:
        return len(self.objects)

    def as_dict(self) -> dict[str, Any]:
        return {
            "query": self.query,
            "kind": self.kind,
            "frame": self.frame,
            "objects": [found.as_dict() for found in self.objects],
            "refused": self.refused,
            "ms": self.ms,
            "timings": self.timings,
            "ts": self.ts,
        }

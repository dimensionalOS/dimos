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

"""What `Hyperspace.find_objects` hands back: every place one query found.

It has no `lcm_encode`, so the transport factory gives it the pickled transport --
which is what carries the colour frames without a second copy of the image codecs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Any, ClassVar

from dimos.mapping.hyperspace.FoundObject import FoundObject


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

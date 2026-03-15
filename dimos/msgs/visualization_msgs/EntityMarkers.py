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

"""Labeled 3D entity markers for Rerun visualization.

Published by modules that track entities in world coordinates.
The Rerun bridge picks these up via ``to_rerun()`` and renders them
as labeled colored points overlaid on the 3D scene.
"""

from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
import json
import struct
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

from dimos.types.timestamped import Timestamped

# Entity type → RGBA color
TYPE_COLORS: dict[str, tuple[int, int, int, int]] = {
    "person": (255, 100, 100, 255),
    "object": (100, 255, 100, 255),
    "location": (100, 100, 255, 255),
}
DEFAULT_COLOR = (200, 200, 200, 255)


@dataclass
class Marker:
    """A single entity marker in world coordinates."""

    entity_id: str
    label: str
    entity_type: str  # person | object | location
    x: float
    y: float
    z: float


class EntityMarkers(Timestamped):
    """A batch of labeled 3D entity markers.

    Wire format: JSON-encoded list of markers over LCM string channel.
    Rerun: ``rr.Points3D`` with per-point labels and colors.
    """

    msg_name = "visualization_msgs.EntityMarkers"

    def __init__(
        self,
        markers: list[Marker] | None = None,
        ts: float | None = None,
    ) -> None:
        self.markers: list[Marker] = markers or []
        self.ts: float = ts or time.time()

    # -- LCM serialization (JSON payload) --

    def _encode_one(self, buf: BytesIO) -> None:
        payload = json.dumps(
            [
                {
                    "id": m.entity_id,
                    "label": m.label,
                    "type": m.entity_type,
                    "x": m.x,
                    "y": m.y,
                    "z": m.z,
                }
                for m in self.markers
            ]
        ).encode()
        buf.write(struct.pack(">I", len(payload)))
        buf.write(payload)

    def encode(self) -> bytes:
        buf = BytesIO()
        self._encode_one(buf)
        return buf.getvalue()

    @classmethod
    def _decode_one(cls, buf: BytesIO) -> EntityMarkers:
        (length,) = struct.unpack(">I", buf.read(4))
        payload = json.loads(buf.read(length).decode())
        markers = [
            Marker(
                entity_id=m["id"],
                label=m["label"],
                entity_type=m["type"],
                x=m["x"],
                y=m["y"],
                z=m["z"],
            )
            for m in payload
        ]
        return cls(markers=markers)

    @classmethod
    def decode(cls, data: bytes) -> EntityMarkers:
        return cls._decode_one(BytesIO(data))

    # -- Rerun conversion --

    def to_rerun(self) -> Archetype:
        import rerun as rr

        if not self.markers:
            return rr.Points3D([])

        positions = [[m.x, m.y, m.z] for m in self.markers]
        labels = [f"{m.entity_id}: {m.label[:40]}" for m in self.markers]
        colors = [TYPE_COLORS.get(m.entity_type, DEFAULT_COLOR) for m in self.markers]

        return rr.Points3D(
            positions=positions,
            labels=labels,
            colors=colors,
            radii=[0.15] * len(self.markers),
        )

    # -- LCM compat (so autoconnect assigns LCMTransport, not pLCM) --

    def lcm_encode(self) -> bytes:
        return self.encode()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: object) -> EntityMarkers:
        return cls.decode(data)

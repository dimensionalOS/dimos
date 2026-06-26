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

from __future__ import annotations

from dataclasses import dataclass
import json
import time
from typing import BinaryIO, cast

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped


@dataclass(slots=True)
class RegisteredObject(Timestamped):
    """Runtime-registered object with frame-axis-aligned target bounds."""

    object_id: str
    name: str
    center: Vector3
    size: Vector3
    frame_id: str = "world"
    ts: float = 0.0

    msg_name = "perception_msgs.RegisteredObject"

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()

    def lcm_encode(self) -> bytes:
        return json.dumps(
            {
                "object_id": self.object_id,
                "name": self.name,
                "center": [self.center.x, self.center.y, self.center.z],
                "size": [self.size.x, self.size.y, self.size.z],
                "frame_id": self.frame_id,
                "ts": self.ts,
            },
            separators=(",", ":"),
        ).encode("utf-8")

    encode = lcm_encode

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> RegisteredObject:
        raw = data if isinstance(data, bytes) else data.read()
        payload = _as_dict(json.loads(raw.decode("utf-8")))
        center = _as_float_list(payload["center"])
        size = _as_float_list(payload["size"])
        return cls(
            object_id=str(payload["object_id"]),
            name=str(payload["name"]),
            center=Vector3(center),
            size=Vector3(size),
            frame_id=str(payload["frame_id"]),
            ts=_as_float(payload["ts"]),
        )

    decode = lcm_decode

    def to_rerun(self) -> object:
        """Render target bounds as a Rerun box."""
        import rerun as rr  # type: ignore[import-not-found]

        return rr.Boxes3D(
            centers=[[self.center.x, self.center.y, self.center.z]],
            sizes=[[self.size.x, self.size.y, self.size.z]],
            colors=[[0, 200, 255]],
            labels=[f"{self.name}:{self.object_id}"],
        )


def _as_dict(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError("RegisteredObject payload must be a JSON object")
    return cast("dict[str, object]", value)


def _as_float_list(value: object) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("Expected a 3-element numeric list")
    return [float(item) for item in value]


def _as_float(value: object) -> float:
    if not isinstance(value, int | float | str):
        raise ValueError(f"Expected a float-compatible value, got {type(value).__name__}")
    return float(value)

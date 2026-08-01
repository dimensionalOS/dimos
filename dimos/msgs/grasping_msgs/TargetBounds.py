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
class TargetBounds(Timestamped):
    """Frame-axis-aligned bounds for a grasp target."""

    center: Vector3
    size: Vector3
    frame_id: str = "world"
    ts: float = 0.0
    label: str = "target"

    msg_name = "grasping_msgs.TargetBounds"

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()

    def expanded(self, cushion_m: float) -> TargetBounds:
        cushion = max(0.0, float(cushion_m)) * 2.0
        return TargetBounds(
            center=self.center,
            size=Vector3(self.size.x + cushion, self.size.y + cushion, self.size.z + cushion),
            frame_id=self.frame_id,
            ts=self.ts,
            label=self.label,
        )

    def lcm_encode(self) -> bytes:
        return json.dumps(
            {
                "center": [self.center.x, self.center.y, self.center.z],
                "size": [self.size.x, self.size.y, self.size.z],
                "frame_id": self.frame_id,
                "ts": self.ts,
                "label": self.label,
            },
            separators=(",", ":"),
        ).encode("utf-8")

    encode = lcm_encode

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> TargetBounds:
        raw = data if isinstance(data, bytes) else data.read()
        payload = _as_dict(json.loads(raw.decode("utf-8")))
        center = _as_float_list(payload["center"])
        size = _as_float_list(payload["size"])
        return cls(
            center=Vector3(center),
            size=Vector3(size),
            frame_id=str(payload["frame_id"]),
            ts=_as_float(payload["ts"]),
            label=str(payload.get("label", "target")),
        )

    decode = lcm_decode

    def to_rerun(self) -> object:
        import rerun as rr  # type: ignore[import-not-found]

        return rr.Boxes3D(
            centers=[[self.center.x, self.center.y, self.center.z]],
            sizes=[[self.size.x, self.size.y, self.size.z]],
            colors=[[0, 200, 255]],
            labels=[self.label],
        )


def _as_dict(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError("TargetBounds payload must be a JSON object")
    return cast("dict[str, object]", value)


def _as_float_list(value: object) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("Expected a 3-element numeric list")
    return [float(item) for item in value]


def _as_float(value: object) -> float:
    if not isinstance(value, int | float | str):
        raise ValueError(f"Expected a float-compatible value, got {type(value).__name__}")
    return float(value)

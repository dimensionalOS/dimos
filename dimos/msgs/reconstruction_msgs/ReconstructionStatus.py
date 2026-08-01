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

import json
import time
from typing import BinaryIO, cast

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped


class ReconstructionStatus(Timestamped):
    """Status for a streaming scene reconstruction module."""

    msg_name = "reconstruction_msgs.ReconstructionStatus"

    def __init__(
        self,
        *,
        integrated_frames: int = 0,
        dropped_frames: int = 0,
        last_error: str = "",
        active: bool = True,
        paused: bool = False,
        latest_integration_ts: float | None = None,
        workspace_origin: Vector3 | None = None,
        workspace_size: float = 0.3,
        frame_id: str = "world",
        ts: float | None = None,
    ) -> None:
        self.integrated_frames = int(integrated_frames)
        self.dropped_frames = int(dropped_frames)
        self.last_error = last_error
        self.active = active
        self.paused = paused
        self.latest_integration_ts = latest_integration_ts
        self.workspace_origin = workspace_origin if workspace_origin is not None else Vector3()
        self.workspace_size = float(workspace_size)
        self.frame_id = frame_id
        self.ts = ts if ts is not None else time.time()

    def lcm_encode(self) -> bytes:
        payload = {
            "integrated_frames": self.integrated_frames,
            "dropped_frames": self.dropped_frames,
            "last_error": self.last_error,
            "active": self.active,
            "paused": self.paused,
            "latest_integration_ts": self.latest_integration_ts,
            "workspace_origin": [
                self.workspace_origin.x,
                self.workspace_origin.y,
                self.workspace_origin.z,
            ],
            "workspace_size": self.workspace_size,
            "frame_id": self.frame_id,
            "ts": self.ts,
        }
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

    encode = lcm_encode

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> ReconstructionStatus:
        raw = _read_bytes(data)
        payload = _as_payload(json.loads(raw.decode("utf-8")))
        origin = _as_float_list(payload["workspace_origin"], expected_len=3)
        return cls(
            integrated_frames=_as_int(payload["integrated_frames"]),
            dropped_frames=_as_int(payload["dropped_frames"]),
            last_error=str(payload["last_error"]),
            active=bool(payload.get("active", True)),
            paused=bool(payload["paused"]),
            latest_integration_ts=_as_optional_float(payload.get("latest_integration_ts")),
            workspace_origin=Vector3(origin),
            workspace_size=_as_float(payload["workspace_size"]),
            frame_id=str(payload["frame_id"]),
            ts=_as_float(payload["ts"]),
        )

    decode = lcm_decode


def _as_payload(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError("ReconstructionStatus payload must be a JSON object")
    return cast("dict[str, object]", value)


def _read_bytes(data: bytes | BinaryIO) -> bytes:
    if isinstance(data, bytes):
        return data
    return data.read()


def _as_int(value: object) -> int:
    if not isinstance(value, int | float | str):
        raise ValueError(f"Expected an int-compatible value, got {type(value).__name__}")
    return int(value)


def _as_float(value: object) -> float:
    if not isinstance(value, int | float | str):
        raise ValueError(f"Expected a float-compatible value, got {type(value).__name__}")
    return float(value)


def _as_optional_float(value: object) -> float | None:
    if value is None:
        return None
    return _as_float(value)


def _as_float_list(value: object, *, expected_len: int) -> list[float]:
    if not isinstance(value, list) or len(value) != expected_len:
        raise ValueError(f"Expected a list of {expected_len} floats")
    return [float(item) for item in value]

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

"""SONIC ZMQ wire codec: decode the GEAR-SONIC packed message format.

Wire layout (gear_sonic_deploy zmq_packed_message_subscriber.hpp, and the
Python builders in gear_sonic/utils/teleop/zmq/zmq_planner_sender.py):

    [topic_prefix][1280-byte null-padded JSON header][concatenated fields]

The header declares ``{"v", "endian", "count", "fields": [{name, dtype,
shape}, ...]}``; payload fields are concatenated little-endian arrays in
declaration order. Topics: ``command``, ``planner``, ``pose``.

This module is the receive side of D2 (feature parity by wire
compatibility): NVIDIA's senders — the pico VR server, planner senders,
recorded traffic — must decode without modification.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Any

import numpy as np

HEADER_SIZE = 1280

_DTYPES: dict[str, np.dtype] = {
    "u8": np.dtype("uint8"),
    "i32": np.dtype("<i4"),
    "i64": np.dtype("<i8"),
    "f32": np.dtype("<f4"),
    "f64": np.dtype("<f8"),
    "bool": np.dtype("bool"),
}

KNOWN_TOPICS = ("command", "planner", "pose")


@dataclass
class DecodedMessage:
    topic: str
    version: int
    count: int
    fields: dict[str, np.ndarray] = field(default_factory=dict)

    def get(self, name: str) -> np.ndarray | None:
        return self.fields.get(name)


def split_topic(raw: bytes) -> tuple[str, bytes]:
    """Split the topic prefix off a raw single-part ZMQ message."""
    for topic in KNOWN_TOPICS:
        prefix = topic.encode()
        if raw.startswith(prefix):
            return topic, raw[len(prefix) :]
    raise ValueError(f"unknown topic prefix: {raw[:16]!r}")


def decode(raw: bytes) -> DecodedMessage:
    """Decode one packed message (topic prefix included)."""
    topic, body = split_topic(raw)
    if len(body) < HEADER_SIZE:
        raise ValueError(f"{topic}: body shorter than header ({len(body)} < {HEADER_SIZE})")
    header_json = body[:HEADER_SIZE].rstrip(b"\x00")
    header: dict[str, Any] = json.loads(header_json)
    if header.get("endian", "le") != "le":
        raise ValueError(f"{topic}: unsupported endianness {header.get('endian')!r}")

    msg = DecodedMessage(
        topic=topic,
        version=int(header.get("v", 1)),
        count=int(header.get("count", 1)),
    )
    payload = body[HEADER_SIZE:]
    offset = 0
    for f in header.get("fields", []):
        name = f["name"]
        dtype = _DTYPES.get(f["dtype"])
        if dtype is None:
            raise ValueError(f"{topic}: unknown dtype {f['dtype']!r} for {name!r}")
        shape = tuple(int(v) for v in f.get("shape", [1]))
        nbytes = int(np.prod(shape)) * dtype.itemsize
        if offset + nbytes > len(payload):
            raise ValueError(
                f"{topic}: payload underrun at field {name!r} ({offset + nbytes} > {len(payload)})"
            )
        arr = np.frombuffer(payload, dtype=dtype, count=int(np.prod(shape)), offset=offset)
        msg.fields[name] = arr.reshape(shape).copy()
        offset += nbytes
    return msg


@dataclass
class CommandUpdate:
    """Accumulated 'command' topic state (C++ OR-accumulates start/stop)."""

    start: bool = False
    stop: bool = False
    planner: bool = False
    delta_heading: float | None = None

    def merge(self, msg: DecodedMessage) -> None:
        start = msg.get("start")
        stop = msg.get("stop")
        planner = msg.get("planner")
        dh = msg.get("delta_heading")
        if start is not None:
            self.start = self.start or bool(start.flat[0])
        if stop is not None:
            self.stop = self.stop or bool(stop.flat[0])
        if planner is not None:
            self.planner = bool(planner.flat[0])
        if dh is not None:
            self.delta_heading = float(dh.flat[0])


@dataclass
class PlannerUpdate:
    """One decoded 'planner' topic message."""

    mode: int = 0
    movement: np.ndarray = field(default_factory=lambda: np.zeros(3))
    facing: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    speed: float = -1.0
    height: float = -1.0
    upper_body_position: np.ndarray | None = None
    upper_body_velocity: np.ndarray | None = None
    left_hand_joints: np.ndarray | None = None
    right_hand_joints: np.ndarray | None = None
    vr_position: np.ndarray | None = None
    vr_orientation: np.ndarray | None = None
    vr_compliance: np.ndarray | None = None

    @classmethod
    def from_message(cls, msg: DecodedMessage) -> PlannerUpdate:
        u = cls()
        mode = msg.get("mode")
        if mode is not None:
            u.mode = int(mode.flat[0])
        for name in ("movement", "facing"):
            arr = msg.get(name)
            if arr is not None:
                setattr(u, name, arr.astype(np.float64).reshape(3))
        for name in ("speed", "height"):
            arr = msg.get(name)
            if arr is not None:
                setattr(u, name, float(arr.flat[0]))
        for name in (
            "upper_body_position",
            "upper_body_velocity",
            "left_hand_joints",
            "right_hand_joints",
            "vr_position",
            "vr_orientation",
            "vr_compliance",
        ):
            arr = msg.get(name)
            if arr is not None:
                setattr(u, name, arr.astype(np.float64).ravel())
        return u

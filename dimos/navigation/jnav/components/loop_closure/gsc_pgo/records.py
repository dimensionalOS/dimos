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

"""Decode GscPGO's packed-record PointCloud2 outputs (rust/src/utils.rs build_*_records).

dimos' PointCloud2 keeps only x/y/z, so a Python consumer reads these channels as the raw
LCM PointCloud2: declare an ``In[RawPointCloud2]`` under its own name and pin it to the
module's channel with ``record_transports``."""

from __future__ import annotations

from typing import Any

from dimos_lcm.sensor_msgs import PointCloud2 as RawPointCloud2
import numpy as np

from dimos.core.transport_factory import make_transport

# pose_graph: one per keyframe at its current pose (quaternion x, y, z, w)
POSE_RECORD = np.dtype([("xyz", "<f4", 3), ("id", "<u4"), ("ts", "<f8"), ("q", "<f4", 4)])
# loop_closure_event: a keyframe's pose before a solve moved it, then the delta (post = delta * pre)
CLOSURE_RECORD = np.dtype([*POSE_RECORD.descr, ("dt", "<f4", 3), ("dq", "<f4", 4)])
# closure_edges: one closure candidate lifecycle event, both keyframes where they are now
CLOSURE_EDGE_RECORD = np.dtype(
    [
        ("xyz", "<f4", 3),
        ("target_xyz", "<f4", 3),
        ("source_id", "<u4"),
        ("target_id", "<u4"),
        ("state", "<u4"),
        ("confidence", "<f4"),
        ("yank_z", "<f4"),
        ("ts", "<f8"),
    ]
)
# ClosureEdgeState values (rust/src/gsc_pgo.rs): GNC voted the closure into / out of the graph
CLOSURE_ACCEPTED = 4
CLOSURE_REJECTED = 5


def decode_records(msg: RawPointCloud2, dtype: np.dtype) -> np.ndarray:
    """The message's records as a structured array of ``dtype``."""
    if msg.width * msg.height and msg.point_step != dtype.itemsize:
        raise ValueError(f"point_step {msg.point_step} != {dtype.itemsize}-byte record")
    return np.frombuffer(bytes(msg.data), dtype, msg.width * msg.height)


def record_transports(channels: dict[str, str]) -> dict[tuple[str, type], Any]:
    """Blueprint transports reading each module channel (value) as raw records under its own stream name (key)."""
    return {
        (name, RawPointCloud2): make_transport(channel, RawPointCloud2)
        for name, channel in channels.items()
    }

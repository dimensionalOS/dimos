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

"""Read recording streams (mem2.db) for loop-closure evaluation.

SqliteStore access keyed by path (one shared store per db), stream iteration,
and a nearest-pose lookup over an odometry stream. Stream names are always
passed in by the caller — nothing here is tied to a particular recording layout.
"""

from __future__ import annotations

import atexit
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.navigation.jnav.utils.trajectory_metrics import PoseLookup, trajectory_lookup

# max |Δt| pairing a query time to an odom sample
ODOM_MATCH_TOLERANCE_S = 0.2

# Fixed-rate replay pacing + sizing caps (used by the eval replay drivers).
REPLAY_PUBLISH_HZ = 50.0
REPLAY_DRAIN_MARGIN_S = 30.0
MAX_REPLAY_SCANS = 4000
MAX_REPLAY_ODOM = 16000

# One shared store per db path (SqliteStore can't take absolute paths through the
# replay adapter, so streams are read directly).
_stores: dict[str, SqliteStore] = {}


def store(db_path: Path) -> SqliteStore:
    key = str(db_path)
    cached = _stores.get(key)
    if cached is None:
        cached = SqliteStore(path=key, must_exist=True)
        cached.start()
        _stores[key] = cached
    return cached


def close_all() -> None:
    """Stop every cached store, releasing their file handles."""
    for cached in _stores.values():
        cached.stop()
    _stores.clear()


atexit.register(close_all)


def list_streams(db_path: Path) -> list[str]:
    return store(db_path).list_streams()


def stream_count(db_path: Path, stream_name: str) -> int:
    return int(store(db_path).stream(stream_name).count())


def iterate_stream(
    db_path: Path, stream_name: str, *, stride: int = 1
) -> Iterator[tuple[float, Any]]:
    """Yield ``(timestamp, decoded message)``, decoding only kept rows."""
    stream: Any = store(db_path).stream(stream_name)
    for index, observation in enumerate(stream):
        if stride > 1 and index % stride:
            continue
        yield (float(observation.ts), observation.data)


def payload_pose(payload: Any) -> Pose:
    """Normalize a recorded odometry payload to a ``Pose``.

    Handles both shapes found in recordings: ``Odometry`` (``.pose``) and
    ``PoseStamped`` (flat position + ``.orientation``)."""
    if hasattr(payload, "pose"):  # Odometry
        return payload.pose  # type: ignore[no-any-return]
    return Pose(  # PoseStamped
        payload.x,
        payload.y,
        payload.z,
        payload.orientation.x,
        payload.orientation.y,
        payload.orientation.z,
        payload.orientation.w,
    )


def odometry_lookup(db_path: Path, stream_name: str) -> PoseLookup:
    """Nearest-position (``[x, y, z]``) lookup over a recording's odometry stream."""
    times: list[float] = []
    positions: list[list[float]] = []
    for timestamp, payload in iterate_stream(db_path, stream_name):
        pose = payload_pose(payload)
        times.append(timestamp)
        positions.append([pose.position.x, pose.position.y, pose.position.z])
    return trajectory_lookup(
        np.asarray(times, dtype=np.float64),
        np.asarray(positions, dtype=np.float64),
        ODOM_MATCH_TOLERANCE_S,
    )

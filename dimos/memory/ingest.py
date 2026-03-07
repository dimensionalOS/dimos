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

"""Helpers for ingesting timestamped data into memory streams."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Iterable

    from dimos.memory.stream import Stream


def ingest(
    stream: Stream[Any],
    source: Iterable[tuple[float, Any]],
    *,
    pose_source: Any | None = None,
) -> int:
    """Ingest (timestamp, payload) pairs into a stream.

    Accepts any iterable of ``(ts, data)`` — e.g. ``replay.iterate_ts(seek=5, duration=60)``.

    Args:
        pose_source: Optional replay with ``find_closest(ts)`` returning a pose
            to attach to each frame (e.g. odom replay).

    Returns:
        Number of items ingested.
    """
    count = 0
    for ts, payload in source:
        pose = None
        if pose_source is not None:
            pose = pose_source.find_closest(ts)
        stream.append(payload, ts=ts, pose=pose)
        count += 1
    return count

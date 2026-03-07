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

"""Send memory stream contents to Rerun.

Iterates a Stream, calls ``.to_rerun()`` on each observation's data
payload, and logs it at the observation's timestamp.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.memory.stream import Stream


def _infer_entity_path(stream: Any) -> str:
    """Derive an entity path from the stream's backend name."""
    backend = getattr(stream, "_backend", None)
    if backend is not None:
        name = getattr(backend, "stream_name", None)
        if name and name != "<memory>":
            return f"memory/{name}"
    raise ValueError(
        "Cannot infer entity_path — stream has no named backend "
        "(e.g. ObservationSet from .fetch()). Pass entity_path explicitly."
    )


def to_rerun(
    stream: Stream[Any] | Any,
    entity_path: str | None = None,
) -> int:
    """Log stream observations to Rerun.

    For each observation whose ``.data`` has a ``to_rerun()`` method,
    logs the result at the observation's timestamp on a custom "time"
    timeline (no wall-clock contamination).

    Args:
        stream: Any Stream or iterable of Observations.
        entity_path: Rerun entity path. Auto-derived from stream name if None.

    Returns:
        Number of items logged.
    """
    import rerun as rr

    if entity_path is None:
        entity_path = _infer_entity_path(stream)

    rr.disable_timeline("log_time")
    rr.disable_timeline("log_tick")

    count = 0
    for obs in stream:
        if obs.ts is not None:
            rr.set_time("time", duration=obs.ts)

        data = obs.data
        if hasattr(data, "to_rerun"):
            rr.log(entity_path, data.to_rerun())
            count += 1

        if obs.pose is not None and hasattr(obs.pose, "to_rerun_arrow"):
            rr.log(f"{entity_path}/pose", obs.pose.to_rerun_arrow())

    rr.reset_time()
    return count

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

"""Opening and writing the belief stream.

A plain derived stream in the existing store -- no second database -- which buys
timestamps, capture pose, tag pushdown, replay and ``align()`` for free.

Two invariants hold by construction rather than by review: ``append_belief``
derives tags from the payload and takes no parameter to override them, and the
envelope timestamp is valid time, because "what was in the kitchen at 2pm" is the
query that has to be fast.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, TypeVar

from dimos.experimental.memory_belief.types import STREAM_NAME, BeliefObservation, belief_tags

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream
    from dimos.memory.type.observation import Observation

T = TypeVar("T")


def derived_stream(store: Store, name: str, payload_type: type[T]) -> Stream[T]:
    """Open (or create) a derived belief stream on ``store``.

    Every derived record type is stored the same way, and the codec is the part
    worth keeping in one place: these streams are read by tools outside this
    package, so a payload that stopped being readable JSON would break them
    silently.
    """
    return store.stream(name, payload_type, codec="json")


def belief_stream(store: Store, *, name: str = STREAM_NAME) -> Stream[BeliefObservation]:
    """Open (or create) the belief stream on ``store``.

    Stored as JSON rather than pickle: these records outlive the release that
    wrote them and other tools have to read them.
    """
    return store.stream(name, BeliefObservation, codec="json")


def append_belief(
    stream: Stream[BeliefObservation],
    record: BeliefObservation,
    *,
    capture_pose: Any | None = None,
) -> Observation[BeliefObservation]:
    """Append one record, indexing it by valid time and capture pose.

    ``capture_pose`` is the vantage point, deliberately separate from
    ``record.target_pose``: conflating them turns "seen from the kitchen" into
    "in the kitchen".
    """
    return stream.append(
        record,
        ts=record.valid_ts,
        pose=capture_pose,
        tags=belief_tags(record),
    )

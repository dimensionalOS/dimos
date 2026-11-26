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

"""Shared recorder policies for source timestamps and existing artifacts."""

from collections.abc import Iterator
import enum
import time
from typing import Any

from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.types.timestamped import to_timestamp
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OnExisting(str, enum.Enum):
    OVERWRITE = "overwrite"
    ERROR = "error"
    BACKUP = "backup"
    APPEND = "append"


def message_timestamp(msg: Any, reception_ts: float | None = None) -> float:
    """Return the message's source time, falling back to the receiving clock."""
    ts = getattr(msg, "ts", None)
    if ts is not None:
        return float(ts)
    for field in ("source_ts", "timestamp"):
        value = getattr(msg, field, None)
        if isinstance(value, (int, float)):
            return float(value)
    stamp = getattr(getattr(msg, "header", msg), "stamp", None)
    if stamp is not None:
        try:
            return to_timestamp(stamp)
        except TypeError:
            pass  # An opaque payload may use "stamp" for a non-clock value.
    return time.time() if reception_ts is None else reception_ts


class SourceStamps:
    """Index recorded rows on their source stamps, one row per TF transform.

    A zero stamp means the publisher never set it: that row is indexed at
    arrival, and the topic is reported once.
    """

    def __init__(self) -> None:
        self._unstamped_topics: set[str] = set()

    def rows(
        self, topic: str, msg: Any, ts: float | None = None, *, reception_ts: float
    ) -> Iterator[tuple[Any, float]]:
        """Yield ``(payload, ts)``: one per TF transform, otherwise *msg* at *ts*."""
        if isinstance(msg, TFMessage) and msg.transforms:
            for transform in msg.transforms:
                yield TFMessage(transform), self._stamp(topic, transform.ts, reception_ts)
        elif ts is not None:
            yield msg, self._stamp(topic, ts, reception_ts)

    def _stamp(self, topic: str, ts: float, reception_ts: float) -> float:
        if ts:
            return ts
        if topic not in self._unstamped_topics:
            self._unstamped_topics.add(topic)
            logger.warning(
                "[%s] publisher does not stamp its messages (source stamp 0); "
                "indexing them at arrival time",
                topic,
            )
        return reception_ts

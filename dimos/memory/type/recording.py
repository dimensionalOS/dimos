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
from typing import Any

from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OnExisting(str, enum.Enum):
    OVERWRITE = "overwrite"
    ERROR = "error"
    BACKUP = "backup"
    APPEND = "append"


class SourceStamps:
    """Index recorded rows on their source stamps, one row per TF transform.

    A message's source time is its ``ts``. A missing or zero ``ts`` means the
    publisher never set it: that row is indexed at arrival, and the topic is
    reported once.
    """

    def __init__(self) -> None:
        self._unstamped_topics: set[str] = set()

    def rows(
        self, topic: str, msg: Any, *, reception_ts: float, ts: float | None = None
    ) -> Iterator[tuple[Any, float]]:
        """Yield ``(payload, ts)``: one per TF transform, otherwise *msg* at *ts* (its ``ts`` by default)."""
        if isinstance(msg, TFMessage) and msg.transforms:
            for transform in msg.transforms:
                yield TFMessage(transform), self._stamp(topic, transform.ts, reception_ts)
        else:
            source_ts = getattr(msg, "ts", None) if ts is None else ts
            yield msg, self._stamp(topic, source_ts, reception_ts)

    def _stamp(self, topic: str, ts: float | None, reception_ts: float) -> float:
        if ts:
            return ts
        if topic not in self._unstamped_topics:
            self._unstamped_topics.add(topic)
            logger.warning(
                "[%s] publisher does not stamp its messages (no source ts); "
                "indexing them at arrival time",
                topic,
            )
        return reception_ts

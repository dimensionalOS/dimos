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

"""Bounded ordinary publications, shared across provider reconnections."""

import asyncio
from contextlib import suppress
import time
from typing import Literal

import zenoh

from dimos.utils.logging_config import setup_logger

logger = setup_logger()
PutOutcome = Literal["accepted", "rejected", "unknown"]


class TopicPuts:
    """Keep at most one Zenoh put alive, even after its receipt times out.

    A thread cannot cancel a blocking put. Report its outcome as unknown and
    reject later writes as busy until it finishes, rather than accumulating
    threads or disconnecting subscriptions. The Zenoh session owner keeps this
    worker across relay reconnects and waits for it after closing the session.
    """

    def __init__(self, session: zenoh.Session, *, timeout: float = 5.0) -> None:
        self._session = session
        self._timeout = timeout
        self._pending: asyncio.Task[None] | None = None

    async def put(
        self, key: str, payload: bytes, encoding: str, *, received_at: float | None = None
    ) -> tuple[PutOutcome, str]:
        if self._pending is not None and not self._pending.done():
            return "rejected", "topic_put_busy"
        remaining = self._timeout
        if received_at is not None:
            remaining -= max(0.0, time.monotonic() - received_at)
        if remaining <= 0:
            return "rejected", "topic_put_queue_timeout"
        task = asyncio.create_task(
            asyncio.to_thread(self._session.put, key, payload, encoding=zenoh.Encoding(encoding))
        )
        self._pending = task
        # Observe a late exception even when its original connection is gone.
        task.add_done_callback(
            lambda completed: None if completed.cancelled() else completed.exception()
        )
        try:
            await asyncio.wait_for(asyncio.shield(task), remaining)
        except asyncio.TimeoutError:  # not the builtin on Python 3.10
            logger.warning("topic put exceeded receipt deadline")
            return "unknown", "topic_put_timeout"
        except zenoh.ZError:
            return "unknown", "topic_put_failed"
        return "accepted", ""

    async def wait_idle(self) -> None:
        if self._pending is not None:
            with suppress(zenoh.ZError):
                await asyncio.shield(self._pending)

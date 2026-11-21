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

import asyncio
from contextlib import suppress
import threading
import time

import pytest
import zenoh

from dimos.web.relay_bridge.topic_put import TopicPuts


@pytest.fixture
async def worker(mocker):
    release = threading.Event()
    started = asyncio.Event()
    loop = asyncio.get_running_loop()

    def blocked_put(*args, **kwargs):
        loop.call_soon_threadsafe(started.set)
        assert release.wait(5), "test did not release the Zenoh put"

    session = mocker.Mock(spec=zenoh.Session)
    session.put.side_effect = blocked_put
    puts = TopicPuts(session, timeout=0.02)
    try:
        yield puts, session.put, started, release
    finally:
        release.set()
        await puts.wait_idle()


async def test_slow_put_returns_unknown_and_rejects_more_work_until_it_finishes(worker):
    puts, put, _, release = worker
    assert await puts.put("slow", b"first", "application/octet-stream") == (
        "unknown",
        "topic_put_timeout",
    )
    assert await puts.put("next", b"second", "application/octet-stream") == (
        "rejected",
        "topic_put_busy",
    )
    put.assert_called_once()
    release.set()
    await puts.wait_idle()
    assert await puts.put("next", b"third", "application/octet-stream") == ("accepted", "")
    assert [call.args for call in put.call_args_list] == [("slow", b"first"), ("next", b"third")]


async def test_reconnect_cancellation_cannot_start_another_put_thread(worker):
    puts, put, started, _ = worker
    pending = asyncio.create_task(puts.put("slow", b"original", "application/octet-stream"))
    try:
        await asyncio.wait_for(started.wait(), 1)
        pending.cancel()
        with pytest.raises(asyncio.CancelledError):
            await pending
        assert await puts.put("next", b"reconnected", "application/octet-stream") == (
            "rejected",
            "topic_put_busy",
        )
        put.assert_called_once()
    finally:
        pending.cancel()
        with suppress(asyncio.CancelledError):
            await pending


async def test_queue_wait_counts_toward_the_put_deadline(worker):
    puts, put, _, _ = worker
    assert await puts.put(
        "queued", b"stale", "application/octet-stream", received_at=time.monotonic() - 1
    ) == (
        "rejected",
        "topic_put_queue_timeout",
    )
    put.assert_not_called()

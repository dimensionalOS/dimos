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

"""Small descriptor snapshots and liveliness; never an all-payload observer."""

import asyncio
from collections import OrderedDict
from collections.abc import Callable
from contextlib import suppress
import threading
import time

from pydantic import BaseModel, ConfigDict, Field, StrictInt, ValidationError
import zenoh

from dimos.web.relay_bridge.topic_catalog import TopicCatalog
from dimos.web.relay_bridge.topic_protocol import MAX_TOPIC_KEY_BYTES, TopicDescriptor, TopicSource

METADATA_PREFIX = "_dimos/topics/metadata/"
METADATA_PATTERN = METADATA_PREFIX + "*"
MAX_METADATA_BYTES = 512 * 1024
MAX_METADATA_ENTRIES = 512
METADATA_REFRESH_SECONDS = 5.0


class DescriptorSnapshot(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    v: StrictInt = Field(default=1, ge=1, le=1)
    owner: TopicSource
    revision: StrictInt = Field(ge=0, le=9007199254740991)
    entries: list[TopicDescriptor] = Field(max_length=MAX_METADATA_ENTRIES)


class MetadataReceiver:
    """One coalesced, byte-bounded metadata inbox for a standalone session.

    Discovery may omit entries during a metadata flood; normal snapshots repeat.
    Its 4 MiB/4096-key pending bound is independent of the 4096-key catalog.
    No metadata failure changes the ability to subscribe to an explicit raw key.
    """

    def __init__(self, session: zenoh.Session, catalog: TopicCatalog) -> None:
        self._session = session
        self._catalog = catalog
        self._loop = asyncio.get_running_loop()
        self._changed = asyncio.Event()
        self._lock = threading.Lock()
        self._pending: OrderedDict[tuple[str, str], bytes | bool] = OrderedDict()
        self._bytes = 0
        self._closed = False
        self._subscribers: list[zenoh.Subscriber[None]] = []
        self._task: asyncio.Task[None] | None = None
        self.discarded = 0

    @staticmethod
    def _size(key: tuple[str, str], value: bytes | bool) -> int:
        return len(key[1].encode()) + (len(value) if isinstance(value, bytes) else 1)

    def _offer(self, kind: str, key: str, value: bytes | bool) -> None:
        if len(key.encode()) > MAX_TOPIC_KEY_BYTES:
            return
        item = (kind, key)
        with self._lock:
            if self._closed:
                return
            wake = not self._pending
            previous = self._pending.pop(item, None)
            if previous is not None:
                self._bytes -= self._size(item, previous)
            self._pending[item] = value
            self._bytes += self._size(item, value)
            while len(self._pending) > 4096 or self._bytes > 4 * 1024 * 1024:
                dropped, payload = self._pending.popitem(last=False)
                self._bytes -= self._size(dropped, payload)
                self.discarded += 1
        if wake:
            self._loop.call_soon_threadsafe(self._changed.set)

    def _metadata(self, sample: zenoh.Sample) -> None:
        if len(sample.payload) <= MAX_METADATA_BYTES and sample.kind == zenoh.SampleKind.PUT:
            self._offer("metadata", str(sample.key_expr), bytes(sample.payload))

    def _liveliness(self, sample: zenoh.Sample) -> None:
        self._offer("liveliness", str(sample.key_expr), sample.kind == zenoh.SampleKind.PUT)

    def _reply(self, reply: zenoh.Reply) -> None:
        sample = reply.ok
        if sample is not None:
            self._metadata(sample)

    async def _open(self, factory: Callable[[], zenoh.Subscriber[None]]) -> None:
        opening = asyncio.create_task(asyncio.to_thread(factory))
        try:
            self._subscribers.append(await asyncio.shield(opening))
        except asyncio.CancelledError:
            subscriber = await opening
            await asyncio.to_thread(subscriber.undeclare)
            raise

    async def start(self) -> None:
        try:
            await self._open(
                lambda: self._session.declare_subscriber(METADATA_PATTERN, self._metadata)
            )
            await self._open(
                lambda: self._session.liveliness().declare_subscriber(
                    "**", self._liveliness, history=True
                )
            )
            self._task = asyncio.create_task(self._run())
            # A bounded metadata-only query gets quiet publishers' last snapshot.
            await asyncio.to_thread(self._session.get, METADATA_PATTERN, self._reply, timeout=2)
        except BaseException:
            await self.close()
            raise

    async def _run(self) -> None:
        while True:
            with suppress(asyncio.TimeoutError):
                await asyncio.wait_for(self._changed.wait(), 2)
            self._changed.clear()
            with self._lock:
                pending, self._pending = self._pending, OrderedDict()
                self._bytes = 0
            for (kind, key), value in pending.items():
                if kind == "metadata" and isinstance(value, bytes):
                    try:
                        snapshot = DescriptorSnapshot.model_validate_json(value)
                        if key != METADATA_PREFIX + snapshot.owner:
                            continue
                        # Descriptor data is inert. Neither imports nor the
                        # general DimOS CBOR object loader are involved here.
                        self._catalog.replace_known(
                            snapshot.owner, snapshot.revision, snapshot.entries, time.monotonic()
                        )
                    except ValidationError:
                        self.discarded += 1
                elif kind == "liveliness":
                    if key.startswith(METADATA_PREFIX):
                        if not value:
                            self._catalog.remove_owner(key.removeprefix(METADATA_PREFIX))
                    else:
                        self._catalog.advertise(key, bool(value))
            self._catalog.expire(time.monotonic())

    async def close(self) -> None:
        with self._lock:
            self._closed = True
            self._pending.clear()
            self._bytes = 0
        if self._task is not None:
            self._task.cancel()
            with suppress(asyncio.CancelledError):
                await self._task
            self._task = None
        for subscriber in self._subscribers:
            await asyncio.to_thread(subscriber.undeclare)
        self._subscribers.clear()

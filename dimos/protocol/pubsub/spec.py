# Copyright 2025 Dimensional Inc.
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
import json
from abc import ABC, abstractmethod
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Any, Callable, Generic, TypeVar

MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")


class PubSub(ABC, Generic[TopicT, MsgT]):
    """Abstract base class for pub/sub implementations with sugar methods."""

    @abstractmethod
    def publish(self, topic: TopicT, message: MsgT) -> None:
        """Publish a message to a topic."""
        ...

    @abstractmethod
    def subscribe(self, topic: TopicT, callback: Callable[[MsgT], None]) -> None:
        """Subscribe to a topic with a callback."""
        ...

    @abstractmethod
    def unsubscribe(self, topic: TopicT, callback: Callable[[MsgT], None]) -> None:
        """Unsubscribe a callback from a topic."""
        ...

    @dataclass(slots=True)
    class _Subscription:
        _bus: "PubSub[Any, Any]"
        _topic: Any
        _cb: Callable[[Any], None]

        def unsubscribe(self) -> None:
            self._bus.unsubscribe(self._topic, self._cb)

        # context-manager helper
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.unsubscribe()

    # public helper: returns disposable object
    def sub(self, topic: TopicT, cb: Callable[[MsgT], None]) -> "_Subscription":
        self.subscribe(topic, cb)
        return self._Subscription(self, topic, cb)

    # async iterator
    async def aiter(self, topic: TopicT, *, max_pending: int | None = None) -> AsyncIterator[MsgT]:
        q: asyncio.Queue[MsgT] = asyncio.Queue(maxsize=max_pending or 0)

        def _cb(msg: MsgT):
            q.put_nowait(msg)

        self.subscribe(topic, _cb)
        try:
            while True:
                yield await q.get()
        finally:
            self.unsubscribe(topic, _cb)

    # async context manager returning a queue
    @asynccontextmanager
    async def queue(self, topic: TopicT, *, max_pending: int | None = None):
        q: asyncio.Queue[MsgT] = asyncio.Queue(maxsize=max_pending or 0)
        self.subscribe(topic, q.put_nowait)
        try:
            yield q
        finally:
            self.unsubscribe(topic, q.put_nowait)


class PubSubEncoderMixin(Generic[TopicT, MsgT]):
    """Mixin that encodes messages before publishing and decodes them after receiving.

    Usage: Just specify encoder and decoder as class attributes:

    class MyPubSubWithJSON(PubSubEncoderMixin, MyPubSub):
        encoder = lambda msg: json.dumps(msg).encode('utf-8')
        decoder = lambda data: json.loads(data.decode('utf-8'))
    """

    encoder: Callable[[MsgT], bytes]
    decoder: Callable[[bytes], MsgT]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Track callback mappings for proper unsubscribe
        self._encoder_callback_map: dict = {}

    def publish(self, topic: TopicT, message: MsgT) -> None:
        """Encode the message and publish it."""
        encoded_message = self.encoder(message)
        super().publish(topic, encoded_message)  # type: ignore[misc]

    def subscribe(self, topic: TopicT, callback: Callable[[MsgT], None]) -> None:
        """Subscribe with automatic decoding."""

        def wrapper_cb(encoded_data: bytes):
            decoded_message = self.decoder(encoded_data)
            callback(decoded_message)

        # Store the wrapper callback for proper unsubscribe
        callback_key = (topic, id(callback))
        self._encoder_callback_map[callback_key] = wrapper_cb

        super().subscribe(topic, wrapper_cb)  # type: ignore[misc]

    def unsubscribe(self, topic: TopicT, callback: Callable[[MsgT], None]) -> None:
        """Unsubscribe a callback."""
        callback_key = (topic, id(callback))
        if callback_key in self._encoder_callback_map:
            wrapper_cb = self._encoder_callback_map[callback_key]
            super().unsubscribe(topic, wrapper_cb)  # type: ignore[misc]
            del self._encoder_callback_map[callback_key]


class JSONEncoder(PubSubEncoderMixin[str, Any]):
    @staticmethod
    def encoder(msg: Any) -> bytes:
        return json.dumps(msg).encode("utf-8")

    @staticmethod
    def decoder(data: bytes) -> Any:
        return json.loads(data.decode("utf-8"))

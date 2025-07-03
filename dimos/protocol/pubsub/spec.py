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
import pickle
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
    def subscribe(
        self, topic: TopicT, callback: Callable[[MsgT, TopicT], None]
    ) -> Callable[[], None]:
        """Subscribe to a topic with a callback. returns unsubscribe function"""
        ...

    @dataclass(slots=True)
    class _Subscription:
        _bus: "PubSub[Any, Any]"
        _topic: Any
        _cb: Callable[[Any, Any], None]
        _unsubscribe_fn: Callable[[], None]

        def unsubscribe(self) -> None:
            self._unsubscribe_fn()

        # context-manager helper
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.unsubscribe()

    # public helper: returns disposable object
    def sub(self, topic: TopicT, cb: Callable[[MsgT, TopicT], None]) -> "_Subscription":
        unsubscribe_fn = self.subscribe(topic, cb)
        return self._Subscription(self, topic, cb, unsubscribe_fn)

    # async iterator
    async def aiter(self, topic: TopicT, *, max_pending: int | None = None) -> AsyncIterator[MsgT]:
        q: asyncio.Queue[MsgT] = asyncio.Queue(maxsize=max_pending or 0)

        def _cb(msg: MsgT, topic: TopicT):
            q.put_nowait(msg)

        unsubscribe_fn = self.subscribe(topic, _cb)
        try:
            while True:
                yield await q.get()
        finally:
            unsubscribe_fn()

        # async context manager returning a queue

    @asynccontextmanager
    async def queue(self, topic: TopicT, *, max_pending: int | None = None):
        q: asyncio.Queue[MsgT] = asyncio.Queue(maxsize=max_pending or 0)

        def _queue_cb(msg: MsgT, topic: TopicT):
            q.put_nowait(msg)

        unsubscribe_fn = self.subscribe(topic, _queue_cb)
        try:
            yield q
        finally:
            unsubscribe_fn()


class PubSubEncoderMixin(ABC, Generic[TopicT, MsgT]):
    """Mixin that encodes messages before publishing and decodes them after receiving.

    Usage: Just specify encoder and decoder as a subclass:

    class MyPubSubWithJSON(PubSubEncoderMixin, MyPubSub):
        def encoder(msg, topic):
            json.dumps(msg).encode('utf-8')
        def decoder(msg, topic):
            data: json.loads(data.decode('utf-8'))
    """

    @abstractmethod
    def encode(self, msg: MsgT, topic: TopicT) -> bytes: ...

    @abstractmethod
    def decode(self, msg: bytes, topic: TopicT) -> MsgT: ...

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._encode_callback_map: dict = {}

    def publish(self, topic: TopicT, message: MsgT) -> None:
        """Encode the message and publish it."""
        encoded_message = self.encode(message, topic)
        super().publish(topic, encoded_message)  # type: ignore[misc]

    def subscribe(
        self, topic: TopicT, callback: Callable[[MsgT, TopicT], None]
    ) -> Callable[[], None]:
        """Subscribe with automatic decoding."""

        def wrapper_cb(encoded_data: bytes, topic: TopicT):
            decoded_message = self.decode(encoded_data, topic)
            callback(decoded_message, topic)

        return super().subscribe(topic, wrapper_cb)  # type: ignore[misc]


class PickleEncoderMixin(PubSubEncoderMixin[TopicT, MsgT]):
    def encode(self, msg: MsgT, *_: TopicT) -> bytes:
        return pickle.dumps(msg)

    def decode(self, msg: bytes, _: TopicT) -> MsgT:
        return pickle.loads(msg)

# Copyright 2025-2026 Dimensional Inc.
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

from abc import ABC, abstractmethod
import asyncio
from collections.abc import AsyncIterator, Callable
from typing import Any, Generic, TypeVar

MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")


def accept_all(_: Any) -> bool:
    return True


class PubSub(Generic[TopicT, MsgT], ABC):
    @abstractmethod
    def publish(self, topic: TopicT, message: MsgT) -> None: ...

    @abstractmethod
    def subscribe(
        self, topic: TopicT, callback: Callable[[MsgT, TopicT], None]
    ) -> Callable[[], None]: ...

    async def aiter(self, topic: TopicT, max_pending: int = 0) -> AsyncIterator[MsgT]:
        queue: asyncio.Queue[MsgT] = asyncio.Queue(maxsize=max_pending)
        unsubscribe = self.subscribe(topic, lambda message, _: queue.put_nowait(message))
        try:
            while True:
                yield await queue.get()
        finally:
            unsubscribe()


class AllPubSub(PubSub[TopicT, MsgT], ABC):
    @abstractmethod
    def subscribe_all(
        self,
        callback: Callable[[MsgT, TopicT], Any],
        accept: Callable[[TopicT], bool] = accept_all,
    ) -> Callable[[], None]: ...

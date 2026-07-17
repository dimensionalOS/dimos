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

from collections import defaultdict
from collections.abc import Callable
from typing import Any

from dimos.protocol.encode import encoder as encode
from dimos.protocol.pubsub.spec import PubSub


class Memory(PubSub[str, Any]):
    def __init__(self) -> None:
        self._map: defaultdict[str, list[Callable[[Any, str], None]]] = defaultdict(list)

    def publish(self, topic: str, message: Any) -> None:
        for cb in self._map[topic]:
            cb(message, topic)

    def subscribe(self, topic: str, callback: Callable[[Any, str], None]) -> Callable[[], None]:
        self._map[topic].append(callback)

        def unsubscribe() -> None:
            self._map[topic].remove(callback)
            if not self._map[topic]:
                del self._map[topic]

        return unsubscribe


class MemoryWithJSONEncoder(Memory):
    """Memory PubSub with JSON encoding/decoding."""

    def publish(self, topic: str, message: Any) -> None:
        super().publish(topic, encode.JSON.encode(message))

    def subscribe(self, topic: str, callback: Callable[[Any, str], None]) -> Callable[[], None]:
        return super().subscribe(
            topic, lambda message, name: callback(encode.JSON.decode(message), name)
        )

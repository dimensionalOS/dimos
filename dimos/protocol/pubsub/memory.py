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

from collections import defaultdict
from collections.abc import Callable
from typing import Any

from dimos.protocol import encode
from dimos.protocol.pubsub.spec import PubSub, PubSubEncoderMixin


class Memory(PubSub[str, Any]):
    def __init__(self) -> None:
        self._map: defaultdict[str, list[Callable[[Any, str], None]]] = defaultdict(list)

    def publish(self, topic: str, message: Any) -> None:
        for cb in self._map[topic]:
            cb(message, topic)

    def subscribe(self, topic: str, callback: Callable[[Any, str], None]) -> Callable[[], None]:
        self._map[topic].append(callback)

        def unsubscribe() -> None:
            try:
                self._map[topic].remove(callback)
                if not self._map[topic]:
                    del self._map[topic]
            except (KeyError, ValueError):
                pass

        return unsubscribe

    def unsubscribe(self, topic: str, callback: Callable[[Any, str], None]) -> None:
        try:
            self._map[topic].remove(callback)
            if not self._map[topic]:
                del self._map[topic]
        except (KeyError, ValueError):
            pass


class MemoryWithJSONEncoder(PubSubEncoderMixin, Memory):  # type: ignore[type-arg]
    """Memory PubSub with JSON encoding/decoding."""

    def encode(self, msg: Any, topic: str) -> bytes:
        return encode.JSON.encode(msg)

    def decode(self, msg: bytes, topic: str) -> Any:
        return encode.JSON.decode(msg)

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

from __future__ import annotations

from typing import Any, TypeVar

import dimos.core.colors as colors

T = TypeVar("T")

from typing import (
    TYPE_CHECKING,
    TypeVar,
)

from dimos.core.stream import In, Transport
from dimos.protocol.pubsub.jpeg_shm import JpegSharedMemory
from dimos.protocol.pubsub.lcmpubsub import LCM, JpegLCM, PickleLCM, Topic as LCMTopic
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory, SharedMemory

if TYPE_CHECKING:
    from collections.abc import Callable

T = TypeVar("T")  # type: ignore[misc]


class PubSubTransport(Transport[T]):
    topic: Any

    def __init__(self, topic: Any) -> None:
        self.topic = topic

    def __str__(self) -> str:
        return (
            colors.green(f"{self.__class__.__name__}(")
            + colors.blue(self.topic)
            + colors.green(")")
        )


class pLCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        self.lcm = PickleLCM(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (pLCMTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.lcm.start()
            self._started = True
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[return-value]

    def start(self) -> None: ...

    def stop(self) -> None:
        self.lcm.stop()


class LCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, type: type, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(LCMTopic(topic, type))
        if not hasattr(self, "lcm"):
            self.lcm = LCM(**kwargs)

    def start(self) -> None: ...

    def stop(self) -> None:
        self.lcm.stop()

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (LCMTransport, (self.topic.topic, self.topic.lcm_type))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.lcm.start()
            self._started = True
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[return-value]


class JpegLcmTransport(LCMTransport):  # type: ignore[type-arg]
    def __init__(self, topic: str, type: type, **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.lcm = JpegLCM(**kwargs)  # type: ignore[assignment]
        super().__init__(topic, type)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (JpegLcmTransport, (self.topic.topic, self.topic.lcm_type))

    def start(self) -> None: ...

    def stop(self) -> None:
        self.lcm.stop()


class pSHMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        self.shm = PickleSharedMemory(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (pSHMTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.shm.start()
            self._started = True

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[return-value]

    def start(self) -> None: ...

    def stop(self) -> None:
        self.shm.stop()


class SHMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        self.shm = SharedMemory(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (SHMTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.shm.start()
            self._started = True

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] | None = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[arg-type, return-value]

    def start(self) -> None: ...

    def stop(self) -> None:
        self.shm.stop()


class JpegShmTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, quality: int = 75, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        self.shm = JpegSharedMemory(quality=quality, **kwargs)
        self.quality = quality

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (JpegShmTransport, (self.topic, self.quality))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.shm.start()
            self._started = True

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] | None = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[arg-type, return-value]

    def start(self) -> None: ...

    def stop(self) -> None: ...


class ZenohTransport(PubSubTransport[T]): ...

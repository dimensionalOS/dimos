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

import traceback
from typing import TypeVar

import dimos.core.colors as colors

T = TypeVar("T")

from typing import (
    TYPE_CHECKING,
    TypeVar,
)

from dimos.core.stream import In, RemoteIn, Transport
from dimos.protocol.pubsub.jpeg_shm import JpegSharedMemory
from dimos.protocol.pubsub.lcmpubsub import LCM, JpegLCM, PickleLCM, Topic as LCMTopic
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory, SharedMemory

if TYPE_CHECKING:
    from collections.abc import Callable

T = TypeVar("T")


class PubSubTransport(Transport[T]):
    topic: any

    def __init__(self, topic: any) -> None:
        self.topic = topic

    def __str__(self) -> str:
        return (
            colors.green(f"{self.__class__.__name__}(")
            + colors.blue(self.topic)
            + colors.green(")")
        )


class pLCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:
        super().__init__(topic)
        self.lcm = PickleLCM(**kwargs)

    def __reduce__(self):
        return (pLCMTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:
        if not self._started:
            self.lcm.start()
            self._started = True
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))


class LCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, type: type, **kwargs) -> None:
        super().__init__(LCMTopic(topic, type))
        if not hasattr(self, "lcm"):
            self.lcm = LCM(**kwargs)

    def __reduce__(self):
        return (LCMTransport, (self.topic.topic, self.topic.lcm_type))

    def broadcast(self, _, msg) -> None:
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:
        if not self._started:
            self.lcm.start()
            self._started = True
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))


class JpegLcmTransport(LCMTransport):
    def __init__(self, topic: str, type: type, **kwargs):
        self.lcm = JpegLCM(**kwargs)
        super().__init__(topic, type)

    def __reduce__(self):
        return (JpegLcmTransport, (self.topic.topic, self.topic.lcm_type))


class pSHMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:
        super().__init__(topic)
        self.shm = PickleSharedMemory(**kwargs)

    def __reduce__(self):
        return (pSHMTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:
        if not self._started:
            self.shm.start()
            self._started = True

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))


class SHMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:
        super().__init__(topic)
        self.shm = SharedMemory(**kwargs)

    def __reduce__(self):
        return (SHMTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:
        if not self._started:
            self.shm.start()
            self._started = True

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))


class JpegShmTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, quality: int = 75, **kwargs):
        super().__init__(topic)
        self.shm = JpegSharedMemory(quality=quality, **kwargs)
        self.quality = quality

    def __reduce__(self):
        return (JpegShmTransport, (self.topic, self.quality))

    def broadcast(self, _, msg):
        if not self._started:
            self.shm.start()
            self._started = True

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))


class DaskTransport(Transport[T]):
    subscribers: list[Callable[[T], None]]
    _started: bool = False

    def __init__(self) -> None:
        self.subscribers = []

    def __str__(self) -> str:
        return colors.yellow("DaskTransport")

    def __reduce__(self):
        return (DaskTransport, ())

    def broadcast(self, selfstream: RemoteIn[T], msg: T) -> None:
        for subscriber in self.subscribers:
            # there is some sort of a bug here with losing worker loop
            #            print(subscriber.owner, subscriber.owner._worker, subscriber.owner._client)
            #            subscriber.owner._try_bind_worker_client()
            #            print(subscriber.owner, subscriber.owner._worker, subscriber.owner._client)

            subscriber.owner.dask_receive_msg(subscriber.name, msg).result()

    def dask_receive_msg(self, msg) -> None:
        for subscriber in self.subscribers:
            try:
                subscriber(msg)
            except Exception as e:
                print(
                    colors.red("Error in DaskTransport subscriber callback:"),
                    e,
                    traceback.format_exc(),
                )

    # for outputs
    def dask_register_subscriber(self, remoteInput: RemoteIn[T]) -> None:
        self.subscribers.append(remoteInput)

    # for inputs
    def subscribe(self, callback: Callable[[T], None], selfstream: In[T]) -> None:
        if not self._started:
            selfstream.connection.owner.dask_register_subscriber(
                selfstream.connection.name, selfstream
            ).result()
            self._started = True
        self.subscribers.append(callback)


class ZenohTransport(PubSubTransport[T]): ...

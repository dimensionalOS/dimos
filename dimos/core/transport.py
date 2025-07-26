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
from typing import Any, Callable, Generic, List, Optional, Protocol, TypeVar

import dimos.core.colors as colors

T = TypeVar("T")

import traceback
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    List,
    Optional,
    Protocol,
    TypeVar,
    get_args,
    get_origin,
    get_type_hints,
)

import dimos.core.colors as colors
from dimos.core.stream import In, Transport
from dimos.protocol.pubsub.lcmpubsub import LCM, PickleLCM
from dimos.protocol.pubsub.lcmpubsub import Topic as LCMTopic

T = TypeVar("T")


class PubSubTransport(Transport[T]):
    topic: any

    def __init__(self, topic: any):
        self.topic = topic

    def __str__(self) -> str:
        return (
            colors.green(f"{self.__class__.__name__}(")
            + colors.blue(self.topic)
            + colors.green(")")
        )


class pLCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs):
        super().__init__(topic)
        self.lcm = PickleLCM(**kwargs)

    def __reduce__(self):
        return (pLCMTransport, (self.topic,))

    def broadcast(self, _, msg):
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, selfstream: In[T], callback: Callable[[T], None]) -> None:
        if not self._started:
            self.lcm.start()
            self._started = True
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))


class LCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, type: type, **kwargs):
        super().__init__(LCMTopic(topic, type))
        self.lcm = LCM(**kwargs)

    def __reduce__(self):
        return (LCMTransport, (self.topic.topic, self.topic.lcm_type))

    def broadcast(self, _, msg):
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, selfstream: In[T], callback: Callable[[T], None]) -> None:
        if not self._started:
            self.lcm.start()
            self._started = True
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))


class DaskTransport(Transport[T]):
    subscribers: List[Callable[[T], None]]
    _started: bool = False

    def __init__(self):
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
    def subscribe(self, selfstream: In[T], callback: Callable[[T], None]) -> None:
        if not self._started:
            selfstream.connection.owner.dask_register_subscriber(
                selfstream.connection.name, selfstream
            ).result()
            self._started = True
        self.subscribers.append(callback)


class ZenohTransport(PubSubTransport[T]): ...

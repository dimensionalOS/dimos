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

from functools import reduce

# class MagnitudeTSBufferCollection
from typing import Protocol, TypeVar, Union

import reactivex as rx
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.msgs.geometry_msgs import Vector3
from dimos.types.timestamped import Timestamped, TSBufferCollection


class WithMagnitude(Protocol):
    def magnitude(self) -> float: ...


M = TypeVar("M", bound=WithMagnitude)


class MagnitudeTSBufferCollection(TSBufferCollection[M]):
    def magnitude(self) -> float:
        return (
            reduce(lambda acc, x: acc + x.magnitude(), self, 0.0) / len(self)
            if len(self) > 0
            else 0.0
        )

    def avg_magnitude(self) -> float:
        return self.magnitude() / len(self) if len(self) > 0 else 0.0


class FrameAlignment:
    collections: dict[str, TSBufferCollection]
    streams: dict[str, Observable]
    subscriptions: dict[str, any]

    def __init__(self):
        self.collections = {}
        self.streams = {}
        self.subscriptions = {}

    def add_stream(self, name: str, stream: Observable[Union[Timestamped, WithMagnitude]]):
        self.streams[name] = stream

    def get_magnitude_stream(self, name: str, window_duration: float = 1.0) -> Observable[float]:
        return rx.interval(window_duration).pipe(
            ops.map(lambda _: avg_magnitude(self, self.collections[name], window_duration))
        )

    def start(self):
        for name, stream in self.streams.items():
            self.collections[name] = TSBufferCollection(window_duration=60)
            self.subscriptions[name] = stream.pipe(ops.map(lambda x: x.magnitude())).subscribe(
                lambda x: self.collections[name].add(x)
            )

    def stop(self):
        for sub in self.subscriptions.values():
            sub.dispose()
        self.subscriptions.clear()
        self.collections.clear()

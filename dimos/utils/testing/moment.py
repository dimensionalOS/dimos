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

from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.core.resource import Resource
from dimos.utils.testing.replay import TimedSensorReplay

if TYPE_CHECKING:
    from dimos.core import Transport

T = TypeVar("T")


class SensorMoment(Generic[T], Resource):
    value: T | None = None

    def __init__(self, name: str, transport: Transport[T]) -> None:
        self.replay: TimedSensorReplay[T] = TimedSensorReplay(name)
        self.transport = transport

    def seek(self, timestamp: float) -> None:
        self.value = self.replay.find_closest_seek(timestamp)

    def publish(self) -> None:
        if self.value is not None:
            self.transport.publish(self.value)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.transport.stop()


class OutputMoment(Generic[T], Resource):
    value: T | None = None
    transport: Transport[T]

    def __init__(self, transport: Transport[T]):
        self.transport = transport

    def set(self, value: T) -> None:
        self.value = value

    def publish(self) -> None:
        if self.value is not None:
            self.transport.publish(self.value)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.transport.stop()


class Moment(Resource):
    def moments(
        self, *classes: type[SensorMoment[Any]] | type[OutputMoment[Any]]
    ) -> list[SensorMoment[Any] | OutputMoment[Any]]:
        moments: list[SensorMoment[Any] | OutputMoment[Any]] = []
        for attr_name in dir(self):
            attr_value = getattr(self, attr_name)
            if isinstance(attr_value, classes):
                moments.append(attr_value)  # type: ignore[arg-type]
        return moments

    def seekable_moments(self) -> list[SensorMoment[Any]]:
        return [m for m in self.moments(SensorMoment) if isinstance(m, SensorMoment)]

    def publishable_moments(self) -> list[SensorMoment[Any] | OutputMoment[Any]]:
        return self.moments(OutputMoment, SensorMoment)

    def seek(self, timestamp: float) -> None:
        for moment in self.seekable_moments():
            moment.seek(timestamp)

    def publish(self) -> None:
        for moment in self.publishable_moments():
            moment.publish()

    def start(self) -> None: ...

    def stop(self) -> None:
        for moment in self.publishable_moments():
            moment.stop()

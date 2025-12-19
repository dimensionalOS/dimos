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

from typing import Generic, TypeVar

from dimos.core import Transport
from dimos.core.resource import Resource
from dimos.utils.testing.replay import TimedSensorReplay

T = TypeVar("T")


class SensorMoment(Generic[T], Resource):
    value: T | None = None

    def __init__(self, name: str, transport: Transport[T]) -> None:
        self.replay = TimedSensorReplay(name)
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


class OutputMoment(Generic[T]):
    value: T | None = None
    transport: Transport[T]

    def __init__(self, transport: Transport[T]):
        self.transport = transport

    def set(self, value: T) -> None:
        self.value = value


class Moment(Resource):
    def moments(self) -> list[SensorMoment]:
        # enumerate all SensorMoment attributes set by subclasses
        moments = []
        for attr_name in dir(self):
            attr_value = getattr(self, attr_name)
            if isinstance(attr_value, (SensorMoment, OutputMoment)):
                moments.append(attr_value)
        return moments

    def seek(self, timestamp: float) -> None:
        for moment in self.moments():
            moment.seek(timestamp)

    def publish(self):
        for moment in self.moments():
            moment.publish()

    def start(self): ...

    def stop(self):
        for moment in self.moments():
            moment.stop()

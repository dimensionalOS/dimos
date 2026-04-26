# Copyright 2026 Dimensional Inc.
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

from abc import abstractmethod
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.core.resource import Resource
from dimos.memory2.registry import qual
from dimos.protocol.service.spec import BaseConfig, Configurable

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.type.observation import Observation

T = TypeVar("T")


class NotifierConfig(BaseConfig):
    pass


class Notifier(Configurable, Resource, Generic[T]):
    """Push-notification for live observation delivery.

    Decouples the notification mechanism from storage.  The built-in
    ``SubjectNotifier`` handles same-process fan-out (thread-safe, zero
    config).  External implementations (Redis pub/sub, Postgres
    LISTEN/NOTIFY, inotify) can be injected for cross-process use.
    """

    config: NotifierConfig

    def __init__(self, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    @abstractmethod
    def subscribe(self, buf: BackpressureBuffer[Observation[T]]) -> DisposableBase:
        """Register *buf* to receive new observations. Returns a
        disposable that unsubscribes when disposed."""
        ...

    @abstractmethod
    def notify(self, obs: Observation[T]) -> None:
        """Fan out *obs* to all current subscribers."""
        ...

    def serialize(self) -> dict[str, Any]:
        return {"class": qual(type(self)), "config": self.config.model_dump()}

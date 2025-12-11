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

from abc import abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Generic, TypeVar

from dimos.protocol.pubsub.lcmpubsub import PickleLCM
from dimos.protocol.service import Service  # type: ignore[attr-defined]
from dimos.protocol.skill.type import SkillMsg

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.protocol.pubsub.spec import PubSub

# defines a protocol for communication between skills and agents
# it has simple requirements of pub/sub semantics capable of sending and receiving SkillMsg objects


class SkillCommsSpec:
    @abstractmethod
    def publish(self, msg: SkillMsg) -> None: ...  # type: ignore[type-arg]

    @abstractmethod
    def subscribe(self, cb: Callable[[SkillMsg], None]) -> None: ...  # type: ignore[type-arg]

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...


MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")


@dataclass
class PubSubCommsConfig(Generic[TopicT, MsgT]):
    topic: TopicT | None = None
    pubsub: type[PubSub[TopicT, MsgT]] | PubSub[TopicT, MsgT] | None = None
    autostart: bool = True


# implementation of the SkillComms using any standard PubSub mechanism
class PubSubComms(Service[PubSubCommsConfig], SkillCommsSpec):  # type: ignore[type-arg]
    default_config: type[PubSubCommsConfig] = PubSubCommsConfig  # type: ignore[type-arg]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        pubsub_config = getattr(self.config, "pubsub", None)
        if pubsub_config is not None:
            if callable(pubsub_config):
                self.pubsub = pubsub_config()
            else:
                self.pubsub = pubsub_config
        else:
            raise ValueError("PubSub configuration is missing")

        if getattr(self.config, "autostart", True):
            self.start()

    def start(self) -> None:
        self.pubsub.start()

    def stop(self) -> None:
        self.pubsub.stop()

    def publish(self, msg: SkillMsg) -> None:  # type: ignore[type-arg]
        self.pubsub.publish(self.config.topic, msg)

    def subscribe(self, cb: Callable[[SkillMsg], None]) -> None:  # type: ignore[type-arg]
        self.pubsub.subscribe(self.config.topic, lambda msg, topic: cb(msg))


@dataclass
class LCMCommsConfig(PubSubCommsConfig[str, SkillMsg]):  # type: ignore[type-arg]
    topic: str = "/skill"
    pubsub: type[PubSub] | PubSub | None = PickleLCM  # type: ignore[type-arg]
    # lcm needs to be started only if receiving
    # skill comms are broadcast only in modules so we don't autostart
    autostart: bool = False


class LCMSkillComms(PubSubComms):
    default_config: type[LCMCommsConfig] = LCMCommsConfig

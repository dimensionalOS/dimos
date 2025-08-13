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

import time
from abc import abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Generic, Optional, TypeVar, Union

from dimos.protocol.pubsub.lcmpubsub import PickleLCM, Topic
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service import Service
from dimos.protocol.skill.types import AgentMsg, Call, MsgType, Reducer, SkillConfig, Stream
from dimos.types.timestamped import Timestamped


# defines a protocol for communication between skills and agents
class SkillCommsSpec:
    @abstractmethod
    def publish(self, msg: AgentMsg) -> None: ...

    @abstractmethod
    def subscribe(self, cb: Callable[[AgentMsg], None]) -> None: ...

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...


MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")


@dataclass
class PubSubCommsConfig(Generic[TopicT, MsgT]):
    topic: Optional[TopicT] = None  # Required field but needs default for dataclass inheritance
    pubsub: Union[type[PubSub[TopicT, MsgT]], PubSub[TopicT, MsgT], None] = None
    autostart: bool = True


class PubSubComms(Service[PubSubCommsConfig], SkillCommsSpec):
    default_config: type[PubSubCommsConfig] = PubSubCommsConfig

    def __init__(self, **kwargs) -> None:
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

    def stop(self):
        self.pubsub.stop()

    def publish(self, msg: AgentMsg) -> None:
        self.pubsub.publish(self.config.topic, msg)

    def subscribe(self, cb: Callable[[AgentMsg], None]) -> None:
        self.pubsub.subscribe(self.config.topic, lambda msg, topic: cb(msg))


@dataclass
class LCMCommsConfig(PubSubCommsConfig[str, AgentMsg]):
    topic: str = "/agent"
    pubsub: Union[type[PubSub], PubSub, None] = PickleLCM
    # lcm needs to be started only if receiving
    # skill comms are broadcast only in modules so we don't autostart
    autostart: bool = False


class LCMSkillComms(PubSubComms):
    default_config: type[LCMCommsConfig] = LCMCommsConfig

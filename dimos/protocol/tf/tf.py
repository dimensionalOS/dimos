#!/usr/bin/env python3

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
from datetime import datetime
from typing import Optional, TypeVar

import dimos_lcm
import numpy as np
import pytest

import lcm
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.tf2_msgs import TFMessage
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.lcmservice import LCMConfig, LCMService, Service

CONFIG = TypeVar("CONFIG")


# generic configuration for transform service
@dataclass
class TFConfig:
    topic: str = "/tf"
    buffer_size: float = 10.0  # seconds
    rate_limit: float = 10.0  # Hz
    autostart: bool = True


# generic specification for transform service
class TFSpec(Service[TFConfig]):
    @abstractmethod
    def send(self, *args: Transform) -> None: ...

    @abstractmethod
    def send_static(self, *args: Transform) -> None: ...

    @abstractmethod
    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: Optional[float] = None,
        time_tolerance: Optional[float] = None,
    ): ...


@dataclass
class PubSubTFConfig(TFConfig):
    pubsub: Optional[PubSub] = None


class PubSubTF:
    config: PubSubTFConfig

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.pubsub = self.config.pubsub

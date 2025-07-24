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


class TFSpec(Service[CONFIG]):
    @abstractmethod
    def send(self, *args: Transform) -> None: ...

    @abstractmethod
    def send_static(self, *args: Transform) -> None: ...

    @abstractmethod
    def lookup(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: Optional[float] = None,
        time_tolerance: Optional[float] = None,
    ): ...


@dataclass
class TFConfig(LCMConfig):
    topic: str = "/tf"
    buffer_size: float = 10.0  # seconds
    rate_limit: float = 10.0  # Hz
    autostart: bool = True


@dataclass
class GenericTFConfig(TFConfig):
    pubsub: PubSub = None


class GenericTF(TFSpec[TFConfig]):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if self.config.autostart:
            self.start()

    def start():
        self.pubsub.subscribe(self.topic, self.receive_transform)

    def receive_transform(self, msg: TFMessage, topic: Topic) -> None: ...

    def send(self, *args: Transform) -> None: ...

    def send_static(self, *args: Transform) -> None: ...

    def lookup(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: Optional[float] = None,
        time_tolerance: Optional[float] = None,
    ): ...

    def stop(): ...


# this doesn't work due to tf_lcm_py package
class TFLCM(LCMService, TFSpec[TFConfig]):
    """A service for managing and broadcasting transforms using LCM.
    This is not a separete module, You can include this in your module
    if you need to access transforms.

    Ideally we would have a generic pubsub for transforms so we are
    transport agnostic (TODO)

    For now we are not doing this because we want to use cpp buffer/lcm
    implementation. We also don't want to manually hook up tf stream
    for each module.
    """

    default_config = TFConfig

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        import tf_lcm_py as tf

        self.l = tf.LCM()
        self.buffer = tf.Buffer(self.config.buffer_size)
        self.listener = tf.TransformListener(self.l, self.buffer)
        self.broadcaster = tf.TransformBroadcaster()
        self.static_broadcaster = tf.StaticTransformBroadcaster()

        # will call the underlying LCMService.start
        self.start()

    def send(self, *args: Transform) -> None:
        for t in args:
            self.broadcaster.send_transform(t.lcm_transform())

    def send_static(self, *args: Transform) -> None:
        for t in args:
            self.static_broadcaster.send_static_transform(t)

    def lookup(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: Optional[float] = None,
        time_tolerance: Optional[float] = None,
    ):
        return self.buffer.lookup_transform(
            parent_frame,
            child_frame,
            datetime.now(),
            lcm_module=dimos_lcm,
        )

    def can_transform(
        self, parent_frame: str, child_frame: str, time_point: Optional[float | datetime] = None
    ) -> bool:
        if not time_point:
            time_point = datetime.now()

        if isinstance(time_point, float):
            time_point = datetime.fromtimestamp(time_point)

        return self.buffer.can_transform(parent_frame, child_frame, time_point)

    def get_frames(self) -> list[str]:
        return self.buffer.get_all_frame_names()

    def start(self):
        super().start()
        ...

    def stop(self): ...

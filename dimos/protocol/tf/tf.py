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
from typing import Optional, TypeVar

from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.tf2_msgs import TFMessage
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.lcmservice import Service
from dimos.types.timestamped import TimestampedCollection

CONFIG = TypeVar("CONFIG")


# generic configuration for transform service
@dataclass
class TFConfig:
    buffer_size: float = 10.0  # seconds
    rate_limit: float = 10.0  # Hz


# generic specification for transform service
class TFSpec(Service[TFConfig]):
    def __init__(self, *kwargs):
        super().__init__(*kwargs)

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

    def receive_transform(self, *args: Transform) -> None: ...

    def receive_tfmessage(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.receive_transform(transform)


MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")


# stores a single transform
class TBuffer(TimestampedCollection[Transform]):
    def __init__(self, buffer_size: float = 10.0):
        super().__init__()
        self.buffer_size = buffer_size

    def add(self, transform: Transform) -> None:
        super().add(transform)
        self._prune_old_transforms()

    def _prune_old_transforms(self) -> None:
        if not self._items:
            return

        current_time = time.time()
        cutoff_time = current_time - self.buffer_size

        while self._items and self._items[0].ts < cutoff_time:
            self._items.pop(0)

    def __str__(self) -> str:
        if not self._items:
            return "TBuffer(empty)"

        # Get unique frame info from the transforms
        frame_pairs = set()
        if self._items:
            frame_pairs.add((self._items[0].frame_id, self._items[0].child_frame_id))

        time_range = self.time_range()
        if time_range:
            start_time = time.strftime("%H:%M:%S", time.localtime(time_range[0]))
            end_time = time.strftime("%H:%M:%S", time.localtime(time_range[1]))
            duration = time_range[1] - time_range[0]

            frame_str = (
                f"{self._items[0].frame_id} -> {self._items[0].child_frame_id}"
                if self._items
                else "unknown"
            )

            return (
                f"TBuffer({len(self._items)} msgs, "
                f"{duration:.2f}s [{start_time} - {end_time}], "
                f"{frame_str})"
            )

        return f"TBuffer({len(self._items)} msgs)"


# stores multiple transform buffers
# creates a new buffer on demand when new transform is detected
class TTBuffer:
    def __init__(self, buffer_size: float = 10.0):
        self.buffers: dict[tuple[str, str], TBuffer] = {}
        self.buffer_size = buffer_size

    def receive_transform(self, *args: Transform) -> None:
        for transform in args:
            key = (transform.frame_id, transform.child_frame_id)
            if key not in self.buffers:
                self.buffers[key] = TBuffer(self.buffer_size)
            self.buffers[key].add(transform)

    def get_transform(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: Optional[float] = None,
        time_tolerance: Optional[float] = None,
    ) -> Optional[Transform]:
        key = (parent_frame, child_frame)
        if key not in self.buffers:
            return None

        buffer = self.buffers[key]

        if time_point is None:
            # Return the latest transform
            return buffer[-1] if len(buffer) > 0 else None

        # Find closest transform within tolerance
        closest = buffer.find_closest(time_point)
        if closest is None:
            return None

        if time_tolerance is not None:
            if abs(closest.ts - time_point) > time_tolerance:
                return None

        return closest

    def __str__(self) -> str:
        if not self.buffers:
            return "TTBuffer(empty)"

        lines = [f"TTBuffer({len(self.buffers)} buffers):"]
        for buffer in self.buffers.values():
            lines.append(f"  {buffer}")

        return "\n".join(lines)


@dataclass
class PubSubTFConfig(TFConfig):
    topic: TopicT = None  # Required field but needs default for dataclass inheritance
    pubsub: Optional[PubSub[TopicT, MsgT]] = None


class PubSubTF(TTBuffer, TFSpec):
    config: PubSubTFConfig

    def __init__(self, **kwargs) -> None:
        TFSpec.__init__(self, **kwargs)
        TTBuffer.__init__(self, self.config.buffer_size)
        self.pubsub = self.config.pubsub

    def start(self):
        self.pubsub.start()
        self.pubsub.subscribe(self.config.topic, self.receive_msg)

    def send(self, *args: Transform) -> None:
        """Send transforms using the configured PubSub."""
        if not self.pubsub:
            raise ValueError("PubSub is not configured.")

        self.pubsub.publish(self.config.topic, TFMessage(*args))

    def send_static(self, *args: Transform) -> None:
        raise NotImplementedError("Static transforms not implemented in PubSubTF.")

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: Optional[float] = None,
        time_tolerance: Optional[float] = None,
    ) -> Optional[Transform]:
        return self.get_transform(parent_frame, child_frame, time_point, time_tolerance)

    def receive_msg(self, channel: str, data: bytes) -> None:
        msg = TFMessage.lcm_decode(data)
        self.receive_tfmessage(msg)

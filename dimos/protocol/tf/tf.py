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

from abc import abstractmethod
from collections import deque
from dataclasses import dataclass, field
from functools import reduce
from typing import TypeVar

from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.tf2_msgs import TFMessage
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.lcmservice import Service  # type: ignore[attr-defined]
from dimos.types.timestamped import TimestampedCollection

CONFIG = TypeVar("CONFIG")


# generic configuration for transform service
@dataclass
class TFConfig:
    buffer_size: float = 10.0  # seconds
    rate_limit: float = 10.0  # Hz


# generic specification for transform service
class TFSpec(Service[TFConfig]):
    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)

    @abstractmethod
    def publish(self, *args: Transform) -> None: ...

    @abstractmethod
    def publish_static(self, *args: Transform) -> None: ...

    def get_frames(self) -> set[str]:
        return set()

    @abstractmethod
    def get(  # type: ignore[no-untyped-def]
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ): ...

    def receive_transform(self, *args: Transform) -> None: ...

    def receive_tfmessage(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.receive_transform(transform)


MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")


# stores a single transform
class TBuffer(TimestampedCollection[Transform]):
    def __init__(self, buffer_size: float = 10.0) -> None:
        super().__init__()
        self.buffer_size = buffer_size

    def add(self, transform: Transform) -> None:
        super().add(transform)
        self._prune_old_transforms(transform.ts)

    def _prune_old_transforms(self, current_time) -> None:  # type: ignore[no-untyped-def]
        if not self._items:
            return

        cutoff_time = current_time - self.buffer_size

        while self._items and self._items[0].ts < cutoff_time:
            self._items.pop(0)

    def get(self, time_point: float | None = None, time_tolerance: float = 1.0) -> Transform | None:
        """Get transform at specified time or latest if no time given."""
        if time_point is None:
            # Return the latest transform
            return self[-1] if len(self) > 0 else None

        return self.find_closest(time_point, time_tolerance)

    def __str__(self) -> str:
        if not self._items:
            return "TBuffer(empty)"

        # Get unique frame info from the transforms
        frame_pairs = set()
        if self._items:
            frame_pairs.add((self._items[0].frame_id, self._items[0].child_frame_id))

        time_range = self.time_range()
        if time_range:
            from dimos.types.timestamped import to_human_readable

            start_time = to_human_readable(time_range[0])
            end_time = to_human_readable(time_range[1])
            duration = time_range[1] - time_range[0]

            frame_str = (
                f"{self._items[0].frame_id} -> {self._items[0].child_frame_id}"
                if self._items
                else "unknown"
            )

            return (
                f"TBuffer("
                f"{frame_str}, "
                f"{len(self._items)} msgs, "
                f"{duration:.2f}s [{start_time} - {end_time}])"
            )

        return f"TBuffer({len(self._items)} msgs)"


# stores multiple transform buffers
# creates a new buffer on demand when new transform is detected
class MultiTBuffer:
    def __init__(self, buffer_size: float = 10.0) -> None:
        self.buffers: dict[tuple[str, str], TBuffer] = {}
        self.buffer_size = buffer_size

    def receive_transform(self, *args: Transform) -> None:
        for transform in args:
            key = (transform.frame_id, transform.child_frame_id)
            if key not in self.buffers:
                self.buffers[key] = TBuffer(self.buffer_size)
            self.buffers[key].add(transform)

    def get_frames(self) -> set[str]:
        frames = set()
        for parent, child in self.buffers:
            frames.add(parent)
            frames.add(child)
        return frames

    def get_connections(self, frame_id: str) -> set[str]:
        """Get all frames connected to the given frame (both as parent and child)."""
        connections = set()
        for parent, child in self.buffers:
            if parent == frame_id:
                connections.add(child)
            if child == frame_id:
                connections.add(parent)
        return connections

    def get_transform(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        # Check forward direction
        key = (parent_frame, child_frame)
        if key in self.buffers:
            return self.buffers[key].get(time_point, time_tolerance)  # type: ignore[arg-type]

        # Check reverse direction and return inverse
        reverse_key = (child_frame, parent_frame)
        if reverse_key in self.buffers:
            transform = self.buffers[reverse_key].get(time_point, time_tolerance)  # type: ignore[arg-type]
            return transform.inverse() if transform else None

        return None

    def get(self, *args, **kwargs) -> Transform | None:  # type: ignore[no-untyped-def]
        simple = self.get_transform(*args, **kwargs)
        if simple is not None:
            return simple

        complex = self.get_transform_search(*args, **kwargs)

        if complex is None:
            return None

        return reduce(lambda t1, t2: t1 + t2, complex)

    def get_transform_search(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> list[Transform] | None:
        """Search for shortest transform chain between parent and child frames using BFS."""
        # Check if direct transform exists (already checked in get_transform, but for clarity)
        direct = self.get_transform(parent_frame, child_frame, time_point, time_tolerance)
        if direct is not None:
            return [direct]

        # BFS to find shortest path
        queue: deque[tuple[str, list[Transform]]] = deque([(parent_frame, [])])
        visited = {parent_frame}

        while queue:
            current_frame, path = queue.popleft()

            if current_frame == child_frame:
                return path

            # Get all connections for current frame
            connections = self.get_connections(current_frame)

            for next_frame in connections:
                if next_frame not in visited:
                    visited.add(next_frame)

                    # Get the transform between current and next frame
                    transform = self.get_transform(
                        current_frame, next_frame, time_point, time_tolerance
                    )
                    if transform:
                        queue.append((next_frame, [*path, transform]))

        return None

    def graph(self) -> str:
        import subprocess

        def connection_str(connection: tuple[str, str]) -> str:
            (frame_from, frame_to) = connection
            return f"{frame_from} -> {frame_to}"

        graph_str = "\n".join(map(connection_str, self.buffers.keys()))

        try:
            result = subprocess.run(
                ["diagon", "GraphDAG", "-style=Unicode"],
                input=graph_str,
                capture_output=True,
                text=True,
            )
            return result.stdout if result.returncode == 0 else graph_str
        except Exception:
            return "no diagon installed"

    def __str__(self) -> str:
        if not self.buffers:
            return f"{self.__class__.__name__}(empty)"

        lines = [f"{self.__class__.__name__}({len(self.buffers)} buffers):"]
        for buffer in self.buffers.values():
            lines.append(f"  {buffer}")

        return "\n".join(lines)


@dataclass
class PubSubTFConfig(TFConfig):
    topic: Topic | None = None  # Required field but needs default for dataclass inheritance
    pubsub: type[PubSub] | PubSub | None = None  # type: ignore[type-arg]
    autostart: bool = True


class PubSubTF(MultiTBuffer, TFSpec):
    default_config: type[PubSubTFConfig] = PubSubTFConfig

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        TFSpec.__init__(self, **kwargs)
        MultiTBuffer.__init__(self, self.config.buffer_size)

        pubsub_config = getattr(self.config, "pubsub", None)
        if pubsub_config is not None:
            if callable(pubsub_config):
                self.pubsub = pubsub_config()
            else:
                self.pubsub = pubsub_config
        else:
            raise ValueError("PubSub configuration is missing")

        if self.config.autostart:  # type: ignore[attr-defined]
            self.start()

    def start(self, sub: bool = True) -> None:
        self.pubsub.start()
        if sub:
            topic = getattr(self.config, "topic", None)
            if topic:
                self.pubsub.subscribe(topic, self.receive_msg)

    def stop(self) -> None:
        self.pubsub.stop()

    def publish(self, *args: Transform) -> None:
        """Send transforms using the configured PubSub."""
        if not self.pubsub:
            raise ValueError("PubSub is not configured.")

        self.receive_transform(*args)
        topic = getattr(self.config, "topic", None)
        if topic:
            self.pubsub.publish(topic, TFMessage(*args))

    def publish_static(self, *args: Transform) -> None:
        raise NotImplementedError("Static transforms not implemented in PubSubTF.")

    def publish_all(self) -> None:
        """Publish all transforms currently stored in all buffers."""
        all_transforms = []
        for buffer in self.buffers.values():
            # Get the latest transform from each buffer
            latest = buffer.get()  # get() with no args returns latest
            if latest:
                all_transforms.append(latest)

        if all_transforms:
            self.publish(*all_transforms)

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        return super().get(parent_frame, child_frame, time_point, time_tolerance)

    def receive_msg(self, msg: TFMessage, topic: Topic) -> None:
        self.receive_tfmessage(msg)


@dataclass
class LCMPubsubConfig(PubSubTFConfig):
    topic: Topic = field(default_factory=lambda: Topic("/tf", TFMessage))
    pubsub: type[PubSub] | PubSub | None = LCM  # type: ignore[type-arg]
    autostart: bool = True


class LCMTF(PubSubTF):
    default_config: type[LCMPubsubConfig] = LCMPubsubConfig


TF = LCMTF

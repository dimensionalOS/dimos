#!/usr/bin/env python3

# Copyright 2025-2026 Dimensional Inc.
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
from dataclasses import field
from functools import reduce
import threading
import time
from typing import TYPE_CHECKING, Protocol, runtime_checkable

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.spec import BaseConfig, Service
from dimos.types.timestamped import to_human_readable
from dimos.utils.logging_config import setup_logger
from dimos.utils.timeseries.inmemory import InMemoryStore

logger = setup_logger()


@runtime_checkable
class TFLookup(Protocol):
    """Read side of a tf service: resolve ``parent ← child`` at a time point.

    Satisfied by the live services (:class:`MultiTBuffer`, :class:`PubSubTF`)
    and by replay backends like ``dimos.memory2.tf.StreamTF``. Code that only
    queries transforms should accept this instead of a concrete service.
    """

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None: ...


# generic configuration for transform service
class TFConfig(BaseConfig):
    buffer_size: float = 10.0  # seconds
    rate_limit: float = 10.0  # Hz


# generic specification for transform service
class TFSpec(Service):
    config: TFConfig

    @abstractmethod
    def publish(self, *args: Transform) -> None: ...

    @abstractmethod
    def publish_static(self, *args: Transform) -> None: ...

    def get_frames(self) -> set[str]:
        return set()

    @abstractmethod
    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
    ) -> Transform | None: ...

    def get_pose(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
    ) -> PoseStamped | None:
        tf = self.get(
            parent_frame,
            child_frame,
            time_point,
            time_tolerance,
            forward_tolerance=forward_tolerance,
        )
        if not tf:
            return None
        return tf.to_pose()

    def receive_transform(self, *args: Transform) -> None: ...

    def receive_tfmessage(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.receive_transform(transform)


class TBuffer(InMemoryStore[Transform]):
    def __init__(self, buffer_size: float = 10.0) -> None:
        super().__init__()
        self.buffer_size = buffer_size

    def add(self, transform: Transform) -> None:
        self.save(transform)
        self.prune_old(transform.ts - self.buffer_size)

    def get(self, time_point: float | None = None, time_tolerance: float = 1.0) -> Transform | None:
        """Get transform at specified time or latest if no time given."""
        if time_point is None:
            return self.last()
        return self.find_closest(time_point, time_tolerance)

    def __str__(self) -> str:
        if len(self) == 0:
            return "TBuffer(empty)"

        first_item = self.first()
        time_range = self.time_range()
        if time_range and first_item:
            start_time = to_human_readable(time_range[0])
            end_time = to_human_readable(time_range[1])
            duration = time_range[1] - time_range[0]

            frame_str = f"{first_item.frame_id} -> {first_item.child_frame_id}"

            return (
                f"TBuffer("
                f"{frame_str}, "
                f"{len(self)} msgs, "
                f"{duration:.2f}s [{start_time} - {end_time}])"
            )

        return f"TBuffer({len(self)} msgs)"


# stores multiple transform buffers
# creates a new buffer on demand when new transform is detected
class MultiTBuffer:
    def __init__(self, buffer_size: float = 10.0) -> None:
        self.buffers: dict[tuple[str, str], TBuffer] = {}
        self.buffer_size = buffer_size
        self._statics: dict[tuple[str, str], Transform] = {}
        self._cv = threading.Condition()

    def receive_transform(self, *args: Transform) -> None:
        with self._cv:
            for transform in args:
                key = (transform.frame_id, transform.child_frame_id)
                if key not in self.buffers:
                    self.buffers[key] = TBuffer(self.buffer_size)
                self.buffers[key].add(transform)
            # add() prunes old transforms; backfill to keep statics from being pruned
            # TODO: replace with latching topics
            self._backfill_statics()
            self._cv.notify_all()

    def receive_static_transform(self, *args: Transform) -> None:
        with self._cv:
            for transform in args:
                key = (transform.frame_id, transform.child_frame_id)
                if key not in self.buffers:
                    self.buffers[key] = TBuffer(self.buffer_size)
                self.buffers[key].save(transform)
                self._statics[key] = transform
            self._cv.notify_all()

    def _backfill_statics(self) -> None:
        """Re-add static transforms to buffers if pruning removed them."""
        for key, transform in self._statics.items():
            buffer = self.buffers.get(key)
            if buffer is None or len(buffer) == 0:
                if buffer is None:
                    self.buffers[key] = TBuffer(self.buffer_size)
                self.buffers[key].save(transform)

    def get_frames(self) -> set[str]:
        frames = set()
        with self._cv:
            for parent, child in self.buffers:
                frames.add(parent)
                frames.add(child)
        return frames

    def get_connections(self, frame_id: str) -> set[str]:
        """Get all frames connected to the given frame (both as parent and child)."""
        connections = set()
        with self._cv:
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
        if parent_frame == child_frame:
            return Transform(
                frame_id=parent_frame,
                child_frame_id=child_frame,
                ts=time_point if time_point is not None else time.time(),
            )

        # No explicit tolerance means "anything still buffered" — the buffer
        # holds at most buffer_size seconds, so that is the effective reach.
        tolerance = time_tolerance if time_tolerance is not None else self.buffer_size

        with self._cv:
            key = (parent_frame, child_frame)
            reverse_key = (child_frame, parent_frame)

            # Check statics first (no tolerance needed)
            if key in self._statics:
                return self._statics[key]
            if reverse_key in self._statics:
                return self._statics[reverse_key].inverse()

            # Check dynamic buffers
            if key in self.buffers:
                return self.buffers[key].get(time_point, tolerance)
            if reverse_key in self.buffers:
                transform = self.buffers[reverse_key].get(time_point, tolerance)
                return transform.inverse() if transform else None

            return None

    def _get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        with self._cv:
            simple = self.get_transform(parent_frame, child_frame, time_point, time_tolerance)

            if simple is not None:
                return simple

            complex = self.get_transform_search(
                parent_frame, child_frame, time_point, time_tolerance
            )

            if complex is None:
                return None

            return reduce(lambda t1, t2: t1 + t2, complex)

    def _wait_get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None,
        time_tolerance: float | None,
        forward_tolerance: float,
    ) -> Transform | None:
        deadline = time.monotonic() + forward_tolerance
        with self._cv:
            while True:
                result = self._get(parent_frame, child_frame, time_point, time_tolerance)
                if result is not None:
                    return result
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cv.wait(timeout=remaining)

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
    ) -> Transform | None:
        result = self._get(parent_frame, child_frame, time_point, time_tolerance)
        if result is None and forward_tolerance > 0:
            result = self._wait_get(
                parent_frame, child_frame, time_point, time_tolerance, forward_tolerance
            )
        if result is None:
            logger.warning(
                f"No direct transform found between '{parent_frame}' and '{child_frame}' at '{to_human_readable(time_point or time.time())}'"
            )
        return result

    def get_transform_search(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> list[Transform] | None:
        """Search for shortest transform chain between parent and child frames using BFS."""
        with self._cv:
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

    @property
    def tree_str(self) -> str:
        with self._cv:
            keys = list(self.buffers.keys())

        if not keys:
            return "(empty)"

        children: dict[str, list[str]] = {}
        all_children: set[str] = set()
        for parent, child in keys:
            children.setdefault(parent, []).append(child)
            all_children.add(child)

        roots = [frame for frame in children if frame not in all_children]
        if not roots:
            roots = sorted(children.keys())[:1]

        for root in roots:
            children.setdefault(root, [])
        for frame_list in children.values():
            frame_list.sort()

        lines: list[str] = []

        def walk(frame: str, prefix: str, is_last: bool, is_root: bool) -> None:
            connector = "" if is_root else ("└── " if is_last else "├── ")
            lines.append(f"{prefix}{connector}{frame}")
            child_prefix = prefix if is_root else prefix + ("    " if is_last else "│   ")
            kids = children.get(frame, [])
            for index, kid in enumerate(kids):
                walk(kid, child_prefix, index == len(kids) - 1, False)

        for index, root in enumerate(sorted(roots)):
            if index > 0:
                lines.append("")
            walk(root, "", index == len(roots) - 1, True)

        return "\n".join(lines)

    def graph(self) -> str:
        import subprocess

        def connection_str(connection: tuple[str, str]) -> str:
            (frame_from, frame_to) = connection
            return f"{frame_from} -> {frame_to}"

        with self._cv:
            keys = list(self.buffers.keys())
        graph_str = "\n".join(map(connection_str, keys))

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
        with self._cv:
            buffers = list(self.buffers.values())

        if not buffers:
            return f"{self.__class__.__name__}(empty)"

        lines = [f"{self.__class__.__name__}({len(buffers)} buffers):"]
        for buffer in buffers:
            lines.append(f"  {buffer}")

        return "\n".join(lines)


class PubSubTFConfig(TFConfig):
    topic: Topic | None = None  # Required field but needs default for dataclass inheritance
    static_topic: Topic | None = None
    pubsub: type[PubSub] | PubSub | None = None  # type: ignore[type-arg]
    autostart: bool = True


class PubSubTF(MultiTBuffer, TFSpec):
    config: PubSubTFConfig

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

        if self.config.autostart:
            self.start()

    def start(self, sub: bool = True) -> None:
        self.pubsub.start()
        if sub:
            topic = getattr(self.config, "topic", None)
            if topic:
                self.pubsub.subscribe(topic, self.receive_msg)
            static_topic = getattr(self.config, "static_topic", None)
            if static_topic:
                self.pubsub.subscribe(static_topic, self._receive_static_msg)

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
        self.receive_static_transform(*args)
        static_topic = getattr(self.config, "static_topic", None)
        if static_topic:
            self.pubsub.publish(static_topic, TFMessage(*args))

    def publish_all(self) -> None:
        """Publish all transforms currently stored in all buffers."""
        dynamic = []
        static = []
        with self._cv:
            for key, buffer in self.buffers.items():
                latest = buffer.get()  # get() with no args returns latest
                if latest:
                    if key in self._statics:
                        static.append(latest)
                    else:
                        dynamic.append(latest)

        if dynamic:
            self.publish(*dynamic)
        if static:
            self.publish_static(*static)

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
    ) -> Transform | None:
        return super().get(
            parent_frame,
            child_frame,
            time_point,
            time_tolerance,
            forward_tolerance=forward_tolerance,
        )

    def receive_msg(self, msg: TFMessage, topic: Topic) -> None:
        self.receive_tfmessage(msg)

    def _receive_static_msg(self, msg: TFMessage, topic: Topic) -> None:
        self.receive_static_transform(*msg.transforms)


class LCMPubsubConfig(PubSubTFConfig):
    topic: Topic = field(default_factory=lambda: Topic("/tf", TFMessage))
    static_topic: Topic = field(default_factory=lambda: Topic("/tf_static", TFMessage))
    pubsub: type[PubSub] | PubSub | None = LCM  # type: ignore[type-arg]
    autostart: bool = True


class LCMTF(PubSubTF):
    config: LCMPubsubConfig


class ZenohPubsubConfig(PubSubTFConfig):
    # Zenoh key expressions can't start with '/'; namespace under 'dimos'.
    topic: Topic = field(default_factory=lambda: Topic("dimos/tf", TFMessage))
    pubsub: type[PubSub] | PubSub | None = Zenoh  # type: ignore[type-arg]
    autostart: bool = True


class ZenohTF(PubSubTF):
    config: ZenohPubsubConfig


TF = LCMTF

if TYPE_CHECKING:
    # mypy conformance checks: the live services satisfy the read-side protocol.
    _lookup_impls: tuple[type[TFLookup], ...] = (MultiTBuffer, PubSubTF)

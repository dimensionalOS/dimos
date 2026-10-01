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

from collections import deque
from copy import copy
from functools import reduce
import math
import subprocess
import threading
import time
from typing import TYPE_CHECKING, Any, Protocol, cast, runtime_checkable

from dimos_generated.geometry_msgs.msg import PoseStamped, TransformStamped
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from sortedcontainers import SortedDict  # type: ignore[import-untyped]

from dimos.msgs.geometry import compose_transforms, inverse_transform, pose_from_transform
from dimos.msgs.time import time_from_seconds, to_nanoseconds, to_seconds
from dimos.types.timestamped import to_human_readable
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()


@runtime_checkable
class TFLookup(Protocol):
    """Read side of a tf buffer: resolve ``parent ← child`` at a time point.

    Satisfied by the live buffers (:class:`MultiTBuffer`, :class:`TF`) and by
    replay backends like ``dimos.memory.tf.StreamTF``. Code that only queries
    transforms should accept this instead of a concrete implementation.
    """

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> TransformStamped | None: ...


class TBuffer:
    """A bounded time index of generated transforms, keyed by integer nanoseconds."""

    def __init__(self, buffer_size: float = 10.0) -> None:
        if math.isnan(buffer_size) or buffer_size <= 0:
            raise ValueError("buffer_size must be positive")
        self.buffer_size = buffer_size
        self._entries: SortedDict = SortedDict()

    def __len__(self) -> int:
        return len(self._entries)

    def add(self, transform: TransformStamped) -> None:
        stamp = to_nanoseconds(transform.header.stamp)
        self._entries[stamp] = TransformStamped(
            header=transform.header,
            child_frame_id=transform.child_frame_id,
            transform=transform.transform,
        )
        if math.isfinite(self.buffer_size):
            oldest = self._entries.peekitem(-1)[0] - round(self.buffer_size * 1_000_000_000)
            while self._entries and self._entries.peekitem(0)[0] < oldest:
                self._entries.popitem(0)

    def first(self) -> TransformStamped | None:
        return copy(self._entries.peekitem(0)[1]) if self._entries else None

    def last(self) -> TransformStamped | None:
        return copy(self._entries.peekitem(-1)[1]) if self._entries else None

    def get(
        self, time_point: float | None = None, time_tolerance: float = 1.0
    ) -> TransformStamped | None:
        if time_point is None:
            return self.last()
        stamp = to_nanoseconds(time_from_seconds(time_point))
        if math.isnan(time_tolerance) or time_tolerance < 0:
            raise ValueError("time_tolerance must be nonnegative")
        index = self._entries.bisect_left(stamp)
        candidates = [
            self._entries.peekitem(i) for i in (index - 1, index) if 0 <= i < len(self._entries)
        ]
        if not candidates:
            return None
        nearest, result = min(candidates, key=lambda item: (abs(item[0] - stamp), -item[0]))
        if abs(nearest - stamp) / 1_000_000_000 > time_tolerance:
            return None
        return copy(cast("TransformStamped", result))

    def __str__(self) -> str:
        first, last = self.first(), self.last()
        if first is None or last is None:
            return "TBuffer(empty)"
        start, end = to_seconds(first.header.stamp), to_seconds(last.header.stamp)
        return (
            f"TBuffer({first.header.frame_id} -> {first.child_frame_id}, "
            f"{len(self)} msgs, {end - start:.2f}s "
            f"[{to_human_readable(start)} - {to_human_readable(end)}])"
        )


# stores multiple transform buffers
# creates a new buffer on demand when new transform is detected
class MultiTBuffer:
    def __init__(self, buffer_size: float = 10.0) -> None:
        self.buffers: dict[tuple[str, str], TBuffer] = {}
        self.buffer_size = buffer_size
        self._cv = threading.Condition()

    def receive_transform(self, *args: TransformStamped) -> None:
        with self._cv:
            for transform in args:
                key = (transform.header.frame_id, transform.child_frame_id)
                if key not in self.buffers:
                    self.buffers[key] = TBuffer(self.buffer_size)
                self.buffers[key].add(transform)
            self._cv.notify_all()

    def receive_tfmessage(self, msg: TFMessage) -> None:
        self.receive_transform(*msg.transforms)

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
    ) -> TransformStamped | None:
        if parent_frame == child_frame:
            return TransformStamped(
                header=Header(
                    frame_id=parent_frame,
                    stamp=time_from_seconds(time_point if time_point is not None else time.time()),
                ),
                child_frame_id=child_frame,
            )

        # No explicit tolerance means "anything still buffered" — the buffer
        # holds at most buffer_size seconds, so that is the effective reach.
        tolerance = time_tolerance if time_tolerance is not None else self.buffer_size

        with self._cv:
            # Check forward direction
            key = (parent_frame, child_frame)
            if key in self.buffers:
                return self.buffers[key].get(time_point, tolerance)

            # Check reverse direction and return inverse
            reverse_key = (child_frame, parent_frame)
            if reverse_key in self.buffers:
                transform = self.buffers[reverse_key].get(time_point, tolerance)
                return inverse_transform(transform) if transform else None

            return None

    def _get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> TransformStamped | None:
        with self._cv:
            simple = self.get_transform(parent_frame, child_frame, time_point, time_tolerance)

            if simple is not None:
                return simple

            complex = self.get_transform_search(
                parent_frame, child_frame, time_point, time_tolerance
            )

            if complex is None:
                return None

            return reduce(compose_transforms, complex)

    def _wait_get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None,
        time_tolerance: float | None,
        forward_tolerance: float,
    ) -> TransformStamped | None:
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
        warn: bool = True,
    ) -> TransformStamped | None:
        result = self._get(parent_frame, child_frame, time_point, time_tolerance)
        if result is None and forward_tolerance > 0:
            result = self._wait_get(
                parent_frame, child_frame, time_point, time_tolerance, forward_tolerance
            )
        if result is None and warn:
            logger.warning(
                f"No direct transform found between '{parent_frame}' and '{child_frame}' at '{to_human_readable(time_point or time.time())}'"
            )
        return result

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
        return pose_from_transform(tf)

    def get_transform_search(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> list[TransformStamped] | None:
        """Search for shortest transform chain between parent and child frames using BFS."""
        with self._cv:
            # Check if direct transform exists (already checked in get_transform, but for clarity)
            direct = self.get_transform(parent_frame, child_frame, time_point, time_tolerance)
            if direct is not None:
                return [direct]

            # BFS to find shortest path
            queue: deque[tuple[str, list[TransformStamped]]] = deque([(parent_frame, [])])
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


class TF(MultiTBuffer):
    """Transform buffer over a tf stream — a disposable tf view.

    Wraps anything that speaks ``TFMessage`` through ``subscribe``/``publish``
    — typically a module's ``tf: IO[TFMessage]`` port, but a raw ``Transport``
    works too. Received messages fill the buffer for ``get()`` lookups;
    ``publish()`` stores locally and sends out on the stream. Without a stream
    it is a purely local buffer.
    """

    def __init__(self, stream: Any | None = None, buffer_size: float = 10.0) -> None:
        super().__init__(buffer_size)
        self._stream = stream
        self._unsubscribe: Callable[[], None] | None = None
        if stream is not None:
            self._unsubscribe = stream.subscribe(self.receive_tfmessage)

    def publish(self, *transforms: TransformStamped) -> None:
        self.receive_transform(*transforms)
        if self._stream is not None:
            self._stream.publish(TFMessage(transforms=list(transforms)))

    def dispose(self) -> None:
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None


if TYPE_CHECKING:
    # mypy conformance checks: the live buffers satisfy the read-side protocol.
    _lookup_impls: tuple[type[TFLookup], ...] = (MultiTBuffer, TF)

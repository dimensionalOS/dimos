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

"""Typed per-message transforms, composable with any transport.

A transform is a callable ``(A) -> B | None`` applied to the typed message
before it reaches the wire; returning ``None`` drops the message. Transforms
are plain picklable values (frozen dataclasses), so a blueprint can pin them
per stream and they survive into worker processes like the transports they
wrap:

    blueprint.transports({
        ("lidar", PointCloud2): TransformTransport(
            ZenohTransport("lidar", PointCloud2), VoxelDownsample(0.05)
        ),
        ("color_image", Image): TransformTransport(
            WebRTCTransport("color_image", Image), Throttle(5.0), ResizeImage(640, 480)
        ),
    })

This keeps bandwidth policy (decimation, throttling) out of the individual
transport backends: the same transform runs unchanged over LCM, Zenoh, SHM,
ROS, or WebRTC, and the wire layer only ever sees the reduced message.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import time
from typing import TYPE_CHECKING, Any, TypeVar

from dimos.core.stream import Out, Stream, Transport

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

A = TypeVar("A")

# A transform maps a typed message to a (possibly different) message, or None
# to drop it. Must be picklable — module-level functions or dataclasses, not
# lambdas — because transports (and their transforms) pickle into workers.
MsgTransform = Callable[[Any], Any]


@dataclass(frozen=True)
class VoxelDownsample:
    """Voxel-grid downsample a PointCloud2 before it reaches the wire."""

    voxel_size: float = 0.1

    def __call__(self, msg: PointCloud2) -> PointCloud2:
        return msg.voxel_downsample(self.voxel_size)


@dataclass(frozen=True)
class ResizeImage:
    """Scale an Image down to fit the given bounds (aspect ratio preserved)."""

    max_width: int = 640
    max_height: int = 480

    def __call__(self, msg: Image) -> Image:
        resized, _ = msg.resize_to_fit(self.max_width, self.max_height)
        return resized


@dataclass
class Throttle:
    """Drop messages arriving faster than ``max_hz``, keeping the first of
    each interval. Per-process state: after pickling into a worker the rate
    cap restarts there, which is the correct behavior for a publisher-side
    cap."""

    max_hz: float
    _last: float = field(default=0.0, repr=False, compare=False)

    def __call__(self, msg: A) -> A | None:
        now = time.monotonic()
        if now - self._last < 1.0 / self.max_hz:
            return None
        self._last = now
        return msg


class TransformTransport(Transport[A]):
    """Applies transforms, in order, before the wrapped transport broadcasts.

    A transform returning None drops the message. Subscribing and lifecycle
    are pass-through, so this composes with any Transport backend.
    """

    def __init__(self, transport: Transport[Any], *transforms: MsgTransform) -> None:
        self._transport = transport
        self._transforms = transforms

    def broadcast(self, selfstream: Out[A] | None, value: A) -> None:
        out: Any = value
        for transform in self._transforms:
            out = transform(out)
            if out is None:
                return
        self._transport.broadcast(selfstream, out)

    def subscribe(
        self, callback: Callable[[A], Any], selfstream: Stream[A] | None = None
    ) -> Callable[[], None]:
        return self._transport.subscribe(callback, selfstream)

    def start(self) -> None:
        self._transport.start()

    def stop(self) -> None:
        self._transport.stop()

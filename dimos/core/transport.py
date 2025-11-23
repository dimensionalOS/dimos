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

"""Pub/sub transports for streaming data between modules.

This module provides transport implementations that connect module stream
endpoints (`In[T]` and `Out[T]`). Transports handle underlying protocol
details, allowing modules to communicate without knowing whether data travels
via shared memory, LCM multicast, or other mechanisms.

Transport categories:

- **Network-capable** (LCM variants): For distributed systems where modules
  may run on different machines. Supports serialization and multicast.
- **Local-only** (SHM variants): For high-throughput communication between
  processes on the same machine. Uses shared memory for near-zero-copy transfer.

Selection guidance:

- `LCMTransport`: Default for typed messages with `lcm_encode` support
- `pLCMTransport`: Python objects without LCM encoding, network-capable
- `JpegLcmTransport`: Images over network with compression
- `pSHMTransport`: High-throughput local data (images, point clouds)
- `SHMTransport`: Raw bytes over shared memory
- `JpegShmTransport`: Local images with reduced memory footprint

Example:
    Configure transports via `.transports()` on blueprint sets:

        from dimos.core.blueprints import autoconnect
        from dimos.core.transport import LCMTransport, pSHMTransport
        from dimos.msgs.sensor_msgs import Image

        blueprint = autoconnect(
            connection(),
            perception(),
        ).transports({
            # Key: (stream_property_name, Type)
            ("color_image", Image): LCMTransport("/robot/camera", Image),
        })

All transports lazily initialize on first `broadcast()` or `subscribe()`
call. For the abstract interface, see `Transport` in `stream.py`.
"""

from __future__ import annotations

from typing import Annotated, Any, TypeVar

from annotated_doc import Doc

import dimos.core.colors as colors

T = TypeVar("T")

from typing import (
    TYPE_CHECKING,
    TypeVar,
)

from dimos.core.stream import In, Transport
from dimos.protocol.pubsub.jpeg_shm import JpegSharedMemory
from dimos.protocol.pubsub.lcmpubsub import LCM, JpegLCM, PickleLCM, Topic as LCMTopic
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory, SharedMemory

if TYPE_CHECKING:
    from collections.abc import Callable

T = TypeVar("T")  # type: ignore[misc]


class PubSubTransport(Transport[T]):
    """Topic-based publish-subscribe transport.

    Extends `Transport` with a topic attribute for routing messages. Use this
    when modules need to communicate over named channels rather than direct
    point-to-point connections.

    The topic serves as a logical address that publishers broadcast to and
    subscribers listen on, enabling many-to-many communication patterns.
    """

    topic: Any

    def __init__(self, topic: Any) -> None:
        self.topic = topic

    def __str__(self) -> str:
        return (
            colors.green(f"{self.__class__.__name__}(")
            + colors.blue(self.topic)
            + colors.green(")")
        )


class pLCMTransport(PubSubTransport[T]):
    """LCM (Lightweight Communications and Marshalling) transport with pickle serialization for arbitrary Python objects.

    Uses UDP multicast via LCM for low-latency pub/sub messaging across processes
    and machines. The "p" prefix indicates pickle serialization.

    Use when you need network messaging with Python objects that lack `lcm_encode`
    support. For native LCM types, prefer `LCMTransport` (faster, cross-language).

    See also:
        LCMTransport: Native LCM encoding (faster, cross-language).
        pSHMTransport: Pickle over shared memory (single-machine only).
    """

    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        self.lcm = PickleLCM(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (pLCMTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.lcm.start()
            self._started = True
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[return-value]


class LCMTransport(PubSubTransport[T]):
    """Publish-subscribe transport using LCM (Lightweight Communications and Marshalling) native encoding over UDP multicast.

    Default transport for typed messages (poses, images, point clouds, sensor readings)
    that need to be shared across network boundaries. Uses LCM's native serialization
    rather than pickle, enabling cross-language interoperability but requiring message
    types that implement `lcm_encode`/`lcm_decode` methods.

    For pickle-based serialization (Python-only, any type), use `pLCMTransport`.

    More on LCM:
      - It's a publish-subscribe messaging system that uses UDP multicast for its underlying transport.
      - "provides a best-effort packet delivery mechanism and gives strong preference to the expedient delivery of recent messages" (LCM paper)

    Further reading
      - [The LCM paper](https://people.csail.mit.edu/albert/pubs/2010-huang-olson-moore-lcm-iros.pdf)
    """

    _started: bool = False

    def __init__(
        self,
        topic: Annotated[str, Doc("Channel name for message routing.")],
        type: Annotated[
            type,
            Doc(
                """LCM message type (must have `lcm_encode`/`lcm_decode` methods,
                typically auto-generated from `.lcm` schema files)."""
            ),
        ],
        **kwargs: Annotated[Any, Doc("Passed to the underlying LCM instance.")],
    ) -> None:
        super().__init__(LCMTopic(topic, type))
        if not hasattr(self, "lcm"):
            self.lcm = LCM(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (LCMTransport, (self.topic.topic, self.topic.lcm_type))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.lcm.start()
            self._started = True
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[return-value]


class JpegLcmTransport(LCMTransport):  # type: ignore[type-arg]
    """LCM transport with JPEG compression for image transmission over networks.

    Reduces bandwidth when transmitting images across network boundaries.

    Trade-offs:
        - Lower bandwidth via compression
        - Lossy compression (some quality loss)
        - CPU overhead for encode/decode

    See also:
        LCMTransport: Uncompressed LCM transport for general data.
        JpegShmTransport: JPEG compression over shared memory (same-machine).
    """

    def __init__(self, topic: str, type: type, **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.lcm = JpegLCM(**kwargs)  # type: ignore[assignment]
        super().__init__(topic, type)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (JpegLcmTransport, (self.topic.topic, self.topic.lcm_type))


class pSHMTransport(PubSubTransport[T]):
    """Local-only transport using POSIX shared memory with pickle serialization.

    Provides high-throughput, low-latency communication between processes on the
    same machine. Data is shared via memory-mapped regions rather than copied
    over network sockets, making this ideal for large payloads like camera frames
    or point clouds.

    Unlike network-capable transports (`pLCMTransport`, `LCMTransport`), this cannot
    communicate across machines. Use `pLCMTransport` instead when network distribution is
    needed.
    """

    _started: bool = False

    def __init__(
        self,
        topic: Annotated[str, Doc("Channel identifier for publish/subscribe routing.")],
        **kwargs: Annotated[
            Any,
            Doc(
                """Passed to PickleSharedMemory. Key option:
                default_capacity: Max payload size in bytes (default ~3.5MB).
                This should be increased for very large data."""
            ),
        ],
    ) -> None:
        super().__init__(topic)
        self.shm = PickleSharedMemory(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (pSHMTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.shm.start()
            self._started = True

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[return-value]


class SHMTransport(PubSubTransport[T]):
    """Shared memory transport for raw bytes data.

    Uses POSIX shared memory with minimal encoding overhead, providing high
    throughput for local inter-process communication when data is already
    in bytes format. Unlike `pSHMTransport`, which pickle-serializes Python
    objects, this transport expects bytes-like data (bytes, bytearray,
    or memoryview).

    Use this transport when:
        - Processes run on the same machine
        - Data is already bytes-like (e.g., sensor buffers, encoded frames)
        - Maximum throughput is critical

    Use `pSHMTransport` instead when you need to send arbitrary Python objects.
    """

    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        self.shm = SharedMemory(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (SHMTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.shm.start()
            self._started = True

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[arg-type, return-value]


class JpegShmTransport(PubSubTransport[T]):
    """Shared memory transport with JPEG compression for Image objects.

    Uses shared memory for fast local inter-process communication while applying
    JPEG compression to reduce memory footprint. Only works for local consumers
    (same machine); not suitable for network transport.

    Trade-offs:
        - Lower memory usage than uncompressed shared memory
        - Adds CPU overhead for encode/decode
        - Lossy compression (quality parameter controls fidelity vs size)
    """

    _started: bool = False

    def __init__(
        self,
        topic: Annotated[str, Doc("Channel identifier for pub/sub routing.")],
        quality: Annotated[
            int,
            Doc(
                """JPEG compression quality (1-100). Lower values produce smaller
                images with more artifacts."""
            ),
        ] = 75,
        **kwargs: Annotated[
            Any, Doc("Additional arguments passed to the underlying shared memory.")
        ],
    ) -> None:
        super().__init__(topic)
        self.shm = JpegSharedMemory(quality=quality, **kwargs)
        self.quality = quality

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (JpegShmTransport, (self.topic, self.quality))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.shm.start()
            self._started = True

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[arg-type, return-value]


class ZenohTransport(PubSubTransport[T]): ...

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

from __future__ import annotations

from typing import Any, TypeVar

import dimos.core.colors as colors

T = TypeVar("T")

from typing import (
    TYPE_CHECKING,
    TypeVar,
)

from dimos.core.stream import In, Out, Stream, Transport
from dimos.protocol.pubsub.jpeg_shm import JpegSharedMemory
from dimos.protocol.pubsub.lcmpubsub import LCM, JpegLCM, PickleLCM, Topic as LCMTopic
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory, SharedMemory
from dimos.protocol.pubsub.zenohpubsub import (
    CongestionControl,
    PickleZenoh,
    Priority,
    Reliability,
    Zenoh,
    ZenohConfig,
    ZenohQoS,
    normalize_topic,
)

if TYPE_CHECKING:
    from collections.abc import Callable

T = TypeVar("T")  # type: ignore[misc]


class PubSubTransport(Transport[T]):
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
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        self.lcm = PickleLCM(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (pLCMTransport, (self.topic,))

    def broadcast(self, _: Out[T] | None, msg: T) -> None:
        if not self._started:
            self.lcm.start()
            self._started = True

        self.lcm.publish(self.topic, msg)

    def subscribe(
        self, callback: Callable[[T], Any], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        if not self._started:
            self.lcm.start()
            self._started = True
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))

    def start(self) -> None: ...

    def stop(self) -> None:
        self.lcm.stop()


class LCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, type: type, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(LCMTopic(topic, type))
        if not hasattr(self, "lcm"):
            self.lcm = LCM(**kwargs)

    def start(self) -> None: ...

    def stop(self) -> None:
        self.lcm.stop()

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
    def __init__(self, topic: str, type: type, **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.lcm = JpegLCM(**kwargs)  # type: ignore[assignment]
        super().__init__(topic, type)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (JpegLcmTransport, (self.topic.topic, self.topic.lcm_type))

    def start(self) -> None: ...

    def stop(self) -> None:
        self.lcm.stop()


class pSHMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
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

    def start(self) -> None: ...

    def stop(self) -> None:
        self.shm.stop()


class SHMTransport(PubSubTransport[T]):
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

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] | None = None) -> None:  # type: ignore[override]
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[arg-type, return-value]

    def start(self) -> None: ...

    def stop(self) -> None:
        self.shm.stop()


class JpegShmTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, quality: int = 75, **kwargs) -> None:  # type: ignore[no-untyped-def]
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

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] | None = None) -> None:  # type: ignore[override]
        if not self._started:
            self.shm.start()
            self._started = True
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[arg-type, return-value]

    def start(self) -> None: ...

    def stop(self) -> None: ...


class ZenohTransport(PubSubTransport[T]):
    """Zenoh-based transport with configurable QoS.

    Drop-in replacement for LCMTransport with additional QoS options.
    Uses pickle serialization by default for maximum compatibility.

    Example:
        # Basic usage (pickle serialization, reliable QoS)
        transport = ZenohTransport("/cmd_vel")

        # With QoS configuration
        transport = ZenohTransport(
            "/lidar",
            qos=ZenohQoS(
                reliability=Reliability.BEST_EFFORT,
                congestion_control=CongestionControl.DROP,
            ),
        )

        # With typed encoding (uses zenoh_encode/lcm_encode)
        transport = ZenohTransport("/pose", msg_type=PoseStamped)
    """

    _started: bool = False

    def __init__(
        self,
        topic: str,
        msg_type: type[T] | None = None,
        qos: ZenohQoS | None = None,
        config: ZenohConfig | None = None,
        **kwargs,  # type: ignore[no-untyped-def]
    ) -> None:
        super().__init__(topic)
        self.qos = qos or ZenohQoS()
        self.config = config or ZenohConfig()
        self.msg_type = msg_type

        # Use typed Zenoh if msg_type provided, else pickle
        if msg_type is not None:
            self.zenoh: Zenoh | PickleZenoh = Zenoh(
                msg_type=msg_type,
                config=self.config,
                qos=self.qos,
                **kwargs,
            )
        else:
            self.zenoh = PickleZenoh(
                config=self.config,
                qos=self.qos,
                **kwargs,
            )

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (
            ZenohTransport,
            (self.topic, self.msg_type),
            {"qos": self.qos, "config": self.config},
        )

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        self.qos = state.get("qos", ZenohQoS())
        self.config = state.get("config", ZenohConfig())

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.zenoh.start()
            self._started = True

        self.zenoh.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] | None = None) -> None:  # type: ignore[override]
        if not self._started:
            self.zenoh.start()
            self._started = True

        return self.zenoh.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[return-value]

    def start(self) -> None:
        if not self._started:
            self.zenoh.start()
            self._started = True

    def stop(self) -> None:
        if self._started:
            self.zenoh.stop()
            self._started = False


class pZenohTransport(PubSubTransport[T]):
    """Pickle-based Zenoh transport for arbitrary Python objects.

    Convenience wrapper that always uses pickle serialization.
    Equivalent to ZenohTransport without msg_type.

    Example:
        transport = pZenohTransport("/data")
        transport.broadcast(None, {"any": "python", "object": 123})
    """

    _started: bool = False

    def __init__(
        self,
        topic: str,
        qos: ZenohQoS | None = None,
        config: ZenohConfig | None = None,
        **kwargs,  # type: ignore[no-untyped-def]
    ) -> None:
        super().__init__(topic)
        self.qos = qos or ZenohQoS()
        self.config = config or ZenohConfig()
        self.zenoh = PickleZenoh(
            config=self.config,
            qos=self.qos,
            **kwargs,
        )

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (pZenohTransport, (self.topic,))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.zenoh.start()
            self._started = True

        self.zenoh.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] | None = None) -> None:  # type: ignore[override]
        if not self._started:
            self.zenoh.start()
            self._started = True

        return self.zenoh.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[return-value]

    def start(self) -> None:
        if not self._started:
            self.zenoh.start()
            self._started = True

    def stop(self) -> None:
        if self._started:
            self.zenoh.stop()
            self._started = False

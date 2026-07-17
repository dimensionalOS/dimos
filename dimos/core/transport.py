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

from collections.abc import Callable
from typing import Any, TypeVar, cast

from dimos.core.stream import Transport
from dimos.msgs.protocol import DimosMsg
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, PickleLCM
from dimos.protocol.pubsub.impl.rospubsub import DimosROS, ROSTopic
from dimos.protocol.pubsub.impl.shmpubsub import BytesSharedMemory, PickleSharedMemory
from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig
from dimos.protocol.pubsub.impl.webrtc.providers.spec import ProviderConfig
from dimos.protocol.pubsub.impl.webrtc.webrtcpubsub import WebRTCPubSub
from dimos.protocol.pubsub.impl.zenohpubsub import PickleZenoh, Zenoh
from dimos.protocol.pubsub.topic import Topic
from dimos.utils import colors

try:
    import cyclonedds as _cyclonedds  # noqa: F401

    DDS_AVAILABLE = True
except ImportError:
    DDS_AVAILABLE = False

T = TypeVar("T")
M = TypeVar("M", bound=DimosMsg)


class PubSubTransport(Transport[T]):
    def __init__(self, topic: Any, pubsub: Any, bus_topic: Any = None) -> None:
        self.topic = topic
        self.pubsub = pubsub
        self._bus_topic = topic if bus_topic is None else bus_topic
        self._started = False

    def __str__(self) -> str:
        return colors.green(f"{type(self).__name__}(") + colors.blue(self.topic) + colors.green(")")

    @property
    def channel(self) -> str:
        return str(self._bus_topic)

    @classmethod
    def spec(cls, *args: Any, **kwargs: Any) -> Any:
        from dimos.core.coordination.blueprints import TransportSpec

        return TransportSpec(cls, args, kwargs)

    def start(self) -> None:
        self.pubsub.start()
        self._started = True

    def stop(self) -> None:
        if self._started:
            self.pubsub.stop()
            self._started = False

    def publish(self, message: T) -> None:
        if not self._started:
            self.start()
        self.pubsub.publish(self._bus_topic, message)

    def subscribe(self, callback: Callable[[T], Any]) -> Callable[[], None]:
        if not self._started:
            self.start()
        return cast(
            "Callable[[], None]",
            self.pubsub.subscribe(self._bus_topic, lambda message, _: callback(message)),
        )


class pLCMTransport(PubSubTransport[T]):
    def __init__(self, topic: str, **kwargs: Any) -> None:
        self.lcm = PickleLCM(**kwargs)
        super().__init__(topic, self.lcm, Topic(topic))

    def __reduce__(self) -> tuple[Any, ...]:
        return (type(self), (self.topic,))


class LCMTransport(PubSubTransport[T]):
    lcm: Any

    def __init__(self, topic: str, type: type, **kwargs: Any) -> None:
        self.lcm = LCM(**kwargs)
        super().__init__(Topic(topic, type), self.lcm)

    def __reduce__(self) -> tuple[Any, ...]:
        return (type(self), (self.topic.topic, self.topic.lcm_type))


class JpegLcmTransport(LCMTransport[Any]):
    def __init__(self, topic: str, type: type, **kwargs: Any) -> None:
        from dimos.protocol.pubsub.impl.jpeg_lcm import JpegLCM

        self.lcm = JpegLCM(**kwargs)
        PubSubTransport.__init__(self, Topic(topic, type), self.lcm)


def _rebuild_shm(cls: Callable[..., Any], topic: str, capacity: int) -> PubSubTransport[Any]:
    return cast("PubSubTransport[Any]", cls(topic, default_capacity=capacity))


class pSHMTransport(PubSubTransport[T]):
    def __init__(self, topic: str, **kwargs: Any) -> None:
        self.shm = PickleSharedMemory(**kwargs)
        super().__init__(topic, self.shm)

    def __reduce__(self) -> tuple[Any, ...]:
        return (_rebuild_shm, (type(self), self.topic, self.shm.config.default_capacity))


class SHMTransport(PubSubTransport[T]):
    def __init__(self, topic: str, **kwargs: Any) -> None:
        self.shm = BytesSharedMemory(**kwargs)
        super().__init__(topic, self.shm)

    def __reduce__(self) -> tuple[Any, ...]:
        return (_rebuild_shm, (type(self), self.topic, self.shm.config.default_capacity))


class JpegShmTransport(PubSubTransport[T]):
    def __init__(self, topic: str, quality: int = 75, **kwargs: Any) -> None:
        from dimos.protocol.pubsub.impl.jpeg_shm import JpegSharedMemory

        self.shm = JpegSharedMemory(quality=quality, **kwargs)
        self.quality = quality
        super().__init__(topic, self.shm)

    def __reduce__(self) -> tuple[Any, ...]:
        return (type(self), (self.topic, self.quality))


class ROSTransport(PubSubTransport[DimosMsg]):
    def __init__(self, topic: str, msg_type: type[DimosMsg], **kwargs: Any) -> None:
        self._kwargs = kwargs
        super().__init__(ROSTopic(topic, msg_type), None)

    def __reduce__(self) -> tuple[Any, ...]:
        return (type(self), (self.topic.topic, self.topic.msg_type))

    def start(self) -> None:
        self.pubsub = DimosROS(**self._kwargs)
        super().start()

    def stop(self) -> None:
        super().stop()
        self.pubsub = None


if DDS_AVAILABLE:
    from dimos.protocol.pubsub.impl.ddspubsub import DDS, Topic as DDSTopic

    class DDSTransport(PubSubTransport[T]):
        def __init__(self, topic: str, type: type, **kwargs: Any) -> None:
            self.dds = DDS(**kwargs)
            super().__init__(DDSTopic(topic, type), self.dds)


def _rebuild_webrtc_transport(
    cls: type[WebRTCTransport[M]], topic: str, msg_type: type[M] | None, config: ProviderConfig
) -> WebRTCTransport[M]:
    return cls(topic, msg_type, config=config)


class WebRTCTransport(PubSubTransport[M]):
    _config_cls: type[ProviderConfig]

    def __init__(
        self,
        topic: str,
        msg_type: type[M] | None = None,
        *,
        config: ProviderConfig | None = None,
        **config_kwargs: Any,
    ) -> None:
        self._msg_type = msg_type
        self._config = config or self._config_cls(**config_kwargs)
        super().__init__(topic, None)

    def __reduce__(self) -> tuple[Any, ...]:
        return (_rebuild_webrtc_transport, (type(self), self.topic, self._msg_type, self._config))

    def start(self) -> None:
        self.pubsub = WebRTCPubSub(provider=self._config.provider())
        super().start()

    def stop(self) -> None:
        self._started = False

    def publish(self, message: M) -> None:
        if not self._started:
            self.start()
        data = message.lcm_encode() if self._msg_type is not None else message
        self.pubsub.publish(self.topic, data)

    def subscribe(self, callback: Callable[[M], Any]) -> Callable[[], None]:
        if not self._started:
            self.start()
        if self._msg_type is None:
            unsubscribe: Callable[[], None] = self.pubsub.subscribe(
                self.topic, lambda message, _: callback(cast("M", message))
            )
            return unsubscribe
        msg_type = self._msg_type

        def receive(data: bytes, _: str) -> None:
            try:
                callback(cast("M", msg_type.lcm_decode(data)))
            except ValueError:
                return

        unsubscribe = self.pubsub.subscribe(self.topic, receive)
        return unsubscribe


class CloudflareTransport(WebRTCTransport[M]):
    _config_cls = BrokerConfig


class WebRTCVideoTransport(Transport[Any]):
    _config_cls: type[ProviderConfig]

    def __init__(self, *, config: ProviderConfig | None = None, **config_kwargs: Any) -> None:
        self._config = config or self._config_cls(**config_kwargs)

    @classmethod
    def spec(cls, *args: Any, **kwargs: Any) -> Any:
        from dimos.core.coordination.blueprints import TransportSpec

        return TransportSpec(cls, args, kwargs)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def publish(self, message: Any) -> None:
        provider = self._config.provider()
        if not provider.is_connected:
            provider.start()
        provider.set_video_frame(message)  # type: ignore[attr-defined]

    def subscribe(self, callback: Callable[[Any], Any]) -> Callable[[], None]:
        return lambda: None


class CloudflareVideoTransport(WebRTCVideoTransport):
    _config_cls = BrokerConfig


class ZenohTransport(PubSubTransport[T]):
    def __init__(self, topic: str | Topic, type: type | None = None, **kwargs: Any) -> None:
        ztopic = Topic(topic, type) if isinstance(topic, str) else topic
        self.zenoh = Zenoh(**kwargs)
        super().__init__(ztopic, self.zenoh)

    @property
    def channel(self) -> str:
        return str(self.topic.key_expr)

    @property
    def publish_qos(self) -> dict[str, str] | None:
        return self.topic.qos.to_wire() if self.topic.qos is not None else None

    def __reduce__(self) -> tuple[Any, ...]:
        return (type(self), (self.topic,))


class pZenohTransport(PubSubTransport[T]):
    def __init__(self, topic: str | Topic, **kwargs: Any) -> None:
        self._zenoh_topic = Topic(topic) if isinstance(topic, str) else topic
        self.zenoh = PickleZenoh(**kwargs)
        super().__init__(self._zenoh_topic.pattern, self.zenoh, self._zenoh_topic)

    @property
    def channel(self) -> str:
        return self._zenoh_topic.key_expr

    @property
    def publish_qos(self) -> dict[str, str] | None:
        qos = self._zenoh_topic.qos
        return qos.to_wire() if qos is not None else None

    def __reduce__(self) -> tuple[Any, ...]:
        return (type(self), (self._zenoh_topic,))

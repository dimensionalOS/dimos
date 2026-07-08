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

import functools
import threading
from typing import (
    TYPE_CHECKING,
    Any,
    TypeVar,
    cast,
)

from dimos.core.stream import In, Out, Stream, Transport
from dimos.msgs.protocol import DimosMsg
from dimos.utils import colors

try:
    import cyclonedds as _cyclonedds  # noqa: F401

    DDS_AVAILABLE = True
except ImportError:
    DDS_AVAILABLE = False

from dimos.protocol.pubsub.impl.lcmpubsub import LCM, PickleLCM, Topic as LCMTopic
from dimos.protocol.pubsub.impl.rospubsub import DimosROS, ROSTopic
from dimos.protocol.pubsub.impl.shmpubsub import BytesSharedMemory, PickleSharedMemory
from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig
from dimos.protocol.pubsub.impl.webrtc.providers.spec import ProviderConfig
from dimos.protocol.pubsub.impl.webrtc.webrtcpubsub import WebRTCPubSub
from dimos.protocol.pubsub.impl.zenohpubsub import (
    PickleZenoh,
    Topic as ZenohTopic,
    Zenoh,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.core.coordination.blueprints import TransportSpec

T = TypeVar("T")

# TODO
# Transports need to be rewritten and simplified,
#
# there is no need for them to get a reference to "a stream" on publish/subscribe calls
# this is a legacy from dask transports.
#
# new transport should literally have 2 functions (next to start/stop)
# "send(msg)" and "receive(callback)" and that's all
#
# we can also consider pubsubs conforming directly to Transport specs
# and removing PubSubTransport glue entirely
#
# Why not ONLY pubsubs without Transport abstraction?
#
# General idea for transports (and why they exist at all)
# is that they can be * anything * like
#
# a web camera rtsp stream for Image, audio stream from mic, etc
# http binary streams, tcp connections etc


class PubSubTransport(Transport[T]):
    topic: Any

    def __init__(self, topic: Any) -> None:
        self.topic = topic

    @classmethod
    def spec(cls, *args: Any, **kwargs: Any) -> TransportSpec:
        """Defer construction: capture ctor args for the coordinator to build later."""
        from dimos.core.coordination.blueprints import TransportSpec

        return TransportSpec(cls, args, kwargs)

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
            self.start()

        self.lcm.publish(self.topic, msg)

    def subscribe(
        self, callback: Callable[[T], Any], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        if not self._started:
            self.start()
        return self.lcm.subscribe(LCMTopic(self.topic), lambda msg, topic: callback(msg))

    def start(self) -> None:
        self.lcm.start()
        self._started = True

    def stop(self) -> None:
        self.lcm.stop()
        self._started = False


class LCMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, type: type, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(LCMTopic(topic, type))
        if not hasattr(self, "lcm"):
            self.lcm = LCM(**kwargs)

    def start(self) -> None:
        self.lcm.start()
        self._started = True

    def stop(self) -> None:
        self.lcm.stop()
        self._started = False

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (LCMTransport, (self.topic.topic, self.topic.lcm_type))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.start()

        self.lcm.publish(self.topic, msg)

    def subscribe(
        self, callback: Callable[[T], Any], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        if not self._started:
            self.start()
        return self.lcm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[arg-type]


class JpegLcmTransport(LCMTransport):  # type: ignore[type-arg]
    def __init__(self, topic: str, type: type, **kwargs) -> None:  # type: ignore[no-untyped-def]
        from dimos.protocol.pubsub.impl.jpeg_lcm import (
            JpegLCM,
        )  # ~330ms: deferred to avoid pulling in Image/cv2/rerun

        self.lcm = JpegLCM(**kwargs)  # type: ignore[assignment]
        super().__init__(topic, type)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (JpegLcmTransport, (self.topic.topic, self.topic.lcm_type))

    def start(self) -> None:
        self.lcm.start()
        self._started = True

    def stop(self) -> None:
        self.lcm.stop()
        self._started = False


class pSHMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        self.shm = PickleSharedMemory(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (
            functools.partial(pSHMTransport, default_capacity=self.shm.config.default_capacity),
            (self.topic,),
        )

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.start()

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] = None) -> None:  # type: ignore[assignment, override]
        if not self._started:
            self.start()
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[return-value]

    def start(self) -> None:
        self.shm.start()
        self._started = True

    def stop(self) -> None:
        self.shm.stop()
        self._started = False


class SHMTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        self.shm = BytesSharedMemory(**kwargs)

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (
            functools.partial(SHMTransport, default_capacity=self.shm.config.default_capacity),
            (self.topic,),
        )

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.start()

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] | None = None) -> None:  # type: ignore[override]
        if not self._started:
            self.start()
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[arg-type, return-value]

    def start(self) -> None:
        self.shm.start()
        self._started = True

    def stop(self) -> None:
        self.shm.stop()
        self._started = False


class JpegShmTransport(PubSubTransport[T]):
    _started: bool = False

    def __init__(self, topic: str, quality: int = 75, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(topic)
        from dimos.protocol.pubsub.impl.jpeg_shm import (
            JpegSharedMemory,
        )  # deferred to avoid pulling in Image/cv2/rerun

        self.shm = JpegSharedMemory(quality=quality, **kwargs)
        self.quality = quality

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (JpegShmTransport, (self.topic, self.quality))

    def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
        if not self._started:
            self.start()

        self.shm.publish(self.topic, msg)

    def subscribe(self, callback: Callable[[T], None], selfstream: In[T] | None = None) -> None:  # type: ignore[override]
        if not self._started:
            self.start()
        return self.shm.subscribe(self.topic, lambda msg, topic: callback(msg))  # type: ignore[arg-type, return-value]

    def start(self) -> None:
        self.shm.start()
        self._started = True

    def stop(self) -> None:
        self.shm.stop()
        self._started = False


class ROSTransport(PubSubTransport[DimosMsg]):
    _ros: DimosROS | None = None

    def __init__(self, topic: str, msg_type: type[DimosMsg], **kwargs: Any) -> None:
        super().__init__(ROSTopic(topic, msg_type))
        self._kwargs = kwargs

    def __reduce__(self) -> tuple[Any, ...]:
        return (ROSTransport, (self.topic.topic, self.topic.msg_type))

    def broadcast(self, _: Out[DimosMsg] | None, msg: DimosMsg) -> None:
        if self._ros is None:
            self.start()
            assert self._ros is not None  # for type narrowing
        self._ros.publish(self.topic, msg)

    def subscribe(
        self, callback: Callable[[DimosMsg], Any], selfstream: Stream[DimosMsg] | None = None
    ) -> Callable[[], None]:
        if self._ros is None:
            self.start()
            assert self._ros is not None  # for type narrowing
        return self._ros.subscribe(self.topic, lambda msg, topic: callback(msg))

    def start(self) -> None:
        if self._ros is None:
            self._ros = DimosROS(**self._kwargs)
            self._ros.start()

    def stop(self) -> None:
        if self._ros is not None:
            self._ros.stop()
            self._ros = None


if DDS_AVAILABLE:
    from dimos.protocol.pubsub.impl.ddspubsub import DDS, Topic as DDSTopic

    class DDSTransport(PubSubTransport[T]):
        def __init__(self, topic: str, type: type, **kwargs) -> None:  # type: ignore[no-untyped-def]
            super().__init__(DDSTopic(topic, type))
            self.dds = DDS(**kwargs)
            self._started: bool = False
            self._start_lock = threading.RLock()

        def start(self) -> None:
            with self._start_lock:
                if not self._started:
                    self.dds.start()
                    self._started = True

        def stop(self) -> None:
            with self._start_lock:
                if self._started:
                    self.dds.stop()
                    self._started = False

        def broadcast(self, _, msg) -> None:  # type: ignore[no-untyped-def]
            if not self._started:
                self.start()
            self.dds.publish(self.topic, msg)

        def subscribe(
            self, callback: Callable[[T], None], selfstream: Stream[T] | None = None
        ) -> Callable[[], None]:
            if not self._started:
                self.start()
            return self.dds.subscribe(self.topic, lambda msg, topic: callback(msg))


M = TypeVar("M", bound=DimosMsg)


def _rebuild_webrtc_transport(
    cls: type[WebRTCTransport[M]], topic: str, msg_type: type[M] | None, config: ProviderConfig
) -> WebRTCTransport[M]:
    return cls(topic, msg_type, config=config)


class WebRTCTransport(PubSubTransport[M]):
    """Transport over WebRTC DataChannels.

    Subclasses bind a backend by setting ``_config_cls``; the base class can
    also be used directly with an explicit ``config``. Two modes:

    * **Raw bytes** (``msg_type=None``): messages pass through as ``bytes``.
    * **Typed LCM** (``msg_type=SomeMsg``): LCM-encoded on ``broadcast()``,
      LCM-decoded on ``subscribe()`` (foreign types on the shared channel are skipped) — so multiple
      transports sharing one multiplexed DataChannel each receive only
      their own message type.

    The transport itself holds no connection: the picklable ``config``
    resolves to a per-process singleton provider on first use, so transports
    survive being pickled into module worker processes and all transports in
    a process share one session. ``stop()`` intentionally leaves the shared
    provider running (it is process-scoped).
    """

    _config_cls: type[ProviderConfig]
    _config: ProviderConfig
    _started: bool = False

    def __init__(
        self,
        topic: str,
        msg_type: type[M] | None = None,
        *,
        config: ProviderConfig | None = None,
        **config_kwargs: Any,
    ) -> None:
        super().__init__(topic)
        self._msg_type = msg_type
        self._config = config or self._config_cls(**config_kwargs)
        self._pubsub: WebRTCPubSub | None = None
        # Guards first-use init: concurrent subscribe()/broadcast() must not
        # construct two WebRTCPubSub wrappers (one would silently orphan any
        # subscribe_all state). Never pickled — __reduce__ rebuilds via the
        # constructor.
        self._init_lock = threading.Lock()

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (_rebuild_webrtc_transport, (type(self), self.topic, self._msg_type, self._config))

    def broadcast(self, _: Out[M] | None, msg: M) -> None:
        if not self._started:
            self.start()
        assert self._pubsub is not None
        data = msg.lcm_encode() if self._msg_type is not None else msg
        self._pubsub.publish(self.topic, data)  # type: ignore[arg-type]

    def subscribe(
        self, callback: Callable[[M], None], selfstream: Stream[M] | None = None
    ) -> Callable[[], None]:
        if not self._started:
            self.start()
        assert self._pubsub is not None

        if self._msg_type is not None:
            msg_type = self._msg_type

            def _typed_cb(data: bytes, _topic: str) -> None:
                # The channel is multiplexed (e.g. the browser sends Twists and
                # Poses on cmd_unreliable); lcm_decode verifies the wire
                # fingerprint and raises on other types — skip those.
                try:
                    msg = msg_type.lcm_decode(data)
                except ValueError:
                    return
                callback(msg)  # type: ignore[arg-type]

            return self._pubsub.subscribe(self.topic, _typed_cb)
        return self._pubsub.subscribe(self.topic, lambda msg, _topic: callback(msg))  # type: ignore[arg-type]

    def start(self) -> None:
        with self._init_lock:
            if self._pubsub is None:
                self._pubsub = WebRTCPubSub(provider=self._config.provider())
            self._pubsub.start()
            self._started = True

    def stop(self) -> None:
        self._started = False


class CloudflareTransport(WebRTCTransport[M]):
    """WebRTC via the hosted teleop broker + Cloudflare Realtime SFU.

    Config kwargs flow into :class:`BrokerConfig`; unset fields fall back to
    the blueprint config flow (``-o transports.broker.<field>=...`` or the
    ``TRANSPORTS__BROKER__<FIELD>=...`` env form).

    Blueprint usage::

        unitree_go2_hosted = unitree_go2_basic.transports({
            ("cmd_vel", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("color_image", Image): CloudflareVideoTransport.spec(),
        })
    """

    _config_cls = BrokerConfig


class WebRTCVideoTransport(Transport[Any]):
    """Robot camera → remote viewer as a WebRTC video track (provider-agnostic).

    ``broadcast()`` feeds each Image into the shared provider's sendonly media
    track — the same provider/PeerConnection the DataChannel transports use
    (identical config resolves to the same per-process singleton). Session
    negotiation of the track is the provider's job; any provider exposing
    ``set_video_frame()`` works. The remote side consumes RTP (e.g. the teleop
    web client pulling the track), so there is nothing to ``subscribe()`` to
    locally and subscribers get a no-op.

    Subclasses bind a backend by setting ``_config_cls``; the base class can
    also be used directly with an explicit ``config``.
    """

    _config_cls: type[ProviderConfig]
    _config: ProviderConfig

    def __init__(self, *, config: ProviderConfig | None = None, **config_kwargs: Any) -> None:
        self._config = config or self._config_cls(**config_kwargs)

    @classmethod
    def spec(cls, *args: Any, **kwargs: Any) -> TransportSpec:
        """Defer construction: capture ctor args for the coordinator to build later."""
        from dimos.core.coordination.blueprints import TransportSpec

        return TransportSpec(cls, args, kwargs)

    def start(self) -> None:
        pass  # provider starts lazily on first broadcast

    def stop(self) -> None:
        pass  # shared provider is process-scoped (see WebRTCTransport.stop)

    def broadcast(self, _: Out[Any] | None, msg: Any) -> None:
        provider = self._config.provider()
        set_frame = getattr(provider, "set_video_frame", None)
        if set_frame is None:
            raise NotImplementedError(f"{type(provider).__name__} does not support media tracks")
        if not provider.is_connected:
            provider.start()
        set_frame(msg)

    def subscribe(
        self, callback: Callable[[Any], None], selfstream: Stream[Any] | None = None
    ) -> Callable[[], None]:
        logger.warning(
            "%s is publish-only on the robot; local subscriber gets no frames",
            type(self).__name__,
        )
        return lambda: None


class CloudflareVideoTransport(WebRTCVideoTransport):
    """Camera → teleop web client via the hosted broker (see WebRTCVideoTransport)."""

    _config_cls = BrokerConfig


class ZenohTransport(PubSubTransport[T]):
    """Zenoh transport with LCM encoding for typed DimosMsg.

    Accepts either a plain topic string plus message type, or a full
    `ZenohTopic` carrying per-topic settings: `ZenohTransport(ZenohTopic("bla",
    Image, qos=...))`.
    """

    _started: bool = False

    def __init__(self, topic: str | ZenohTopic, type: type | None = None, **kwargs: Any) -> None:
        if isinstance(topic, str):
            topic = ZenohTopic(topic, type)
        super().__init__(topic)
        self.zenoh = Zenoh(**kwargs)
        self._start_lock = threading.RLock()

    def __reduce__(self) -> tuple[Any, ...]:
        return (ZenohTransport, (self.topic,))

    def start(self) -> None:
        with self._start_lock:
            if not self._started:
                self.zenoh.start()
                self._started = True

    def stop(self) -> None:
        with self._start_lock:
            if self._started:
                self.zenoh.stop()
                self._started = False

    def broadcast(self, _: Out[T] | None, msg: T) -> None:
        if not self._started:
            self.start()
        self.zenoh.publish(self.topic, cast("DimosMsg", msg))

    def subscribe(
        self, callback: Callable[[T], None], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        if not self._started:
            self.start()
        return self.zenoh.subscribe(self.topic, lambda msg, topic: callback(cast("T", msg)))


class pZenohTransport(PubSubTransport[T]):
    """Zenoh transport with pickle encoding for arbitrary Python objects.

    Accepts either a plain topic string or a full `ZenohTopic` carrying
    per-topic settings (QoS). `self.topic` stays the plain string.
    """

    _started: bool = False

    def __init__(self, topic: str | ZenohTopic, **kwargs: Any) -> None:
        self._zenoh_topic = ZenohTopic(topic) if isinstance(topic, str) else topic
        super().__init__(self._zenoh_topic.pattern)
        self.zenoh = PickleZenoh(**kwargs)
        self._start_lock = threading.RLock()

    def __reduce__(self) -> tuple[Any, ...]:
        return (pZenohTransport, (self._zenoh_topic,))

    def start(self) -> None:
        with self._start_lock:
            if not self._started:
                self.zenoh.start()
                self._started = True

    def stop(self) -> None:
        with self._start_lock:
            if self._started:
                self.zenoh.stop()
                self._started = False

    def broadcast(self, _: Out[T] | None, msg: T) -> None:
        if not self._started:
            self.start()
        self.zenoh.publish(self._zenoh_topic, msg)

    def subscribe(
        self, callback: Callable[[T], None], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        if not self._started:
            self.start()
        return self.zenoh.subscribe(self._zenoh_topic, lambda msg, topic: callback(msg))

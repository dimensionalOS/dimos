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

"""CDR encoding for generated messages over existing raw transports."""

from __future__ import annotations

from collections.abc import Callable
from functools import partial
import threading
from typing import Any, Literal, TypeVar

from dimos_cdrgen.registry import encode, message_types

from dimos.core.global_config import global_config
from dimos.core.stream import Stream
from dimos.core.transport import PubSubTransport
from dimos.msgs.protocol import decode_cdr as decode
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase
from dimos.protocol.pubsub.impl.zenohpubsub import Topic, ZenohPubSubBase, ZenohQoS
from dimos.protocol.service.lcmservice import LCMConfig
from dimos.protocol.service.zenohservice import ZenohConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
T = TypeVar("T")


class CdrTransport(PubSubTransport[T]):
    """Plain XCDR1 values over raw LCM or Zenoh.

    The default transport factory selects this class for generated message types.
    Omit backend in a blueprint spec to resolve it when the spec is built.
    An explicit backend retains its own topic and session choices.

    The explicit /cdr/<package>/msg/<Type> suffix separates these channels
    from existing LCM-encoded streams. Both peers must select the same schema;
    this is not runtime schema negotiation. Native ports use ``channel``.
    """

    def __init__(
        self,
        topic: str,
        msg_type: type[T],
        *,
        backend: Literal["lcm", "zenoh"] | None = None,
        qos: ZenohQoS | None = None,
        **kwargs: Any,
    ) -> None:
        if not topic.strip("/") or any(character in topic for character in "*?#"):
            raise ValueError("CDR transports require a concrete, nonempty topic")
        if backend is None:
            # The factory imports this class; defer this import to avoid the cycle.
            from dimos.core.transport_factory import default_zenoh_qos, transport_topic

            backend = global_config.transport
            if qos is None and backend == "zenoh":
                qos = default_zenoh_qos(topic, msg_type)
            topic = transport_topic(topic, global_config)
        name = next((name for name, cls in message_types().items() if cls is msg_type), None)
        if name is None:
            raise TypeError("CDR transport requires a generated, registry-owned message class")
        if backend not in {"lcm", "zenoh"}:
            raise ValueError("CDR transport backend must be lcm or zenoh")
        if qos is not None and backend != "zenoh":
            raise ValueError("Publisher QoS applies only to Zenoh")
        self._base_topic = topic
        self._msg_type = msg_type
        self._backend = backend
        self._qos = qos
        self._config = (LCMConfig if backend == "lcm" else ZenohConfig)(**kwargs)
        # Zenoh topics are also raw LCM topics when lcm_type is absent.
        super().__init__(Topic(topic.strip("/") + "/cdr/" + name, qos=qos))
        self._pubsub: LCMPubSubBase | ZenohPubSubBase | None = None
        self._lock = threading.RLock()

    @property
    def publish_qos(self) -> dict[str, str] | None:
        return self._qos.to_wire() if self._qos is not None else None

    def __reduce__(self) -> tuple[Any, ...]:
        return (
            partial(
                type(self),
                backend=self._backend,
                qos=self._qos,
                **self._config.model_dump(exclude={"lcm"}),
            ),
            (self._base_topic, self._msg_type),
        )

    def start(self) -> None:
        with self._lock:
            if self._pubsub is None:
                cls = LCMPubSubBase if self._backend == "lcm" else ZenohPubSubBase
                pubsub = cls(**self._config.model_dump())
                try:
                    pubsub.start()
                except BaseException:
                    pubsub.stop()
                    raise
                self._pubsub = pubsub

    def stop(self) -> None:
        with self._lock:
            pubsub, self._pubsub = self._pubsub, None
        if pubsub is not None:
            pubsub.stop()

    def broadcast(self, _: Stream[T] | None, msg: T) -> None:
        if type(msg) is not self._msg_type:
            raise TypeError("Message does not match the CDR transport's declared type")
        payload = encode(msg)
        with self._lock:
            self.start()
            assert self._pubsub is not None
            self._pubsub.publish(self.topic, payload)

    def subscribe(
        self, callback: Callable[[T], Any], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        def receive(payload: bytes, _: Any) -> None:
            try:
                value = decode(payload, self._msg_type)
            except ValueError as error:
                logger.warning("Invalid CDR payload", channel=self.channel, error=str(error))
                return
            callback(value)

        with self._lock:
            self.start()
            assert self._pubsub is not None
            return self._pubsub.subscribe(self.topic, receive)

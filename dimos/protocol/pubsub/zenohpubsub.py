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

"""Zenoh-based pub/sub implementation with QoS support.

This module provides a Zenoh transport layer that maps ROS-like QoS concepts
to Zenoh's native reliability and congestion control mechanisms.

Key features:
- Configurable reliability (RELIABLE vs BEST_EFFORT)
- Congestion control (BLOCK vs DROP)
- Shared memory support for intra-host communication
- Clean encoder mixin pattern for serialization
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import threading
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

import zenoh

from dimos.protocol.pubsub.spec import PickleEncoderMixin, PubSub, PubSubEncoderMixin
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()


class Reliability(Enum):
    """QoS reliability setting - maps to ROS 2 reliability QoS."""

    RELIABLE = "reliable"  # Retransmit until acknowledged
    BEST_EFFORT = "best_effort"  # Fire and forget, lowest latency


class CongestionControl(Enum):
    """Publisher behavior when subscriber can't keep up."""

    BLOCK = "block"  # Block publisher until space available (backpressure)
    DROP = "drop"  # Drop messages when buffer full (for real-time)


class Priority(Enum):
    """Message priority - affects scheduling and batching."""

    REAL_TIME = "real_time"  # Bypass batching, immediate send
    INTERACTIVE_HIGH = "interactive_high"
    INTERACTIVE_LOW = "interactive_low"
    DATA_HIGH = "data_high"
    DATA = "data"  # Default
    DATA_LOW = "data_low"
    BACKGROUND = "background"


@dataclass
class ZenohQoS:
    """Quality of Service configuration for Zenoh pub/sub.

    Maps ROS 2 QoS concepts to Zenoh equivalents:
    - reliability: RELIABLE (default) or BEST_EFFORT
    - congestion_control: BLOCK (backpressure) or DROP (real-time)
    - priority: Affects message scheduling
    - express: Bypass batching for lowest latency

    Example usage:
        # For reliable command topics (like ROS RELIABLE)
        qos = ZenohQoS(reliability=Reliability.RELIABLE)

        # For sensor data (like ROS BEST_EFFORT)
        qos = ZenohQoS(
            reliability=Reliability.BEST_EFFORT,
            congestion_control=CongestionControl.DROP,
        )

        # For low-latency control
        qos = ZenohQoS(
            reliability=Reliability.BEST_EFFORT,
            priority=Priority.REAL_TIME,
            express=True,
        )
    """

    reliability: Reliability = Reliability.RELIABLE
    congestion_control: CongestionControl = CongestionControl.BLOCK
    priority: Priority = Priority.DATA
    express: bool = False  # Bypass batching for minimum latency


@dataclass
class ZenohConfig:
    """Zenoh session configuration.

    Attributes:
        locators: List of endpoints to connect to (e.g., ["tcp/192.168.1.1:7447"])
                  Empty list means peer-to-peer discovery
        shm_enabled: Enable shared memory for intra-host zero-copy
        shm_size: Shared memory buffer size in bytes (default 64MB)
        listen_endpoints: Endpoints to listen on (for router mode)
        mode: Session mode - "peer" (default), "client", or "router"
    """

    locators: list[str] = field(default_factory=list)
    shm_enabled: bool = True
    shm_size: int = 64 * 1024 * 1024  # 64MB
    listen_endpoints: list[str] = field(default_factory=list)
    mode: str = "peer"  # "peer", "client", or "router"


@runtime_checkable
class ZenohSerializable(Protocol):
    """Protocol for messages that can be serialized for Zenoh transport."""

    msg_name: str

    def zenoh_encode(self) -> bytes:
        """Encode this message to bytes for Zenoh transport."""
        ...

    @classmethod
    def zenoh_decode(cls, data: bytes) -> ZenohSerializable:
        """Decode bytes from Zenoh transport into a message instance."""
        ...


def _map_priority(priority: Priority) -> zenoh.Priority:
    """Map our Priority enum to Zenoh's Priority."""
    mapping = {
        Priority.REAL_TIME: zenoh.Priority.REAL_TIME,
        Priority.INTERACTIVE_HIGH: zenoh.Priority.INTERACTIVE_HIGH,
        Priority.INTERACTIVE_LOW: zenoh.Priority.INTERACTIVE_LOW,
        Priority.DATA_HIGH: zenoh.Priority.DATA_HIGH,
        Priority.DATA: zenoh.Priority.DATA,
        Priority.DATA_LOW: zenoh.Priority.DATA_LOW,
        Priority.BACKGROUND: zenoh.Priority.BACKGROUND,
    }
    return mapping[priority]


def _map_congestion_control(cc: CongestionControl) -> zenoh.CongestionControl:
    """Map our CongestionControl enum to Zenoh's."""
    if cc == CongestionControl.BLOCK:
        return zenoh.CongestionControl.BLOCK
    return zenoh.CongestionControl.DROP


def _map_reliability(rel: Reliability) -> zenoh.Reliability:
    """Map our Reliability enum to Zenoh's."""
    if rel == Reliability.RELIABLE:
        return zenoh.Reliability.RELIABLE
    return zenoh.Reliability.BEST_EFFORT


def normalize_topic(topic: str) -> str:
    """Normalize a topic to a valid Zenoh key expression.

    Zenoh key expressions don't allow:
    - Leading slashes
    - Trailing slashes
    - Empty chunks (double slashes)

    This converts ROS-style topics to Zenoh key expressions:
    - "/cmd_vel" -> "cmd_vel"
    - "/robot/sensors/lidar" -> "robot/sensors/lidar"
    - "topic" -> "topic" (unchanged)

    Args:
        topic: ROS-style topic name or Zenoh key expression

    Returns:
        Valid Zenoh key expression
    """
    # Remove leading and trailing slashes
    result = topic.strip("/")
    # Remove any double slashes
    while "//" in result:
        result = result.replace("//", "/")
    return result


class ZenohPubSubBase(PubSub[str, bytes]):
    """Base Zenoh pub/sub implementation.

    Handles session lifecycle, publisher/subscriber management,
    and QoS configuration. Use with encoder mixins for serialization.

    Thread-safe: Publishers and subscribers are managed with locks.
    """

    def __init__(
        self,
        config: ZenohConfig | None = None,
        qos: ZenohQoS | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self.config = config or ZenohConfig()
        self.qos = qos or ZenohQoS()

        self._session: zenoh.Session | None = None
        self._publishers: dict[str, zenoh.Publisher] = {}
        self._subscribers: dict[str, zenoh.Subscriber] = {}
        self._lock = threading.Lock()
        self._started = False
        self._stop_event = threading.Event()

    def _build_zenoh_config(self) -> zenoh.Config:
        """Build Zenoh configuration from our config dataclass."""
        zconfig = zenoh.Config()

        # Session mode
        zconfig.insert_json5("mode", f'"{self.config.mode}"')

        # Connect endpoints (locators)
        if self.config.locators:
            endpoints_json = "[" + ",".join(f'"{e}"' for e in self.config.locators) + "]"
            zconfig.insert_json5("connect/endpoints", endpoints_json)

        # Listen endpoints
        if self.config.listen_endpoints:
            endpoints_json = (
                "[" + ",".join(f'"{e}"' for e in self.config.listen_endpoints) + "]"
            )
            zconfig.insert_json5("listen/endpoints", endpoints_json)

        # Shared memory configuration
        if self.config.shm_enabled:
            zconfig.insert_json5("transport/shared_memory/enabled", "true")

        return zconfig

    def start(self) -> None:
        """Start the Zenoh session.

        Opens a Zenoh session with the configured parameters.
        Safe to call multiple times - subsequent calls are no-ops.
        """
        with self._lock:
            if self._started:
                return

            try:
                zconfig = self._build_zenoh_config()
                self._session = zenoh.open(zconfig)
                self._started = True
                self._stop_event.clear()
                logger.debug(
                    f"Zenoh session started (mode={self.config.mode}, "
                    f"shm={self.config.shm_enabled})"
                )
            except Exception as e:
                logger.error(f"Failed to start Zenoh session: {e}")
                raise

    def stop(self) -> None:
        """Stop the Zenoh session and clean up resources.

        Undeclares all publishers and subscribers, then closes the session.
        Safe to call multiple times.
        """
        with self._lock:
            if not self._started:
                return

            self._stop_event.set()

            # Clean up publishers
            for topic, pub in self._publishers.items():
                try:
                    pub.undeclare()
                except Exception as e:
                    logger.warning(f"Error undeclaring publisher for {topic}: {e}")
            self._publishers.clear()

            # Clean up subscribers
            for topic, sub in self._subscribers.items():
                try:
                    sub.undeclare()
                except Exception as e:
                    logger.warning(f"Error undeclaring subscriber for {topic}: {e}")
            self._subscribers.clear()

            # Close session
            if self._session is not None:
                try:
                    self._session.close()
                except Exception as e:
                    logger.warning(f"Error closing Zenoh session: {e}")
                self._session = None

            self._started = False
            logger.debug("Zenoh session stopped")

    def _ensure_started(self) -> None:
        """Ensure session is started, raise if not."""
        if not self._started or self._session is None:
            raise RuntimeError(
                "Zenoh session not started. Call start() before publish/subscribe."
            )

    def _get_or_create_publisher(self, topic: str) -> zenoh.Publisher:
        """Get existing publisher or create a new one for the topic."""
        # Normalize topic to valid Zenoh key expression
        key_expr = normalize_topic(topic)

        if key_expr not in self._publishers:
            self._ensure_started()
            assert self._session is not None

            pub = self._session.declare_publisher(
                key_expr,
                congestion_control=_map_congestion_control(self.qos.congestion_control),
                priority=_map_priority(self.qos.priority),
                express=self.qos.express,
                reliability=_map_reliability(self.qos.reliability),
            )
            self._publishers[key_expr] = pub
            logger.debug(f"Created Zenoh publisher for key: {key_expr}")

        return self._publishers[key_expr]

    def publish(self, topic: str, message: bytes) -> None:
        """Publish a message to the specified topic.

        Args:
            topic: The key expression (topic) to publish to
            message: Raw bytes to publish

        Raises:
            RuntimeError: If session not started
        """
        if self._stop_event.is_set():
            return

        with self._lock:
            pub = self._get_or_create_publisher(topic)

        try:
            pub.put(message)
        except Exception as e:
            logger.error(f"Failed to publish to {topic}: {e}")
            raise

    def subscribe(
        self,
        topic: str,
        callback: Callable[[bytes, str], Any],
    ) -> Callable[[], None]:
        """Subscribe to a topic with a callback.

        Args:
            topic: The key expression (topic) to subscribe to
            callback: Function called with (message_bytes, topic) for each message

        Returns:
            Unsubscribe function - call to stop receiving messages

        Raises:
            RuntimeError: If session not started

        Note:
            Reliability is configured on the publisher side in Zenoh 1.x.
            Subscribers receive all messages published by matching publishers.
        """
        self._ensure_started()
        assert self._session is not None

        # Normalize topic to valid Zenoh key expression
        key_expr = normalize_topic(topic)

        def _handler(sample: zenoh.Sample) -> None:
            if self._stop_event.is_set():
                return
            try:
                payload = sample.payload.to_bytes()
                callback(payload, topic)  # Return original topic to callback
            except Exception as e:
                logger.error(f"Error in subscriber callback for {key_expr}: {e}")

        with self._lock:
            sub = self._session.declare_subscriber(
                key_expr,
                _handler,
            )
            self._subscribers[key_expr] = sub

        logger.debug(f"Subscribed to key: {key_expr}")

        def unsubscribe() -> None:
            with self._lock:
                if key_expr in self._subscribers:
                    try:
                        self._subscribers[key_expr].undeclare()
                        del self._subscribers[key_expr]
                        logger.debug(f"Unsubscribed from key: {key_expr}")
                    except Exception as e:
                        logger.warning(f"Error unsubscribing from {topic}: {e}")

        return unsubscribe

    def __enter__(self) -> ZenohPubSubBase:
        """Context manager entry - starts session."""
        self.start()
        return self

    def __exit__(self, *exc: Any) -> None:
        """Context manager exit - stops session."""
        self.stop()


class ZenohEncoderMixin(PubSubEncoderMixin[str, Any]):
    """Encoder mixin for messages implementing ZenohSerializable protocol.

    Use this when messages have zenoh_encode()/zenoh_decode() methods.
    Messages MUST implement the ZenohSerializable protocol.
    """

    _msg_type: type[ZenohSerializable] | None = None

    def __init__(self, msg_type: type[ZenohSerializable] | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._msg_type = msg_type

    def encode(self, msg: Any, _topic: str) -> bytes:
        """Encode message to bytes using zenoh_encode()."""
        if not hasattr(msg, "zenoh_encode"):
            raise TypeError(
                f"Message type {type(msg).__name__} does not implement zenoh_encode(). "
                f"Use PickleZenoh for arbitrary Python objects or implement ZenohSerializable protocol."
            )
        return msg.zenoh_encode()

    def decode(self, data: bytes, _topic: str) -> Any:
        """Decode bytes to message using zenoh_decode().

        Requires msg_type to be set for decoding.
        """
        if self._msg_type is None:
            raise ValueError("Cannot decode: msg_type not set")

        if not hasattr(self._msg_type, "zenoh_decode"):
            raise TypeError(
                f"Message type {self._msg_type.__name__} does not implement zenoh_decode(). "
                f"Use PickleZenoh for arbitrary Python objects or implement ZenohSerializable protocol."
            )
        return self._msg_type.zenoh_decode(data)


class Zenoh(ZenohEncoderMixin, ZenohPubSubBase):
    """Zenoh pub/sub with native message encoding.

    Use for messages implementing ZenohSerializable or LCMMsg protocol.

    Example:
        zenoh = Zenoh(msg_type=PoseStamped, qos=ZenohQoS(reliability=Reliability.RELIABLE))
        zenoh.start()
        zenoh.publish("/pose", pose_msg)
    """

    pass


class PickleZenoh(PickleEncoderMixin[str, Any], ZenohPubSubBase):
    """Zenoh pub/sub with pickle serialization.

    Use for arbitrary Python objects. Convenient but not recommended
    for cross-language or security-sensitive scenarios.

    Example:
        zenoh = PickleZenoh()
        zenoh.start()
        zenoh.publish("/data", {"key": "value", "array": [1, 2, 3]})
    """

    pass


__all__ = [
    "CongestionControl",
    "PickleZenoh",
    "Priority",
    "Reliability",
    "Zenoh",
    "ZenohConfig",
    "ZenohEncoderMixin",
    "ZenohPubSubBase",
    "ZenohQoS",
    "ZenohSerializable",
    "normalize_topic",
]

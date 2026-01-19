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

from collections.abc import Callable
from dataclasses import dataclass
import importlib
import threading
from typing import Any, Protocol, TypeAlias, TypeVar, runtime_checkable

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    rclpy = None  # type: ignore[assignment]
    SingleThreadedExecutor = None  # type: ignore[assignment, misc]
    Node = None  # type: ignore[assignment, misc]

from dimos.protocol.pubsub.spec import MsgT, PubSub, PubSubEncoderMixin, TopicT


# Type definitions for LCM and ROS messages, be gentle for now
# just a sketch until proper translation is written
@runtime_checkable
class DimosMessage(Protocol):
    """Protocol for LCM message types (from dimos_lcm or lcm_msgs)."""

    msg_name: str
    __slots__: tuple[str, ...]


@runtime_checkable
class ROSMessage(Protocol):
    """Protocol for ROS message types."""

    def get_fields_and_field_types(self) -> dict[str, str]: ...


@dataclass
class ROSTopic:
    """Topic descriptor for ROS pubsub."""

    topic: str
    ros_type: type
    qos: "QoSProfile | None" = None  # Optional per-topic QoS override


class RawROS(PubSub[ROSTopic, Any]):
    """ROS 2 PubSub implementation following the PubSub spec.

    This allows direct comparison of ROS messaging performance against
    native LCM and other pubsub implementations.
    """

    def __init__(
        self, node_name: str = "dimos_ros_pubsub", qos: "QoSProfile | None" = None
    ) -> None:
        """Initialize the ROS pubsub.

        Args:
            node_name: Name for the ROS node
            qos: Optional QoS profile (defaults to BEST_EFFORT for throughput)
        """
        if not ROS_AVAILABLE:
            raise ImportError("rclpy is not installed. ROS pubsub requires ROS 2.")

        self._node_name = node_name
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._running = False

        # Track publishers and subscriptions
        self._publishers: dict[str, Any] = {}
        self._subscriptions: dict[str, list[tuple[Any, Callable[[Any, ROSTopic], None]]]] = {}
        self._lock = threading.Lock()

        # QoS profile - use provided or default to best-effort for throughput
        if qos is not None:
            self._qos = qos
        else:
            self._qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1,
            )

    def start(self) -> None:
        """Start the ROS node and executor."""
        if self._running:
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = Node(self._node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._running = True
        self._spin_thread = threading.Thread(target=self._spin, name="ros_pubsub_spin")
        self._spin_thread.start()

    def stop(self) -> None:
        """Stop the ROS node and clean up."""
        if not self._running:
            return

        self._running = False

        # Wake up the executor so spin thread can exit
        if self._executor:
            self._executor.wake()

        # Wait for spin thread to finish
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)

        if self._executor:
            self._executor.shutdown()

        if self._node:
            self._node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        self._publishers.clear()
        self._subscriptions.clear()
        self._spin_thread = None

    def _spin(self) -> None:
        """Background thread for spinning the ROS executor."""
        while self._running:
            executor = self._executor
            if executor is None:
                break
            executor.spin_once(timeout_sec=0)  # Non-blocking for max throughput

    def _get_or_create_publisher(self, topic: ROSTopic) -> Any:
        """Get existing publisher or create a new one."""
        with self._lock:
            if topic.topic not in self._publishers:
                node = self._node
                if node is None:
                    raise RuntimeError("Pubsub must be started before publishing")
                qos = topic.qos if topic.qos is not None else self._qos
                self._publishers[topic.topic] = node.create_publisher(
                    topic.ros_type, topic.topic, qos
                )
            return self._publishers[topic.topic]

    def publish(self, topic: ROSTopic, message: Any) -> None:
        """Publish a message to a ROS topic.

        Args:
            topic: ROSTopic descriptor with topic name and message type
            message: ROS message to publish
        """
        if not self._running or not self._node:
            return

        publisher = self._get_or_create_publisher(topic)
        publisher.publish(message)

    def subscribe(
        self, topic: ROSTopic, callback: Callable[[Any, ROSTopic], None]
    ) -> Callable[[], None]:
        """Subscribe to a ROS topic with a callback.

        Args:
            topic: ROSTopic descriptor with topic name and message type
            callback: Function called with (message, topic) when message received

        Returns:
            Unsubscribe function
        """
        if not self._running or not self._node:
            raise RuntimeError("ROS pubsub not started")

        with self._lock:

            def ros_callback(msg: Any) -> None:
                callback(msg, topic)

            qos = topic.qos if topic.qos is not None else self._qos
            subscription = self._node.create_subscription(
                topic.ros_type, topic.topic, ros_callback, qos
            )

            if topic.topic not in self._subscriptions:
                self._subscriptions[topic.topic] = []
            self._subscriptions[topic.topic].append((subscription, callback))

            def unsubscribe() -> None:
                with self._lock:
                    if topic.topic in self._subscriptions:
                        self._subscriptions[topic.topic] = [
                            (sub, cb)
                            for sub, cb in self._subscriptions[topic.topic]
                            if cb is not callback
                        ]
                        if self._node:
                            self._node.destroy_subscription(subscription)

            return unsubscribe


class Dimos2RosMixin(PubSubEncoderMixin[TopicT, DimosMessage, ROSMessage]):
    """Mixin that converts between dimos_lcm (LCM-based) and ROS messages.

    This enables seamless interop: publish LCM messages to ROS topics
    and receive ROS messages as LCM messages.
    """

    def encode(self, msg: DimosMessage, *_: TopicT) -> ROSMessage:
        """Convert a dimos_lcm message to its equivalent ROS message.

        Args:
            msg: An LCM message (e.g., dimos_lcm.geometry_msgs.Vector3)

        Returns:
            The corresponding ROS message (e.g., geometry_msgs.msg.Vector3)
        """
        raise NotImplementedError("Encode method not implemented")

    def decode(self, msg: ROSMessage, _: TopicT | None = None) -> DimosMessage:
        """Convert a ROS message to its equivalent dimos_lcm message.

        Args:
            msg: A ROS message (e.g., geometry_msgs.msg.Vector3)

        Returns:
            The corresponding LCM message (e.g., dimos_lcm.geometry_msgs.Vector3)
        """
        raise NotImplementedError("Decode method not implemented")


class DimosROS(
    Dimos2RosMixin[ROSTopic],
    RawROS,
):
    """ROS PubSub with automatic dimos.msgs ↔ ROS message conversion."""

    pass


ROS = DimosROS

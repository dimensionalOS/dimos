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
import threading
from typing import Any, Protocol, runtime_checkable

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

import uuid

from dimos.msgs import DimosMsg
from dimos.protocol.pubsub.impl.rospubsub_conversion import (
    derive_ros_type,
    dimos_to_ros,
    ros_to_dimos,
)
from dimos.protocol.pubsub.spec import PubSub


@runtime_checkable
class ROSMessage(Protocol):
    """Protocol for ROS message types."""

    def get_fields_and_field_types(self) -> dict[str, str]: ...


@dataclass
class RawROSTopic:
    """Topic descriptor for raw ROS pubsub (uses ROS types directly)."""

    topic: str
    ros_type: type
    qos: "QoSProfile | None" = None


@dataclass
class ROSTopic:
    """Topic descriptor for DimosROS pubsub (uses dimos message types)."""

    topic: str
    msg_type: type[DimosMsg]
    qos: "QoSProfile | None" = None


class RawROS(PubSub[RawROSTopic, Any]):
    """ROS 2 PubSub implementation following the PubSub spec.

    This allows direct comparison of ROS messaging performance against
    native LCM and other pubsub implementations.
    """

    def __init__(self, node_name: str | None = None, qos: "QoSProfile | None" = None) -> None:
        """Initialize the ROS pubsub.

        Args:
            node_name: Name for the ROS node (auto-generated if None)
            qos: Optional QoS profile (defaults to BEST_EFFORT for throughput)
        """
        if not ROS_AVAILABLE:
            raise ImportError("rclpy is not installed. ROS pubsub requires ROS 2.")

        # Use unique node name to avoid conflicts in tests
        self._node_name = node_name or f"dimos_ros_{uuid.uuid4().hex[:8]}"
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Track publishers and subscriptions
        self._publishers: dict[str, Any] = {}
        self._subscriptions: dict[str, list[tuple[Any, Callable[[Any, RawROSTopic], None]]]] = {}
        self._lock = threading.Lock()

        # QoS profile - use provided or default to best-effort for throughput
        if qos is not None:
            self._qos = qos
        else:
            self._qos = QoSProfile(  # type: ignore[no-untyped-call]
                # Haven't noticed any difference between BEST_EFFORT and RELIABLE for local comms in our tests
                # ./bin/dev python -m pytest -svm tool -k ros dimos/protocol/pubsub/benchmark/test_benchmark.py
                #
                # but RELIABLE seems to have marginally higher throughput
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=5000,
            )

    def start(self) -> None:
        """Start the ROS node and executor."""
        if self._spin_thread is not None:
            return

        if not rclpy.ok():  # type: ignore[attr-defined]
            rclpy.init()

        self._stop_event.clear()
        self._node = Node(self._node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(target=self._spin, name="ros_pubsub_spin")
        self._spin_thread.start()

    def stop(self) -> None:
        """Stop the ROS node and clean up."""
        if self._spin_thread is None:
            return

        # Signal spin thread to stop and shutdown executor
        self._stop_event.set()
        if self._executor:
            self._executor.shutdown()  # This stops spin_once from blocking

        # Wait for spin thread to exit
        self._spin_thread.join(timeout=1.0)

        # Grab references while holding lock, then destroy without lock
        with self._lock:
            subs_to_destroy = [
                sub for topic_subs in self._subscriptions.values() for sub, _ in topic_subs
            ]
            pubs_to_destroy = list(self._publishers.values())
            self._subscriptions.clear()
            self._publishers.clear()

        if self._node:
            for subscription in subs_to_destroy:
                self._node.destroy_subscription(subscription)
            for publisher in pubs_to_destroy:
                self._node.destroy_publisher(publisher)

        if self._node:
            self._node.destroy_node()  # type: ignore[no-untyped-call]
            self._node = None

        self._executor = None
        self._spin_thread = None

    def _spin(self) -> None:
        """Background thread for spinning the ROS executor."""
        while not self._stop_event.is_set():
            executor = self._executor
            if executor is None:
                break
            try:
                executor.spin_once(timeout_sec=0.01)
            except Exception:
                break

    def _get_or_create_publisher(self, topic: RawROSTopic) -> Any:
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

    def publish(self, topic: RawROSTopic, message: Any) -> None:
        """Publish a message to a ROS topic.

        Args:
            topic: RawROSTopic descriptor with topic name and message type
            message: ROS message to publish
        """
        if self._node is None:
            return

        publisher = self._get_or_create_publisher(topic)
        publisher.publish(message)

    def subscribe(
        self, topic: RawROSTopic, callback: Callable[[Any, RawROSTopic], None]
    ) -> Callable[[], None]:
        """Subscribe to a ROS topic with a callback.

        Args:
            topic: RawROSTopic descriptor with topic name and message type
            callback: Function called with (message, topic) when message received

        Returns:
            Unsubscribe function
        """
        if self._node is None:
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


class DimosROS(PubSub[ROSTopic, DimosMsg]):
    """ROS PubSub with automatic dimos.msgs ↔ ROS message conversion.

    Uses ROSTopic (with dimos msg_type) instead of RawROSTopic (with ros_type).
    Automatically converts between dimos and ROS message formats.
    Uses composition with RawROS internally.
    """

    def __init__(self, node_name: str | None = None, qos: "QoSProfile | None" = None) -> None:
        """Initialize the DimosROS pubsub.

        Args:
            node_name: Name for the ROS node (auto-generated if None)
            qos: Optional QoS profile (defaults to BEST_EFFORT for throughput)
        """
        self._raw = RawROS(node_name, qos)

    def start(self) -> None:
        """Start the ROS node and executor."""
        self._raw.start()

    def stop(self) -> None:
        """Stop the ROS node and clean up."""
        self._raw.stop()

    def _to_raw_topic(self, topic: ROSTopic) -> RawROSTopic:
        """Convert a ROSTopic to a RawROSTopic by deriving the ROS type."""
        ros_type = derive_ros_type(topic.msg_type)
        return RawROSTopic(topic=topic.topic, ros_type=ros_type, qos=topic.qos)

    def publish(self, topic: ROSTopic, message: DimosMsg) -> None:
        """Publish a dimos message to a ROS topic.

        Args:
            topic: ROSTopic with dimos msg_type
            message: Dimos message to publish
        """
        raw_topic = self._to_raw_topic(topic)
        ros_message = dimos_to_ros(message, raw_topic.ros_type)
        self._raw.publish(raw_topic, ros_message)

    def subscribe(
        self, topic: ROSTopic, callback: Callable[[DimosMsg, ROSTopic], None]
    ) -> Callable[[], None]:
        """Subscribe to a ROS topic with automatic dimos message conversion.

        Args:
            topic: ROSTopic with dimos msg_type
            callback: Function called with (dimos_message, topic)

        Returns:
            Unsubscribe function
        """
        raw_topic = self._to_raw_topic(topic)

        def wrapped_callback(ros_msg: Any, _raw_topic: RawROSTopic) -> None:
            dimos_msg = ros_to_dimos(ros_msg, topic.msg_type)
            callback(dimos_msg, topic)

        return self._raw.subscribe(raw_topic, wrapped_callback)


ROS = DimosROS

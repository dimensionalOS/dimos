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

from enum import Enum
import logging
import threading
from typing import Any

try:
    import rclpy  # type: ignore[import-untyped]
    from rclpy.executors import SingleThreadedExecutor  # type: ignore[import-untyped]
    from rclpy.node import Node  # type: ignore[import-untyped]
    from rclpy.qos import (  # type: ignore[import-untyped]
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
except ImportError:
    rclpy = None  # type: ignore[assignment]
    SingleThreadedExecutor = None  # type: ignore[assignment, misc]
    Node = None  # type: ignore[assignment, misc]
    QoSProfile = None  # type: ignore[assignment, misc]
    QoSReliabilityPolicy = None  # type: ignore[assignment, misc]
    QoSHistoryPolicy = None  # type: ignore[assignment, misc]
    QoSDurabilityPolicy = None  # type: ignore[assignment, misc]

from dimos.core.resource import Resource
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.INFO)


class BridgeDirection(Enum):
    """Direction of message bridging."""

    ROS_TO_DIMOS = "ros_to_dimos"
    DIMOS_TO_ROS = "dimos_to_ros"


class ROSBridge(Resource):
    """Unidirectional bridge between ROS and DIMOS for message passing."""

    def __init__(self, node_name: str = "dimos_ros_bridge") -> None:
        """Initialize the ROS-DIMOS bridge.

        Args:
            node_name: Name for the ROS node (default: "dimos_ros_bridge")
        """
        if not rclpy.ok():  # type: ignore[attr-defined]
            rclpy.init()

        self.node = Node(node_name)
        self.lcm = LCM()
        self.lcm.start()

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)

        self._spin_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._spin_thread.start()  # TODO: don't forget to shut it down

        self._bridges: dict[str, dict[str, Any]] = {}

        self._qos = QoSProfile(  # type: ignore[no-untyped-call]
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        logger.info(f"ROSBridge initialized with node name: {node_name}")

    def start(self) -> None:
        pass

    def stop(self) -> None:
        """Shutdown the bridge and clean up resources."""
        self._executor.shutdown()
        self.node.destroy_node()  # type: ignore[no-untyped-call]

        if rclpy.ok():  # type: ignore[attr-defined]
            rclpy.shutdown()

        logger.info("ROSBridge shutdown complete")

    def _ros_spin(self) -> None:
        """Background thread for spinning ROS executor."""
        try:
            self._executor.spin()
        finally:
            self._executor.shutdown()

    def add_topic(
        self,
        topic_name: str,
        dimos_type: type,
        ros_type: type,
        direction: BridgeDirection,
        remap_topic: str | None = None,
    ) -> None:
        """Add unidirectional bridging for a topic.

        Args:
            topic_name: Name of the topic (e.g., "/cmd_vel")
            dimos_type: DIMOS message type (e.g., dimos.msgs.geometry_msgs.Twist)
            ros_type: ROS message type (e.g., geometry_msgs.msg.Twist)
            direction: Direction of bridging (ROS_TO_DIMOS or DIMOS_TO_ROS)
            remap_topic: Optional remapped topic name for the other side
        """
        if topic_name in self._bridges:
            logger.warning(f"Topic {topic_name} already bridged")
            return

        # Determine actual topic names for each side
        ros_topic_name = topic_name
        dimos_topic_name = topic_name

        if remap_topic:
            if direction == BridgeDirection.ROS_TO_DIMOS:
                dimos_topic_name = remap_topic
            else:  # DIMOS_TO_ROS
                ros_topic_name = remap_topic

        # Create DIMOS/LCM topic
        dimos_topic = Topic(dimos_topic_name, dimos_type)

        ros_subscription = None
        ros_publisher = None
        dimos_subscription = None

        if direction == BridgeDirection.ROS_TO_DIMOS:

            def ros_callback(msg) -> None:  # type: ignore[no-untyped-def]
                self._ros_to_dimos(msg, dimos_topic, dimos_type, topic_name)

            ros_subscription = self.node.create_subscription(
                ros_type, ros_topic_name, ros_callback, self._qos
            )
            logger.info(f"  ROS → DIMOS: Subscribing to ROS topic {ros_topic_name}")

        elif direction == BridgeDirection.DIMOS_TO_ROS:
            ros_publisher = self.node.create_publisher(ros_type, ros_topic_name, self._qos)

            def dimos_callback(msg, _topic) -> None:  # type: ignore[no-untyped-def]
                self._dimos_to_ros(msg, ros_publisher, topic_name)

            dimos_subscription = self.lcm.subscribe(dimos_topic, dimos_callback)
            logger.info(f"  DIMOS → ROS: Subscribing to DIMOS topic {dimos_topic_name}")
        else:
            raise ValueError(f"Invalid bridge direction: {direction}")

        self._bridges[topic_name] = {
            "dimos_topic": dimos_topic,
            "dimos_type": dimos_type,
            "ros_type": ros_type,
            "ros_subscription": ros_subscription,
            "ros_publisher": ros_publisher,
            "dimos_subscription": dimos_subscription,
            "direction": direction,
            "ros_topic_name": ros_topic_name,
            "dimos_topic_name": dimos_topic_name,
        }

        direction_str = {
            BridgeDirection.ROS_TO_DIMOS: "ROS → DIMOS",
            BridgeDirection.DIMOS_TO_ROS: "DIMOS → ROS",
        }[direction]

        logger.info(f"Bridged topic: {topic_name} ({direction_str})")
        if remap_topic:
            logger.info(f"  Remapped: ROS '{ros_topic_name}' ↔ DIMOS '{dimos_topic_name}'")
        logger.info(f"  DIMOS type: {dimos_type.__name__}, ROS type: {ros_type.__name__}")

    def _ros_to_dimos(
        self, ros_msg: Any, dimos_topic: Topic, dimos_type: type, _topic_name: str
    ) -> None:
        """Convert ROS message to DIMOS and publish.

        Args:
            ros_msg: ROS message
            dimos_topic: DIMOS topic to publish to
            dimos_type: DIMOS message type
            topic_name: Name of the topic for tracking
        """
        dimos_msg = dimos_type.from_ros_msg(ros_msg)  # type: ignore[attr-defined]
        self.lcm.publish(dimos_topic, dimos_msg)

    def _dimos_to_ros(self, dimos_msg: Any, ros_publisher, _topic_name: str) -> None:  # type: ignore[no-untyped-def]
        """Convert DIMOS message to ROS and publish.

        Args:
            dimos_msg: DIMOS message
            ros_publisher: ROS publisher to use
            _topic_name: Name of the topic (unused, kept for consistency)
        """
        ros_msg = dimos_msg.to_ros_msg()
        ros_publisher.publish(ros_msg)

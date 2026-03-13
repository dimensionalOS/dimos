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

from collections import deque
import threading
import time
from typing import Any
import uuid

from dimos.utils.human import human_bytes

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
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message  # type: ignore[import-not-found]

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    rclpy = None  # type: ignore[assignment]
    SingleThreadedExecutor = None  # type: ignore[assignment, misc]
    Node = None  # type: ignore[assignment, misc]


class Topic:
    history_window: float = 60.0

    def __init__(self, name: str, msg_type: str = "", history_window: float = 60.0) -> None:
        self.name = name
        self.msg_type = msg_type
        self.message_history: deque[tuple[float, int]] = deque()
        self.history_window = history_window
        self.total_traffic_bytes = 0

    def msg(self, data_size: int) -> None:
        self.message_history.append((time.time(), data_size))
        self.total_traffic_bytes += data_size
        self._cleanup_old_messages()

    def _cleanup_old_messages(self, max_age: float | None = None) -> None:
        current_time = time.time()
        while self.message_history and current_time - self.message_history[0][0] > (
            max_age or self.history_window
        ):
            self.message_history.popleft()

    def _get_messages_in_window(self, time_window: float) -> list[tuple[float, int]]:
        current_time = time.time()
        cutoff_time = current_time - time_window
        return [(ts, size) for ts, size in list(self.message_history) if ts >= cutoff_time]

    def freq(self, time_window: float) -> float:
        messages = self._get_messages_in_window(time_window)
        if not messages:
            return 0.0
        return len(messages) / time_window

    def kbps(self, time_window: float) -> float:
        messages = self._get_messages_in_window(time_window)
        if not messages:
            return 0.0
        total_bytes = sum(size for _, size in messages)
        return total_bytes / 1000 / time_window

    def kbps_hr(self, time_window: float) -> str:
        bps = self.kbps(time_window) * 1000
        return human_bytes(bps) + "/s"

    def size(self, time_window: float) -> float:
        messages = self._get_messages_in_window(time_window)
        if not messages:
            return 0.0
        return sum(size for _, size in messages) / len(messages)

    def total_traffic(self) -> int:
        return self.total_traffic_bytes

    def total_traffic_hr(self) -> str:
        return human_bytes(self.total_traffic())

    def __str__(self) -> str:
        return f"topic({self.name})"


class ROS2Spy:
    topic_class: type[Topic] = Topic

    def __init__(self, history_window: float = 60.0, poll_interval: float = 1.0) -> None:
        self.history_window = history_window
        self.poll_interval = poll_interval
        self.topic: dict[str, Topic] = {}
        self.total = Topic("total", history_window=history_window)
        self._node: Any = None
        self._executor: Any = None
        self._spin_thread: threading.Thread | None = None
        self._poll_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._subscriptions: dict[str, Any] = {}

    def start(self) -> None:
        if not ROS_AVAILABLE:
            raise ImportError(
                "rclpy is not available. Source a ROS 2 installation before running rosspy."
            )

        if not rclpy.ok():  # type: ignore[union-attr]
            rclpy.init()  # type: ignore[union-attr]

        self._stop_event.clear()
        node_name = f"rosspy_{uuid.uuid4().hex[:8]}"
        self._node = Node(node_name)  # type: ignore[misc]
        self._executor = SingleThreadedExecutor()  # type: ignore[misc]
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True, name="rosspy_spin")
        self._spin_thread.start()

        self._poll_thread = threading.Thread(
            target=self._poll_topics, daemon=True, name="rosspy_poll"
        )
        self._poll_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._executor:
            self._executor.shutdown()
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=2.0)
        if self._node:
            self._node.destroy_node()
            self._node = None
        self._executor = None
        self._spin_thread = None
        self._poll_thread = None

    def _spin(self) -> None:
        while not self._stop_event.is_set():
            executor = self._executor
            if executor is None:
                break
            try:
                executor.spin_once(timeout_sec=0.01)
            except Exception:
                break

    def _poll_topics(self) -> None:
        while not self._stop_event.is_set():
            self._discover_topics()
            self._stop_event.wait(timeout=self.poll_interval)

    def _discover_topics(self) -> None:
        if self._node is None:
            return
        try:
            topic_list = self._node.get_topic_names_and_types()
        except Exception:
            return

        with self._lock:
            for topic_name, type_strings in topic_list:
                if topic_name in self._subscriptions:
                    continue
                if not type_strings:
                    continue
                type_str = type_strings[0]
                try:
                    msg_class = get_message(type_str)  # type: ignore[name-defined]
                    self._subscribe_to_topic(topic_name, msg_class, type_str)
                except Exception:
                    # Mark as attempted so we don't retry every poll cycle
                    self._subscriptions[topic_name] = None

    def _subscribe_to_topic(self, topic_name: str, msg_class: Any, type_str: str) -> None:
        if self._node is None:
            return

        def callback(msg: Any) -> None:
            try:
                data = serialize_message(msg)  # type: ignore[name-defined]
                data_size = len(data)
            except Exception:
                data_size = 0
            self._on_message(topic_name, type_str, data_size)

        qos = QoSProfile(  # type: ignore[misc]
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        try:
            sub = self._node.create_subscription(msg_class, topic_name, callback, qos)
            self._subscriptions[topic_name] = sub
        except Exception:
            self._subscriptions[topic_name] = None

    def _on_message(self, topic_name: str, type_str: str, data_size: int) -> None:
        with self._lock:
            if topic_name not in self.topic:
                self.topic[topic_name] = self.topic_class(
                    topic_name,
                    msg_type=type_str,
                    history_window=self.history_window,
                )
        self.topic[topic_name].msg(data_size)
        self.total.msg(data_size)


class GraphTopic(Topic):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.freq_history: deque[float] = deque(maxlen=20)
        self.bandwidth_history: deque[float] = deque(maxlen=20)

    def update_graphs(self, step_window: float = 1.0) -> None:
        self.freq_history.append(self.freq(step_window))
        self.bandwidth_history.append(self.kbps(step_window))


class GraphROS2Spy(ROS2Spy, GraphTopic):
    topic_class: type[Topic] = GraphTopic

    graph_log_thread: threading.Thread | None = None
    graph_log_stop_event: threading.Event = threading.Event()

    def __init__(self, graph_log_window: float = 1.0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        GraphTopic.__init__(self, name="total", history_window=self.history_window)
        self.graph_log_window = graph_log_window

    def start(self) -> None:
        super().start()
        self.graph_log_stop_event.clear()
        self.graph_log_thread = threading.Thread(
            target=self._graph_log, daemon=True, name="rosspy_graph"
        )
        self.graph_log_thread.start()

    def _graph_log(self) -> None:
        while not self.graph_log_stop_event.is_set():
            self.update_graphs(self.graph_log_window)
            for topic in list(self.topic.values()):
                topic.update_graphs(self.graph_log_window)  # type: ignore[attr-defined]
            self.graph_log_stop_event.wait(timeout=self.graph_log_window)

    def stop(self) -> None:
        self.graph_log_stop_event.set()
        if self.graph_log_thread and self.graph_log_thread.is_alive():
            self.graph_log_thread.join(timeout=1.0)
        super().stop()

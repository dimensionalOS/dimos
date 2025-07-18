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

import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum

import lcm

from dimos.protocol.service.lcmservice import LCMConfig, LCMService


class BandwidthUnit(Enum):
    BP = "B"
    KBP = "kB"
    MBP = "MB"
    GBP = "GB"


def human_readable_bytes(bytes_value: float, round_to: int = 2) -> tuple[float, BandwidthUnit]:
    """Convert bytes to human-readable format with appropriate units"""
    if bytes_value >= 1024**3:  # GB
        return round(bytes_value / (1024**3), round_to), BandwidthUnit.GBP
    elif bytes_value >= 1024**2:  # MB
        return round(bytes_value / (1024**2), round_to), BandwidthUnit.MBP
    elif bytes_value >= 1024:  # KB
        return round(bytes_value / 1024, round_to), BandwidthUnit.KBP
    else:
        return round(bytes_value, round_to), BandwidthUnit.BP


class Topic:
    history_window: float = 60.0

    def __init__(self, name: str, history_window: float = 60.0):
        self.name = name
        # Store (timestamp, data_size) tuples for statistics
        self.message_history = deque()
        self.history_window = history_window
        # Total traffic accumulator (doesn't get cleaned up)
        self.total_traffic_bytes = 0

    def msg(self, data: bytes):
        # print(f"> msg {self.__str__()} {len(data)} bytes")
        datalen = len(data)
        self.message_history.append((time.time(), datalen))
        self.total_traffic_bytes += datalen
        self._cleanup_old_messages()

    def _cleanup_old_messages(self, max_age: float = None):
        """Remove messages older than max_age seconds"""
        current_time = time.time()
        while self.message_history and current_time - self.message_history[0][0] > (
            max_age or self.history_window
        ):
            self.message_history.popleft()

    def _get_messages_in_window(self, time_window: float):
        """Get messages within the specified time window"""
        current_time = time.time()
        cutoff_time = current_time - time_window
        return [(ts, size) for ts, size in self.message_history if ts >= cutoff_time]

    # avg msg freq in the last n seconds
    def freq(self, time_window: float) -> float:
        messages = self._get_messages_in_window(time_window)
        if not messages:
            return 0.0
        return len(messages) / time_window

    # avg bandwidth in kB/s in the last n seconds
    def kbps(self, time_window: float) -> float:
        messages = self._get_messages_in_window(time_window)
        if not messages:
            return 0.0
        total_bytes = sum(size for _, size in messages)
        total_kbytes = total_bytes / 1000  # Convert bytes to kB
        return total_kbytes / time_window

    def kbps_hr(self, time_window: float, round_to: int = 2) -> tuple[float, BandwidthUnit]:
        """Return human-readable bandwidth with appropriate units"""
        kbps_val = self.kbps(time_window)
        # Convert kB/s to B/s for human_readable_bytes
        bps = kbps_val * 1000
        return human_readable_bytes(bps, round_to)

    # avg msg size in the last n seconds
    def size(self, time_window: float) -> float:
        messages = self._get_messages_in_window(time_window)
        if not messages:
            return 0.0
        total_size = sum(size for _, size in messages)
        return total_size / len(messages)

    def total_traffic(self) -> int:
        """Return total traffic passed in bytes since the beginning"""
        return self.total_traffic_bytes

    def total_traffic_hr(self) -> tuple[float, BandwidthUnit]:
        """Return human-readable total traffic with appropriate units"""
        total_bytes = self.total_traffic()
        return human_readable_bytes(total_bytes)

    def __str__(self):
        return f"topic({self.name})"


@dataclass
class LCMSpyConfig(LCMConfig):
    topic_history_window: float = 60.0


class LCMSpy(LCMService, Topic):
    default_config = LCMSpyConfig
    topic = dict[str, Topic]
    graph_log_window: float = 1.0
    topic_class: type[Topic] = Topic

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Topic.__init__(self, name="total", history_window=self.config.topic_history_window)
        self.topic = {}
        self.l = lcm.LCM(self.config.url) if self.config.url else lcm.LCM()

    def start(self):
        super().start()
        self.l.subscribe(".*", self.msg)

    def stop(self):
        """Stop the LCM spy and clean up resources"""
        super().stop()

    def msg(self, topic, data):
        Topic.msg(self, data)

        if topic not in self.topic:
            print(self.config)
            self.topic[topic] = self.topic_class(
                topic, history_window=self.config.topic_history_window
            )
        self.topic[topic].msg(data)


class GraphTopic(Topic):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.freq_history = deque(maxlen=20)
        self.bandwidth_history = deque(maxlen=20)

    def update_graphs(self, step_window: float = 1.0):
        """Update historical data for graphing"""
        freq = self.freq(step_window)
        kbps = self.kbps(step_window)
        self.freq_history.append(freq)
        self.bandwidth_history.append(kbps)


@dataclass
class GraphLCMSpyConfig(LCMSpyConfig):
    graph_log_window: float = 1.0


class GraphLCMSpy(LCMSpy, GraphTopic):
    default_config = GraphLCMSpyConfig

    graph_log_thread: threading.Thread | None = None
    graph_log_stop_event: threading.Event = threading.Event()
    topic_class: type[Topic] = GraphTopic

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        GraphTopic.__init__(self, name="total", history_window=self.config.topic_history_window)

    def start(self):
        super().start()
        self.graph_log_thread = threading.Thread(target=self.graph_log, daemon=True)
        self.graph_log_thread.start()

    def graph_log(self):
        while not self.graph_log_stop_event.is_set():
            self.update_graphs(self.config.graph_log_window)  # Update global history
            for topic in self.topic.values():
                topic.update_graphs(self.config.graph_log_window)
            time.sleep(self.config.graph_log_window)

    def stop(self):
        """Stop the graph logging and LCM spy"""
        self.graph_log_stop_event.set()
        if self.graph_log_thread and self.graph_log_thread.is_alive():
            self.graph_log_thread.join(timeout=1.0)
        super().stop()


if __name__ == "__main__":
    lcm_spy = LCMSpy()
    lcm_spy.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("LCM Spy stopped.")

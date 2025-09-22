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

import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Set

import lcm

from dimos.protocol.service.lcmservice import LCMConfig, LCMService
from dimos.utils.cli.lcmspy.lcmspy import Topic
from dimos.utils.data import _get_data_dir
from dimos.utils.testing import TimedSensorStorage


class TopicInfo(Topic):
    """Extended topic info with selection state."""

    def __init__(self, name: str, history_window: float = 60.0):
        super().__init__(name, history_window)
        self.selected: bool = False


@dataclass
class RecorderConfig(LCMConfig):
    """Configuration for the LCM recorder."""

    topic_history_window: float = 60.0
    recording_base_dir: str = "recordings"


class RecorderService(LCMService):
    """Service for recording selected LCM topics."""

    default_config = RecorderConfig

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.topics: Dict[str, TopicInfo] = {}
        self.l = lcm.LCM(self.config.url) if self.config.url else lcm.LCM()
        self.recording = False
        self.recording_start_time: Optional[float] = None
        self.recording_dir: Optional[Path] = None
        self.storages: Dict[str, TimedSensorStorage] = {}

    def start(self):
        """Start the recorder service and topic discovery."""
        super().start()
        # Subscribe to all topics for discovery
        self.l.subscribe(".*", self._handle_message)

    def stop(self):
        """Stop the recorder service."""
        if self.recording:
            self.stop_recording()
        super().stop()

    def _handle_message(self, topic: str, data: bytes):
        """Handle incoming LCM messages."""
        # Track topic if not already known
        if topic not in self.topics:
            self.topics[topic] = TopicInfo(
                name=topic, history_window=self.config.topic_history_window
            )

        # Update topic stats
        self.topics[topic].msg(data)

        # If recording and topic is selected, save the message
        if self.recording and self.topics[topic].selected and topic in self.storages:
            self.storages[topic].save_one(data)

    def get_selected_topics(self) -> List[str]:
        """Get list of selected topic names."""
        return [name for name, info in self.topics.items() if info.selected]

    def toggle_topic_selection(self, topic_name: str):
        """Toggle selection state of a topic."""
        if topic_name in self.topics:
            self.topics[topic_name].selected = not self.topics[topic_name].selected

    def select_all_topics(self, select: bool = True):
        """Select or deselect all topics."""
        for topic in self.topics.values():
            topic.selected = select

    def start_recording(self) -> bool:
        """Start recording selected topics."""
        selected_topics = self.get_selected_topics()
        if not selected_topics:
            return False

        # Create recording directory with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.recording_dir = _get_data_dir(
            f"{self.config.recording_base_dir}/recording_{timestamp}"
        )
        self.recording_dir.mkdir(parents=True, exist_ok=True)

        # Create storage for each selected topic
        self.storages.clear()
        for topic_name in selected_topics:
            # Clean topic name for filesystem (replace / with _)
            safe_name = topic_name.replace("/", "_").strip("_")
            storage_path = f"{self.config.recording_base_dir}/recording_{timestamp}/{safe_name}"
            self.storages[topic_name] = TimedSensorStorage(storage_path)

        self.recording = True
        self.recording_start_time = time.time()
        return True

    def stop_recording(self) -> Optional[Path]:
        """Stop recording and return the recording directory path."""
        if not self.recording:
            return None

        self.recording = False
        self.recording_start_time = None
        recording_dir = self.recording_dir
        self.recording_dir = None
        self.storages.clear()

        return recording_dir

    def get_recording_duration(self) -> float:
        """Get current recording duration in seconds."""
        if self.recording and self.recording_start_time:
            return time.time() - self.recording_start_time
        return 0.0

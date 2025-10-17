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

from __future__ import annotations

from abc import ABC, abstractmethod

from dimos.perception.detection.type import Detection2DBBox, ImageDetections2D


class IDSystem(ABC):
    """Abstract base class for ID assignment systems."""

    def register_detections(self, detections: ImageDetections2D) -> None:
        """Register multiple detections."""
        for detection in detections.detections:
            if isinstance(detection, Detection2DBBox):
                self.register_detection(detection)

    @abstractmethod
    def register_detection(self, detection: Detection2DBBox) -> int:
        """
        Register a single detection, returning assigned (long term) ID.

        Args:
            detection: Detection to register

        Returns:
            Long-term unique ID for this detection
        """
        ...


class PassthroughIDSystem(IDSystem):
    """Simple ID system that returns track_id with no object permanence."""

    def register_detection(self, detection: Detection2DBBox) -> int:
        """Return detection's track_id as long-term ID (no permanence)."""
        return detection.track_id

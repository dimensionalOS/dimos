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
from typing import Generic, TypeVar

from dimos.models.embedding.type import Embedding, EmbeddingModel
from dimos.perception.detection.type import Detection2DBBox, ImageDetections2D

E = TypeVar("E", bound="Embedding")
F = TypeVar("F")  # Generic feature type


class FeatureExtractor(ABC, Generic[F]):
    """Abstract base class for extracting features from detections."""

    @abstractmethod
    def extract(self, detection: Detection2DBBox) -> F:
        """
        Extract feature from a detection.

        Args:
            detection: Detection to extract features from

        Returns:
            Extracted feature of type F
        """
        pass


class EmbeddingFeatureExtractor(FeatureExtractor[E], Generic[E]):
    """Feature extractor that uses an embedding model to extract features from detection crops."""

    def __init__(self, model: EmbeddingModel[E], padding: int = 0):
        """
        Initialize embedding feature extractor.

        Args:
            model: Embedding model to use for feature extraction
            padding: Padding to add around detection bbox when cropping (default: 0)
        """
        self.model = model
        self.padding = padding

    def extract(self, detection: Detection2DBBox) -> E:
        """
        Extract embedding from detection's cropped image.

        Args:
            detection: Detection to extract embedding from

        Returns:
            Embedding feature (moved to CPU to save GPU memory)
        """
        cropped_image = detection.cropped_image(padding=self.padding)
        embedding = self.model.embed(cropped_image)
        assert not isinstance(embedding, list), "Expected single embedding for single image"
        # Move embedding to CPU immediately to free GPU memory
        embedding = embedding.to_cpu()
        return embedding


class IDSystem(ABC, Generic[F]):
    """Abstract base class for ID assignment systems using features."""

    def __init__(self, feature_extractor: FeatureExtractor[F]):
        """
        Initialize ID system with feature extractor.

        Args:
            feature_extractor: Feature extractor to use for detection features
        """
        self.feature_extractor = feature_extractor

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


class PassthroughIDSystem(IDSystem[F]):
    """Simple ID system that returns track_id with no object permanence."""

    def __init__(self, feature_extractor: FeatureExtractor[F] | None = None):
        """
        Initialize passthrough ID system.

        Args:
            feature_extractor: Optional feature extractor (not used, for interface compatibility)
        """
        # Don't call super().__init__ since we don't need feature_extractor
        self.feature_extractor = feature_extractor  # type: ignore

    def register_detection(self, detection: Detection2DBBox) -> int:
        """Return detection's track_id as long-term ID (no permanence)."""
        return detection.track_id


class EmbeddingIDSystem(IDSystem[Embedding]):
    """ID system using embedding similarity for object permanence."""

    def __init__(
        self,
        feature_extractor: FeatureExtractor[Embedding],
        similarity_threshold: float = 0.75,
    ):
        """
        Initialize embedding-based ID system.

        Args:
            feature_extractor: Feature extractor for embeddings
            similarity_threshold: Minimum similarity for associating tracks (0-1)
        """
        super().__init__(feature_extractor)

        # Import here to avoid circular dependency
        from dimos.perception.detection.reid.trackAssociator import TrackAssociator

        self.associator = TrackAssociator(similarity_threshold=similarity_threshold)

    def register_detection(self, detection: Detection2DBBox) -> int:
        embedding = self.feature_extractor.extract(detection)
        self.associator.update_embedding(detection.track_id, embedding)
        return self.associator.associate(detection.track_id)

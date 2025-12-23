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

from collections.abc import Callable
from typing import Literal

import numpy as np

from dimos.models.embedding.base import Embedding, EmbeddingModel
from dimos.perception.detection.reid.type import IDSystem
from dimos.perception.detection.type import Detection2DBBox


class EmbeddingIDSystem(IDSystem):
    """Associates short-term track_ids to long-term unique detection IDs via embedding similarity.

    Maintains:
    - All embeddings per track_id (as numpy arrays) for robust group comparison
    - Negative constraints from co-occurrence (tracks in same frame = different objects)
    - Mapping from track_id to unique long-term ID
    """

    def __init__(
        self,
        model: Callable[[], EmbeddingModel[Embedding]],
        padding: int = 0,
        similarity_threshold: float = 0.63,
        comparison_mode: Literal["max", "mean", "top_k_mean"] = "top_k_mean",
        top_k: int = 30,
        max_embeddings_per_track: int = 500,
        min_embeddings_for_matching: int = 10,
    ) -> None:
        """Initialize track associator.

        Args:
            model: Callable (class or function) that returns an embedding model for feature extraction
            padding: Padding to add around detection bbox when cropping (default: 0)
            similarity_threshold: Minimum similarity for associating tracks (0-1)
            comparison_mode: How to aggregate similarities between embedding groups
                - "max": Use maximum similarity between any pair
                - "mean": Use mean of all pairwise similarities
                - "top_k_mean": Use mean of top-k similarities
            top_k: Number of top similarities to average (if using top_k_mean)
            max_embeddings_per_track: Maximum number of embeddings to keep per track
            min_embeddings_for_matching: Minimum embeddings before attempting to match tracks
        """
        # Call model factory (class or function) to get model instance
        self.model = model()

        # Call start if available (Resource interface)
        if hasattr(self.model, "start"):
            self.model.start()

        self.padding = padding
        self.similarity_threshold = similarity_threshold
        self.comparison_mode = comparison_mode
        self.top_k = top_k
        self.max_embeddings_per_track = max_embeddings_per_track
        self.min_embeddings_for_matching = min_embeddings_for_matching

        # Track embeddings (list of all embeddings as numpy arrays)
        self.track_embeddings: dict[int, list[np.ndarray]] = {}  # type: ignore[type-arg]

        # Negative constraints (track_ids that co-occurred = different objects)
        self.negative_pairs: dict[int, set[int]] = {}

        # Track ID to long-term unique ID mapping
        self.track_to_long_term: dict[int, int] = {}
        self.long_term_counter: int = 0

        # Similarity history for optional adaptive thresholding
        self.similarity_history: list[float] = []

    def register_detection(self, detection: Detection2DBBox) -> int:
        """
        Register detection and return long-term ID.

        Args:
            detection: Detection to register

        Returns:
            Long-term unique ID for this detection
        """
        # Extract embedding from detection's cropped image
        cropped_image = detection.cropped_image(padding=self.padding)
        embedding = self.model.embed(cropped_image)
        assert not isinstance(embedding, list), "Expected single embedding for single image"
        # Move embedding to CPU immediately to free GPU memory
        embedding = embedding.to_cpu()

        # Update and associate track
        self.update_embedding(detection.track_id, embedding)
        return self.associate(detection.track_id)

    def update_embedding(self, track_id: int, new_embedding: Embedding) -> None:
        """Add new embedding to track's embedding collection.

        Args:
            track_id: Short-term track ID from detector
            new_embedding: New embedding to add to collection
        """
        # Convert to numpy array (already on CPU from feature extractor)
        new_vec = new_embedding.to_numpy()

        # Ensure normalized for cosine similarity
        norm = np.linalg.norm(new_vec)
        if norm > 0:
            new_vec = new_vec / norm

        if track_id not in self.track_embeddings:
            self.track_embeddings[track_id] = []

        embeddings = self.track_embeddings[track_id]
        embeddings.append(new_vec)

        # Keep only most recent embeddings if limit exceeded
        if len(embeddings) > self.max_embeddings_per_track:
            embeddings.pop(0)  # Remove oldest

    def _compute_group_similarity(
        self,
        query_embeddings: list[np.ndarray],  # type: ignore[type-arg]
        candidate_embeddings: list[np.ndarray],  # type: ignore[type-arg]
    ) -> float:
        """Compute similarity between two groups of embeddings.

        Args:
            query_embeddings: List of embeddings for query track
            candidate_embeddings: List of embeddings for candidate track

        Returns:
            Aggregated similarity score
        """
        # Compute all pairwise similarities efficiently
        query_matrix = np.stack(query_embeddings)  # [M, D]
        candidate_matrix = np.stack(candidate_embeddings)  # [N, D]

        # Cosine similarity via matrix multiplication (already normalized)
        similarities = query_matrix @ candidate_matrix.T  # [M, N]

        if self.comparison_mode == "max":
            # Maximum similarity across all pairs
            return float(np.max(similarities))

        elif self.comparison_mode == "mean":
            # Mean of all pairwise similarities
            return float(np.mean(similarities))

        elif self.comparison_mode == "top_k_mean":
            # Mean of top-k similarities
            flat_sims = similarities.flatten()
            k = min(self.top_k, len(flat_sims))
            top_k_sims = np.partition(flat_sims, -k)[-k:]
            return float(np.mean(top_k_sims))

        else:
            raise ValueError(f"Unknown comparison mode: {self.comparison_mode}")

    def add_negative_constraints(self, track_ids: list[int]) -> None:
        """Record that these track_ids co-occurred in same frame (different objects).

        Args:
            track_ids: List of track_ids present in current frame
        """
        # All pairs of track_ids in same frame can't be same object
        for i, tid1 in enumerate(track_ids):
            for tid2 in track_ids[i + 1 :]:
                self.negative_pairs.setdefault(tid1, set()).add(tid2)
                self.negative_pairs.setdefault(tid2, set()).add(tid1)

    def associate(self, track_id: int) -> int:
        """Associate track_id to long-term unique detection ID.

        Args:
            track_id: Short-term track ID to associate

        Returns:
            Long-term unique detection ID
        """
        # Already has assignment
        if track_id in self.track_to_long_term:
            return self.track_to_long_term[track_id]

        # Need embeddings to compare
        if track_id not in self.track_embeddings or not self.track_embeddings[track_id]:
            # Create new ID if no embeddings yet
            new_id = self.long_term_counter
            self.long_term_counter += 1
            self.track_to_long_term[track_id] = new_id
            return new_id

        # Get query embeddings
        query_embeddings = self.track_embeddings[track_id]

        # Don't attempt matching until we have enough embeddings for the query track
        if len(query_embeddings) < self.min_embeddings_for_matching:
            # Not ready yet - return -1
            return -1

        # Build candidate list (only tracks with assigned long_term_ids)
        best_similarity = -1.0
        best_track_id = None

        for other_tid, other_embeddings in self.track_embeddings.items():
            # Skip self
            if other_tid == track_id:
                continue

            # Skip if negative constraint (co-occurred)
            if other_tid in self.negative_pairs.get(track_id, set()):
                continue

            # Skip if no long_term_id yet
            if other_tid not in self.track_to_long_term:
                continue

            # Skip if not enough embeddings
            if len(other_embeddings) < self.min_embeddings_for_matching:
                continue

            # Compute group similarity
            similarity = self._compute_group_similarity(query_embeddings, other_embeddings)

            if similarity > best_similarity:
                best_similarity = similarity
                best_track_id = other_tid

        # Check if best match exceeds threshold
        if best_track_id is not None and best_similarity >= self.similarity_threshold:
            matched_long_term_id = self.track_to_long_term[best_track_id]
            print(
                f"Track {track_id}: matched with track {best_track_id} "
                f"(long_term_id={matched_long_term_id}, similarity={best_similarity:.4f}, "
                f"mode={self.comparison_mode}, embeddings: {len(query_embeddings)} vs {len(self.track_embeddings[best_track_id])}), threshold: {self.similarity_threshold}"
            )

            # Track similarity history
            self.similarity_history.append(best_similarity)

            # Associate with existing long_term_id
            self.track_to_long_term[track_id] = matched_long_term_id
            return matched_long_term_id

        # Create new unique detection ID
        new_id = self.long_term_counter
        self.long_term_counter += 1
        self.track_to_long_term[track_id] = new_id

        if best_track_id is not None:
            print(
                f"Track {track_id}: creating new ID {new_id} "
                f"(best similarity={best_similarity:.4f} with id={self.track_to_long_term[best_track_id]} below threshold={self.similarity_threshold})"
            )

        return new_id

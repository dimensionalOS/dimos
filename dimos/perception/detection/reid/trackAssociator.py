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

from typing import Dict, List, Set

import torch
import torch.nn.functional as F

from dimos.perception.detection.reid.base import Embedding, EmbeddingModel


class TrackAssociator:
    """Associates short-term track_ids to long-term unique detection IDs via embedding similarity.

    Maintains:
    - Running average embeddings per track_id (on GPU)
    - Negative constraints from co-occurrence (tracks in same frame = different objects)
    - Mapping from track_id to unique long-term ID
    """

    def __init__(self, model: EmbeddingModel, similarity_threshold: float = 0.75):
        """Initialize track associator.

        Args:
            model: Embedding model for GPU-accelerated comparisons
            similarity_threshold: Minimum similarity for associating tracks (0-1)
        """
        self.model = model
        self.device = model.device
        self.similarity_threshold = similarity_threshold

        # Track embeddings (running average, kept on GPU)
        self.track_embeddings: Dict[int, torch.Tensor] = {}
        self.embedding_counts: Dict[int, int] = {}

        # Negative constraints (track_ids that co-occurred = different objects)
        self.negative_pairs: Dict[int, Set[int]] = {}

        # Track ID to long-term unique ID mapping
        self.track_to_long_term: Dict[int, int] = {}
        self.long_term_counter: int = 0

        # Similarity history for optional adaptive thresholding
        self.similarity_history: List[float] = []

    def update_embedding(self, track_id: int, new_embedding: Embedding) -> None:
        """Update running average embedding for a track_id.

        Args:
            track_id: Short-term track ID from detector
            new_embedding: New embedding to incorporate into average
        """
        # Convert to torch on device (no-op if already on device)
        new_vec = new_embedding.to_torch(self.device)

        # Debug: check embedding diversity
        print(
            f"Track {track_id}: embedding norm={new_vec.norm().item():.3f}, first 3 values={new_vec[:3].cpu().tolist()}"
        )

        if track_id in self.track_embeddings:
            # Running average
            count = self.embedding_counts[track_id]
            old_avg = self.track_embeddings[track_id]

            # Compute average on GPU
            new_avg = (old_avg * count + new_vec) / (count + 1)

            # Re-normalize (important for cosine similarity)
            new_avg = F.normalize(new_avg, dim=-1)

            self.track_embeddings[track_id] = new_avg
            self.embedding_counts[track_id] += 1
        else:
            # First embedding for this track (normalize for consistency)
            self.track_embeddings[track_id] = F.normalize(new_vec, dim=-1)
            self.embedding_counts[track_id] = 1

    def add_negative_constraints(self, track_ids: List[int]) -> None:
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
            Long-term unique detection ID, or -1 if not ready yet
        """
        # Already has assignment
        if track_id in self.track_to_long_term:
            return self.track_to_long_term[track_id]

        # Need embedding to compare
        if track_id not in self.track_embeddings:
            return -1  # Not ready yet

        # Build candidate list (only tracks with assigned long_term_ids)
        query_vec = self.track_embeddings[track_id]

        candidates = []
        candidate_track_ids = []

        for other_tid, other_vec in self.track_embeddings.items():
            # Skip self
            if other_tid == track_id:
                continue
            # Skip if negative constraint (co-occurred)
            if other_tid in self.negative_pairs.get(track_id, set()):
                continue
            # Skip if no long_term_id yet
            if other_tid not in self.track_to_long_term:
                continue

            candidates.append(other_vec)
            candidate_track_ids.append(other_tid)

        if candidates:
            # GPU-accelerated comparison (single matrix multiplication)
            candidate_stack = torch.stack(candidates)  # [N, D]
            similarities = query_vec @ candidate_stack.T  # [N]

            # Find best match
            best_sim, best_idx = similarities.max(dim=0)
            best_sim_value = best_sim.item()  # Move to CPU only for comparison

            # Debug: show similarity values and check for exact match
            matched_track_id = candidate_track_ids[best_idx]
            matched_long_term_id = self.track_to_long_term[matched_track_id]

            # Check if embeddings are actually identical
            matched_vec = self.track_embeddings[matched_track_id]
            diff = (query_vec - matched_vec).abs().max().item()

            print(
                f"Track {track_id}: best similarity = {best_sim_value:.6f} with track {matched_track_id} "
                f"(long_term_id={matched_long_term_id}, max_diff={diff:.6f}, counts: {self.embedding_counts[track_id]} vs {self.embedding_counts[matched_track_id]})"
            )

            # Track similarity distribution (for future adaptive thresholding)
            self.similarity_history.append(best_sim_value)

            if best_sim_value >= self.similarity_threshold:
                # Associate with existing long_term_id
                matched_track_id = candidate_track_ids[best_idx]
                long_term_id = self.track_to_long_term[matched_track_id]
                self.track_to_long_term[track_id] = long_term_id
                return long_term_id

        # Create new unique detection ID
        new_id = self.long_term_counter
        self.long_term_counter += 1
        self.track_to_long_term[track_id] = new_id
        return new_id

# Copyright 2026 Dimensional Inc.
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

"""Tools for helping compute groundtruth loop closures from a trajectory."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

DEFAULT_MIN_FRAME_GAP = 50
DEFAULT_MAX_LOOP_DISTANCE_M = 4.0


@dataclass
class LoopGroundtruth:
    min_frame_gap: int
    max_distance_m: float
    valid_loops_per_query: dict[int, set[int]]

    @property
    def queries_with_loop(self) -> int:
        return sum(1 for v in self.valid_loops_per_query.values() if v)

    @property
    def total_loop_pairs(self) -> int:
        return sum(len(v) for v in self.valid_loops_per_query.values())


def compute_loop_groundtruth(
    frame_ids: list[int],
    positions_xyz: np.ndarray,
    min_frame_gap: int = DEFAULT_MIN_FRAME_GAP,
    max_distance_m: float = DEFAULT_MAX_LOOP_DISTANCE_M,
) -> LoopGroundtruth:
    """
    Args:
        frame_ids: ordered list of frame IDs (e.g. KITTI frame indices).
        positions_xyz: (N, 3) world-frame translation of each frame.
        min_frame_gap: minimum index distance (in this list) to count.
        max_distance_m: spatial radius for a positive loop.

    Returns:
        ``LoopGroundtruth`` with ``valid_loops_per_query``: query frame_id
        → set of earlier frame_ids that satisfy both thresholds.
    """
    if positions_xyz.shape != (len(frame_ids), 3):
        raise ValueError(
            f"positions_xyz shape {positions_xyz.shape} doesn't match "
            f"len(frame_ids)={len(frame_ids)}"
        )

    valid: dict[int, set[int]] = {frame_id: set() for frame_id in frame_ids}
    for query_index in range(len(frame_ids)):
        if query_index < min_frame_gap:
            continue
        # Bound the search: any candidate with |query - candidate| >= min_frame_gap.
        upper_candidate_index = query_index - min_frame_gap
        if upper_candidate_index < 0:
            continue
        deltas = positions_xyz[: upper_candidate_index + 1] - positions_xyz[query_index]
        distances = np.linalg.norm(deltas, axis=1)
        matches = np.where(distances <= max_distance_m)[0]
        for candidate_index in matches:
            valid[frame_ids[query_index]].add(frame_ids[int(candidate_index)])

    return LoopGroundtruth(
        min_frame_gap=min_frame_gap,
        max_distance_m=max_distance_m,
        valid_loops_per_query=valid,
    )


@dataclass
class LoopMetrics:
    true_positive: int
    false_positive: int
    false_negative: int

    @property
    def precision(self) -> float:
        denom = self.true_positive + self.false_positive
        return self.true_positive / denom if denom > 0 else float("nan")

    @property
    def recall(self) -> float:
        denom = self.true_positive + self.false_negative
        return self.true_positive / denom if denom > 0 else float("nan")

    @property
    def f1(self) -> float:
        precision, recall = self.precision, self.recall
        if not (precision > 0 and recall > 0):
            return 0.0
        return 2.0 * precision * recall / (precision + recall)


def score_detected_loops(
    detected_pairs: list[tuple[int, int]],
    groundtruth: LoopGroundtruth,
) -> LoopMetrics:
    """Score detected (query_id, candidate_id) pairs against groundtruth.

    All three counts are query-level so precision/recall stay
    dimensionally consistent. The "query" of a detected pair is the
    later frame_id. A query contributes 1 TP if any of its detected
    edges matched groundtruth, otherwise 1 FP. Duplicate detections
    for the same query collapse. Match is order-agnostic — PGO may
    report (target, source) or (source, target).
    """
    seen_queries_with_hit: set[int] = set()
    seen_queries_without_hit: set[int] = set()
    queries_with_any_groundtruth = {
        query_frame_id
        for query_frame_id, valid in groundtruth.valid_loops_per_query.items()
        if valid
    }

    for source_frame_id, target_frame_id in detected_pairs:
        source_valid = groundtruth.valid_loops_per_query.get(source_frame_id, set())
        target_valid = groundtruth.valid_loops_per_query.get(target_frame_id, set())
        query_frame_id = max(source_frame_id, target_frame_id)
        if target_frame_id in source_valid or source_frame_id in target_valid:
            seen_queries_with_hit.add(query_frame_id)
        else:
            seen_queries_without_hit.add(query_frame_id)
    seen_queries_without_hit -= seen_queries_with_hit

    return LoopMetrics(
        true_positive=len(seen_queries_with_hit),
        false_positive=len(seen_queries_without_hit),
        false_negative=len(queries_with_any_groundtruth - seen_queries_with_hit),
    )

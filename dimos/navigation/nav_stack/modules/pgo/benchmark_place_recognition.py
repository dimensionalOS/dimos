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

"""How well can Scan Context tell "I've been here before" on KITTI-360?

Score: Average Precision (AP): a single 0-1 number, Higher = better
Replays a real driven trajectory, and for every frame asks the
descriptor: of all the places I saw a while ago, which one looks most
like where I am now? If that "most similar" old place is actually
within a few metres of where I am, that's a correct revisit detection.

The published Scan Context paper gets 0.65-0.78 on this sequence, so that's the bar.

Usage:
    uv run python -m dimos.navigation.nav_stack.modules.pgo.benchmark_place_recognition \\
        --kitti360-root ~/datasets/kitti360 --sequence 2
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree
from sklearn.metrics import average_precision_score  # type: ignore[import-untyped]

from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.kitti360_loader import (
    load_kitti360_sequence,
)
from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.loop_groundtruth import (
    DEFAULT_MAX_LOOP_DISTANCE_M,
    DEFAULT_MIN_FRAME_GAP,
    compute_loop_groundtruth,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class ScanContextConfig:
    """Mirror of cpp/scan_context.h scan_context::Config defaults."""

    num_rings: int = 20
    num_sectors: int = 60
    max_range_m: float = 80.0
    lidar_height_m: float = 2.0


def make_descriptor(points_body: np.ndarray, config: ScanContextConfig) -> np.ndarray:
    """Polar max-z descriptor — matches cpp/scan_context.cpp::make_descriptor.

    ``points_body``: (N, 3+) body-frame point cloud.
    Returns: (num_rings, num_sectors) float32 with cell value = max(z + lidar_height, 0)
    for points falling in that (range, azimuth) bin.
    """
    descriptor = np.zeros((config.num_rings, config.num_sectors), dtype=np.float32)
    if len(points_body) == 0:
        return descriptor

    x = points_body[:, 0]
    y = points_body[:, 1]
    z = points_body[:, 2]

    range_xy = np.sqrt(x * x + y * y)
    valid = (range_xy < config.max_range_m) & (range_xy > 1e-6)
    if not valid.any():
        return descriptor

    range_valid = range_xy[valid]
    azimuth = np.arctan2(y[valid], x[valid])
    azimuth = np.where(azimuth < 0, azimuth + 2 * np.pi, azimuth)
    z_shifted = np.maximum(z[valid] + config.lidar_height_m, 0.0)

    ring_step = config.max_range_m / config.num_rings
    sector_step = 2 * np.pi / config.num_sectors
    rings = np.clip(np.floor(range_valid / ring_step).astype(np.int32), 0, config.num_rings - 1)
    sectors = np.clip(np.floor(azimuth / sector_step).astype(np.int32), 0, config.num_sectors - 1)

    flat_idx = rings * config.num_sectors + sectors
    np.maximum.at(descriptor.ravel(), flat_idx, z_shifted.astype(np.float32))
    return descriptor


def best_scan_context_distance(query: np.ndarray, candidate: np.ndarray) -> tuple[float, int]:
    """Min cosine distance over all column shifts — matches cpp::best_distance.

    Returns (min_distance, best_shift). 0 = identical, 2 = opposite.
    Each shift's score is mean(1 - cosine_sim) across columns whose
    norms are both non-zero (matches reference's "skip empty sector" logic).
    """
    num_sectors = query.shape[1]
    query_norms = np.linalg.norm(query, axis=0)
    candidate_norms = np.linalg.norm(candidate, axis=0)

    best_distance = 2.0
    best_shift = 0
    for shift in range(num_sectors):
        # roll candidate so candidate_shifted[:, j] = candidate[:, (j + shift) % num_sectors]
        shifted_norms = np.roll(candidate_norms, -shift)
        valid = (query_norms > 1e-6) & (shifted_norms > 1e-6)
        if not valid.any():
            continue
        candidate_shifted = np.roll(candidate, -shift, axis=1)
        dot_products = (query * candidate_shifted).sum(axis=0)
        similarities = dot_products[valid] / (query_norms[valid] * shifted_norms[valid])
        distance = float(1.0 - similarities.mean())
        if distance < best_distance:
            best_distance = distance
            best_shift = shift
    return best_distance, best_shift


def main() -> None:
    parser = argparse.ArgumentParser(description="Place-recognition AP eval (KITTI-360)")
    parser.add_argument("--kitti360-root", type=Path, required=True)
    parser.add_argument("--sequence", type=int, default=2)
    parser.add_argument(
        "--max-scans",
        type=int,
        default=None,
        help="cap total frames evaluated (default: full sequence)",
    )
    parser.add_argument("--min-frame-gap", type=int, default=DEFAULT_MIN_FRAME_GAP)
    parser.add_argument("--max-loop-distance-m", type=float, default=DEFAULT_MAX_LOOP_DISTANCE_M)
    parser.add_argument(
        "--candidate-top-k",
        type=int,
        default=10,
        help="ring-key kd-tree prefilter size (Kim & Kim default)",
    )
    parser.add_argument(
        "--brute-force",
        action="store_true",
        help="skip ring-key prefilter; score every past candidate (slow)",
    )
    args = parser.parse_args()

    config = ScanContextConfig()

    logger.info(f"Loading KITTI-360 sequence {args.sequence} from {args.kitti360_root}")
    sequence = load_kitti360_sequence(args.kitti360_root, args.sequence)
    frame_ids = sequence.frame_ids
    if args.max_scans:
        frame_ids = frame_ids[: args.max_scans]
    num_frames = len(frame_ids)
    logger.info(f"{num_frames} frames")

    positions = np.array([sequence.lidar_pose(frame_id)[:3, 3] for frame_id in frame_ids])
    travelled = float(np.linalg.norm(positions[-1] - positions[0]))
    logger.info(f"trajectory ~{travelled:.1f}m end-to-end")

    groundtruth = compute_loop_groundtruth(
        frame_ids,
        positions,
        min_frame_gap=args.min_frame_gap,
        max_distance_m=args.max_loop_distance_m,
    )
    queries_with_gt = sum(1 for v in groundtruth.valid_loops_per_query.values() if v)
    total_pairs = sum(len(v) for v in groundtruth.valid_loops_per_query.values())
    logger.info(
        f"GT: {queries_with_gt} queries have a valid loop "
        f"(min_gap={args.min_frame_gap}, radius={args.max_loop_distance_m}m), "
        f"{total_pairs} total valid pairs"
    )

    logger.info("Building SC descriptors...")
    build_start = time.time()
    descriptors = np.zeros((num_frames, config.num_rings, config.num_sectors), dtype=np.float32)
    ring_keys = np.zeros((num_frames, config.num_rings), dtype=np.float32)
    for i, frame_id in enumerate(frame_ids):
        scan = sequence.scan_xyz(frame_id)
        descriptors[i] = make_descriptor(scan, config)
        ring_keys[i] = descriptors[i].mean(axis=1)
        if (i + 1) % 500 == 0:
            rate = (i + 1) / (time.time() - build_start)
            logger.info(f"  {i + 1}/{num_frames} ({rate:.0f} scans/s)")
    logger.info(f"Built {num_frames} descriptors in {time.time() - build_start:.1f}s")

    logger.info("Computing top-1 SC matches per query...")
    score_start = time.time()
    top_match_distances = np.full(num_frames, 2.0, dtype=np.float64)
    is_true_positive = np.zeros(num_frames, dtype=bool)
    has_any_groundtruth = np.zeros(num_frames, dtype=bool)

    eval_count = 0
    for query_index, frame_id in enumerate(frame_ids):
        max_candidate_index = query_index - args.min_frame_gap
        if max_candidate_index < 0:
            continue
        eval_count += 1
        valid_set = groundtruth.valid_loops_per_query.get(frame_id, set())
        has_any_groundtruth[query_index] = bool(valid_set)

        if args.brute_force:
            candidate_indices: list[int] = list(range(max_candidate_index + 1))
        else:
            past_keys = ring_keys[: max_candidate_index + 1]
            tree = cKDTree(past_keys)
            top_k = min(args.candidate_top_k, max_candidate_index + 1)
            _, neighbor_indices = tree.query(ring_keys[query_index], k=top_k)
            candidate_indices = (
                [int(neighbor_indices)]
                if top_k == 1
                else [int(index) for index in neighbor_indices]
            )

        best_distance = 2.0
        best_candidate_index = -1
        for candidate_index in candidate_indices:
            distance, _shift = best_scan_context_distance(
                descriptors[query_index], descriptors[candidate_index]
            )
            if distance < best_distance:
                best_distance = distance
                best_candidate_index = candidate_index

        top_match_distances[query_index] = best_distance
        if best_candidate_index >= 0 and frame_ids[best_candidate_index] in valid_set:
            is_true_positive[query_index] = True

        if eval_count % 200 == 0:
            elapsed = time.time() - score_start
            logger.info(
                f"  scored {eval_count} queries  ({eval_count / elapsed:.1f} q/s, "
                f"running TP={is_true_positive.sum()}, has_gt={has_any_groundtruth.sum()})"
            )

    logger.info(f"Scoring done in {time.time() - score_start:.1f}s")

    # AP: rank queries by score = -top_match_distances (high = more confident "this is a loop")
    eval_mask = np.arange(num_frames) >= args.min_frame_gap
    y_true = is_true_positive[eval_mask].astype(np.int32)
    y_score = -top_match_distances[eval_mask]
    num_evaluated = int(eval_mask.sum())
    num_with_groundtruth = int(has_any_groundtruth[eval_mask].sum())
    num_true_positives = int(y_true.sum())

    average_precision = (
        float(average_precision_score(y_true, y_score)) if y_true.any() else float("nan")
    )

    # Manual P/R sweep at representative SC-distance thresholds.
    # At threshold T: a query is "predicted loop" iff its top_match_distances <= T.
    #   precision = (#predicted ∧ is_true_positive) / #predicted
    #   recall    = (#predicted ∧ is_true_positive) / #queries_with_any_gt
    eval_distances = top_match_distances[eval_mask]
    pr_rows = []
    for threshold in (0.13, 0.20, 0.30, 0.40, 0.50, 0.60, 0.80, 1.00):
        predicted = eval_distances <= threshold
        true_positives_at_threshold = int(np.logical_and(predicted, y_true).sum())
        num_predicted = int(predicted.sum())
        precision = (
            true_positives_at_threshold / num_predicted if num_predicted > 0 else float("nan")
        )
        recall = (
            true_positives_at_threshold / num_with_groundtruth
            if num_with_groundtruth > 0
            else float("nan")
        )
        if (precision + recall) and not np.isnan(precision + recall):
            f1 = 2 * precision * recall / (precision + recall)
        else:
            f1 = 0.0
        pr_rows.append(
            (threshold, num_predicted, true_positives_at_threshold, precision, recall, f1)
        )

    print("")
    print(f"=== KITTI-360 seq {args.sequence} — Place Recognition (Scan Context) ===")
    print(f"frames evaluated:           {num_evaluated}")
    print(f"queries with any valid GT:  {num_with_groundtruth}")
    print(f"top-1 matches that are TP:  {num_true_positives}")
    print("")
    print(f"Average Precision (AP):     {average_precision:.4f}")
    print("")
    print("PR points (SC distance threshold):")
    print(
        f"  {'threshold':>9s}  {'predicted':>9s}  {'true_positives':>14s}  "
        f"{'precision':>9s}  {'recall':>8s}  {'F1':>6s}"
    )
    for threshold, num_predicted, true_positives, precision, recall, f1 in pr_rows:
        print(
            f"  {threshold:>9.2f}  {num_predicted:>9d}  {true_positives:>14d}  "
            f"{precision:>9.4f}  {recall:>8.4f}  {f1:>6.4f}"
        )


if __name__ == "__main__":
    main()

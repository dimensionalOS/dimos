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

"""Place-recognition AP evaluator for the Scan Context descriptor.

This is the apples-to-apples comparison with published numbers like
Scan Context (Kim & Kim 2018) AP=0.65-0.78 on KITTI-360 seq 02. It
evaluates just the descriptor — no iSAM2, no ICP — so we measure
descriptor discriminative power directly, the same way the reference
papers do (LCDNet Protocol 1).

For each query frame i (with i >= MIN_FRAME_GAP):
    candidates = past frames in [0, i - MIN_FRAME_GAP]
    1. ring-key kd-tree prefilter to top-K (default 10, matching Kim & Kim)
    2. full column-shifted cosine distance against each candidate
    3. top-1 candidate = argmin distance
    is_tp[i]  = top-1's frame_id is within MAX_LOOP_DISTANCE_M of query
    score[i]  = -top1_distance  (high = confident match)

AP = sklearn.metrics.average_precision_score(is_tp, score). Sweeps the
threshold implicitly. Also reports precision/recall at a few specific
thresholds for reference.

The Python descriptor matches cpp/scan_context.cpp exactly: same
polar binning, same lidar_height_m=2.0 shift, same column-cosine
distance over all sector shifts.

Usage:
    uv run python -m dimos.navigation.nav_stack.modules.pgo.benchmark.place_recognition_ap \\
        --kitti360-root ~/datasets/kitti360 --sequence 2
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree
from sklearn.metrics import average_precision_score

from dimos.navigation.nav_stack.modules.pgo.benchmark.kitti360_loader import (
    load_kitti360_sequence,
)
from dimos.navigation.nav_stack.modules.pgo.benchmark.loop_groundtruth import (
    DEFAULT_MAX_LOOP_DISTANCE_M,
    DEFAULT_MIN_FRAME_GAP,
    compute_loop_groundtruth,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class SCConfig:
    """Mirror of cpp/scan_context.h scan_context::Config defaults."""

    n_rings: int = 20
    n_sectors: int = 60
    max_range_m: float = 80.0
    lidar_height_m: float = 2.0


def make_descriptor(points_body: np.ndarray, config: SCConfig) -> np.ndarray:
    """Polar max-z descriptor — matches cpp/scan_context.cpp::make_descriptor.

    ``points_body``: (N, 3+) body-frame point cloud.
    Returns: (n_rings, n_sectors) float32 with cell value = max(z + lidar_height, 0)
    for points falling in that (range, azimuth) bin.
    """
    descriptor = np.zeros((config.n_rings, config.n_sectors), dtype=np.float32)
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

    ring_step = config.max_range_m / config.n_rings
    sector_step = 2 * np.pi / config.n_sectors
    rings = np.clip(np.floor(range_valid / ring_step).astype(np.int32), 0, config.n_rings - 1)
    sectors = np.clip(np.floor(azimuth / sector_step).astype(np.int32), 0, config.n_sectors - 1)

    flat_idx = rings * config.n_sectors + sectors
    np.maximum.at(descriptor.ravel(), flat_idx, z_shifted.astype(np.float32))
    return descriptor


def best_sc_distance(query: np.ndarray, candidate: np.ndarray) -> tuple[float, int]:
    """Min cosine distance over all column shifts — matches cpp::best_distance.

    Returns (min_distance, best_shift). 0 = identical, 2 = opposite.
    Each shift's score is mean(1 - cosine_sim) across columns whose
    norms are both non-zero (matches reference's "skip empty sector" logic).
    """
    n_sectors = query.shape[1]
    query_norms = np.linalg.norm(query, axis=0)
    candidate_norms = np.linalg.norm(candidate, axis=0)

    best_distance = 2.0
    best_shift = 0
    for shift in range(n_sectors):
        # roll candidate so candidate_shifted[:, j] = candidate[:, (j + shift) % n_sectors]
        shifted_norms = np.roll(candidate_norms, -shift)
        valid = (query_norms > 1e-6) & (shifted_norms > 1e-6)
        if not valid.any():
            continue
        candidate_shifted = np.roll(candidate, -shift, axis=1)
        dots = (query * candidate_shifted).sum(axis=0)
        sims = dots[valid] / (query_norms[valid] * shifted_norms[valid])
        distance = float(1.0 - sims.mean())
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

    config = SCConfig()

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
    descriptors = np.zeros((num_frames, config.n_rings, config.n_sectors), dtype=np.float32)
    ring_keys = np.zeros((num_frames, config.n_rings), dtype=np.float32)
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
    top1_dist = np.full(num_frames, 2.0, dtype=np.float64)
    is_tp = np.zeros(num_frames, dtype=bool)
    has_any_gt = np.zeros(num_frames, dtype=bool)

    eval_count = 0
    for i, frame_id in enumerate(frame_ids):
        max_j = i - args.min_frame_gap
        if max_j < 0:
            continue
        eval_count += 1
        valid_set = groundtruth.valid_loops_per_query.get(frame_id, set())
        has_any_gt[i] = bool(valid_set)

        if args.brute_force:
            candidate_indices: list[int] = list(range(max_j + 1))
        else:
            past_keys = ring_keys[: max_j + 1]
            tree = cKDTree(past_keys)
            k = min(args.candidate_top_k, max_j + 1)
            _, raw = tree.query(ring_keys[i], k=k)
            candidate_indices = [int(raw)] if k == 1 else [int(x) for x in raw]

        best_distance = 2.0
        best_j = -1
        for j in candidate_indices:
            distance, _shift = best_sc_distance(descriptors[i], descriptors[j])
            if distance < best_distance:
                best_distance = distance
                best_j = j

        top1_dist[i] = best_distance
        if best_j >= 0 and frame_ids[best_j] in valid_set:
            is_tp[i] = True

        if eval_count % 200 == 0:
            elapsed = time.time() - score_start
            logger.info(
                f"  scored {eval_count} queries  ({eval_count / elapsed:.1f} q/s, "
                f"running TP={is_tp.sum()}, has_gt={has_any_gt.sum()})"
            )

    logger.info(f"Scoring done in {time.time() - score_start:.1f}s")

    # AP: rank queries by score = -top1_dist (high = more confident "this is a loop")
    eval_mask = np.arange(num_frames) >= args.min_frame_gap
    y_true = is_tp[eval_mask].astype(np.int32)
    y_score = -top1_dist[eval_mask]
    n_eval = int(eval_mask.sum())
    n_has_gt = int(has_any_gt[eval_mask].sum())
    n_tp = int(y_true.sum())

    ap = float(average_precision_score(y_true, y_score)) if y_true.any() else float("nan")

    # Manual P/R sweep at representative SC-distance thresholds.
    # At threshold T: a query is "predicted loop" iff its top1_dist <= T.
    #   precision = (#predicted ∧ is_tp) / #predicted
    #   recall    = (#predicted ∧ is_tp) / #queries_with_any_gt
    dists = top1_dist[eval_mask]
    rows = []
    for t in (0.13, 0.20, 0.30, 0.40, 0.50, 0.60, 0.80, 1.00):
        predicted = dists <= t
        tp_at = int(np.logical_and(predicted, y_true).sum())
        n_pred = int(predicted.sum())
        p = tp_at / n_pred if n_pred > 0 else float("nan")
        r = tp_at / n_has_gt if n_has_gt > 0 else float("nan")
        f1 = 2 * p * r / (p + r) if (p + r) and not np.isnan(p + r) else 0.0
        rows.append((t, n_pred, tp_at, p, r, f1))

    print("")
    print(f"=== KITTI-360 seq {args.sequence} — Place Recognition (Scan Context) ===")
    print(f"frames evaluated:           {n_eval}")
    print(f"queries with any valid GT:  {n_has_gt}")
    print(f"top-1 matches that are TP:  {n_tp}")
    print("")
    print(f"Average Precision (AP):     {ap:.4f}")
    print("")
    print("PR points (SC distance threshold):")
    print(
        f"  {'thresh':>8s}  {'n_pred':>7s}  {'n_tp':>6s}  {'precision':>10s}  {'recall':>8s}  {'F1':>6s}"
    )
    for t, n_pred, tp_at, p, r, f1 in rows:
        print(f"  {t:>8.2f}  {n_pred:>7d}  {tp_at:>6d}  {p:>10.4f}  {r:>8.4f}  {f1:>6.4f}")


if __name__ == "__main__":
    main()

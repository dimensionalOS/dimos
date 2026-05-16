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

"""Run the PGO native module against a KITTI-360 sequence and score it.

Usage:
    uv run python -m dimos.navigation.nav_stack.modules.pgo.benchmark.run_kitti360_benchmark \\
        --kitti360-root /data/kitti360 --sequence 9 --max-scans 4000

Pipeline:
1. Load scans + groundtruth poses for the given KITTI-360 sequence.
2. Compute the loop-pair groundtruth (≥50 frame gap, ≤4m radius).
3. Spawn the PGO native binary with private LCM topics.
4. Play (registered_scan, odometry) at controlled rate via LCM.
5. Subscribe to ``pgo_graph_edges`` to extract detected loop pairs
   (traversability=0.4 segments) and ``pgo_loop_closure`` for delta
   events (count only — deltas aren't scored here).
6. Score precision / recall / F1 + write a JSON report.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

import lcm as lcmlib
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.navigation.nav_stack.modules.pgo.benchmark.kitti360_loader import (
    Kitti360Sequence,
    load_kitti360_sequence,
)
from dimos.navigation.nav_stack.modules.pgo.benchmark.loop_groundtruth import (
    DEFAULT_MAX_LOOP_DISTANCE_M,
    DEFAULT_MIN_FRAME_GAP,
    LoopMetrics,
    compute_loop_groundtruth,
    score_detected_loops,
)
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    NativeProcessRunner,
    lcm_handle_loop,
    make_isolated_lcm_url,
    make_odometry_msg,
    make_pointcloud_msg,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PGO_BIN = Path(__file__).resolve().parent.parent / "cpp" / "result" / "bin" / "pgo"


@dataclass
class BenchmarkConfig:
    kitti360_root: Path
    sequence_id: int
    max_scans: int | None
    min_frame_gap: int = DEFAULT_MIN_FRAME_GAP
    max_loop_distance_m: float = DEFAULT_MAX_LOOP_DISTANCE_M
    use_scan_context: bool = True
    sc_match_threshold: float = 0.4
    loop_score_thresh: float = 0.5
    loop_search_radius_m: float = 1.0
    publish_interval_sec: float = 0.02
    drain_sec: float = 10.0
    output_json: Path | None = None


@dataclass
class BenchmarkResult:
    sequence_id: int
    scans_played: int
    groundtruth_queries_with_loop: int
    groundtruth_total_loop_pairs: int
    detected_loop_edges: int
    loop_closure_events: int
    metrics: LoopMetrics
    wallclock_seconds: float


def _matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix → (x, y, z, w) quaternion."""
    quaternion: np.ndarray = Rotation.from_matrix(matrix).as_quat()
    return quaternion


@dataclass
class BenchmarkState:
    """Mutable counters for what PGO published during the run."""

    pgo_keyframe_count: int = 0
    loop_closure_events: int = 0
    detected_pairs: list[tuple[int, int]] = None  # type: ignore[assignment]
    last_graph_node_count: int = 0
    # Maps round(effective_send_timestamp * 1e3) → frame_id. The keys MUST be
    # computed from the exact same timestamps the runner publishes to PGO; using
    # raw KITTI timestamps here de-syncs from the sent ts and silently drops
    # loop pairs (greptile c2 on PR #2099).
    timestamp_ms_to_frame_id: dict[int, int] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.detected_pairs is None:
            self.detected_pairs = []
        if self.timestamp_ms_to_frame_id is None:
            self.timestamp_ms_to_frame_id = {}


def _compute_send_timestamps(
    sequence: Kitti360Sequence, frame_ids_in_order: list[int]
) -> list[float]:
    """Compute the per-frame timestamp the runner will publish.

    PGO's Odometry constructor treats ``ts==0`` as "now", so we clamp the
    first ts away from zero and enforce strict monotonicity afterward. The
    cache built from these values is what ``_on_graph_edges`` consults — keep
    this function as the single source of truth for both.
    """
    if not frame_ids_in_order:
        return []
    first_timestamp = max(sequence.timestamps.get(frame_ids_in_order[0], 1.0), 1.0)
    send_timestamps: list[float] = []
    for index, frame_id in enumerate(frame_ids_in_order):
        raw_timestamp = sequence.timestamps.get(frame_id, float(index))
        send_timestamps.append(max(raw_timestamp, first_timestamp + index * 0.001))
    return send_timestamps


def _on_loop_closure(state: BenchmarkState, _channel: str, data: bytes) -> None:
    message = NavPath.lcm_decode(data)
    state.loop_closure_events += 1
    # The new keyframe count == len(message.poses) for our publisher.
    state.last_graph_node_count = max(state.last_graph_node_count, len(message.poses))


def _on_graph_edges(
    state: BenchmarkState,
    _channel: str,
    data: bytes,
) -> None:
    """Extract loop-closure edges (traversability ~ 0.4) and map each
    endpoint back to the originating KITTI frame_id via its keyframe
    creation timestamp.

    Each loop edge is a pair of consecutive ``poses[i], poses[i+1]``
    with traversability=0.4. PGO stamps each endpoint's header with the
    keyframe's *creation* timestamp (not the message publish time), so
    we can look up which input scan produced each endpoint regardless
    of how much iSAM2 has since shifted the keyframe's world position.

    The ``state.timestamp_ms_to_frame_id`` cache must be pre-populated by
    ``run_benchmark`` using the same per-frame send timestamps the runner
    publishes — otherwise the lookup keys don't line up.
    """
    message = NavPath.lcm_decode(data)

    def _timestamp_to_frame(timestamp_sec: float) -> int | None:
        timestamp_ms = round(timestamp_sec * 1e3)
        # Allow ±1ms slop (PoseStamped ts rounds through int sec + uint nsec)
        for slop_ms in (0, -1, 1):
            frame_id = state.timestamp_ms_to_frame_id.get(timestamp_ms + slop_ms)
            if frame_id is not None:
                return frame_id
        return None

    pose_index = 0
    while pose_index + 1 < len(message.poses):
        start_pose = message.poses[pose_index]
        end_pose = message.poses[pose_index + 1]
        traversability = float(start_pose.orientation.w)
        if abs(traversability - 0.4) < 0.05:
            start_frame_id = _timestamp_to_frame(start_pose.ts)
            end_frame_id = _timestamp_to_frame(end_pose.ts)
            if start_frame_id is not None and end_frame_id is not None:
                pair = (start_frame_id, end_frame_id)
                if pair not in state.detected_pairs:
                    state.detected_pairs.append(pair)
        pose_index += 2


def _build_runner(config: BenchmarkConfig, topic_prefix: str) -> NativeProcessRunner:
    return NativeProcessRunner(
        binary_path=str(PGO_BIN),
        args=[
            "--registered_scan",
            f"/{topic_prefix}_scan#sensor_msgs.PointCloud2",
            "--odometry",
            f"/{topic_prefix}_odom#nav_msgs.Odometry",
            "--corrected_odometry",
            f"/{topic_prefix}_corrected#nav_msgs.Odometry",
            "--global_map",
            f"/{topic_prefix}_global_map#sensor_msgs.PointCloud2",
            "--pgo_tf",
            f"/{topic_prefix}_tf#nav_msgs.Odometry",
            "--pgo_graph_nodes",
            f"/{topic_prefix}_graph_nodes#nav_msgs.GraphNodes3D",
            "--pgo_graph_edges",
            f"/{topic_prefix}_graph_edges#nav_msgs.LineSegments3D",
            "--pgo_loop_closure",
            f"/{topic_prefix}_loop_closure#nav_msgs.Path",
            "--key_pose_delta_deg",
            "10.0",
            "--key_pose_delta_trans",
            "0.5",
            "--loop_search_radius",
            str(config.loop_search_radius_m),
            "--loop_time_thresh",
            "10.0",
            "--loop_score_thresh",
            str(config.loop_score_thresh),
            "--loop_submap_half_range",
            "10",
            "--submap_resolution",
            "0.5",
            "--min_loop_detect_duration",
            "0.0",
            "--global_map_voxel_size",
            "0.5",
            "--global_map_publish_rate",
            "0.5",
            "--unregister_input",
            "true",
            "--use_scan_context",
            "true" if config.use_scan_context else "false",
            "--sc_max_range_m",
            "60.0",
            "--sc_match_threshold",
            str(config.sc_match_threshold),
            "--world_frame",
            "map",
            "--local_frame",
            "odom",
        ],
    )


def run_benchmark(config: BenchmarkConfig) -> BenchmarkResult:
    if not PGO_BIN.exists():
        raise FileNotFoundError(f"PGO binary missing: {PGO_BIN}")

    logger.info(f"Loading KITTI-360 sequence {config.sequence_id} from {config.kitti360_root}")
    sequence = load_kitti360_sequence(config.kitti360_root, config.sequence_id)
    all_frame_ids = sequence.frame_ids
    if config.max_scans is not None:
        frame_ids = all_frame_ids[: config.max_scans]
    else:
        frame_ids = all_frame_ids
    if len(frame_ids) < config.min_frame_gap + 1:
        raise ValueError(
            f"Sequence has {len(frame_ids)} usable frames; need ≥ "
            f"{config.min_frame_gap + 1} to evaluate loop closures."
        )

    positions = np.array([sequence.lidar_pose(frame_id)[:3, 3] for frame_id in frame_ids])
    logger.info(
        f"Trajectory has {len(frame_ids)} frames, "
        f"travelled {float(np.linalg.norm(positions[-1] - positions[0])):.1f}m"
    )

    groundtruth = compute_loop_groundtruth(
        frame_ids,
        positions,
        min_frame_gap=config.min_frame_gap,
        max_distance_m=config.max_loop_distance_m,
    )
    logger.info(
        f"Groundtruth: {groundtruth.queries_with_loop} queries with a loop, "
        f"{groundtruth.total_loop_pairs} total valid loop pairs."
    )

    # Isolate the benchmark from other LCM traffic on the host so we only see
    # the loop-closure events from our own PGO subprocess.
    lcm_url = make_isolated_lcm_url()
    lcm_instance = lcmlib.LCM(lcm_url)
    state = BenchmarkState()
    # Single source of truth: the timestamps the runner will publish are also
    # the keys we cache for the edge-endpoint → frame_id lookup. Building this
    # once before subscribing avoids the cache/publish desync described by
    # greptile c2 on PR #2099.
    send_timestamps = _compute_send_timestamps(sequence, frame_ids)
    for frame_id, send_timestamp in zip(frame_ids, send_timestamps, strict=True):
        state.timestamp_ms_to_frame_id[round(send_timestamp * 1e3)] = frame_id

    topic_prefix = f"kitti360_seq{config.sequence_id:02d}"
    loop_topic = f"/{topic_prefix}_loop_closure#nav_msgs.Path"
    edges_topic = f"/{topic_prefix}_graph_edges#nav_msgs.LineSegments3D"

    loop_sub = lcm_instance.subscribe(
        loop_topic,
        lambda channel, data: _on_loop_closure(state, channel, data),
    )
    edges_sub = lcm_instance.subscribe(
        edges_topic,
        lambda channel, data: _on_graph_edges(state, channel, data),
    )

    stop_event = threading.Event()
    handle_thread = threading.Thread(
        target=lcm_handle_loop, args=(lcm_instance, stop_event), daemon=True
    )
    handle_thread.start()

    runner = _build_runner(config, topic_prefix)
    wallclock_start = time.monotonic()
    try:
        # Capture PGO's stderr so its diagnostic prints (keyframes, sc-search, loop events)
        # are available for debugging. We dump them to the runner's log at the end.
        # Pass the isolated LCM URL through env so the subprocess joins the same
        # bus as our Python publisher/subscriber.
        runner.process = subprocess.Popen(
            [runner.binary_path, *runner.args],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
            env={**os.environ, "LCM_DEFAULT_URL": lcm_url},
        )
        time.sleep(2.0)
        if not runner.is_running:
            raise RuntimeError("PGO native process failed to start")

        scan_topic = f"/{topic_prefix}_scan#sensor_msgs.PointCloud2"
        odom_topic = f"/{topic_prefix}_odom#nav_msgs.Odometry"

        for index, frame_id in enumerate(frame_ids):
            scan_xyz = sequence.scan_xyz(frame_id)
            pose = sequence.lidar_pose(frame_id)
            position = pose[:3, 3]
            quaternion = _matrix_to_quaternion(pose[:3, :3])
            timestamp = send_timestamps[index]

            odometry_message = make_odometry_msg(position, quaternion, ts=timestamp)
            world_xyz = (pose[:3, :3] @ scan_xyz[:, :3].T).T + position
            cloud_message = make_pointcloud_msg(
                np.column_stack([world_xyz, scan_xyz[:, 3:4]]).astype(np.float32),
                ts=timestamp,
            )
            # Odom first so on_registered_scan can read the latest pose.
            lcm_instance.publish(odom_topic, odometry_message.lcm_encode())
            lcm_instance.publish(scan_topic, cloud_message.lcm_encode())

            if config.publish_interval_sec > 0:
                time.sleep(config.publish_interval_sec)
            if (index + 1) % 500 == 0:
                logger.info(
                    f"  played {index + 1}/{len(frame_ids)} scans; "
                    f"loop events: {state.loop_closure_events}, "
                    f"detected pairs: {len(state.detected_pairs)}"
                )

        time.sleep(config.drain_sec)
    finally:
        stderr_bytes = b""
        if runner.process is not None:
            runner.process.terminate()
            try:
                _, stderr_bytes = runner.process.communicate(timeout=3.0)
            except Exception:
                runner.process.kill()
                try:
                    _, stderr_bytes = runner.process.communicate()
                except Exception:
                    pass
            runner.process = None
        stop_event.set()
        handle_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        lcm_instance.unsubscribe(loop_sub)
        lcm_instance.unsubscribe(edges_sub)
        if getattr(config, "print_stderr", False) and stderr_bytes:
            text = stderr_bytes.decode("utf-8", errors="replace")
            logger.info(f"\n--- PGO stderr ({len(text.splitlines())} lines) ---")
            for line in text.splitlines():
                logger.info(f"  {line}")

    wallclock = time.monotonic() - wallclock_start

    # Diagnostic: print every detected pair with positions + verdict, so we can
    # see whether PGO is finding wrong places or whether the scorer is mis-snapping.
    if getattr(config, "print_pairs", False):
        positions_by_frame_id = {
            frame_id: sequence.lidar_pose(frame_id)[:3, 3] for frame_id in frame_ids
        }
        logger.info(f"\n--- {len(state.detected_pairs)} detected pairs ---")
        for source_frame_id, target_frame_id in state.detected_pairs:
            source_position = positions_by_frame_id.get(source_frame_id)
            target_position = positions_by_frame_id.get(target_frame_id)
            if source_position is None or target_position is None:
                logger.info(
                    f"  detected ({source_frame_id}, {target_frame_id}): "
                    "MISSING POSITION (off-window)"
                )
                continue
            world_distance = float(np.linalg.norm(source_position - target_position))
            frame_gap = abs(source_frame_id - target_frame_id)
            source_valid = groundtruth.valid_loops_per_query.get(source_frame_id, set())
            target_valid = groundtruth.valid_loops_per_query.get(target_frame_id, set())
            is_true_positive = target_frame_id in source_valid or source_frame_id in target_valid
            # Find the nearest GT pair for this query
            nearest_groundtruth = "none"
            for query_frame_id in (source_frame_id, target_frame_id):
                if groundtruth.valid_loops_per_query.get(query_frame_id):
                    candidate_positions = np.array(
                        [
                            positions_by_frame_id[candidate_frame_id]
                            for candidate_frame_id in groundtruth.valid_loops_per_query[
                                query_frame_id
                            ]
                            if candidate_frame_id in positions_by_frame_id
                        ]
                    )
                    if len(candidate_positions):
                        query_position = positions_by_frame_id[query_frame_id]
                        distances_to_query = np.linalg.norm(
                            candidate_positions - query_position, axis=1
                        )
                        nearest_groundtruth = (
                            f"q={query_frame_id} has {len(candidate_positions)} "
                            f"valid GT, nearest at {distances_to_query.min():.2f}m"
                        )
                        break
            verdict = "✓TP" if is_true_positive else "✗FP"
            logger.info(
                f"  {verdict} ({source_frame_id}, {target_frame_id}) "
                f"gap={frame_gap} dist={world_distance:.2f}m | gt: {nearest_groundtruth}"
            )

    metrics = score_detected_loops(state.detected_pairs, groundtruth)
    result = BenchmarkResult(
        sequence_id=config.sequence_id,
        scans_played=len(frame_ids),
        groundtruth_queries_with_loop=groundtruth.queries_with_loop,
        groundtruth_total_loop_pairs=groundtruth.total_loop_pairs,
        detected_loop_edges=len(state.detected_pairs),
        loop_closure_events=state.loop_closure_events,
        metrics=metrics,
        wallclock_seconds=wallclock,
    )

    if config.output_json is not None:
        config.output_json.parent.mkdir(parents=True, exist_ok=True)
        config.output_json.write_text(json.dumps(result_to_json(result), indent=2))

    return result


def result_to_json(result: BenchmarkResult) -> dict[str, object]:
    """JSON-safe dict view of a BenchmarkResult (NaN precision/recall → null)."""
    metrics = result.metrics
    payload = asdict(result)
    payload["metrics"] = {
        "true_positive": metrics.true_positive,
        "false_positive": metrics.false_positive,
        "false_negative": metrics.false_negative,
        "precision": metrics.precision if math.isfinite(metrics.precision) else None,
        "recall": metrics.recall if math.isfinite(metrics.recall) else None,
        "f1": metrics.f1,
    }
    return payload


def _format_result(result: BenchmarkResult) -> str:
    metrics = result.metrics
    return (
        f"\n=== KITTI-360 seq {result.sequence_id:02d} — PGO benchmark ===\n"
        f"scans played:            {result.scans_played}\n"
        f"groundtruth queries:     {result.groundtruth_queries_with_loop}\n"
        f"groundtruth loop pairs:  {result.groundtruth_total_loop_pairs}\n"
        f"detected loop edges:     {result.detected_loop_edges}\n"
        f"loop closure events:     {result.loop_closure_events}\n"
        f"true positive:           {metrics.true_positive}\n"
        f"false positive:          {metrics.false_positive}\n"
        f"false negative:          {metrics.false_negative}\n"
        f"precision:               {metrics.precision:.4f}\n"
        f"recall:                  {metrics.recall:.4f}\n"
        f"F1:                      {metrics.f1:.4f}\n"
        f"wallclock:               {result.wallclock_seconds:.1f}s\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kitti360-root", type=Path, required=True)
    parser.add_argument("--sequence", type=int, default=9)
    parser.add_argument(
        "--max-scans",
        type=int,
        default=None,
        help="cap on number of scans (default: full sequence)",
    )
    parser.add_argument(
        "--use-scan-context",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--sc-match-threshold", type=float, default=0.4)
    parser.add_argument("--loop-score-thresh", type=float, default=0.5)
    parser.add_argument("--publish-interval-sec", type=float, default=0.02)
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument(
        "--print-pairs",
        action="store_true",
        help="dump every detected pair + nearest GT (diagnostic)",
    )
    parser.add_argument(
        "--print-stderr", action="store_true", help="dump PGO subprocess stderr (diagnostic)"
    )
    args = parser.parse_args()

    config = BenchmarkConfig(
        kitti360_root=args.kitti360_root,
        sequence_id=args.sequence,
        max_scans=args.max_scans,
        use_scan_context=args.use_scan_context,
        sc_match_threshold=args.sc_match_threshold,
        loop_score_thresh=args.loop_score_thresh,
        publish_interval_sec=args.publish_interval_sec,
        output_json=args.output_json,
    )
    config.print_pairs = args.print_pairs  # type: ignore[attr-defined]
    config.print_stderr = args.print_stderr  # type: ignore[attr-defined]

    result = run_benchmark(config)
    logger.info(_format_result(result))
    print(json.dumps(result_to_json(result), indent=2))


if __name__ == "__main__":
    main()

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

"""Fast PGO liveness probe: feed a small KITTI-360 slice, listen to every
PGO publish topic, capture stderr, report per-topic message counts.

Use this when ``run_kitti360_benchmark`` reports F1=0 to distinguish:
    * PGO crashed or never started        → stderr has the error
    * PGO never promoted any keyframes    → 0 graph_nodes
    * PGO built a graph but no loops      → graph_nodes > 0, edges may be > 0 (odom only), loop_closure == 0
    * Threshold issue (loops, but too few)→ all topics tick, edges > nodes, loop_closure > 0

    uv run python -m dimos.navigation.nav_stack.modules.pgo.benchmark.smoke_test \\
        --kitti360-root ~/datasets/kitti360 --sequence 2 --num-scans 200
"""

from __future__ import annotations

import argparse
from collections import Counter
from collections.abc import Callable
from pathlib import Path
import subprocess
import threading
import time

import lcm as lcmlib
import numpy as np

from dimos.navigation.nav_stack.modules.pgo.benchmark.kitti360_loader import (
    load_kitti360_sequence,
)
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    NativeProcessRunner,
    lcm_handle_loop,
    make_odometry_msg,
    make_pointcloud_msg,
)

PGO_BIN = Path(__file__).resolve().parent.parent / "cpp" / "result" / "bin" / "pgo"

OUTPUT_TOPICS = [
    ("corrected_odometry", "nav_msgs.Odometry"),
    ("global_map", "sensor_msgs.PointCloud2"),
    ("pgo_tf", "nav_msgs.Odometry"),
    ("pgo_graph_nodes", "nav_msgs.Path"),
    ("pgo_graph_edges", "nav_msgs.Path"),
    ("pgo_loop_closure", "nav_msgs.Path"),
]


def _matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    trace = rotation.trace()
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def _build_runner(prefix: str, loop_search_radius_m: float) -> NativeProcessRunner:
    args = [
        "--registered_scan",
        f"/{prefix}_scan#sensor_msgs.PointCloud2",
        "--odometry",
        f"/{prefix}_odom#nav_msgs.Odometry",
    ]
    for name, type_name in OUTPUT_TOPICS:
        args.extend([f"--{name}", f"/{prefix}_{name}#{type_name}"])
    args.extend(
        [
            "--key_pose_delta_deg",
            "10.0",
            "--key_pose_delta_trans",
            "1.0",
            "--loop_search_radius",
            str(loop_search_radius_m),
            "--loop_time_thresh",
            "10.0",
            "--loop_score_thresh",
            "0.5",
            "--loop_submap_half_range",
            "10",
            "--submap_resolution",
            "0.5",
            "--min_loop_detect_duration",
            "1.0",
            "--global_map_voxel_size",
            "0.5",
            "--global_map_publish_rate",
            "0.5",
            "--unregister_input",
            "true",
            "--use_scan_context",
            "true",
            "--sc_max_range_m",
            "60.0",
            "--sc_match_threshold",
            "0.4",
            "--world_frame",
            "map",
            "--local_frame",
            "odom",
        ]
    )
    return NativeProcessRunner(binary_path=str(PGO_BIN), args=args)


def main() -> None:
    parser = argparse.ArgumentParser(description="PGO liveness smoke test")
    parser.add_argument("--kitti360-root", type=Path, required=True)
    parser.add_argument("--sequence", type=int, default=2)
    parser.add_argument("--num-scans", type=int, default=200)
    parser.add_argument(
        "--loop-search-radius",
        type=float,
        default=4.0,
        help="m; default 4.0 matches groundtruth radius (vs runner's 1.0)",
    )
    parser.add_argument("--publish-interval-sec", type=float, default=0.02)
    parser.add_argument("--drain-sec", type=float, default=5.0)
    args = parser.parse_args()

    if not PGO_BIN.exists():
        raise SystemExit(f"PGO binary missing: {PGO_BIN}")

    print(f"loading KITTI-360 sequence {args.sequence} from {args.kitti360_root}")
    sequence = load_kitti360_sequence(args.kitti360_root, args.sequence)
    frame_ids = sequence.frame_ids[: args.num_scans]
    positions = np.array([sequence.lidar_pose(fid)[:3, 3] for fid in frame_ids])
    travelled = float(np.linalg.norm(positions[-1] - positions[0]))
    print(f"playing {len(frame_ids)} scans, ~{travelled:.1f}m of trajectory")

    counts: Counter[str] = Counter()
    prefix = "pgo_smoke"

    lcm_instance = lcmlib.LCM()
    subs = []

    def make_handler(topic_name: str) -> Callable[[str, bytes], None]:
        def handler(_ch: str, _data: bytes) -> None:
            counts.update([topic_name])

        return handler

    for name, type_name in OUTPUT_TOPICS:
        topic = f"/{prefix}_{name}#{type_name}"
        subs.append(lcm_instance.subscribe(topic, make_handler(name)))

    stop_event = threading.Event()
    handle_thread = threading.Thread(
        target=lcm_handle_loop, args=(lcm_instance, stop_event), daemon=True
    )
    handle_thread.start()

    runner = _build_runner(prefix, args.loop_search_radius)
    # Capture stderr ourselves so we get the binary's own diagnostics.
    runner.process = subprocess.Popen(
        [runner.binary_path, *runner.args],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )

    try:
        time.sleep(2.0)
        if not runner.is_running:
            raise SystemExit("PGO subprocess died before playback started")

        scan_topic = f"/{prefix}_scan#sensor_msgs.PointCloud2"
        odom_topic = f"/{prefix}_odom#nav_msgs.Odometry"
        first_ts = max(sequence.timestamps.get(frame_ids[0], 1.0), 1.0)

        for index, frame_id in enumerate(frame_ids):
            pose = sequence.lidar_pose(frame_id)
            position = pose[:3, 3]
            quaternion = _matrix_to_quaternion(pose[:3, :3])
            ts = max(
                sequence.timestamps.get(frame_id, float(index)),
                first_ts + index * 0.001,
            )

            scan_xyz = sequence.scan_xyz(frame_id)
            world_xyz = (pose[:3, :3] @ scan_xyz[:, :3].T).T + position
            cloud = np.column_stack([world_xyz, scan_xyz[:, 3:4]]).astype(np.float32)

            lcm_instance.publish(
                odom_topic, make_odometry_msg(position, quaternion, ts=ts).lcm_encode()
            )
            lcm_instance.publish(scan_topic, make_pointcloud_msg(cloud, ts=ts).lcm_encode())

            if args.publish_interval_sec > 0:
                time.sleep(args.publish_interval_sec)

        time.sleep(args.drain_sec)
    finally:
        stderr_bytes = b""
        if runner.process is not None:
            runner.process.terminate()
            try:
                _, stderr_bytes = runner.process.communicate(timeout=3.0)
            except subprocess.TimeoutExpired:
                runner.process.kill()
                _, stderr_bytes = runner.process.communicate()
            runner.process = None
        stop_event.set()
        handle_thread.join(timeout=2.0)
        for sub in subs:
            lcm_instance.unsubscribe(sub)

    print("\n=== PGO topic message counts ===")
    for name, _ in OUTPUT_TOPICS:
        print(f"  {name:<24} {counts.get(name, 0):>6}")

    stderr_text = stderr_bytes.decode("utf-8", errors="replace").strip()
    if stderr_text:
        lines = stderr_text.splitlines()
        print(f"\n=== PGO stderr ({len(lines)} lines, last 30) ===")
        for line in lines[-30:]:
            print(f"  {line}")
    else:
        print("\n=== PGO stderr: (empty) ===")

    print("\nverdict:")
    if counts.get("pgo_graph_nodes", 0) == 0:
        print("  ⚠ no graph nodes — PGO never promoted a keyframe. Check --key_pose_delta_*.")
    elif counts.get("pgo_graph_edges", 0) == 0:
        print("  ⚠ nodes but no edges — graph isn't being assembled.")
    elif counts.get("pgo_loop_closure", 0) == 0:
        print(
            "  ⚠ graph builds, no loop closure events — try wider --loop-search-radius "
            "or lower --sc-match-threshold."
        )
    else:
        print("  ✓ all topics firing — PGO is alive end-to-end.")


if __name__ == "__main__":
    main()

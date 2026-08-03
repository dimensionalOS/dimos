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

"""Replay a memory2 recording through the cuVSLAM native module and build a map.

    python -m dimos.mapping.cuvslam_native.demo_cuvslam_replay \
        --db ~/datasets/d455/airbnb.db --out /tmp/cuvslam_map

Writes trajectory.npy, landmarks.npy, metrics.json and topdown.png.

cuVSLAM restarts its world frame on a tracking loss, so the module tags each pose
with a segment id and this demo evaluates per segment. Differencing poses across
a segment boundary reads a frame change as motion and inflates the path by orders
of magnitude.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time

import lcm as lcmlib
import numpy as np

from dimos.mapping.cuvslam_native.cuvslam import (
    D455_FACTORY_BASELINE_M,
    CuvslamConfig,
)
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.cmu_nav.tests.rosbag_fixtures import (
    LcmCollector,
    NativeProcessRunner,
    lcm_handle_loop,
)

_TOPICS = {
    "image_left": "/cuvslam_demo_left#sensor_msgs.Image",
    "image_right": "/cuvslam_demo_right#sensor_msgs.Image",
    "camera_info": "/cuvslam_demo_info#sensor_msgs.CameraInfo",
    "odometry": "/cuvslam_demo_odom#nav_msgs.Odometry",
    "landmarks": "/cuvslam_demo_landmarks#sensor_msgs.PointCloud2",
}

_PROCESS_STARTUP_SEC = 3.0
_POST_FEED_DRAIN_SEC = 2.0
# camera_info is recorded at 1 Hz but the tracker needs it before the first pair.
_CAMERA_INFO_REPEAT = 10
# One frame costs ~10 ms on the GPU; waiting for its odometry keeps the 800 kB
# stereo pair from outrunning the socket buffer, and this bounds a lost frame.
_FRAME_REPLY_TIMEOUT_S = 0.12
_PAIR_SKEW_S = 0.001
# Landmarks repeat heavily between frames; every 10th message is plenty of map.
_LANDMARK_STRIDE = 10
_LANDMARK_VOXEL_M = 0.05

# A handheld indoor rig cannot move at this speed; a step implying it is a
# coordinate-frame change, not motion.
_IMPOSSIBLE_SPEED_MPS = 15.0
_MIN_SEGMENT_POSES = 30
_MAP_MARGIN_M = 2.0
_MOVING_SEGMENT_MIN_PATH_M = 2.0


def umeyama(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rigid alignment, rotation + translation only so ATE stays metric."""
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    cov = (dst - mu_dst).T @ (src - mu_src) / len(src)
    u, _sigma, vt = np.linalg.svd(cov)
    correction = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[2, 2] = -1
    rotation = u @ correction @ vt
    return rotation, mu_dst - rotation @ mu_src


def segment_bounds(
    times: np.ndarray, segment_ids: np.ndarray, positions: np.ndarray
) -> list[tuple[int, int]]:
    """Split on the module's segment id, and defensively on impossible speed."""
    breaks = set(np.where(np.diff(segment_ids) != 0)[0] + 1)
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    deltas = np.maximum(np.diff(times), 1e-6)
    breaks |= set(np.where(steps / deltas > _IMPOSSIBLE_SPEED_MPS)[0] + 1)
    edges = [0, *sorted(breaks), len(positions)]
    return [
        (edges[i], edges[i + 1])
        for i in range(len(edges) - 1)
        if edges[i + 1] - edges[i] >= _MIN_SEGMENT_POSES
    ]


def _interpolate_reference(times: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return np.stack([np.interp(times, reference[:, 0], reference[:, i]) for i in (1, 2, 3)], axis=1)


def evaluate(
    times: np.ndarray, positions: np.ndarray, segment_ids: np.ndarray, reference: np.ndarray
) -> dict:
    segments = []
    for start, end in segment_bounds(times, segment_ids, positions):
        estimate = positions[start:end]
        truth = _interpolate_reference(times[start:end], reference)
        rotation, translation = umeyama(estimate, truth)
        error = np.linalg.norm((rotation @ estimate.T).T + translation - truth, axis=1)
        steps = np.linalg.norm(np.diff(estimate, axis=0), axis=1)
        estimate_length = float(steps.sum())
        truth_length = float(np.linalg.norm(np.diff(truth, axis=0), axis=1).sum())
        segments.append(
            {
                "poses": int(end - start),
                "duration_s": round(float(times[end - 1] - times[start]), 2),
                "est_path_m": round(estimate_length, 2),
                "gt_path_m": round(truth_length, 2),
                "path_ratio": (
                    round(estimate_length / truth_length, 3) if truth_length > 0.05 else None
                ),
                "ate_rmse_m": round(float(np.sqrt((error**2).mean())), 3),
                "max_step_m": round(float(steps.max()), 3) if len(steps) else 0.0,
                "steps_above_1m": int((steps > 1.0).sum()),
            }
        )

    reference_length = float(np.linalg.norm(np.diff(reference[:, 1:4], axis=0), axis=1).sum())
    weights = np.array([s["poses"] for s in segments], dtype=float)
    ates = np.array([s["ate_rmse_m"] for s in segments])
    moving = [s for s in segments if s["gt_path_m"] >= _MOVING_SEGMENT_MIN_PATH_M]
    return {
        "poses_total": len(positions),
        "poses_in_segments": int(weights.sum()) if segments else 0,
        "n_segments": len(segments),
        "weighted_ate_rmse_m": (
            round(float((ates * weights).sum() / weights.sum()), 3) if segments else None
        ),
        "max_in_seg_step_m": round(max((s["max_step_m"] for s in segments), default=0.0), 3),
        "in_seg_steps_above_1m": sum(s["steps_above_1m"] for s in segments),
        "path_weighted_ratio": (
            round(
                sum(s["est_path_m"] for s in moving) / sum(s["gt_path_m"] for s in moving),
                3,
            )
            if moving
            else None
        ),
        "reference_path_m": round(reference_length, 2),
        "segments": segments,
    }


def segment_alignments(
    times: np.ndarray, positions: np.ndarray, segment_ids: np.ndarray, reference: np.ndarray
) -> list[tuple[int, int, np.ndarray, np.ndarray]]:
    """Per-segment rigid transform from cuVSLAM's frame into the reference frame."""
    alignments = []
    for start, end in segment_bounds(times, segment_ids, positions):
        truth = _interpolate_reference(times[start:end], reference)
        rotation, translation = umeyama(positions[start:end], truth)
        alignments.append((start, end, rotation, translation))
    return alignments


def aligned_map(
    times: np.ndarray,
    landmark_frames: list[tuple[float, np.ndarray]],
    alignments: list[tuple[int, int, np.ndarray, np.ndarray]],
) -> np.ndarray:
    """Landmarks from every segment, each carried into the reference frame.

    A landmark is expressed in whichever world frame cuVSLAM held at the time, so
    stacking the raw clouds across a tracking loss overlays unrelated frames.
    """
    clouds = []
    for start, end, rotation, translation in alignments:
        span = (times[start], times[end - 1])
        points = [p for stamp, p in landmark_frames if span[0] <= stamp <= span[1] and len(p)]
        if points:
            clouds.append((rotation @ np.vstack(points).T).T + translation)
    return np.vstack(clouds) if clouds else np.zeros((0, 3), dtype=np.float32)


def render(
    out_dir: Path,
    positions: np.ndarray,
    landmarks: np.ndarray,
    reference: np.ndarray,
    alignments: list[tuple[int, int, np.ndarray, np.ndarray]],
    metrics: dict,
) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(15, 7))
    axis = axes[0]
    # cuVSLAM exports far, poorly-conditioned landmarks too; drawn unclipped they
    # stretch the axes by two orders of magnitude and hide the room.
    low = reference[:, 1:3].min(0) - _MAP_MARGIN_M
    high = reference[:, 1:3].max(0) + _MAP_MARGIN_M
    if len(landmarks):
        inside = landmarks[np.all((landmarks[:, :2] >= low) & (landmarks[:, :2] <= high), axis=1)]
        axis.scatter(
            inside[:, 0],
            inside[:, 1],
            s=0.4,
            c="#888888",
            alpha=0.35,
            label=f"cuVSLAM landmarks ({len(inside)} of {len(landmarks)} in frame)",
            zorder=1,
        )
    axis.set_xlim(low[0], high[0])
    axis.set_ylim(low[1], high[1])
    axis.plot(
        reference[:, 1],
        reference[:, 2],
        color="#0074d9",
        lw=2.0,
        label="Point-LIO reference",
        zorder=2,
    )
    axis.set_title("Map: cuVSLAM landmarks, each segment aligned to the reference")
    axis.set_aspect("equal")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=8)

    axis = axes[1]
    axis.plot(
        reference[:, 1],
        reference[:, 2],
        color="#0074d9",
        lw=2.0,
        label="Point-LIO reference",
        zorder=1,
    )
    colors = plt.get_cmap("autumn")(np.linspace(0, 0.75, 12))
    for index, (start, end, rotation, translation) in enumerate(alignments):
        aligned = (rotation @ positions[start:end].T).T + translation
        axis.plot(
            aligned[:, 0],
            aligned[:, 1],
            color=colors[index % 12],
            lw=1.4,
            label=f"segment {index + 1} ({end - start})",
            zorder=3,
        )
    axis.set_title(
        f"Trajectory, {metrics['n_segments']} segments\n"
        f"weighted ATE {metrics['weighted_ate_rmse_m']} m · "
        f"path ratio {metrics['path_weighted_ratio']}"
    )
    axis.set_aspect("equal")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=7)

    figure.suptitle("cuVSLAM native module — replay of a memory2 recording", fontweight="bold")
    figure.tight_layout()
    path = out_dir / "topdown.png"
    figure.savefig(path, dpi=120)
    return path


def reference_from_db(db: str) -> np.ndarray:
    from dimos.memory2.replay import Replay
    from dimos.memory2.store.sqlite import SqliteStore

    replay = Replay(store=SqliteStore(path=db))
    rows = [
        (message.ts, message.x, message.y, message.z)
        for _timestamp, message in replay.stream("pointlio_odometry").iterate_ts()
    ]
    return np.asarray(rows)


def _stdin_blob(config: CuvslamConfig) -> bytes:
    """The line NativeModule.start() would write, derived from the config itself."""
    blob = {"topics": _TOPICS, "config": config.to_config_dict()}
    return json.dumps(blob).encode() + b"\n"


def _stereo_pairs(db: str) -> tuple[CameraInfo, list[tuple[Image, Image]]]:
    from dimos.memory2.replay import Replay
    from dimos.memory2.store.sqlite import SqliteStore

    replay = Replay(store=SqliteStore(path=db))
    info = next(iter(replay.stream("realsense_infra_left_camera_info").iterate_ts()))[1]
    rights = {
        round(message.ts, 4): message
        for _timestamp, message in replay.stream("realsense_infra_right").iterate_ts()
    }
    pairs = []
    for _timestamp, left in replay.stream("realsense_infra_left").iterate_ts():
        right = rights.get(round(left.ts, 4))
        if right is not None and abs(right.ts - left.ts) <= _PAIR_SKEW_S:
            pairs.append((left, right))
    return info, pairs


def run_replay(
    db: str, config: CuvslamConfig, limit: int | None
) -> tuple[list[Odometry], list[tuple[float, np.ndarray]], str]:
    """Feed the recording's stereo stream to the native process over LCM."""
    executable = Path(config.cwd or ".") / config.executable
    if not executable.exists():
        raise SystemExit(f"binary not found: {executable}\nrun: {config.build_command}")

    info, pairs = _stereo_pairs(db)
    if limit is not None:
        pairs = pairs[:limit]
    print(f"{len(pairs)} stereo pairs, {pairs[-1][0].ts - pairs[0][0].ts:.1f} s of recording")

    lcm_instance = lcmlib.LCM()
    odometry = LcmCollector(topic=_TOPICS["odometry"], msg_type=Odometry)
    landmarks = LcmCollector(topic=_TOPICS["landmarks"], msg_type=PointCloud2)
    odometry.start(lcm_instance)
    landmarks.start(lcm_instance)

    stop_event = threading.Event()
    handler = threading.Thread(target=lcm_handle_loop, args=(lcm_instance, stop_event), daemon=True)
    handler.start()

    runner = NativeProcessRunner(
        binary_path=str(executable),
        stdin_blob=_stdin_blob(config),
        env={**os.environ, **config.extra_env, "DIMOS_TRANSPORT": "lcm"},
    )
    try:
        runner.start(capture_stderr=True)
        if not runner.is_running:
            raise SystemExit("cuvslam_odometry failed to start")
        time.sleep(_PROCESS_STARTUP_SEC)

        info_blob = info.lcm_encode()
        for _ in range(_CAMERA_INFO_REPEAT):
            lcm_instance.publish(_TOPICS["camera_info"], info_blob)
            time.sleep(0.02)

        started = time.monotonic()
        for index, (left, right) in enumerate(pairs):
            before = len(odometry.messages)
            lcm_instance.publish(_TOPICS["image_left"], left.lcm_encode())
            lcm_instance.publish(_TOPICS["image_right"], right.lcm_encode())
            deadline = time.monotonic() + _FRAME_REPLY_TIMEOUT_S
            while len(odometry.messages) == before and time.monotonic() < deadline:
                time.sleep(0.001)
            if index % 500 == 0:
                print(
                    f"  {index}/{len(pairs)} fed, {len(odometry.messages)} poses, "
                    f"{time.monotonic() - started:.0f}s",
                    flush=True,
                )
            if not runner.is_running:
                raise SystemExit(f"cuvslam_odometry died at frame {index}{runner.exit_report()}")

        time.sleep(_POST_FEED_DRAIN_SEC)
    finally:
        runner.stop()
        stop_event.set()
        handler.join(timeout=5.0)
        odometry.stop(lcm_instance)
        landmarks.stop(lcm_instance)

    frames = [
        (message.ts, message.points_f32())
        for index, message in enumerate(landmarks.messages)
        if index % _LANDMARK_STRIDE == 0
    ]
    return odometry.messages, frames, runner.stderr


def _downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    if not len(points):
        return points
    keys = np.round(points / voxel_m).astype(np.int64)
    _unique, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", default=str(Path.home() / "datasets/d455/airbnb.db"))
    parser.add_argument("--out", default="/tmp/cuvslam_map")
    parser.add_argument("--baseline-m", type=float, default=D455_FACTORY_BASELINE_M)
    parser.add_argument("--limit", type=int, default=None, help="stop after N stereo pairs")
    parser.add_argument("--no-landmarks", action="store_true", help="trajectory only, no map")
    parser.add_argument(
        "--max-speed-mps",
        type=float,
        default=None,
        help="override the module's frame-restart threshold",
    )
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    overrides = {} if args.max_speed_mps is None else {"max_speed_mps": args.max_speed_mps}
    config = CuvslamConfig(
        baseline_m=args.baseline_m, publish_landmarks=not args.no_landmarks, **overrides
    )
    messages, landmark_frames, stderr = run_replay(args.db, config, args.limit)
    if not messages:
        print(f"no poses produced\n{stderr}", file=sys.stderr)
        return 1

    times = np.array([m.ts for m in messages])
    positions = np.array([[m.x, m.y, m.z] for m in messages])
    segment_ids = np.array(
        [
            int(m.child_frame_id.rsplit("_", 1)[-1]) if "_" in m.child_frame_id else 0
            for m in messages
        ]
    )

    reference = reference_from_db(args.db)
    origin = reference[0, 0]
    reference[:, 0] -= origin
    times -= origin
    landmark_frames = [(stamp - origin, points) for stamp, points in landmark_frames]

    alignments = segment_alignments(times, positions, segment_ids, reference)
    landmarks = _downsample(aligned_map(times, landmark_frames, alignments), _LANDMARK_VOXEL_M)

    np.save(out_dir / "trajectory.npy", np.column_stack([times, positions, segment_ids]))
    np.save(out_dir / "landmarks.npy", landmarks)

    metrics = evaluate(times, positions, segment_ids, reference)
    metrics["db"] = args.db
    metrics["baseline_m"] = args.baseline_m
    metrics["landmarks"] = len(landmarks)
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2))

    plot = render(out_dir, positions, landmarks, reference, alignments, metrics)
    print(json.dumps({k: v for k, v in metrics.items() if k != "segments"}, indent=2))
    print(f"map      -> {out_dir / 'landmarks.npy'}")
    print(f"topdown  -> {plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

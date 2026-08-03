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

cuVSLAM restarts its world frame on a tracking loss. The module rebases each restart
so the published odometry is ONE continuous path in ONE frame, which is all a robot
ever sees. The resets are debugging output on the module log; this demo marks them
as nodes on the path and evaluates the whole trajectory with a single rigid fit.
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

_MAP_MARGIN_M = 2.0


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


def _interpolate_reference(times: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return np.stack([np.interp(times, reference[:, 0], reference[:, i]) for i in (1, 2, 3)], axis=1)


def reset_times(stderr: str) -> list[float]:
    """Reset stamps from the module's log.

    The odometry stream itself carries no reset marker on purpose -- a robot sees
    one path in one frame. These are nodes on that path for debugging: the pose
    either side is in the same frame, only the motion across a node is unmeasured.
    """
    stamps = []
    for line in stderr.splitlines():
        if "world frame restarted" not in line:
            continue
        try:
            stamps.append(json.loads(line)["timestamp_ns"] / 1e9)
        except (ValueError, KeyError):
            continue
    return stamps


def evaluate(
    times: np.ndarray, positions: np.ndarray, resets: np.ndarray, reference: np.ndarray
) -> dict:
    truth = _interpolate_reference(times, reference)
    rotation, translation = umeyama(positions, truth)
    error = np.linalg.norm((rotation @ positions.T).T + translation - truth, axis=1)
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    estimate_length = float(steps.sum())
    truth_length = float(np.linalg.norm(np.diff(truth, axis=0), axis=1).sum())
    reference_length = float(np.linalg.norm(np.diff(reference[:, 1:4], axis=0), axis=1).sum())
    return {
        "poses": len(positions),
        "duration_s": round(float(times[-1] - times[0]), 2),
        "ate_rmse_m": round(float(np.sqrt((error**2).mean())), 3),
        "ate_max_m": round(float(error.max()), 3),
        "est_path_m": round(estimate_length, 2),
        "gt_path_m": round(truth_length, 2),
        "path_ratio": round(estimate_length / truth_length, 3) if truth_length > 0.05 else None,
        "max_step_m": round(float(steps.max()), 3),
        "steps_above_1m": int((steps > 1.0).sum()),
        "resets": len(resets),
        "reference_path_m": round(reference_length, 2),
    }


def aligned_map(
    landmark_frames: list[tuple[float, np.ndarray]],
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """Landmarks carried into the reference frame by the single trajectory fit.

    The module publishes landmarks in its own continuous world frame, so one
    transform serves the whole run.
    """
    points = [p for _stamp, p in landmark_frames if len(p)]
    if not points:
        return np.zeros((0, 3), dtype=np.float32)
    return (rotation @ np.vstack(points).T).T + translation


def render(
    out_dir: Path,
    aligned: np.ndarray,
    landmarks: np.ndarray,
    reference: np.ndarray,
    resets: np.ndarray,
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
    axis.set_title("Map: cuVSLAM landmarks, one rigid fit to the reference")
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
    axis.plot(
        aligned[:, 0], aligned[:, 1], color="#ff4136", lw=1.3, label="cuVSLAM odometry", zorder=3
    )
    if len(resets):
        axis.scatter(
            aligned[resets, 0],
            aligned[resets, 1],
            s=26,
            facecolors="none",
            edgecolors="#111111",
            lw=0.8,
            label=f"world-frame resets ({len(resets)})",
            zorder=4,
        )
    axis.set_title(
        f"Trajectory, one continuous path\n"
        f"ATE {metrics['ate_rmse_m']} m · path ratio {metrics['path_ratio']} · "
        f"{metrics['resets']} resets"
    )
    axis.set_aspect("equal")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=8)

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

    reference = reference_from_db(args.db)
    origin = reference[0, 0]
    reference[:, 0] -= origin
    times -= origin
    landmark_frames = [(stamp - origin, points) for stamp, points in landmark_frames]
    resets = np.searchsorted(times, np.array(reset_times(stderr)) - origin)
    resets = resets[(resets > 0) & (resets < len(times))]

    rotation, translation = umeyama(positions, _interpolate_reference(times, reference))
    landmarks = _downsample(aligned_map(landmark_frames, rotation, translation), _LANDMARK_VOXEL_M)

    np.save(out_dir / "trajectory.npy", np.column_stack([times, positions]))
    np.save(out_dir / "landmarks.npy", landmarks)

    metrics = evaluate(times, positions, resets, reference)
    metrics["db"] = args.db
    metrics["baseline_m"] = args.baseline_m
    metrics["landmarks"] = len(landmarks)
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2))

    aligned = (rotation @ positions.T).T + translation
    plot = render(out_dir, aligned, landmarks, reference, resets, metrics)
    print(json.dumps({k: v for k, v in metrics.items() if k != "segments"}, indent=2))
    print(f"map      -> {out_dir / 'landmarks.npy'}")
    print(f"topdown  -> {plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

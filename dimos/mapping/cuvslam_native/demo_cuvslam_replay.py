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
from dataclasses import dataclass
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
    "corrected_odometry": "/cuvslam_demo_corrected#nav_msgs.Odometry",
    "map_tf": "/cuvslam_demo_maptf#nav_msgs.Odometry",
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
# Below this the Slam track is a prefix, not a trajectory, and must not be scored.
_SLAM_MIN_COVERAGE = 0.8


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


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def correct_landmarks(
    landmark_frames: list[tuple[float, np.ndarray]], corrections: list[Odometry]
) -> np.ndarray:
    """Carry each landmark cloud from the odom frame into the map frame.

    Slam's map->odom is what turns a drifting live view into a map, so each cloud
    is corrected with the transform in force when it was published.
    """
    if not corrections:
        return np.vstack([p for _t, p in landmark_frames if len(p)])
    stamps = np.array([c.ts for c in corrections])
    clouds = []
    for stamp, points in landmark_frames:
        if not len(points):
            continue
        correction = corrections[max(0, int(np.searchsorted(stamps, stamp)) - 1)]
        rotation = quaternion_matrix(
            correction.pose.orientation.x,
            correction.pose.orientation.y,
            correction.pose.orientation.z,
            correction.pose.orientation.w,
        )
        offset = np.array(
            [
                correction.pose.position.x,
                correction.pose.position.y,
                correction.pose.position.z,
            ]
        )
        clouds.append((rotation @ points.T).T + offset)
    return np.vstack(clouds) if clouds else np.zeros((0, 3), dtype=np.float32)


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


def render(
    out_dir: Path,
    aligned: np.ndarray,
    aligned_times: np.ndarray,
    landmarks: np.ndarray,
    reference: np.ndarray,
    reset_stamps: np.ndarray,
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
    axis.set_title("Map: cuVSLAM landmarks through map->odom, one rigid fit to the reference")
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
        aligned[:, 0],
        aligned[:, 1],
        color="#ff4136",
        lw=1.3,
        label="cuVSLAM SLAM" if metrics.get("slam_usable") else "cuVSLAM odometry",
        zorder=3,
    )
    # The resets are the odometry's; index them onto whichever track is drawn.
    resets = np.searchsorted(aligned_times, reset_stamps)
    resets = resets[(resets > 0) & (resets < len(aligned_times))]
    if len(resets):
        axis.scatter(
            aligned[resets, 0],
            aligned[resets, 1],
            s=26,
            facecolors="none",
            edgecolors="#111111",
            lw=0.8,
            label=f"VO world-frame resets ({len(resets)})",
            zorder=4,
        )
    usable = metrics.get("slam_usable")
    shown = metrics["slam"] if usable else metrics["raw_vo"]
    coverage = metrics.get("slam_pose_coverage", 0.0)
    axis.set_title(
        f"{'SLAM-corrected' if usable else 'raw VO'} trajectory, one continuous path\n"
        f"ATE {shown['ate_rmse_m']} m · path ratio {shown['path_ratio']} · "
        f"{metrics['raw_vo']['resets']} VO world-frame resets\n"
        + (
            f"pose graph diverged, {coverage:.0%} of frames corrected before it was rejected"
            if not usable
            else f"{metrics['loop_closures']} loop closures"
        )
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


@dataclass
class Replayed:
    odometry: list[Odometry]
    corrected: list[Odometry]
    map_tf: list[Odometry]
    landmark_frames: list[tuple[float, np.ndarray]]
    stderr: str


def run_replay(db: str, config: CuvslamConfig, limit: int | None) -> Replayed:
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
    corrected = LcmCollector(topic=_TOPICS["corrected_odometry"], msg_type=Odometry)
    map_tf = LcmCollector(topic=_TOPICS["map_tf"], msg_type=Odometry)
    for collector in (odometry, landmarks, corrected, map_tf):
        collector.start(lcm_instance)

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
        for collector in (odometry, landmarks, corrected, map_tf):
            collector.stop(lcm_instance)

    frames = [
        (message.ts, message.points_f32())
        for index, message in enumerate(landmarks.messages)
        if index % _LANDMARK_STRIDE == 0
    ]
    return Replayed(odometry.messages, corrected.messages, map_tf.messages, frames, runner.stderr)


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
    parser.add_argument("--no-slam", action="store_true", help="odometry only, no pose graph")
    parser.add_argument("--slam-async", action="store_true", help="run Slam on its own thread")
    parser.add_argument(
        "--slam-max-map-size", type=int, default=None, help="poses in the graph, 0 = unlimited"
    )
    parser.add_argument(
        "--max-speed-mps",
        type=float,
        default=None,
        help="override the module's frame-restart threshold",
    )
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    overrides: dict[str, float | int] = {}
    if args.max_speed_mps is not None:
        overrides["max_speed_mps"] = args.max_speed_mps
    if args.slam_max_map_size is not None:
        overrides["slam_max_map_size"] = args.slam_max_map_size
    config = CuvslamConfig(
        baseline_m=args.baseline_m,
        publish_landmarks=not args.no_landmarks,
        enable_slam=not args.no_slam,
        slam_sync_mode=not args.slam_async,
        **overrides,
    )
    run = run_replay(args.db, config, args.limit)
    if not run.odometry:
        print(f"no poses produced\n{run.stderr}", file=sys.stderr)
        return 1

    reference = reference_from_db(args.db)
    origin = reference[0, 0]
    reference[:, 0] -= origin

    def track(messages: list[Odometry]) -> tuple[np.ndarray, np.ndarray]:
        return (
            np.array([m.ts for m in messages]) - origin,
            np.array([[m.x, m.y, m.z] for m in messages]),
        )

    times, positions = track(run.odometry)
    reset_stamps = np.array(reset_times(run.stderr)) - origin
    resets = np.searchsorted(times, reset_stamps)
    resets = resets[(resets > 0) & (resets < len(times))]

    # The map is the landmarks put through Slam's map->odom, so what is plotted is
    # the corrected map rather than the drifting live view.
    landmark_frames = [(stamp, points) for stamp, points in run.landmark_frames]
    corrected_points = correct_landmarks(landmark_frames, run.map_tf)
    # Slam only counts as the trajectory if it survived the run. On a recording it
    # diverges on, the corrections are rejected and what is left is a short prefix
    # whose ATE looks wonderful because it covers 15 s of a 223 s walk.
    slam_covers = len(run.corrected) >= _SLAM_MIN_COVERAGE * len(run.odometry)
    map_times, map_positions = track(run.corrected) if slam_covers else (times, positions)
    rotation, translation = umeyama(map_positions, _interpolate_reference(map_times, reference))
    landmarks = _downsample(
        (rotation @ corrected_points.T).T + translation
        if len(corrected_points)
        else corrected_points,
        _LANDMARK_VOXEL_M,
    )

    (out_dir / "module.log").write_text(run.stderr)
    np.save(out_dir / "trajectory.npy", np.column_stack([times, positions]))
    np.save(out_dir / "landmarks.npy", landmarks)
    if run.corrected:
        np.save(out_dir / "corrected.npy", np.column_stack([map_times, map_positions]))

    metrics = {
        "raw_vo": evaluate(times, positions, resets, reference),
        "slam": evaluate(*track(run.corrected), np.array([]), reference) if run.corrected else None,
        "slam_pose_coverage": round(len(run.corrected) / max(len(run.odometry), 1), 3),
        "slam_usable": slam_covers,
        "loop_closures": run.stderr.count("cuvslam loop closure"),
        "map_corrections": len(run.map_tf),
        "max_map_to_odom_m": round(
            max(
                (float(np.linalg.norm([m.x, m.y, m.z])) for m in run.map_tf),
                default=0.0,
            ),
            3,
        ),
        "db": args.db,
        "baseline_m": args.baseline_m,
        "landmarks": len(landmarks),
    }
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2))

    aligned = (rotation @ map_positions.T).T + translation
    plot = render(out_dir, aligned, map_times, landmarks, reference, reset_stamps, metrics)
    print(json.dumps(metrics, indent=2))
    print(f"map      -> {out_dir / 'landmarks.npy'}")
    print(f"topdown  -> {plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

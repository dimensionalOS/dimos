#!/usr/bin/env python
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

"""Track the Spot 5-camera rig with pycuvslam, straight from a memory2 recording.

Spot provides rectified mono grayscale plus registered depth per camera — no IMU and
no stereo pairs — so this uses cuVSLAM's Multisensor mode with every camera declared
as a depth camera. Plain Multicamera mode never initializes on this data (mono
cameras without overlap give it no scale), and it rejects depth images outright.

The rig frame is base_link and camera extrinsics come from the recording's tf tree,
so world_from_rig is directly comparable to the recorded odom->base_link stream.

Run with `python -m dimos.mapping.cuvslam.demo_cuvslam_spot` — invoking the file
directly would shadow pycuvslam with the sibling cuvslam.py module.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cuvslam as vslam
import cv2
import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

CAMERAS = ["front_left", "front_right", "left", "right", "back"]
NANOSECONDS_PER_SECOND = 1e9

# Rotating an image 90 deg clockwise (np.rot90 k=-1, u' = (H-1)-v, v' = u) is a -90 deg
# rotation of the camera frame about its optical axis.
QUATERNION_Z_MINUS_90 = [0.0, 0.0, -float(np.sqrt(0.5)), float(np.sqrt(0.5))]


def optical_frame(camera: str) -> str:
    return f"{camera.replace('_', '')}_camera_optical"


def quaternion_multiply(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def rig_extrinsics(store: SqliteStore) -> dict[str, tuple[list[float], list[float]]]:
    """base_link -> camera_optical (translation, xyzw quaternion) for every camera."""
    wanted = {optical_frame(camera): camera for camera in CAMERAS}
    found: dict[str, tuple[list[float], list[float]]] = {}
    for observation in store.stream("tf", TFMessage):
        for transform in observation.data.transforms:
            camera = wanted.get(transform.child_frame_id)
            if camera is not None and camera not in found:
                translation = [float(value) for value in transform.translation]
                rotation = [float(value) for value in transform.rotation]
                found[camera] = (translation, rotation)
        if len(found) == len(CAMERAS):
            return found
    raise SystemExit(f"tf tree is missing cameras: {sorted(set(CAMERAS) - set(found))}")


def camera_infos(store: SqliteStore, stream: str) -> dict[str, CameraInfo]:
    wanted = {optical_frame(camera): camera for camera in CAMERAS}
    found: dict[str, CameraInfo] = {}
    for observation in store.stream(stream, CameraInfo):
        camera = wanted.get(observation.data.frame_id)
        if camera is not None and camera not in found:
            found[camera] = observation.data
        if len(found) == len(CAMERAS):
            return found
    raise SystemExit(f"{stream} is missing cameras: {sorted(set(CAMERAS) - set(found))}")


def is_portrait(info: CameraInfo) -> bool:
    return info.width < info.height


def build_camera(info: CameraInfo, extrinsic: tuple[list[float], list[float]]) -> vslam.Camera:
    """Build a pycuvslam camera; portrait cameras are described as their landscape rotation.

    cuVSLAM requires every camera in the rig to share one resolution, and Spot's two
    front cameras are mounted 90 deg rotated. Their images get np.rot90(k=-1) at feed
    time, so the intrinsics/extrinsics here describe that rotated camera.
    """
    camera = vslam.Camera()
    camera.distortion = vslam.Distortion(vslam.Distortion.Model.Pinhole)
    translation, rotation = extrinsic
    if is_portrait(info):
        camera.focal = (info.K[4], info.K[0])
        camera.principal = (info.height - 1 - info.K[5], info.K[2])
        camera.size = (info.height, info.width)
        rotation = quaternion_multiply(rotation, QUATERNION_Z_MINUS_90)
    else:
        camera.focal = (info.K[0], info.K[4])
        camera.principal = (info.K[2], info.K[5])
        camera.size = (info.width, info.height)
    camera.rig_from_camera = vslam.Pose(rotation=rotation, translation=translation)
    return camera


def depth_to_gray_affine(gray: CameraInfo, depth: CameraInfo) -> np.ndarray:
    """Affine mapping gray pixels onto depth pixels (same optical frame, different K)."""
    scale_x = depth.K[0] / gray.K[0]
    scale_y = depth.K[4] / gray.K[4]
    return np.array(
        [
            [scale_x, 0.0, depth.K[2] - scale_x * gray.K[2]],
            [0.0, scale_y, depth.K[5] - scale_y * gray.K[5]],
        ],
        dtype=np.float32,
    )


class NearestStream:
    """Walk a timestamp-ordered stream, returning the observation nearest each query."""

    def __init__(self, stream: Any) -> None:
        self._iterator = iter(stream)
        self._head = next(self._iterator, None)
        self._lookahead = next(self._iterator, None)

    def at(self, stamp: float) -> Any:
        while (
            self._head is not None
            and self._lookahead is not None
            and abs(self._lookahead.ts - stamp) < abs(self._head.ts - stamp)
        ):
            self._head = self._lookahead
            self._lookahead = next(self._iterator, None)
        return self._head


def recorded_odometry(store: SqliteStore) -> list[list[float]]:
    rows = []
    for observation in store.stream("odometry", Odometry):
        position = observation.data.pose.position
        rows.append([observation.ts, float(position.x), float(position.y), float(position.z)])
    return rows


def umeyama_alignment(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rigid (no scale) alignment mapping source points onto target."""
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (target - target_mean).T @ (source - source_mean) / len(source)
    u, _, vt = np.linalg.svd(covariance)
    sign = np.sign(np.linalg.det(u @ vt))
    correction = np.diag([1.0, 1.0, sign])
    rotation = u @ correction @ vt
    return rotation, target_mean - rotation @ source_mean


def compare_and_plot(
    trajectory: list[list[float]], odometry: list[list[float]], out: Path
) -> dict[str, float]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    track = np.asarray(trajectory, dtype=float)
    reference = np.asarray(odometry, dtype=float)
    matched = np.column_stack(
        [np.interp(track[:, 0], reference[:, 0], reference[:, axis]) for axis in (1, 2, 3)]
    )
    rotation, translation = umeyama_alignment(track[:, 1:4], matched)
    aligned = track[:, 1:4] @ rotation.T + translation
    errors = np.linalg.norm(aligned - matched, axis=1)
    metrics = {
        "ate_rmse_m": float(np.sqrt((errors**2).mean())),
        "ate_max_m": float(errors.max()),
        "final_drift_m": float(errors[-1]),
        "odom_path_m": float(np.linalg.norm(np.diff(reference[:, 1:3], axis=0), axis=1).sum()),
        "cuvslam_path_m": float(np.linalg.norm(np.diff(aligned[:, :2], axis=0), axis=1).sum()),
    }

    figure, axes = plt.subplots(1, 2, figsize=(14, 7), width_ratios=[3, 1])
    axes[0].plot(
        reference[:, 1], reference[:, 2], color="black", linewidth=1, label="spot odometry"
    )
    axes[0].plot(
        aligned[:, 0], aligned[:, 1], color="tab:red", linewidth=1, label="cuVSLAM (aligned)"
    )
    axes[0].scatter([reference[0, 1]], [reference[0, 2]], color="green", zorder=5, label="start")
    axes[0].set_aspect("equal")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].legend()
    axes[0].set_title(
        f"spot_small_loop: cuVSLAM multisensor vs Spot odometry\n"
        f"ATE RMSE {metrics['ate_rmse_m']:.2f} m, max {metrics['ate_max_m']:.2f} m"
    )
    axes[1].plot(track[:, 0] - track[0, 0], errors, linewidth=0.8)
    axes[1].set_xlabel("t [s]")
    axes[1].set_ylabel("position error [m]")
    axes[1].set_title("error over time")
    figure.tight_layout()
    figure.savefig(out, dpi=120)
    plt.close(figure)
    return metrics


def replay_spot(
    store: SqliteStore,
    cameras: list[str],
    depth_cameras: list[str],
    warmup: int = 10,
    max_frames: int = 0,
    tolerance_s: float = 0.09,
    use_gpu: bool = True,
) -> dict[str, Any]:
    """Replay the recording through a Multisensor tracker; returns trajectory and stats."""
    gray_infos = camera_infos(store, "grayscale_info")
    depth_infos = camera_infos(store, "depth_info")
    extrinsics = rig_extrinsics(store)
    affines = {
        camera: depth_to_gray_affine(gray_infos[camera], depth_infos[camera]) for camera in cameras
    }

    rig = vslam.Rig()
    rig.cameras = [build_camera(gray_infos[camera], extrinsics[camera]) for camera in cameras]

    config = vslam.Tracker.OdometryConfig(
        async_sba=False,
        odometry_mode=vslam.Tracker.OdometryMode.Multisensor,
        use_gpu=use_gpu,
    )
    config.multisensor_settings.depth_camera_ids = [
        cameras.index(camera) for camera in depth_cameras
    ]
    config.multisensor_settings.depth_scale_factor = 1000.0
    tracker = vslam.Tracker(rig, config)

    reference, others = cameras[0], cameras[1:]
    gray_streams = {
        camera: NearestStream(store.stream(f"grayscale_image_{camera}", Image)) for camera in others
    }
    depth_streams = {
        camera: NearestStream(store.stream(f"depth_image_{camera}", Image))
        for camera in depth_cameras
    }
    tolerance = tolerance_s

    trajectory: list[list[float]] = []
    frames = 0
    lost = 0
    dropped_inputs = 0
    last_stamp: int | None = None
    for index, observation in enumerate(store.stream(f"grayscale_image_{reference}", Image)):
        if index < warmup:
            for camera in others:
                gray_streams[camera].at(observation.ts)
            for camera in depth_cameras:
                depth_streams[camera].at(observation.ts)
            continue
        if max_frames and frames >= max_frames:
            break
        stamp_s = observation.ts
        stamp = int(stamp_s * NANOSECONDS_PER_SECOND)
        if last_stamp is not None and stamp <= last_stamp:
            continue
        last_stamp = stamp

        images: list[np.ndarray] = []
        depths: list[np.ndarray] = []
        for camera in cameras:
            if camera == reference:
                gray = observation
            else:
                gray = gray_streams[camera].at(stamp_s)
            depth = depth_streams[camera].at(stamp_s) if camera in depth_streams else None
            needs_depth = camera in depth_streams
            if (
                gray is None
                or abs(gray.ts - stamp_s) > tolerance
                or (needs_depth and (depth is None or abs(depth.ts - stamp_s) > tolerance))
            ):
                dropped_inputs += 1
                images.append(np.empty((0, 0), dtype=np.uint8))
                depths.append(np.empty((0, 0), dtype=np.uint16))
                continue
            pixels = np.asanyarray(gray.data.data)
            info = gray_infos[camera]
            if needs_depth and depth is not None:
                aligned = cv2.warpAffine(
                    np.asanyarray(depth.data.data),
                    affines[camera],
                    (info.width, info.height),
                    flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP,
                )
            else:
                aligned = np.empty((0, 0), dtype=np.uint16)
            if is_portrait(info):
                pixels = np.ascontiguousarray(np.rot90(pixels, k=-1))
                if aligned.size:
                    aligned = np.ascontiguousarray(np.rot90(aligned, k=-1))
            images.append(pixels)
            depths.append(aligned)

        if not any(image.size for image in images):
            dropped_inputs += 1
            continue
        estimate, _ = tracker.track(stamp, images, depths=depths)
        frames += 1
        if estimate.world_from_rig is None:
            lost += 1
            continue
        pose = estimate.world_from_rig.pose
        trajectory.append(
            [float(stamp_s), *map(float, pose.translation), *map(float, pose.rotation)]
        )
        if frames % 250 == 0:
            print(f"[spot] {frames} frames, {len(trajectory)} poses, {lost} lost", flush=True)

    points = np.asarray([row[1:4] for row in trajectory], dtype=float)
    length = (
        float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()) if len(points) > 1 else 0.0
    )
    return {
        "trajectory": trajectory,
        "frames": frames,
        "lost": lost,
        "dropped_inputs": dropped_inputs,
        "path_m": length,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("db", nargs="?", default="/home/dimos/datasets/spot/spot_small_loop.db")
    parser.add_argument(
        "--cams", default=",".join(CAMERAS), help="comma list, first is the sync reference"
    )
    parser.add_argument(
        "--depth-cams",
        default="left,right,back",
        help="cameras whose depth anchors scale; the front pair's floor-dominated depth "
        "degrades the solve badly (ATE 14.5 m vs 1.7 m on spot_small_loop)",
    )
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--warmup", type=int, default=10, help="reference frames to skip")
    parser.add_argument("--tolerance-ms", type=float, default=90.0, help="cross-camera sync window")
    parser.add_argument("--out", default="/tmp/cuvslam_spot_trajectory.json")
    parser.add_argument("--plot", default="", help="write a comparison PNG here")
    args = parser.parse_args()

    cameras = [camera.strip() for camera in args.cams.split(",") if camera.strip()]
    depth_cameras = [camera.strip() for camera in args.depth_cams.split(",") if camera.strip()]
    depth_cameras = [camera for camera in depth_cameras if camera in cameras]
    unknown = sorted(set(cameras) - set(CAMERAS))
    if unknown:
        raise SystemExit(f"unknown cameras: {unknown}")

    store = SqliteStore(path=args.db, must_exist=True)
    store.start()
    result = replay_spot(
        store,
        cameras,
        depth_cameras,
        warmup=args.warmup,
        max_frames=args.max_frames,
        tolerance_s=args.tolerance_ms / 1000.0,
    )
    trajectory = result["trajectory"]
    print(
        f"[spot] done: frames={result['frames']} poses={len(trajectory)} lost={result['lost']} "
        f"dropped_inputs={result['dropped_inputs']} path={result['path_m']:.1f} m",
        flush=True,
    )

    odometry = recorded_odometry(store)
    payload = {
        "cameras": cameras,
        "odometry_mode": "Multisensor",
        "trajectory": trajectory,
        "odometry": odometry,
        "stats": {
            "frames": result["frames"],
            "lost": result["lost"],
            "dropped_inputs": result["dropped_inputs"],
            "path_m": result["path_m"],
        },
    }
    if args.plot and len(trajectory) > 1:
        metrics = compare_and_plot(trajectory, odometry, Path(args.plot))
        payload["metrics"] = metrics
        print(f"[spot] {metrics}", flush=True)
        print(f"[spot] plot -> {args.plot}", flush=True)
    Path(args.out).write_text(json.dumps(payload))
    print(f"[spot] wrote {args.out}", flush=True)


if __name__ == "__main__":
    main()

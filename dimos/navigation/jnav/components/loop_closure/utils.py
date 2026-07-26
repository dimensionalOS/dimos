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

"""Self-consistency helpers for loop-closure evaluation.

Tag placement + spread scoring, lidar-scan registration + map accumulation, and
the before/after top-down render — everything the eval driver composes into a
score. Poses are resolved through a `RecordingTF` tree; corrections are applied via a
plain-stretch delta lookup (see `trajectory_metrics.drift_delta_lookup`).
"""

from __future__ import annotations

from collections.abc import Iterator
from itertools import islice
from pathlib import Path
from typing import Any

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.utils.apriltags import (
    VISIT_GAP_S,
    AgreementReport,
    agreement_improvement,
    agreement_report,
    ensure_april_streams,
    read_raw_tag_stream,
    split_visits,
)
from dimos.navigation.jnav.utils.recording_tf import RecordingTF
from dimos.navigation.jnav.utils.trajectory_metrics import matrix_from_pose

RAW_TAGS_STREAM = "raw_april_tags"

# Cap accumulated scans so the map fits in memory / renders quickly.
MAP_MAX_SCANS = 400


def place_tags(
    detections: list[dict[str, Any]],
    tf: RecordingTF,
    odom_parent: str,
    tag_frame: str,
    delta_lookup: Any,
) -> tuple[dict[int, list[tuple[float, np.ndarray]]], dict[int, list[tuple[float, np.ndarray]]]]:
    """Place each tag detection in the map by geometry, raw vs corrected.

    Returns ``{marker_id: [(ts, xyz), ...]}`` for the raw odom placement and the
    Δ-corrected placement. The raw placement resolves the camera pose through the
    tf tree at the detection time (``odom_parent <- tag_frame``, which walks the
    overridden odom edge plus the static camera extrinsic). Detections whose time
    falls outside the odom coverage are dropped from both.
    """
    raw: dict[int, list[tuple[float, np.ndarray]]] = {}
    corrected: dict[int, list[tuple[float, np.ndarray]]] = {}
    for detection in detections:
        timestamp = float(detection["ts"])
        world_from_camera = tf.get(odom_parent, tag_frame, timestamp)
        delta = delta_lookup(timestamp)
        if world_from_camera is None or delta is None:
            continue
        camera_from_tag = matrix_from_pose(np.asarray(detection["t_cam_marker"], dtype=np.float64))
        tag_in_map_raw = (world_from_camera.to_matrix() @ camera_from_tag)[:3, 3]
        rotation_delta, translation_delta = delta
        tag_in_map_corrected = rotation_delta @ tag_in_map_raw + translation_delta
        marker_id = int(detection["marker_id"])
        raw.setdefault(marker_id, []).append((timestamp, tag_in_map_raw))
        corrected.setdefault(marker_id, []).append((timestamp, tag_in_map_corrected))
    return raw, corrected


def visit_medians(
    placements: dict[int, list[tuple[float, np.ndarray]]], *, gap_s: float
) -> dict[int, np.ndarray]:
    """One median map position per tag VISIT (sightings clustered by time gap)."""
    positions: dict[int, np.ndarray] = {}
    for marker_id, sightings in placements.items():
        by_time = {timestamp: xyz for timestamp, xyz in sightings}
        medians: list[np.ndarray] = []
        for visit_times in split_visits(list(by_time), gap_s=gap_s):
            medians.append(np.median(np.vstack([by_time[t] for t in visit_times]), axis=0))
        if medians:
            positions[marker_id] = np.vstack(medians)
    return positions


def registered_scans(
    db_path: Path,
    lidar_stream: str,
    stride: int,
    tf: RecordingTF,
    odom_parent: str,
) -> Iterator[tuple[float, np.ndarray]]:
    """Yield ``(ts, world-frame points)`` for each scan, registered via the tf tree.

    Each scan's frame resolves to ``odom_parent`` through
    ``tf.get(odom_parent, frame, ts)``, which walks the overridden odom edge plus
    any static sensor extrinsics. Scans already in ``odom_parent`` resolve to
    identity; scans whose frame can't be reached at their timestamp are dropped."""
    with SqliteStore(path=db_path, must_exist=True) as store:
        for observation in islice(store.stream(lidar_stream, PointCloud2), 0, None, stride):
            cloud = observation.data
            timestamp = float(observation.ts)
            points = np.asarray(cloud.points_f32(), dtype=np.float64)[:, :3]
            frame_id = cloud.frame_id or odom_parent
            world_from_sensor = tf.get(odom_parent, frame_id, timestamp)
            if world_from_sensor is None:
                continue
            matrix = world_from_sensor.to_matrix()
            yield timestamp, points @ matrix[:3, :3].T + matrix[:3, 3]


def accumulate_maps(
    scans: Iterator[tuple[float, np.ndarray]],
    delta_lookup: Any,
    *,
    max_points_per_scan: int = 4000,
) -> tuple[np.ndarray, np.ndarray]:
    """Stack raw + Δ-corrected map points from registered scans."""
    raw_clouds: list[np.ndarray] = []
    corrected_clouds: list[np.ndarray] = []
    for timestamp, points in scans:
        delta = delta_lookup(timestamp)
        if delta is None:
            continue
        if len(points) > max_points_per_scan:
            points = points[:: -(-len(points) // max_points_per_scan)]
        rotation_delta, translation_delta = delta
        raw_clouds.append(points)
        corrected_clouds.append(points @ rotation_delta.T + translation_delta)
    if not raw_clouds:
        return np.empty((0, 3)), np.empty((0, 3))
    return np.vstack(raw_clouds), np.vstack(corrected_clouds)


def read_camera_info(
    store: Any, stream_name: str = "camera_info"
) -> tuple[np.ndarray, np.ndarray, str] | None:
    """``(K 3x3, distortion, optical frame_id)`` from the ``camera_info`` stream, or None if absent."""
    if stream_name not in store.list_streams():
        return None
    observation = next(iter(store.stream(stream_name, CameraInfo)), None)
    if observation is None:
        return None
    info = observation.data
    return info.get_K_matrix(), info.get_D_coeffs(), info.frame_id


def load_tag_detections(
    db_path: Path,
    camera_stream: str | None,
    camera_info_stream: str,
    streams: list[str],
    dynamic_tags: set[int],
) -> list[dict[str, Any]]:
    """Every gated tag glimpse (camera_optical pose), minus the dynamic tags."""
    if camera_stream is None or camera_stream not in streams:
        print("no camera stream — voxel agreement only")
        return []
    with SqliteStore(path=db_path, must_exist=True) as db_store:
        if RAW_TAGS_STREAM not in db_store.list_streams():
            camera_info = read_camera_info(db_store, camera_info_stream)
            if camera_info is None:
                raise SystemExit(
                    f"no {RAW_TAGS_STREAM!r} and no {camera_info_stream!r} stream — "
                    f"can't detect tags. Disable the camera stream or add camera_info."
                )
            camera_matrix, distortion, _optical_frame = camera_info
            ensure_april_streams(
                db_store,
                camera_matrix,
                distortion,
                image_stream=camera_stream,
            )
        detections = [
            detection
            for detection in read_raw_tag_stream(db_store, RAW_TAGS_STREAM)
            if int(detection["marker_id"]) not in dynamic_tags
        ]
    ids = sorted({int(detection["marker_id"]) for detection in detections})
    print(
        f"tag detections: {len(detections)} glimpses across ids {ids} (dynamic held out: {sorted(dynamic_tags)})"
    )
    return detections


def score_tags(
    detections: list[dict[str, Any]],
    tf: RecordingTF,
    odom_parent: str,
    tag_frame: str,
    delta_lookup: Any,
) -> tuple[
    AgreementReport, AgreementReport, float | None, dict[int, np.ndarray], dict[int, np.ndarray]
]:
    """Raw vs Δ-corrected tag agreement reports + per-tag median map positions."""
    if not detections:
        empty = agreement_report({})
        return empty, empty, None, {}, {}
    raw_placements, corrected_placements = place_tags(
        detections, tf, odom_parent, tag_frame, delta_lookup
    )
    raw_medians = visit_medians(raw_placements, gap_s=VISIT_GAP_S)
    corrected_medians = visit_medians(corrected_placements, gap_s=VISIT_GAP_S)
    raw_report = agreement_report(raw_medians)
    corrected_report = agreement_report(corrected_medians)
    improvement = agreement_improvement(raw_report, corrected_report)
    return raw_report, corrected_report, improvement, raw_medians, corrected_medians


def report_dict(report: AgreementReport) -> dict[str, Any]:
    """Flatten an `AgreementReport` into a JSON-serializable summary."""
    return {
        "mean_spread_m": report.mean_spread,
        "total_observations": report.total_observations,
        "per_tag": [
            {"tag_id": tag.tag_id, "observations": tag.observations, "spread_m": tag.spread}
            for tag in report.per_tag
        ],
    }


def write_topdown_png(
    png_path: Path,
    raw_map: np.ndarray,
    corrected_map: np.ndarray,
    raw_tags: dict[int, np.ndarray],
    corrected_tags: dict[int, np.ndarray],
    raw_path: np.ndarray,
    corrected_path: np.ndarray,
    recording_name: str,
) -> None:
    """Two-panel top-down (x-y) scatter: before vs after correction."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(16, 8), sharex=True, sharey=True)
    for axis, cloud, tags, path, title in (
        (axes[0], raw_map, raw_tags, raw_path, "raw odom (before)"),
        (axes[1], corrected_map, corrected_tags, corrected_path, "corrected (after)"),
    ):
        if len(cloud):
            axis.scatter(cloud[:, 0], cloud[:, 1], s=0.4, c="0.55", linewidths=0, rasterized=True)
        if len(path):
            axis.plot(path[:, 0], path[:, 1], c="tab:blue", linewidth=1.5, zorder=4)
            axis.scatter(
                path[0, 0], path[0, 1], s=60, c="green", marker="o", zorder=6, label="start"
            )
            axis.scatter(path[-1, 0], path[-1, 1], s=60, c="red", marker="s", zorder=6, label="end")
            axis.legend(loc="upper right", fontsize=8)
        for marker_id, positions in tags.items():
            axis.scatter(
                positions[:, 0], positions[:, 1], s=90, marker="X", edgecolors="black", zorder=5
            )
            centroid = positions.mean(axis=0)
            axis.annotate(f"tag {marker_id}", centroid[:2], fontsize=9, zorder=7)
        axis.set_title(title)
        axis.set_aspect("equal")
        axis.set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    figure.suptitle(f"{recording_name}: top-down lidar map, before vs after loop closure")
    figure.tight_layout()
    figure.savefig(png_path, dpi=130)
    plt.close(figure)


# Isometric view angle (degrees): a 3/4 bird's-eye that reveals z structure the
# top-down flattens away (drift that tilts the map into vertical shows up here).
_ISO_ELEV_DEG = 35.0
_ISO_AZIM_DEG = -60.0


def write_isometric_png(
    png_path: Path,
    raw_map: np.ndarray,
    corrected_map: np.ndarray,
    raw_path: np.ndarray,
    corrected_path: np.ndarray,
    recording_name: str,
) -> None:
    """Two-panel isometric (3D) scatter of the lidar map: before vs after correction.

    Complements the top-down: false closures often tilt the map into z (horizontal
    travel bleaking into vertical), which is invisible flattened but obvious here.
    Points are colored by height so the ground plane and any tilt read at a glance."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(16, 8))
    panels = (
        (raw_map, raw_path, "raw odom (before)"),
        (corrected_map, corrected_path, "corrected (after)"),
    )
    for index, (cloud, path, title) in enumerate(panels):
        axis = figure.add_subplot(1, 2, index + 1, projection="3d")
        if len(cloud):
            axis.scatter(
                cloud[:, 0],
                cloud[:, 1],
                cloud[:, 2],
                s=0.3,
                c=cloud[:, 2],
                cmap="viridis",
                linewidths=0,
                rasterized=True,
            )
        if len(path):
            axis.plot(path[:, 0], path[:, 1], path[:, 2], c="tab:red", linewidth=1.2, zorder=4)
        axis.set_title(title)
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.set_zlabel("z (m)")
        axis.view_init(elev=_ISO_ELEV_DEG, azim=_ISO_AZIM_DEG)
        if len(cloud):
            _set_equal_3d_aspect(axis, cloud)
    figure.suptitle(f"{recording_name}: isometric lidar map, before vs after loop closure")
    figure.tight_layout()
    figure.savefig(png_path, dpi=130)
    plt.close(figure)


_RRD_RAW_COLOR = np.array([220, 60, 60], dtype=float)
_RRD_CORRECTED_COLOR = np.array([60, 120, 230], dtype=float)


def _z_shaded(cloud: np.ndarray, base_color: np.ndarray) -> np.ndarray:
    """Per-point colors: ``base_color`` shaded dark (low z) to light (high z)."""
    low, high = np.percentile(cloud[:, 2], (2.0, 98.0))
    fraction = np.clip((cloud[:, 2] - low) / ((high - low) or 1.0), 0.0, 1.0)[:, None]
    dark = base_color * 0.3
    light = base_color + (255.0 - base_color) * 0.7
    shaded: np.ndarray = (dark + (light - dark) * fraction).astype(np.uint8)
    return shaded


def write_comparison_rrd(
    rrd_path: Path,
    raw_map: np.ndarray,
    corrected_map: np.ndarray,
    raw_tags: dict[int, np.ndarray],
    corrected_tags: dict[int, np.ndarray],
    raw_path: np.ndarray,
    corrected_path: np.ndarray,
    recording_name: str,
) -> None:
    """Rerun rrd: raw vs corrected lidar cloud (z-shaded), trajectories, and tag medians."""
    import rerun as rr

    rr.init(f"eval_{recording_name}")
    rr.save(str(rrd_path))
    for name, cloud, base_color, path, path_color in (
        ("raw", raw_map, _RRD_RAW_COLOR, raw_path, [255, 120, 120]),
        ("corrected", corrected_map, _RRD_CORRECTED_COLOR, corrected_path, [120, 180, 255]),
    ):
        if len(cloud):
            rr.log(
                f"{name}/cloud",
                rr.Points3D(cloud, colors=_z_shaded(cloud, base_color), radii=0.02),
                static=True,
            )
        if len(path):
            segments = np.stack([path[:-1], path[1:]], axis=1)
            rr.log(f"{name}/trajectory", rr.LineStrips3D(segments, colors=path_color), static=True)
    for name, tags in (("raw", raw_tags), ("corrected", corrected_tags)):
        centers = [positions.mean(axis=0) for positions in tags.values()]
        if centers:
            rr.log(
                f"{name}/tags",
                rr.Points3D(
                    np.asarray(centers),
                    colors=[255, 230, 0],
                    radii=0.05,
                    labels=[f"tag{marker_id}" for marker_id in tags],
                ),
                static=True,
            )


def _set_equal_3d_aspect(axis: Any, cloud: np.ndarray) -> None:
    """Equal x/y/z scale so tilt isn't hidden by axis stretching."""
    mins = cloud.min(axis=0)
    maxs = cloud.max(axis=0)
    centers = (mins + maxs) / 2.0
    half = float((maxs - mins).max()) / 2.0 or 1.0
    axis.set_xlim(centers[0] - half, centers[0] + half)
    axis.set_ylim(centers[1] - half, centers[1] + half)
    axis.set_zlim(centers[2] - half, centers[2] + half)

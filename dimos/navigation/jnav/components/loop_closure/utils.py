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
import math
from pathlib import Path
from typing import Any

import numpy as np

from dimos.memory.store.sqlite import SqliteStore
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

# Cap the scans fed to the voxel-agreement metric so it stays quick.
MAP_MAX_SCANS = 400

# Rendered maps keep one point per voxel instead of striding scans, so the ground
# fills in as a surface rather than a set of isolated per-scan lidar rings.
MAP_VOXEL_SIZE_M = 0.1
# Deduplicating in chunks amortizes the sort over many scans.
_VOXEL_FLUSH_POINTS = 4_000_000
# Biases grid indices non-negative so xyz packs into one int64 (21 bits each).
_VOXEL_INDEX_BITS = 21
_VOXEL_INDEX_OFFSET = 1 << (_VOXEL_INDEX_BITS - 1)


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
        try:
            world_from_camera = tf.get(odom_parent, tag_frame, timestamp)
        except LookupError:
            continue
        delta = delta_lookup(timestamp)
        if delta is None:
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
    tf_failure_budget: int = 30,
) -> Iterator[tuple[float, np.ndarray]]:
    """Yield ``(ts, world-frame points)`` for each scan, registered via the tf tree.

    Each scan's frame resolves to ``odom_parent`` through
    ``tf.get(odom_parent, frame, ts)``, which walks the overridden odom edge plus
    any static sensor extrinsics. Scans already in ``odom_parent`` resolve to
    identity. A few lookups normally fail at the recording boundaries (scan ts
    outside the fed odom coverage); each failure spends one ``tf_failure_budget``
    and exhausting it raises, so a broken tree can't silently yield an empty map."""
    with SqliteStore(path=db_path, must_exist=True) as store:
        for observation in islice(store.stream(lidar_stream, PointCloud2), 0, None, stride):
            cloud = observation.data
            timestamp = float(observation.ts)
            points = np.asarray(cloud.points_f32(), dtype=np.float64)[:, :3]
            frame_id = cloud.frame_id or odom_parent
            try:
                world_from_sensor = tf.get(odom_parent, frame_id, timestamp)
            except LookupError as error:
                tf_failure_budget -= 1
                if tf_failure_budget < 0:
                    raise LookupError(
                        f"too many failed tf lookups registering {lidar_stream!r} scans; if "
                        "this recording just has more startup noise than usual, raise the "
                        f"limit with --tf-failure-budget. Last failure: {error}"
                    ) from error
                continue
            matrix = world_from_sensor.to_matrix()
            yield timestamp, points @ matrix[:3, :3].T + matrix[:3, 3]


class _VoxelGrid:
    """Keeps the first point seen in each voxel across a stream of scans."""

    def __init__(self, voxel_size_m: float) -> None:
        self.voxel_size_m = voxel_size_m
        self.points = np.empty((0, 3))
        self._keys = np.empty(0, dtype=np.int64)
        self._pending: list[np.ndarray] = []
        self._pending_points = 0

    def add(self, points: np.ndarray) -> None:
        self._pending.append(points)
        self._pending_points += len(points)
        if self._pending_points >= _VOXEL_FLUSH_POINTS:
            self.flush()

    def flush(self) -> None:
        if not self._pending:
            return
        added = np.concatenate(self._pending)
        index = np.floor(added / self.voxel_size_m).astype(np.int64) + _VOXEL_INDEX_OFFSET
        added_keys = (
            (index[:, 0] << (2 * _VOXEL_INDEX_BITS)) | (index[:, 1] << _VOXEL_INDEX_BITS)
        ) | index[:, 2]
        keys = np.concatenate([self._keys, added_keys])
        points = np.concatenate([self.points, added])
        _, first_seen = np.unique(keys, return_index=True)
        self._keys, self.points = keys[first_seen], points[first_seen]
        self._pending, self._pending_points = [], 0


def accumulate_maps(
    scans: Iterator[tuple[float, np.ndarray]],
    delta_lookup: Any,
    *,
    voxel_size_m: float = MAP_VOXEL_SIZE_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Voxel-downsampled raw + Δ-corrected map points from registered scans."""
    raw_grid = _VoxelGrid(voxel_size_m)
    corrected_grid = _VoxelGrid(voxel_size_m)
    for timestamp, points in scans:
        delta = delta_lookup(timestamp)
        if delta is None:
            continue
        points = points[np.isfinite(points).all(axis=1)]
        rotation_delta, translation_delta = delta
        raw_grid.add(points)
        corrected_grid.add(points @ rotation_delta.T + translation_delta)
    raw_grid.flush()
    corrected_grid.flush()
    return raw_grid.points, corrected_grid.points


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


def resolve_camera_info(
    store: Any, camera_stream: str, override: str = ""
) -> tuple[tuple[np.ndarray, np.ndarray, str] | None, list[str]]:
    """``(camera info, stream names tried)``. Without an override, rigs that name their
    intrinsics after the image stream (``<camera>_camera_info``) resolve before the
    generic ``camera_info``."""
    tried = [override] if override else [f"{camera_stream}_camera_info", "camera_info"]
    for stream_name in tried:
        camera_info = read_camera_info(store, stream_name)
        if camera_info is not None:
            return camera_info, tried
    return None, tried


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
                print(
                    f"WARNING: no {RAW_TAGS_STREAM!r} and no {camera_info_stream!r} stream — "
                    f"can't detect tags, voxel agreement only",
                    flush=True,
                )
                return []
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
    closure_segments: np.ndarray | None = None,
) -> None:
    """Two-panel top-down (x-y) scatter: before vs after correction.

    ``closure_segments`` is ``(N, 6)`` ``[x_start, y_start, z_start, x_end, y_end, z_end]``
    loop-closure edges on the raw path, drawn on the BEFORE panel."""
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.collections import LineCollection
    from matplotlib.colors import LinearSegmentedColormap
    import matplotlib.pyplot as plt

    # single background everywhere, matching inferno's zero-density color
    background = "#000004"
    text_color = "#e8eaed"
    muted_color = "#9aa0a6"
    path_colormap = LinearSegmentedColormap.from_list("path_time", ["#8a5cff", "#ff9f43"])
    # lidar palette avoids the path colors (purple/orange) and closure-edge red
    heat_colormap = LinearSegmentedColormap.from_list(
        "lidar_heat", ["#000004", "#15417e", "#1f9e89", "#a8e063", "#fdf8bf"]
    )
    figure, axes = plt.subplots(1, 2, figsize=(16, 8.5), sharex=True, sharey=True)
    figure.patch.set_facecolor(background)
    # shared bins across both panels so densities are comparable
    stacked_xy = [cloud[:, :2] for cloud in (raw_map, corrected_map) if len(cloud)]
    heatmap_edges: tuple[np.ndarray, np.ndarray] | None = None
    if stacked_xy:
        all_xy = np.vstack(stacked_xy)
        # coarse cells so walls (vertically stacked points) accumulate visibly more
        # heat than floor sweeps
        heatmap_bins = 300
        bin_width = max(np.ptp(all_xy, axis=0).max() / heatmap_bins, 1e-6)
        heatmap_edges = (
            np.arange(all_xy[:, 0].min(), all_xy[:, 0].max() + bin_width, bin_width),
            np.arange(all_xy[:, 1].min(), all_xy[:, 1].max() + bin_width, bin_width),
        )
    for axis, cloud, tags, path, title in (
        (axes[0], raw_map, raw_tags, raw_path, "BEFORE  ·  raw odom"),
        (axes[1], corrected_map, corrected_tags, corrected_path, "AFTER  ·  loop closed"),
    ):
        axis.set_facecolor(background)
        if len(cloud) and heatmap_edges is not None:
            x_edges, y_edges = heatmap_edges
            # voxelize first so heat = occupied z-voxels per cell (vertical structure
            # like walls), not raw point count (biased toward oft-rescanned spots);
            # voxels with almost no point support are stray lidar noise, drop them
            min_points_per_voxel = 4
            occupied_voxels, points_per_voxel = np.unique(
                np.floor(cloud / bin_width).astype(np.int64), axis=0, return_counts=True
            )
            occupied_voxels = occupied_voxels[points_per_voxel >= min_points_per_voxel]
            voxel_centers = (occupied_voxels[:, :2] + 0.5) * bin_width
            density, _, _ = np.histogram2d(
                voxel_centers[:, 0], voxel_centers[:, 1], bins=(x_edges, y_edges)
            )
            # saturate at a full floor-to-ceiling stack: walls hit max heat, noise
            # streaks of a few voxels stay dark
            wall_height_meters = 2.5
            axis.imshow(
                density.T,
                origin="lower",
                extent=(x_edges[0], x_edges[-1], y_edges[0], y_edges[-1]),
                cmap=heat_colormap,
                interpolation="nearest",
                vmax=max(wall_height_meters / bin_width, 1.0),
                rasterized=True,
            )
        if len(path):
            # gradient along the path shows time: purple (start) -> yellow (end)
            xy_points = path[:, :2].reshape(-1, 1, 2)
            segments = np.concatenate([xy_points[:-1], xy_points[1:]], axis=1)
            axis.add_collection(
                LineCollection(
                    segments,
                    cmap=path_colormap,
                    array=np.linspace(0.0, 1.0, max(len(segments), 1)),
                    linewidth=2.0,
                    capstyle="round",
                    zorder=4,
                )
            )
            axis.update_datalim(path[:, :2])
            if axis is axes[0] and closure_segments is not None and len(closure_segments):
                axis.add_collection(
                    LineCollection(
                        closure_segments[:, [0, 1, 3, 4]].reshape(-1, 2, 2).tolist(),
                        colors="#ff3b3b",
                        linewidth=1.0,
                        alpha=0.9,
                        zorder=5,
                    )
                )
            for point, label, marker in ((path[0], "start", "o"), (path[-1], "end", "s")):
                axis.scatter(
                    point[0],
                    point[1],
                    s=70,
                    c="white",
                    marker=marker,
                    edgecolors=background,
                    zorder=6,
                )
                axis.annotate(
                    label,
                    point[:2],
                    xytext=(8, 8),
                    textcoords="offset points",
                    fontsize=10,
                    color=text_color,
                    zorder=7,
                )
        for marker_id, positions in tags.items():
            axis.scatter(
                positions[:, 0], positions[:, 1], s=90, marker="X", edgecolors="white", zorder=5
            )
            centroid = positions.mean(axis=0)
            axis.annotate(f"tag {marker_id}", centroid[:2], fontsize=9, color=text_color, zorder=7)
        axis.set_title(title, loc="left", fontsize=15, fontweight="bold", color=text_color, pad=12)
        axis.set_aspect("equal")
        axis.locator_params(nbins=5)
        axis.tick_params(colors=muted_color, labelsize=8, length=3, width=0.6, direction="out")
        for spine in axis.spines.values():
            spine.set_visible(False)
    # crop to the trajectory (plus room for walls) — the full cloud extent is
    # blown out by stray long-range returns
    stacked_paths = [path[:, :2] for path in (raw_path, corrected_path) if len(path)]
    if stacked_paths:
        paths_xy = np.vstack(stacked_paths)
        low_corner = paths_xy.min(axis=0)
        high_corner = paths_xy.max(axis=0)
        crop_pad = np.maximum(0.15 * (high_corner - low_corner), 8.0)
        axes[0].set_xlim(low_corner[0] - crop_pad[0], high_corner[0] + crop_pad[0])
        axes[0].set_ylim(low_corner[1] - crop_pad[1], high_corner[1] + crop_pad[1])
    # scale bar instead of axis ticks: pick a round length near a fifth of the width
    x_low, x_high = axes[0].get_xlim()
    y_low, y_high = axes[0].get_ylim()
    rough_bar = (x_high - x_low) / 5.0
    magnitude = 10.0 ** math.floor(math.log10(rough_bar)) if rough_bar > 0 else 1.0
    bar_length = max(
        (step * magnitude for step in (1.0, 2.0, 5.0) if step * magnitude <= rough_bar),
        default=magnitude,
    )
    margin_fraction = 0.04
    bar_x = x_low + (x_high - x_low) * margin_fraction
    bar_y = y_low + (y_high - y_low) * margin_fraction
    for axis in axes:
        axis.plot([bar_x, bar_x + bar_length], [bar_y, bar_y], color=muted_color, linewidth=3)
        axis.annotate(
            f"{bar_length:g} m",
            (bar_x + bar_length / 2.0, bar_y),
            xytext=(0, 6),
            textcoords="offset points",
            ha="center",
            fontsize=10,
            color=muted_color,
        )
    figure.suptitle(
        f"{recording_name} — top-down lidar map",
        fontsize=17,
        fontweight="bold",
        color=text_color,
        x=0.06,
        y=0.99,
        ha="left",
    )
    figure.text(
        0.06,
        0.945,
        "heat = vertical structure (occupied z-voxels per cell)  ·  path colored by time (purple start → orange end)",
        fontsize=10,
        color=muted_color,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
    figure.savefig(png_path, dpi=130, facecolor=background)
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
    closure_segments: np.ndarray | None = None,
) -> None:
    """Two-panel isometric (3D) scatter of the lidar map: before vs after correction.

    Complements the top-down: false closures often tilt the map into z (horizontal
    travel bleaking into vertical), which is invisible flattened but obvious here.
    Points are colored by height so the ground plane and any tilt read at a glance.
    ``closure_segments`` is ``(N, 6)`` ``[x_start, y_start, z_start, x_end, y_end, z_end]``
    loop-closure edges on the raw path, drawn on the BEFORE panel."""
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.colors import LinearSegmentedColormap, ListedColormap
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Line3DCollection  # type: ignore[import-untyped]

    background = "#000004"
    text_color = "#e8eaed"
    muted_color = "#9aa0a6"
    path_colormap = LinearSegmentedColormap.from_list("path_time", ["#8a5cff", "#ff9f43"])
    # the low end of turbo only (blue -> yellow-green); the warm half collides with the
    # orange path and red closure edges
    heat_colormap = ListedColormap(plt.get_cmap("turbo")(np.linspace(0.0, 0.4, 256)))
    figure = plt.figure(figsize=(16, 8))
    figure.patch.set_facecolor(background)
    panels = (
        (raw_map, raw_path, "BEFORE  ·  raw odom"),
        (corrected_map, corrected_path, "AFTER  ·  loop closed"),
    )
    for index, (cloud, path, title) in enumerate(panels):
        axis = figure.add_subplot(1, 2, index + 1, projection="3d")
        axis.set_facecolor(background)
        # manual zorder: the path and closure edges must draw over the cloud,
        # not be depth-sorted into it
        axis.computed_zorder = False
        if len(cloud) and len(path):
            # crop to the trajectory neighborhood — stray long-range returns blow
            # out the plot range otherwise
            xy_low = path[:, :2].min(axis=0) - 8.0
            xy_high = path[:, :2].max(axis=0) + 8.0
            z_low = path[:, 2].min() - 2.0
            z_high = path[:, 2].max() + 4.0
            inside = np.all((cloud >= [*xy_low, z_low]) & (cloud <= [*xy_high, z_high]), axis=1)
            cloud = cloud[inside]
        if len(cloud):
            # percentile limits, not min/max: the crop window reaches below the ground
            # and a few stray high returns stretch the top, so autoscaling wastes most
            # of the ramp on empty height
            color_low, color_high = np.percentile(cloud[:, 2], (2.0, 98.0))
            axis.scatter(
                cloud[:, 0],
                cloud[:, 1],
                cloud[:, 2],
                s=0.3,
                c=cloud[:, 2],
                cmap=heat_colormap,
                vmin=color_low,
                vmax=color_high,
                linewidths=0,
                rasterized=True,
                zorder=1,
            )
        if len(path):
            # gradient along the path shows time: purple (start) -> orange (end)
            xyz_points = path[:, :3].reshape(-1, 1, 3)
            segments = np.concatenate([xyz_points[:-1], xyz_points[1:]], axis=1)
            axis.add_collection3d(
                Line3DCollection(
                    segments,
                    cmap=path_colormap,
                    array=np.linspace(0.0, 1.0, max(len(segments), 1)),
                    linewidth=2.2,
                    capstyle="round",
                    zorder=4,
                )
            )
            if index == 0 and closure_segments is not None and len(closure_segments):
                axis.add_collection3d(
                    Line3DCollection(
                        closure_segments.reshape(-1, 2, 3),
                        colors="#ff3b3b",
                        linewidth=1.0,
                        alpha=0.9,
                        zorder=5,
                    )
                )
            for point, label, marker in ((path[0], "start", "o"), (path[-1], "end", "s")):
                axis.scatter(
                    point[0],
                    point[1],
                    point[2],
                    s=60,
                    c="white",
                    marker=marker,
                    edgecolors=background,
                    zorder=6,
                )
                axis.text(point[0], point[1], point[2] + 1.0, label, color=text_color, fontsize=9)
        axis.set_title(title, color=text_color, fontsize=13, fontweight="bold")
        for named_axis in (axis.xaxis, axis.yaxis, axis.zaxis):
            named_axis.set_pane_color((0.0, 0.0, 0.0, 0.0))
            named_axis.label.set_color(muted_color)
        axis.tick_params(colors=muted_color, labelsize=7)
        axis.grid(False)
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.set_zlabel("z (m)")
        axis.view_init(elev=_ISO_ELEV_DEG, azim=_ISO_AZIM_DEG)
        if len(cloud):
            _set_equal_3d_aspect(axis, cloud)
    figure.suptitle(
        f"{recording_name} — isometric lidar map",
        fontsize=16,
        fontweight="bold",
        color=text_color,
        x=0.06,
        y=0.99,
        ha="left",
    )
    figure.text(
        0.06,
        0.945,
        "points colored by height  ·  path colored by time (purple start → orange end)",
        fontsize=10,
        color=muted_color,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.92))
    figure.savefig(png_path, dpi=130, facecolor=background)
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
    closure_segments: np.ndarray | None = None,
) -> None:
    """Rerun rrd: raw vs corrected lidar cloud (z-shaded), trajectories, tag medians, and
    loop-closure edges (on the raw trajectory, like the top-down BEFORE panel)."""
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
    if closure_segments is not None and len(closure_segments):
        rr.log(
            "raw/loop_closures",
            rr.LineStrips3D(closure_segments.reshape(-1, 2, 3), colors=[255, 59, 59]),
            static=True,
        )
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
    """Equal x/y/z scale so tilt isn't hidden by axis stretching.

    Limits hug the data and the box aspect matches the spans, so a flat cloud
    fills the panel instead of floating inside a mostly-empty cube."""
    mins = cloud.min(axis=0)
    maxs = cloud.max(axis=0)
    spans = np.maximum(maxs - mins, 1.0)
    axis.set_xlim(mins[0], maxs[0])
    axis.set_ylim(mins[1], maxs[1])
    axis.set_zlim(mins[2], maxs[2])
    axis.set_box_aspect(tuple(spans))

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

"""Combined comparison rrd: raw lidar cloud + EVERY *_corrected*_lidar version present in the db,
each as its own colored entity, plus AprilTag landmarks + trajectories. Re-run after adding a new
corrected method and it picks the new stream up automatically.

Importable: `build(...)` writes the rrd and returns its path (used by post_process.py).
Standalone: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/make_rrd.py --rec=PATH [--lidar=...] [--odom=...] [--tags=...] [--out=...]
"""

import colorsys
import json
from pathlib import Path
import sys

import cv2
from gtsam import Point3, Pose3, Rot3
import numpy as np
import rerun as rr

from dimos.navigation.jnav.utils import recording_db as rdb
from dimos.navigation.jnav.utils.apriltags import filter_glimpses, read_raw_tag_stream
from dimos.navigation.jnav.utils.trajectory_metrics import nearest_index

SCAN_STRIDE, VOXEL = 8, 0.10
TAG_SIZE_M = 0.10  # matches post_process --tag-size default
TAG_DICT = "DICT_APRILTAG_36h11"
TAG_IMAGE_PX = 200  # 36h11 incl. border is 10 modules; render each as 20 px
# marker-frame corners (OpenCV aruco: x right, y up, z out), texcoord order TL TR BR BL
TAG_CORNERS = np.array(
    [
        [-TAG_SIZE_M / 2, TAG_SIZE_M / 2, 0.0],
        [TAG_SIZE_M / 2, TAG_SIZE_M / 2, 0.0],
        [TAG_SIZE_M / 2, -TAG_SIZE_M / 2, 0.0],
        [-TAG_SIZE_M / 2, -TAG_SIZE_M / 2, 0.0],
    ]
)


def tag_image(marker_id):
    """RGB bitmap of the actual AprilTag, for texturing its 3D placement."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, TAG_DICT))
    grayscale = cv2.aruco.generateImageMarker(dictionary, marker_id, TAG_IMAGE_PX)
    return np.repeat(grayscale[:, :, None], 3, axis=2)


COLORS = {"raw": [220, 60, 60]}
PALETTE = [
    [60, 120, 230],
    [60, 210, 90],
    [230, 180, 50],
    [200, 80, 220],
    [80, 220, 220],
    [240, 130, 60],
]
# relaxed vs the detection defaults: landmarks are display markers, not PGO factors
LANDMARK_GATES = dict(
    min_sharpness=25.0,
    max_reproj_px=3.5,
    min_tag_px=12.0,
    max_distance_m=1.5,
    max_view_angle_deg=65.0,
    max_linear_speed_mps=1.5,
    max_angular_speed_dps=150.0,
)


def cli_arg(flag, default=""):
    """``--flag=value`` lookup in sys.argv."""
    return next(
        (item.split("=", 1)[1] for item in sys.argv if item.startswith(flag + "=")), default
    )


Z_GRADIENT_PERCENTILES = (2.0, 98.0)  # clip outlier floors/ceilings out of the color range
GRADIENT_DARK = 0.3  # gradient start: this fraction of the stream color
GRADIENT_LIGHT = 0.7  # gradient end: blended this far toward white


def shade(base_color, t):
    """Colors along the dark->light gradient of ``base_color`` at fractions ``t`` in [0, 1]."""
    base = np.asarray(base_color, float)
    dark = base * GRADIENT_DARK
    light = base + (255.0 - base) * GRADIENT_LIGHT
    return (dark + (light - dark) * np.asarray(t, float)[:, None]).astype(np.uint8)


def z_gradient_colors(points, base_color):
    """Per-point colors: the stream color shaded dark (low z) to light (high z)."""
    z_values = points[:, 2]
    low, high = np.percentile(z_values, Z_GRADIENT_PERCENTILES)
    return shade(base_color, np.clip((z_values - low) / ((high - low) or 1.0), 0.0, 1.0))


TRAJECTORY_DARK = 0.45  # gradient start: this fraction of the full-vibrance color


def vibrant(base_color):
    """The fully-saturated pure hue of ``base_color`` (so paths pop against the muted clouds)."""
    red, green, blue = (channel / 255.0 for channel in base_color)
    hue, _lightness, _saturation = colorsys.rgb_to_hls(red, green, blue)
    return [round(channel * 255) for channel in colorsys.hls_to_rgb(hue, 0.5, 1.0)]


def gradient_trajectory(positions, base_color):
    """(segments, colors) for a path shaded dark (start) to full vibrance (finish)."""
    segments = np.stack([positions[:-1], positions[1:]], axis=1)
    t = np.linspace(0.0, 1.0, len(segments))[:, None]
    full = np.asarray(vibrant(base_color), float)
    return segments, (full * (TRAJECTORY_DARK + (1.0 - TRAJECTORY_DARK) * t)).astype(np.uint8)


def pose3_from_xyzquat(xyzquat):
    """(x, y, z, qx, qy, qz, qw) -> Pose3."""
    return Pose3(
        Rot3.Quaternion(xyzquat[6], xyzquat[3], xyzquat[4], xyzquat[5]),
        Point3(xyzquat[0], xyzquat[1], xyzquat[2]),
    )


def build(
    rec,
    lidar_stream="pointlio_lidar",
    odom_stream="pointlio_odometry",
    tag_stream="raw_april_tags",
    out_name="corrected_compare.rrd",
):
    rec_path = Path(rec).expanduser()
    db_path = rec_path / "mem2.db" if rec_path.is_dir() else rec_path
    recording_dir = db_path.parent
    out_path = recording_dir / out_name
    store = rdb.store(db_path)
    intrinsics_path = recording_dir / "camera_intrinsics.json"
    base_to_optical = (
        pose3_from_xyzquat(
            np.array(json.loads(intrinsics_path.read_text())["optical_in_base"], float)
        )
        if intrinsics_path.exists()
        else None
    )

    def accumulate(stream_name):
        scans = []
        for scan_index, observation in enumerate(store.stream(stream_name)):
            if scan_index % SCAN_STRIDE:
                continue
            points = np.asarray(observation.data.points_f32())
            if len(points):
                scans.append(points[::3])
        if not scans:
            sys.exit(f"stream {stream_name!r} has no points in {db_path}")
        all_points = np.concatenate(scans, 0)
        _, unique_indices = np.unique(
            np.floor(all_points / VOXEL).astype(np.int64), axis=0, return_index=True
        )
        return all_points[unique_indices]

    def traj(stream_name):
        return rdb.odometry_rows(db_path, stream_name)[:, 1:4].astype(np.float32)

    def landmarks(gt_odom):
        odom_rows = rdb.odometry_rows(db_path, gt_odom)
        detections = filter_glimpses(
            read_raw_tag_stream(store, tag_stream), exclude_tags=(), **LANDMARK_GATES
        )
        positions_by_marker = {}
        best_by_marker = {}  # lowest-reproj detection: its rotation orients the tag square
        for detection in detections:
            base_pose = pose3_from_xyzquat(
                odom_rows[nearest_index(odom_rows[:, 0], detection["ts"])][1:]
            )
            tag_in_world = base_pose.compose(base_to_optical).compose(
                pose3_from_xyzquat(detection["t_cam_marker"])
            )
            marker_id = detection["marker_id"]
            positions_by_marker.setdefault(marker_id, []).append(
                np.asarray(tag_in_world.translation())
            )
            if (
                marker_id not in best_by_marker
                or detection["reproj_px"] < best_by_marker[marker_id][0]
            ):
                best_by_marker[marker_id] = (detection["reproj_px"], tag_in_world)
        marker_ids = sorted(positions_by_marker)
        mean_positions = [np.mean(positions_by_marker[mid], 0) for mid in marker_ids]
        rotations = [np.asarray(best_by_marker[mid][1].rotation().matrix()) for mid in marker_ids]
        return np.array(mean_positions), rotations, marker_ids

    streams = store.list_streams()
    corrected_lidars = sorted(
        stream_name
        for stream_name in streams
        if "_corrected" in stream_name and "lidar" in stream_name
    )
    print("raw + corrected lidar streams:", corrected_lidars)

    rr.init("corrected_compare")
    rr.save(str(out_path))
    raw_cloud = accumulate(lidar_stream)
    rr.log(
        "raw/cloud",
        rr.Points3D(raw_cloud, colors=z_gradient_colors(raw_cloud, COLORS["raw"]), radii=0.02),
        static=True,
    )
    raw_segments, raw_traj_colors = gradient_trajectory(traj(odom_stream), [255, 120, 120])
    rr.log(
        "raw/trajectory",
        rr.LineStrips3D(raw_segments, colors=raw_traj_colors),
        static=True,
    )
    for lidar_index, lidar_name in enumerate(corrected_lidars):
        color = PALETTE[lidar_index % len(PALETTE)]
        cloud = accumulate(lidar_name)
        rr.log(
            f"{lidar_name}/cloud",
            rr.Points3D(cloud, colors=z_gradient_colors(cloud, color), radii=0.02),
            static=True,
        )
        print(f"  logged {lidar_name}: {len(cloud):,} pts")
        odom_candidates = [lidar_name.replace("lidar", stem) for stem in ("odometry", "odom")]
        odom_name = next((name for name in odom_candidates if name in streams), "")
        if odom_name:
            segments, traj_colors = gradient_trajectory(traj(odom_name), color)
            rr.log(
                f"{lidar_name}/trajectory",
                rr.LineStrips3D(segments, colors=traj_colors),
                static=True,
            )
    # landmarks placed against the first available corrected odometry
    corrected_odoms = sorted(
        stream_name
        for stream_name in streams
        if "_corrected" in stream_name and "odom" in stream_name
    )
    if corrected_odoms and base_to_optical is None:
        print(f"no {intrinsics_path.name} — skipping tag landmarks")
    elif corrected_odoms:
        landmark_positions, landmark_rotations, marker_ids = landmarks(corrected_odoms[0])
        for center, rotation, marker_id in zip(
            landmark_positions, landmark_rotations, marker_ids, strict=True
        ):
            vertices = TAG_CORNERS @ rotation.T + center
            rr.log(
                f"landmarks/tag{marker_id}",
                rr.Mesh3D(
                    vertex_positions=vertices,
                    triangle_indices=[[0, 1, 2], [0, 2, 3]],
                    vertex_texcoords=[[0, 0], [1, 0], [1, 1], [0, 1]],
                    albedo_texture=tag_image(marker_id),
                ),
                static=True,
            )
        if marker_ids:
            rr.log(
                "landmarks/labels",
                rr.Points3D(
                    landmark_positions,
                    colors=[255, 230, 0],
                    radii=0.005,
                    labels=[f"tag{marker_id}" for marker_id in marker_ids],
                ),
                static=True,
            )
            print(f"  logged {len(marker_ids)} landmarks")
    print("wrote", out_path)
    return out_path


if __name__ == "__main__":
    rec_arg = cli_arg("--rec")
    if not rec_arg:
        sys.exit(
            "usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/make_rrd.py --rec=PATH [--lidar=...] [--odom=...] "
            "[--tags=...] [--out=...]   (--rec is required)"
        )
    build(
        rec_arg,
        lidar_stream=cli_arg("--lidar", "pointlio_lidar"),
        odom_stream=cli_arg("--odom", "pointlio_odometry"),
        tag_stream=cli_arg("--tags", "raw_april_tags"),
        out_name=cli_arg("--out", "corrected_compare.rrd"),
    )

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
Standalone: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/make_rrd.py --db=PATH.db [--lidar=...] [--odom=...] [--tags=...] [--out=...]
"""

import colorsys
from pathlib import Path
import sys
from typing import Any

import cv2
from gtsam import Point3, Pose3, Rot3
import numpy as np
import rerun as rr

from dimos.memory2.store.sqlite import SqliteStore
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.recording_scans import (
    default_odom_edge,
)
from dimos.navigation.jnav.components.loop_closure.utils import read_camera_info
from dimos.navigation.jnav.utils.apriltags import filter_glimpses, read_raw_tag_stream
from dimos.navigation.jnav.utils.recording_tf import RecordingTF
from dimos.navigation.jnav.utils.trajectory_metrics import nearest_index

SCAN_STRIDE, VOXEL = 8, 0.10
MAX_RENDER_POINTS = 200_000  # cap each accumulated cloud so rerun stays responsive
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


def tag_image(marker_id: int) -> np.ndarray:
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


def cli_arg(flag: str, default: str = "") -> str:
    """``--flag=value`` lookup in sys.argv."""
    return next(
        (item.split("=", 1)[1] for item in sys.argv if item.startswith(flag + "=")), default
    )


Z_GRADIENT_PERCENTILES = (2.0, 98.0)  # clip outlier floors/ceilings out of the color range
GRADIENT_DARK = 0.35  # low-z end: this fraction of the stream color


def shade(base_color: Any, t: Any) -> np.ndarray:
    """Colors ramping low-z (dark stream color) -> high-z (its vivid saturated hue), at
    fractions ``t`` in [0, 1]. Both ends stay colored, so height never washes out to white."""
    low: np.ndarray = np.asarray(base_color, float) * GRADIENT_DARK
    high: np.ndarray = np.asarray(vibrant(base_color), float)
    colors: np.ndarray = (low + (high - low) * np.asarray(t, float)[:, None]).astype(np.uint8)
    return colors


def z_gradient_colors(points: np.ndarray, base_color: Any) -> np.ndarray:
    """Per-point colors: the stream color shaded dark (low z) to light (high z)."""
    z_values = points[:, 2]
    low, high = np.percentile(z_values, Z_GRADIENT_PERCENTILES)
    return shade(base_color, np.clip((z_values - low) / ((high - low) or 1.0), 0.0, 1.0))


TRAJECTORY_DARK = 0.45  # gradient start: this fraction of the full-vibrance color


def vibrant(base_color: Any) -> list[int]:
    """The fully-saturated pure hue of ``base_color`` (so paths pop against the muted clouds)."""
    red, green, blue = (channel / 255.0 for channel in base_color)
    hue, _lightness, _saturation = colorsys.rgb_to_hls(red, green, blue)
    return [round(channel * 255) for channel in colorsys.hls_to_rgb(hue, 0.5, 1.0)]


def gradient_trajectory(positions: np.ndarray, base_color: Any) -> tuple[np.ndarray, np.ndarray]:
    """(segments, colors) for a path shaded dark (start) to full vibrance (finish)."""
    segments = np.stack([positions[:-1], positions[1:]], axis=1)
    t = np.linspace(0.0, 1.0, len(segments))[:, None]
    full = np.asarray(vibrant(base_color), float)
    return segments, (full * (TRAJECTORY_DARK + (1.0 - TRAJECTORY_DARK) * t)).astype(np.uint8)


def pose3_from_xyzquat(xyzquat: np.ndarray) -> Pose3:
    """(x, y, z, qx, qy, qz, qw) -> Pose3."""
    return Pose3(
        Rot3.Quaternion(xyzquat[6], xyzquat[3], xyzquat[4], xyzquat[5]),
        Point3(xyzquat[0], xyzquat[1], xyzquat[2]),
    )


def build(
    db: str | Path,
    lidar_stream: str = "pointlio_lidar",
    odom_stream: str = "pointlio_odometry",
    tag_stream: str = "raw_april_tags",
    out_name: str = "corrected_compare.rrd",
    world_frame: str = "world",
) -> Path:
    db_path = Path(db).expanduser()
    if db_path.is_dir():
        sys.exit(f"--db must be a .db file, not a directory: {db_path}")
    recording_dir = db_path.parent
    out_path = recording_dir / out_name
    store = SqliteStore(path=db_path, must_exist=True)
    store.start()
    odom_tf = default_odom_edge(store, odom_stream)
    body_frame = odom_tf.split(":", 1)[1] if odom_tf else world_frame
    lidar_frame = body_frame
    store_tf = RecordingTF.from_store(store, odom_tf=odom_tf or None, odom_stream=odom_stream)

    # base<-optical camera extrinsic, read from the tf tree (was the json's optical_in_base)
    camera_info = read_camera_info(store)
    base_to_optical = None
    if camera_info is not None and store_tf is not None:
        optical_frame = camera_info[2] or "camera_optical"
        extrinsic = store_tf.get(body_frame, optical_frame)
        if extrinsic is not None:
            base_to_optical = Pose3(extrinsic.to_matrix())

    def tf_world_points(
        observation: Any, tf: RecordingTF | None, world: str, fallback_frame: str
    ) -> np.ndarray:
        points = np.asarray(observation.data.points_f32())
        scan_frame = getattr(observation.data, "frame_id", "") or fallback_frame
        transform = tf.get(world, scan_frame, float(observation.ts), None) if tf else None
        if transform is None or not len(points):
            return points
        rotation = np.asarray(transform.rotation.to_rotation_matrix(), float).reshape(3, 3)
        offset = transform.translation
        translation = np.array([offset.x, offset.y, offset.z], float)
        world_points: np.ndarray = (points @ rotation.T + translation).astype(np.float32)
        return world_points

    def accumulate(stream_name: str, register: bool = False) -> np.ndarray:
        scans = []
        observation: Any
        for scan_index, observation in enumerate(store.stream(stream_name)):
            if scan_index % SCAN_STRIDE:
                continue
            if register:
                points = tf_world_points(observation, store_tf, world_frame, lidar_frame)
            else:
                points = np.asarray(observation.data.points_f32())
            if len(points):
                scans.append(points[::3])
        if not scans:
            sys.exit(f"stream {stream_name!r} has no points in {db_path}")
        all_points = np.concatenate(scans, 0)
        _, unique_indices = np.unique(
            np.floor(all_points / VOXEL).astype(np.int64), axis=0, return_index=True
        )
        voxelized: np.ndarray = all_points[unique_indices]
        if len(voxelized) > MAX_RENDER_POINTS:
            keep = np.random.default_rng(0).choice(len(voxelized), MAX_RENDER_POINTS, replace=False)
            voxelized = voxelized[keep]
        return voxelized

    def odom_samples(stream_name: str) -> np.ndarray:
        rows = []
        observation: Any
        for observation in store.stream(stream_name).order_by("ts"):
            pose = observation.data.pose.pose
            rows.append(
                (
                    float(observation.ts),
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            )
        samples: np.ndarray = np.asarray(rows, dtype=np.float64).reshape(-1, 8)
        return samples

    def traj(stream_name: str) -> np.ndarray:
        positions: np.ndarray = odom_samples(stream_name)[:, 1:4].astype(np.float32)
        return positions

    def landmarks(gt_odom: str) -> tuple[np.ndarray, list[np.ndarray], list[int]]:
        odom_rows = odom_samples(gt_odom)
        detections = filter_glimpses(
            read_raw_tag_stream(store, tag_stream), exclude_tags=(), **LANDMARK_GATES
        )
        positions_by_marker: dict[int, list[np.ndarray]] = {}
        best_by_marker: dict[
            int, tuple[float, Pose3]
        ] = {}  # lowest-reproj detection orients the tag square
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
    raw_cloud = accumulate(lidar_stream, register=True)
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
        print("no CameraInfo stream or optical tf edge — skipping tag landmarks")
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
    store.stop()
    print("wrote", out_path)
    return out_path


if __name__ == "__main__":
    db_arg = cli_arg("--db")
    if not db_arg:
        sys.exit(
            "usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/make_rrd.py --db=PATH.db [--lidar=...] [--odom=...] "
            "[--tags=...] [--out=...] [--world-frame=...]   (--db is required)"
        )
    build(
        db_arg,
        lidar_stream=cli_arg("--lidar", "pointlio_lidar"),
        odom_stream=cli_arg("--odom", "pointlio_odometry"),
        tag_stream=cli_arg("--tags", "raw_april_tags"),
        out_name=cli_arg("--out", "corrected_compare.rrd"),
        world_frame=cli_arg("--world-frame", "world"),
    )

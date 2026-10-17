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

from pathlib import Path
import sys
from typing import Any

from gtsam import Point3, Pose3, Rot3
import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.recording_scans import (
    default_odom_edge,
)
from dimos.navigation.jnav.components.loop_closure.utils import resolve_camera_info
from dimos.navigation.jnav.utils.apriltags import (
    DEFAULT_ROTATION_WEIGHT_M_PER_RAD,
    Detection,
    cluster_medoid,
    filter_glimpses,
    read_raw_tag_stream,
)
from dimos.navigation.jnav.utils.recording_tf import RecordingTF
from dimos.navigation.jnav.utils.trajectory_metrics import nearest_index

SCAN_STRIDE, VOXEL = 8, 0.10
MAX_RENDER_POINTS = 200_000  # cap each accumulated cloud so rerun stays responsive
TAG_SIZE_M = 0.10  # matches post_process --tag-size default
TAG_DICT = "DICT_APRILTAG_36h11"
TAG_IMAGE_PX = 200  # 36h11 incl. border is 10 modules; render each as 20 px
CAMERA_MATCH_SEC = 0.2  # how far from a medoid glimpse to look for its color frame
FRUSTUM_PLANE_M = 0.6  # image plane distance of the placed medoid views
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
    import cv2

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, TAG_DICT))
    grayscale = cv2.aruco.generateImageMarker(dictionary, marker_id, TAG_IMAGE_PX)
    return np.repeat(grayscale[:, :, None], 3, axis=2)


# each entry fades between two distinct hues. Both endpoints are kept bright so the ramp
# reads as a change of color, not of brightness (and neither end sinks into the background).
Gradient = tuple[list[int], list[int]]
RAW_CLOUD_GRADIENT: Gradient = ([235, 45, 95], [250, 205, 60])  # crimson -> amber
RAW_TRAJECTORY_GRADIENT: Gradient = ([255, 95, 165], [255, 240, 120])  # pink -> gold
CLOUD_GRADIENTS: list[Gradient] = [
    ([60, 110, 255], [90, 245, 180]),  # blue -> aqua
    ([70, 200, 90], [235, 230, 70]),  # green -> yellow
    ([170, 90, 250], [250, 110, 170]),  # violet -> pink
    ([250, 130, 60], [245, 225, 130]),  # orange -> sand
]
TRAJECTORY_GRADIENTS: list[Gradient] = [
    ([120, 175, 255], [140, 255, 220]),
    ([130, 245, 140], [245, 255, 130]),
    ([210, 150, 255], [255, 165, 210]),
    ([255, 175, 110], [255, 240, 180]),
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


def ramp(gradient: Gradient, t: Any) -> np.ndarray:
    """Colors interpolated from the gradient's first color to its second, at fractions
    ``t`` in [0, 1]."""
    start, end = (np.asarray(color, float) for color in gradient)
    colors: np.ndarray = (start + (end - start) * np.asarray(t, float)[:, None]).astype(np.uint8)
    return colors


def z_gradient_colors(points: np.ndarray, gradient: Gradient) -> np.ndarray:
    """Per-point colors fading across the gradient with height."""
    z_values = points[:, 2]
    low, high = np.percentile(z_values, Z_GRADIENT_PERCENTILES)
    return ramp(gradient, np.clip((z_values - low) / ((high - low) or 1.0), 0.0, 1.0))


def gradient_trajectory(positions: np.ndarray, gradient: Gradient) -> tuple[np.ndarray, np.ndarray]:
    """(segments, colors) for a path fading across the gradient from start to finish."""
    segments = np.stack([positions[:-1], positions[1:]], axis=1)
    return segments, ramp(gradient, np.linspace(0.0, 1.0, len(segments)))


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
    camera_stream: str = "color_image",
    camera_info_stream: str = "",
) -> Path:
    import rerun as rr
    import rerun.blueprint as rrb

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
    camera_info, camera_info_tried = resolve_camera_info(store, camera_stream, camera_info_stream)
    base_to_optical, camera_matrix = None, None
    if camera_info is None:
        print(f"no CameraInfo stream among {camera_info_tried}")
    else:
        camera_matrix = camera_info[0]
        optical_frame = camera_info[2] or "camera_optical"
        try:
            base_to_optical = Pose3(store_tf.get(body_frame, optical_frame).to_matrix())
        except LookupError:
            print(f"no {body_frame} <- {optical_frame} tf edge")

    def tf_world_points(
        observation: Any, tf: RecordingTF, world: str, fallback_frame: str
    ) -> np.ndarray:
        points = np.asarray(observation.data.points_f32())
        if not len(points):
            return points
        scan_frame = getattr(observation.data, "frame_id", "") or fallback_frame
        transform = tf.get(world, scan_frame, float(observation.ts), None)
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

    def landmarks(gt_odom: str) -> list[dict[str, Any]]:
        """Per marker: the mean tag position, plus the medoid glimpse — the detection whose
        pose is most central — which orients the tag square and places the camera frustum."""
        odom_rows = odom_samples(gt_odom)
        if not len(odom_rows) or tag_stream not in streams:
            return []
        detections = filter_glimpses(
            read_raw_tag_stream(store, tag_stream), exclude_tags=(), **LANDMARK_GATES
        )
        by_marker: dict[int, list[tuple[Detection, Pose3, Pose3]]] = {}
        for detection in detections:
            base_pose = pose3_from_xyzquat(
                odom_rows[nearest_index(odom_rows[:, 0], detection["ts"])][1:]
            )
            camera_pose = base_pose.compose(base_to_optical)
            tag_in_world = camera_pose.compose(pose3_from_xyzquat(detection["t_cam_marker"]))
            by_marker.setdefault(detection["marker_id"], []).append(
                (detection, tag_in_world, camera_pose)
            )
        found = []
        for marker_id in sorted(by_marker):
            glimpses = by_marker[marker_id]
            medoid = cluster_medoid(
                [glimpse[0] for glimpse in glimpses], DEFAULT_ROTATION_WEIGHT_M_PER_RAD
            )
            _, medoid_tag, medoid_camera = next(
                glimpse for glimpse in glimpses if glimpse[0] is medoid
            )
            found.append(
                {
                    "marker_id": marker_id,
                    "position": np.mean(
                        [np.asarray(glimpse[1].translation()) for glimpse in glimpses], 0
                    ),
                    "rotation": np.asarray(medoid_tag.rotation().matrix()),
                    "camera_pose": np.asarray(medoid_camera.matrix(), float),
                    "ts": float(medoid["ts"]),
                }
            )
        return found

    def log_medoid_camera(entity: str, camera_pose: np.ndarray, ts: float) -> bool:
        """Place the camera's-eye image on a pinhole frustum at the pose it was taken from."""
        if camera_matrix is None:
            return False
        try:
            image = store.stream(camera_stream, Image).at(ts, CAMERA_MATCH_SEC).first().data
        except LookupError:
            return False
        rr.log(
            entity,
            rr.Transform3D(translation=camera_pose[:3, 3], mat3x3=camera_pose[:3, :3]),
            static=True,
        )
        rr.log(
            entity,
            rr.Pinhole(
                image_from_camera=camera_matrix,
                resolution=[image.width, image.height],
                camera_xyz=rr.ViewCoordinates.RDF,
                image_plane_distance=FRUSTUM_PLANE_M,
            ),
            static=True,
        )
        # raw pixels, not Image.to_rerun(): a JPEG EncodedImage under a pinhole hangs the
        # rerun 0.32 viewer forever (black window, "application not responding")
        rr.log(f"{entity}/rgb", rr.Image(image.data, color_model="BGR"), static=True)
        return True

    streams = store.list_streams()
    # the world-registered accumulated corrected clouds; the per-scan `*_corrected` streams are
    # stored sensor-relative (frame `corrected_odom`), so they'd render as a blob without tf.
    corrected_lidars = sorted(
        stream_name
        for stream_name in streams
        if "_corrected" in stream_name and stream_name.endswith("_accumulated")
    )
    print("corrected accumulated lidar streams:", corrected_lidars)

    rr.init("corrected_compare")
    rr.save(str(out_path))
    # a single 3D view: the medoid photos belong on their frustums in the scene, not in the
    # per-image 2D panels rerun's default layout would spawn for them
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Spatial3DView(origin="/"),
            rrb.TimePanel(state="hidden"),
            rrb.SelectionPanel(state="hidden"),
        )
    )
    raw_cloud = accumulate(lidar_stream, register=True)
    rr.log(
        "raw/cloud",
        rr.Points3D(raw_cloud, colors=z_gradient_colors(raw_cloud, RAW_CLOUD_GRADIENT), radii=0.02),
        static=True,
    )
    raw_segments, raw_traj_colors = gradient_trajectory(traj(odom_stream), RAW_TRAJECTORY_GRADIENT)
    rr.log(
        "raw/trajectory",
        rr.LineStrips3D(raw_segments, colors=raw_traj_colors),
        static=True,
    )
    corrected_odoms = sorted(
        stream_name
        for stream_name in streams
        if "_corrected" in stream_name and "odom" in stream_name
    )
    for lidar_index, lidar_name in enumerate(corrected_lidars):
        gradient = CLOUD_GRADIENTS[lidar_index % len(CLOUD_GRADIENTS)]
        cloud = accumulate(lidar_name)
        rr.log(
            f"{lidar_name}/cloud",
            rr.Points3D(cloud, colors=z_gradient_colors(cloud, gradient), radii=0.02),
            static=True,
        )
        print(f"  logged {lidar_name}: {len(cloud):,} pts")
    if corrected_odoms:
        segments, traj_colors = gradient_trajectory(
            traj(corrected_odoms[0]), TRAJECTORY_GRADIENTS[0]
        )
        rr.log(
            "corrected/trajectory",
            rr.LineStrips3D(segments, colors=traj_colors),
            static=True,
        )
    # landmarks placed against the first available corrected odometry
    if corrected_odoms and base_to_optical is None:
        print("no CameraInfo stream or optical tf edge — skipping tag landmarks")
    elif corrected_odoms:
        found = landmarks(corrected_odoms[0])
        images_logged = 0
        for marker in found:
            marker_id = marker["marker_id"]
            rr.log(
                f"landmarks/tag{marker_id}",
                rr.Mesh3D(
                    vertex_positions=TAG_CORNERS @ marker["rotation"].T + marker["position"],
                    triangle_indices=[[0, 1, 2], [0, 2, 3]],
                    vertex_texcoords=[[0, 0], [1, 0], [1, 1], [0, 1]],
                    albedo_texture=tag_image(marker_id),
                ),
                static=True,
            )
            images_logged += log_medoid_camera(
                f"landmarks/tag{marker_id}/medoid_view", marker["camera_pose"], marker["ts"]
            )
        if found:
            rr.log(
                "landmarks/labels",
                rr.Points3D(
                    np.array([marker["position"] for marker in found]),
                    colors=[255, 230, 0],
                    radii=0.005,
                    labels=[f"tag{marker['marker_id']}" for marker in found],
                ),
                static=True,
            )
            print(f"  logged {len(found)} landmarks, {images_logged} medoid views")
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
        camera_stream=cli_arg("--camera", "color_image"),
        camera_info_stream=cli_arg("--camera-info", ""),
    )

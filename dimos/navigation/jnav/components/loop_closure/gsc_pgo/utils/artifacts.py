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

"""Write PGO artifacts back into a recording db: corrected odom/lidar, deformation
nodes, pose graph, aggregated .pc2.lcm, and raycast-accumulated maps."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import time
from typing import TYPE_CHECKING

from gtsam import Pose3, Values
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts.make_rrd import (
    pose3_from_xyzquat,
)
from dimos.navigation.jnav.msgs.DeformationNode import DeformationNode, tf_id_for
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.utils.trajectory_metrics import nearest_index

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.type.observation import Observation
    from dimos.navigation.jnav.utils.recording_tf import RecordingTF

# aggregated .pc2.lcm
LCM_CHUNK_SCANS = 1000  # collapse buffered scans this often to bound memory
LCM_OUTLIER_NN = 20  # statistical outlier removal: neighbor count
LCM_OUTLIER_STD = 2.0  # ...and std-ratio threshold (lower = more aggressive)

# progress logging cadence
ODOM_LOG_EVERY = 20000
SCAN_LOG_EVERY = 2000


def pose_tuple(pose: Pose3) -> tuple[float, float, float, float, float, float, float]:
    translation = pose.translation()
    quaternion = pose.rotation().toQuaternion()
    return (
        translation[0],
        translation[1],
        translation[2],
        quaternion.x(),
        quaternion.y(),
        quaternion.z(),
        quaternion.w(),
    )


def interpolate_correction(
    keyframe_times: np.ndarray, corrections: list[Pose3], ts: float
) -> Pose3:
    """SE(3) interpolation of the keyframe corrections at an arbitrary timestamp."""
    if ts <= keyframe_times[0]:
        return corrections[0]
    if ts >= keyframe_times[-1]:
        return corrections[-1]
    after = int(np.searchsorted(keyframe_times, ts))
    before = after - 1
    alpha = (ts - keyframe_times[before]) / (keyframe_times[after] - keyframe_times[before])
    step = Pose3.Expmap(alpha * Pose3.Logmap(corrections[before].between(corrections[after])))
    return corrections[before].compose(step)


def write_deformation_nodes(
    store: Store,
    name: str,
    keyframe_times: np.ndarray,
    raw_poses: list[Pose3],
    estimate: Values,
    world_frame: str,
    body_frame: str,
) -> None:
    """Per keyframe: the raw pose then the optimized pose, so tf.get can replay the correction."""
    if name in store.list_streams():
        store.delete_stream(name)
    stream = store.stream(name, DeformationNode)
    edge_id = tf_id_for(world_frame, body_frame)
    for index in range(len(keyframe_times)):
        node_ts = float(keyframe_times[index])
        for pose in (raw_poses[index], estimate.atPose3(index)):
            px, py, pz, qx, qy, qz, qw = pose_tuple(pose)
            stream.append(
                DeformationNode(
                    id=index,
                    tf_id=edge_id,
                    pose=PoseStamped(
                        ts=node_ts,
                        frame_id=world_frame,
                        position=[px, py, pz],
                        orientation=[qx, qy, qz, qw],
                    ),
                ),
                ts=node_ts,
                pose=None,
                tags={"tf_id": str(edge_id), "id": str(index)},
            )
    print(f"wrote {name}: {len(keyframe_times)} keyframes (raw+optimized)", flush=True)


def write_pose_graph(
    store: Store, name: str, keyframe_times: np.ndarray, estimate: Values, world_frame: str
) -> None:
    """The optimized keyframe nodes + sequential odom edges as a Graph3D."""
    if name in store.list_streams():
        store.delete_stream(name)
    num_keyframes = len(keyframe_times)
    nodes: list[Graph3D.Node3D] = []
    for index in range(num_keyframes):
        px, py, pz, qx, qy, qz, qw = pose_tuple(estimate.atPose3(index))
        nodes.append(
            Graph3D.Node3D(
                pose=PoseStamped(
                    ts=float(keyframe_times[index]),
                    frame_id=world_frame,
                    position=[px, py, pz],
                    orientation=[qx, qy, qz, qw],
                ),
                id=index,
            )
        )
    edges = [
        Graph3D.Edge(index, index + 1, float(keyframe_times[index + 1]))
        for index in range(num_keyframes - 1)
    ]
    graph_ts = float(keyframe_times[-1])
    store.stream(name, Graph3D).append(
        Graph3D(ts=graph_ts, nodes=nodes, edges=edges), ts=graph_ts, pose=None
    )
    print(f"wrote {name}: {num_keyframes} nodes, {len(edges)} edges", flush=True)


def write_corrected_odom(
    store: Store,
    name: str,
    odom_rows: np.ndarray,
    keyframe_times: np.ndarray,
    corrections: list[Pose3],
    world_frame: str,
    corrected_odom_frame: str,
) -> None:
    """Corrected trajectory as ``world_frame -> corrected_odom_frame`` odometry, i.e. the tf
    edge the per-scan corrected clouds hang on."""
    if name in store.list_streams():
        store.delete_stream(name)
    stream = store.stream(name, Odometry)
    print(f"writing {name} ({len(odom_rows)} poses)...", flush=True)
    started = time.time()
    for count, row in enumerate(odom_rows, 1):
        ts = float(row[0])
        corrected = interpolate_correction(keyframe_times, corrections, ts).compose(
            pose3_from_xyzquat(row[1:])
        )
        x, y, z, qx, qy, qz, qw = pose_tuple(corrected)
        stream.append(
            Odometry(
                ts=ts,
                frame_id=world_frame,
                child_frame_id=corrected_odom_frame,
                pose=Pose(x, y, z, qx, qy, qz, qw),
            ),
            ts=ts,
            pose=(x, y, z, qx, qy, qz, qw),
        )
        if count % ODOM_LOG_EVERY == 0:
            print(f"  {count}/{len(odom_rows)} poses, {time.time() - started:.0f}s", flush=True)
    print(f"wrote {name}: {len(odom_rows)} poses in {time.time() - started:.0f}s", flush=True)


def voxel_downsample(
    points_chunks: list[np.ndarray], intensity_chunks: list[np.ndarray], voxel: float
) -> tuple[np.ndarray, np.ndarray | None]:
    """Merge chunks, voxel-downsample, carrying intensity through open3d's color channel."""
    import open3d as o3d

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.concatenate(points_chunks).astype(np.float64))
    carry = bool(intensity_chunks)
    if carry:
        column = np.concatenate(intensity_chunks).astype(np.float64)[:, None]
        cloud.colors = o3d.utility.Vector3dVector(np.repeat(column, 3, axis=1))
    cloud = cloud.voxel_down_sample(voxel)
    points = np.asarray(cloud.points, np.float32)
    intensities = np.asarray(cloud.colors, np.float32)[:, 0] if carry else None
    return points, intensities


def write_corrected_lidar(
    store: Store,
    name: str,
    lidar_stream: str,
    odom_rows: np.ndarray,
    keyframe_times: np.ndarray,
    corrections: list[Pose3],
    world_points: Callable[[Observation[PointCloud2]], np.ndarray],
    lcm_path: Path | None,
    lcm_voxel: float,
    world_frame: str,
    corrected_odom_frame: str,
) -> None:
    """Per-scan corrected clouds into the db, stored body-relative on ``corrected_odom_frame``
    so rerun/tf can place them via ``<odom>_corrected``; if lcm_path, also one aggregated,
    world-baked .pc2.lcm (a single fused cloud has no tf to hang on)."""
    if name in store.list_streams():
        store.delete_stream(name)
    stream = store.stream(name, PointCloud2)
    odom_times = odom_rows[:, 0]
    aggregated_points: list[np.ndarray] = []
    aggregated_intensities: list[np.ndarray] = []
    buffered_points: list[np.ndarray] = []
    buffered_intensities: list[np.ndarray] = []
    have_intensities = False

    print(f"writing {name} (corrected lidar)...", flush=True)
    started = time.time()
    scan_count = 0
    for observation in store.stream(lidar_stream, PointCloud2):
        scan_count += 1
        ts = float(observation.ts)
        correction = interpolate_correction(keyframe_times, corrections, ts)
        raw_pose = pose3_from_xyzquat(odom_rows[nearest_index(odom_times, ts)][1:])
        points = world_points(observation)
        intensities = observation.data.intensities_f32()
        # body-relative points: place them via the corrected-odom tf, not baked into world
        raw_rotation = raw_pose.rotation().matrix()
        raw_translation = np.asarray(raw_pose.translation())
        body_points = ((points - raw_translation) @ raw_rotation).astype(np.float32)
        cloud_msg = PointCloud2.from_numpy(
            body_points,
            frame_id=corrected_odom_frame,
            intensities=(np.asarray(intensities) if intensities is not None else None),
        )
        cloud_msg.ts = ts
        stream.append(
            cloud_msg,
            ts=ts,
            pose=pose_tuple(correction.compose(raw_pose)),
        )
        if lcm_path:
            rotation_matrix = correction.rotation().matrix()
            translation = np.asarray(correction.translation())
            corrected_points = (points @ rotation_matrix.T + translation).astype(np.float32)
            buffered_points.append(corrected_points)
            if intensities is not None:
                have_intensities = True
                buffered_intensities.append(np.asarray(intensities, np.float32))
            if len(buffered_points) >= LCM_CHUNK_SCANS:
                points_out, intensities_out = voxel_downsample(
                    buffered_points, buffered_intensities if have_intensities else [], lcm_voxel
                )
                aggregated_points.append(points_out)
                if intensities_out is not None:
                    aggregated_intensities.append(intensities_out)
                buffered_points, buffered_intensities = [], []
        if scan_count % SCAN_LOG_EVERY == 0:
            print(f"  {scan_count} scans, {time.time() - started:.0f}s", flush=True)
    print(f"wrote {name}: {scan_count} scans in {time.time() - started:.0f}s", flush=True)

    if lcm_path:
        if buffered_points:
            points_out, intensities_out = voxel_downsample(
                buffered_points, buffered_intensities if have_intensities else [], lcm_voxel
            )
            aggregated_points.append(points_out)
            if intensities_out is not None:
                aggregated_intensities.append(intensities_out)
        write_aggregated_lcm(
            aggregated_points,
            aggregated_intensities if have_intensities else [],
            lcm_voxel,
            lcm_path,
            float(odom_times[0]),
            world_frame,
        )


def write_aggregated_lcm(
    points_chunks: list[np.ndarray],
    intensity_chunks: list[np.ndarray],
    voxel: float,
    lcm_path: Path,
    stamp: float,
    world_frame: str,
) -> None:
    """Final unified voxel pass + statistical outlier removal into a single .pc2.lcm cloud."""
    import open3d as o3d

    points, intensities = voxel_downsample(points_chunks, intensity_chunks, voxel)
    print(
        f"aggregating .pc2.lcm: {len(points):,} pts after voxel, removing outliers...", flush=True
    )
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if intensities is not None:
        cloud.colors = o3d.utility.Vector3dVector(
            np.repeat(intensities.astype(np.float64)[:, None], 3, axis=1)
        )
    cloud, _keep = cloud.remove_statistical_outlier(LCM_OUTLIER_NN, LCM_OUTLIER_STD)
    merged_xyz = np.asarray(cloud.points, np.float32)
    merged_intensities = (
        np.asarray(cloud.colors, np.float32)[:, 0] if intensities is not None else None
    )
    merged = PointCloud2.from_numpy(
        merged_xyz, frame_id=world_frame, intensities=merged_intensities
    )
    merged.ts = stamp
    lcm_path.write_bytes(merged.lcm_encode())
    print(
        f"wrote {lcm_path}: 1 aggregated cloud, {len(merged_xyz):,} pts (voxel {voxel} m)",
        flush=True,
    )


def raycast_accumulate(
    store: Store,
    in_stream: str,
    store_tf: RecordingTF,
    world_frame: str,
    voxel: float,
    max_range: float,
) -> None:
    """Raycast ``in_stream`` into one ``<in_stream>_accumulated`` cloud, carving free space
    along every ray so dynamic objects and registration ghosts get cleared, not smeared."""
    out_stream = f"{in_stream}_accumulated"
    # imported here: needs the dimos_voxel_ray_tracing native ext, absent on plain CI runners
    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper

    mapper = VoxelRayMapper(voxel_size=voxel, max_range=max_range)
    scan_count = 0
    last_ts = 0.0
    start_time = time.time()
    for observation in store.stream(in_stream, PointCloud2):
        raw_points = np.asarray(observation.data.points_f32())
        if not len(raw_points):
            continue
        scan_frame = observation.data.frame_id
        transform = store_tf.get(world_frame, scan_frame, float(observation.ts), None)
        rotation = np.asarray(transform.rotation.to_rotation_matrix(), float).reshape(3, 3)
        origin = (
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        )
        points = (raw_points @ rotation.T + np.asarray(origin)).astype(np.float32)
        mapper.add_frame_world(points, origin)
        last_ts = float(observation.ts)
        scan_count += 1
        if scan_count % SCAN_LOG_EVERY == 0:
            print(
                f"  {scan_count} scans, {mapper.voxel_count():,} voxels, "
                f"{time.time() - start_time:.0f}s",
                flush=True,
            )
    accumulated = np.asarray(mapper.global_map(), np.float32)
    if out_stream in store.list_streams():
        store.delete_stream(out_stream)
    cloud = PointCloud2.from_numpy(accumulated, frame_id=world_frame, timestamp=last_ts)
    store.stream(out_stream, PointCloud2).append(cloud, ts=last_ts, pose=None)
    print(
        f"wrote {out_stream}: {len(accumulated):,} pts from {scan_count} scans "
        f"in {time.time() - start_time:.0f}s",
        flush=True,
    )

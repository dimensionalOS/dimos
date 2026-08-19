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

"""Fuse depth frames into a TSDF instead of hit/miss occupancy.

Occupancy mapping keeps every return, so the ~0.14 m scatter of stereo depth thickens
each real surface into a shell several voxels deep. A TSDF averages the signed distance
along each ray, so repeated views of the same wall collapse onto one zero-crossing.
"""

from __future__ import annotations

import json
import sys
import time

import numpy as np
import open3d as o3d

from dimos.memory.cli.dataset import open_store
from dimos.memory.tf import StreamTF
from dimos.msgs.sensor_msgs.Image import Image
from tools.depth_voxel import recording as recordings
from tools.depth_voxel.maps import VOXEL_SIZE_METERS, compare, shared_window
from tools.depth_voxel.pipeline import DEPTH_UNIT_METERS, DepthProjector, PoseTrack
from tools.depth_voxel.run_experiment import FRAME_STRIDE

SDF_TRUNCATION_METERS = 0.15
"""Three voxels, matching the measured depth scatter: narrower and the shell survives,
wider and thin structure gets averaged away."""

MAX_RANGE_METERS = 4.0
"""The same range gate the occupancy sweep settled on."""


def tsdf_keys(store, projector: DepthProjector, recording, start: float, end: float) -> np.ndarray:
    intrinsics = projector.intrinsics
    camera = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width,
        intrinsics.height,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.cx,
        intrinsics.cy,
    )
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=VOXEL_SIZE_METERS,
        sdf_trunc=SDF_TRUNCATION_METERS,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor,
    )
    blank_color = o3d.geometry.Image(
        np.zeros((intrinsics.height, intrinsics.width, 3), dtype=np.uint8)
    )
    stream = store.stream(recording.depth_stream, Image).after(start).before(end)
    for index, obs in enumerate(stream):
        if index % FRAME_STRIDE:
            continue
        odom_to_lidar = projector.poses.at(float(obs.ts))
        if odom_to_lidar is None:
            continue
        world_to_depth = odom_to_lidar.to_matrix() @ projector.lidar_to_depth
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            blank_color,
            o3d.geometry.Image(np.ascontiguousarray(obs.data.data, dtype=np.uint16)),
            depth_scale=1.0 / DEPTH_UNIT_METERS,
            depth_trunc=MAX_RANGE_METERS,
            convert_rgb_to_intensity=False,
        )
        volume.integrate(rgbd, camera, np.linalg.inv(world_to_depth))
    points = np.asarray(volume.extract_point_cloud().points)
    return np.floor(points / VOXEL_SIZE_METERS).astype(np.int64)


def main(recording_name: str) -> None:
    recording = recordings.get(recording_name)
    results_dir = recording.results_dir
    results_dir.mkdir(parents=True, exist_ok=True)
    store = open_store(recording.db_path)
    with store:
        tf = StreamTF.from_store(store, recording.tf_stream)
        assert tf is not None
        poses = PoseTrack.from_store(store, recording)
        projector = DepthProjector.from_store(store, tf, poses, recording)
        start, end = shared_window(store, recording)
        began = time.time()
        depth_keys = tsdf_keys(store, projector, recording, start, end)
    np.save(results_dir / "tsdf_depth_keys.npy", depth_keys)
    print(f"tsdf depth map: {len(depth_keys)} voxels in {time.time() - began:.0f} s")

    lidar_keys = np.load(results_dir / "raycast_lidar_keys.npy")
    stats = compare(depth_keys, lidar_keys)
    (results_dir / "tsdf.json").write_text(
        json.dumps({"name": "tsdf", "recording": recording.name, **stats.as_dict()}, indent=2)
    )
    print(stats.summary())


if __name__ == "__main__":
    main(sys.argv[1])

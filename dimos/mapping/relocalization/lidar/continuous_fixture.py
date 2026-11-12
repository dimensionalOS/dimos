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

"""Build the continuous relocalization fixture from an M20 walk.

    python -m dimos.mapping.relocalization.lidar.continuous_fixture controller_1793931551.mcap

The walk's first ``premap_s`` seconds, mapped, are the premap. The rest is mapped
afresh in a live ``world`` frame the premap sits ``MAP_IN_WORLD`` inside, as for a
robot started up somewhere else, and every diff the ray tracer publishes for it is kept,
which is what :class:`~dimos.mapping.relocalization.lidar.module.DiffRelocalization`
reads. So the right ``world -> map`` fix is ``MAP_IN_WORLD``. Writes ``data/m20_continuous_reloc/``:
``premap.pc2.lcm`` and ``map_update.db``; ``bin/lfs_push`` uploads it.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import typer

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.dds.ros import decode_odometry, decode_pointcloud2
from dimos.utils.data import get_data_dir

NAME = "m20_continuous_reloc"

# Where the premap sits in the fixture's live frame: the world -> map fix a relocalizer must find.
MAP_IN_WORLD = Transform(
    translation=Vector3(4.0, -2.5, 0.0),
    rotation=Quaternion.from_euler(Vector3(0.0, 0.0, np.radians(25.0))),
    frame_id="world",
    child_frame_id="map",
)


def _mapper() -> VoxelRayMapper:
    """The M20 nav stack's ray tracer (robot/deeprobotics/m20/blueprints/_nav.py)."""
    return VoxelRayMapper(voxel_size=0.08, max_range=25.0, pre_trace_subsample=5, ray_subsample=5)


def _scans(recording: Path) -> list[tuple[float, np.ndarray, Transform]]:
    """Each lidar scan in its body frame with the LIO pose of the same stamp."""
    store = McapStore(path=str(recording))
    store.start()
    poses: dict[float, Transform] = {}
    for obs in store.stream("_slam_odom", bytes):
        odom = decode_odometry(obs.data)
        poses[round(odom.ts, 3)] = Transform(
            translation=odom.pose.position, rotation=odom.pose.orientation
        )
    scans = []
    for obs in store.stream("_slam_body_points", bytes):
        cloud = decode_pointcloud2(obs.data)
        pose = poses.get(round(cloud.ts, 3))
        if pose is not None:
            scans.append((cloud.ts, cloud.points_f32(), pose))
    store.stop()
    return scans


def _add(mapper: VoxelRayMapper, points: np.ndarray, pose: Transform) -> None:
    p, q = pose.translation, pose.rotation
    mapper.add_frame(points, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def main(
    recording: Path,
    premap_s: float = typer.Option(170.0, help="Seconds of the walk the premap is made from"),
    diff_every: int = typer.Option(10, help="Scans per diff (the ray tracer's global_emit_every)"),
) -> None:
    scans = _scans(recording)
    start = scans[0][0]
    premap_scans = [scan for scan in scans if scan[0] - start < premap_s]
    live_scans = scans[len(premap_scans) :]

    premap = _mapper()
    for _, points, pose in premap_scans:
        _add(premap, points, pose)
    out = get_data_dir() / NAME
    out.mkdir(parents=True, exist_ok=True)
    premap_points = premap.global_map()
    (out / "premap.pc2.lcm").write_bytes(
        PointCloud2.from_numpy(premap_points, frame_id="map", timestamp=start).lcm_encode()
    )

    db = out / "map_update.db"
    db.unlink(missing_ok=True)
    live = _mapper()
    live.global_keyframe()
    diffs = 0
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("map_update", PointCloud2)
        for index, (ts, points, pose) in enumerate(live_scans, 1):
            _add(live, points, MAP_IN_WORLD + pose)
            if index % diff_every:
                continue
            rows = live.global_diff()
            if len(rows) == 0:
                continue
            cloud = PointCloud2.from_numpy(
                rows[:, :3],
                frame_id="world",
                timestamp=ts,
                premap=rows[:, 3].astype(np.uint8),
                change=rows[:, 4].astype(np.uint8),
            )
            stream.append(cloud, ts=ts)
            diffs += 1
    print(
        f"{out}: premap {len(premap_points)} voxels from {len(premap_scans)} scans; "
        f"{diffs} diffs from {len(live_scans)} scans"
    )


if __name__ == "__main__":
    typer.run(main)

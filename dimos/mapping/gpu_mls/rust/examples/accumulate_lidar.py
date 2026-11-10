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

"""Accumulate a recording's raw Livox cloud over a still stretch, for surface_truth.

Writes little-endian f32 xyz triples in the odometry frame. Each frame is
placed with the pose interpolated at its timestamp; keep the stretch short
and still, since frames are not deskewed and pose jitter smears the cloud.

    python accumulate_lidar.py mem2.db START_S END_S out.xyz.f32
"""

import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from dimos.memory.store.sqlite import SqliteStore

LIDAR_STREAM = "livox_lidar"
ODOMETRY_STREAM = "pointlio_odometry_corrected"
# The Mid-360 reports a shell of junk returns close in; real ones start past this.
MIN_RANGE_M = 1.0
# Raw z carries outliers tens of meters off; keep the scene around the robot.
KEEP_RADIUS_M = 8.0
KEEP_HEIGHT_M = 4.0


def main(db: str, start_s: float, end_s: float, out: str) -> None:
    store = SqliteStore(path=db)
    odometry = store.stream(ODOMETRY_STREAM).to_list()
    times = np.array([o.ts for o in odometry])
    positions = np.array(
        [[o.data.position.x, o.data.position.y, o.data.position.z] for o in odometry]
    )
    rotations = Rotation.from_quat(
        [
            [o.data.orientation.x, o.data.orientation.y, o.data.orientation.z, o.data.orientation.w]
            for o in odometry
        ]
    )
    slerp = Slerp(times, rotations)
    begin, end = times[0] + start_s, times[0] + end_s
    still = (times >= begin) & (times <= end)
    spread = np.abs(positions[still] - positions[still].mean(axis=0)).max(axis=0)
    chunks = []
    for observation in store.stream(LIDAR_STREAM).time_range(begin, end).to_list():
        points = np.asarray(observation.data.pointcloud.points, dtype=np.float64)
        points = points[np.linalg.norm(points, axis=1) > MIN_RANGE_M]
        position = np.array([np.interp(observation.ts, times, positions[:, k]) for k in range(3)])
        chunks.append(slerp([observation.ts]).apply(points) + position)
    cloud = np.concatenate(chunks)
    robot = positions[still].mean(axis=0)
    keep = np.linalg.norm(cloud[:, :2] - robot[:2], axis=1) < KEEP_RADIUS_M
    keep &= np.abs(cloud[:, 2] - robot[2]) < KEEP_HEIGHT_M
    cloud[keep].astype("<f4").tofile(out)
    print(f"{len(chunks)} frames, {keep.sum()} points; robot pose spread {np.round(spread, 3)} m")


if __name__ == "__main__":
    main(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4])

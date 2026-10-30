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

"""Export a recording's raw Livox frames in the odometry frame, for the bench.

Each frame is a little-endian u32 point count, the sensor origin as 3 f32,
then the points as f32 xyz triples, placed with the pose interpolated at the
frame's timestamp.

    python export_frames.py mem2.db out.frames [odometry_stream] [min_range_m]
"""

import struct
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from dimos.memory.store.sqlite import SqliteStore

LIDAR_STREAM = "livox_lidar"
ODOMETRY_STREAM = "fastlio_odometry"
# The Mid-360's junk shell sits within a few cm of the sensor. Real returns start
# about 0.3 m out, and a stair the robot climbs is only ever that close.
MIN_RANGE_M = 0.3


def main(
    db: str, out: str, odometry_stream: str = ODOMETRY_STREAM, min_range_m: float = MIN_RANGE_M
) -> None:
    store = SqliteStore(path=db)
    odometry = store.stream(odometry_stream).to_list()
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
    frames = 0
    points = 0
    with open(out, "wb") as f:
        for observation in store.stream(LIDAR_STREAM).time_range(times[0], times[-1]):
            cloud = np.asarray(observation.data.pointcloud.points, dtype=np.float64)
            cloud = cloud[np.linalg.norm(cloud, axis=1) > min_range_m]
            origin = np.array([np.interp(observation.ts, times, positions[:, k]) for k in range(3)])
            world = slerp([observation.ts]).apply(cloud) + origin
            f.write(struct.pack("<I3f", len(world), *origin))
            f.write(world.astype("<f4").tobytes())
            frames += 1
            points += len(world)
    print(f"{frames} frames, {points} points")


if __name__ == "__main__":
    main(*sys.argv[1:4], *[float(v) for v in sys.argv[4:5]])

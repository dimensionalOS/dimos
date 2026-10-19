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

"""Write generated CDR SQLite streams and load world-frame navigation data."""

import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory

from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.msgs.time import time_from_seconds
from dimos.navigation.nav_3d.evaluator.recording import iter_world_frames, load_trajectory


def main() -> None:
    with TemporaryDirectory(prefix="dimos-cdr-navigation-") as temporary:
        database = Path(temporary) / "navigation.db"
        with SqliteStore(path=str(database)) as store:
            lidar = store.stream("lidar", PointCloud2)
            odometry = store.stream("odom", Odometry)
            transforms = store.stream("tf", TFMessage)
            for tick in range(3):
                timestamp = float(tick + 1)
                stamp = time_from_seconds(timestamp)
                cloud = pointcloud_from_xyz(
                    np.array([[1, 0, 0]], dtype=np.float32),
                    header=Header(frame_id="lidar", stamp=stamp),
                )
                pose = Pose(
                    position=Point(x=float(tick)),
                    orientation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5)),
                )
                lidar.append(cloud, ts=timestamp)
                transforms.append(
                    TFMessage(
                        transforms=[
                            TransformStamped(
                                header=Header(frame_id="world", stamp=stamp),
                                child_frame_id="lidar",
                                transform=Transform(
                                    translation=Vector3(x=float(tick)), rotation=pose.orientation
                                ),
                            )
                        ]
                    ),
                    ts=timestamp,
                )
                odometry.append(
                    Odometry(
                        header=Header(frame_id="world", stamp=stamp),
                        pose=PoseWithCovariance(pose=pose),
                    ),
                    ts=timestamp,
                )
        frames = list(iter_world_frames(database, "lidar", "odom"))
        trajectory = load_trajectory(database, "odom")
        assert len(frames) == 3
        for tick, frame in enumerate(frames):
            np.testing.assert_allclose(frame.points, [[tick, 1, 0]], atol=1e-6)
            print(f"t={frame.ts:.1f}: sensor [1, 0, 0] → world {frame.points[0].tolist()}")
        np.testing.assert_allclose(trajectory.arc_lengths(), [0, 1, 2])
        print(f"Decoded odometry trajectory: {trajectory.positions.tolist()}; distance=2m")
        output = Path("build/message-codegen/demo/evidence/raytrace-cli.rrd").resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                sys.executable,
                "-m",
                "dimos.mapping.ray_tracing.utils.raytrace_rrd",
                str(database),
                "--out",
                str(output),
                "--lidar-stream",
                "lidar",
                "--world-frame",
                "world",
                "--fine-divisor",
                "0",
            ],
            check=True,
            timeout=30,
        )
        assert output.stat().st_size > 0
        print(f"Native ray-tracing CLI recording: {output}")
    print("Temporary SQLite recording removed")


if __name__ == "__main__":
    main()

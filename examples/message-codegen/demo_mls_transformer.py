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

"""Plan across synthetic terrain using the native MLS backend and CDR messages."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Path
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
import numpy as np

from dimos.memory.type.observation import Observation
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.navigation.nav_3d.mls_planner.start_relay import StartRelay
from dimos.navigation.nav_3d.mls_planner.transformer import MLSPlan
from dimos.protocol.tf.tf import MultiTBuffer


def main() -> None:
    coords = np.arange(-3, 3, 0.1, dtype=np.float32)
    x, y = np.meshgrid(coords, coords)
    points = np.stack([x.ravel(), y.ravel(), np.zeros(x.size, dtype=np.float32)], axis=1)
    cloud = pointcloud_from_xyz(
        points, header=Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789))
    )
    tf = MultiTBuffer()
    edge = TransformStamped(
        header=cloud.header,
        child_frame_id="base_link",
        transform=Transform(translation=Vector3(x=-2.0, y=-2.0, z=1.0), rotation=Quaternion(w=1.0)),
    )
    tf.receive_transform(TransformStamped.decode(edge.encode()))
    relay = StartRelay(world_frame="world")
    relay._tf = tf
    poses: list[PoseStamped] = []
    unsubscribe = relay.start_pose.subscribe(
        lambda value: poses.append(PoseStamped.decode(value.encode()))
    )
    try:
        relay._on_tf(TFMessage(transforms=[edge]))
        assert len(poses) == 1
        print(
            f"TF relay start: ({poses[0].pose.position.x}, {poses[0].pose.position.y}, {poses[0].pose.position.z})"
        )
    finally:
        unsubscribe()
        relay._tf = None
        relay.stop()
    obs = Observation(
        id=0,
        ts=0.0,
        pose=poses[0],
        tags={"region_bounds": (0.0, 0.0, 5.0, -1.0, 2.0)},
        _data=PointCloud2.decode(cloud.encode()),
    )
    [result] = list(MLSPlan(goal=(2.0, 2.0, 0.0), voxel_size=0.2, robot_height=1.0)(iter([obs])))
    path = Path.decode(result.data.encode())
    assert result.tags["planned"] and len(path.poses) >= 2
    assert path.header.stamp.nanosec == 123456789
    print(
        f"CDR terrain: {len(points)} points → {result.tags['voxels']} voxels → {len(path.poses)} path poses"
    )
    print(f"Header: {path.header.frame_id} {path.header.stamp.sec}.{path.header.stamp.nanosec:09d}")
    for index, pose in enumerate(path.poses):
        p = pose.pose.position
        print(f"  waypoint {index}: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")


if __name__ == "__main__":
    main()

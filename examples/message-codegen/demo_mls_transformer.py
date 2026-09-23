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

from pathlib import Path as FilePath
from types import SimpleNamespace

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_generated.geometry_msgs.msg import (
    Point,
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
import rerun as rr

from dimos.mapping.ray_tracing.transformer import RayTraceMap
from dimos.memory.type.observation import Observation
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.navigation.nav_3d.mls_planner.start_relay import StartRelay
from dimos.navigation.nav_3d.mls_planner.transformer import MLSPlan
from dimos.navigation.nav_3d.mls_planner.viz import (
    render_node_edges,
    render_nodes,
    render_surface_map,
)
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.message_helpers import register_colormap_annotation


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
    ray_input = obs.derive(data=cloud, pose_tuple=(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
    [mapped] = list(RayTraceMap(voxel_size=0.2)(iter([ray_input])))
    mapped_cloud = PointCloud2.decode(mapped.data.encode())
    assert mapped_cloud.header.stamp.nanosec == 123456789
    print(
        f"Ray-traced CDR local map: {mapped_cloud.width * mapped_cloud.height} points, frame={mapped_cloud.header.frame_id}"
    )
    [result] = list(MLSPlan(goal=(2.0, 2.0, 0.0), voxel_size=0.2, robot_height=1.0)(iter([obs])))
    path = Path.decode(result.data.encode())
    assert result.tags["planned"] and len(path.poses) >= 2
    assert path.header.stamp.nanosec == 123456789
    print(
        f"CDR terrain: {len(points)} points → {result.tags['voxels']} voxels → {len(path.poses)} path poses"
    )
    print(f"Header: {path.header.frame_id} {path.header.stamp.sec}.{path.header.stamp.nanosec:09d}")
    output = FilePath("build/message-codegen/demo/evidence/mls-planner.rrd")
    output.parent.mkdir(parents=True, exist_ok=True)
    rr.init("generated-mls-planner", spawn=False)
    rr.save(str(output))
    register_colormap_annotation()
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}
    try:
        bridge._on_message(
            TFMessage.decode(TFMessage(transforms=[edge]).encode()), SimpleNamespace(name="/tf")
        )
    finally:
        bridge.stop()
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}
    try:
        bridge._on_message(PointCloud2.decode(cloud.encode()), SimpleNamespace(name="/terrain"))
    finally:
        bridge.stop()
    rr.log("world/ray_map", render_surface_map(mapped_cloud))
    nodes = pointcloud_from_xyz(
        np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path.poses]),
        header=path.header,
    )
    rr.log("world/path_nodes", render_nodes(nodes))
    segments = LineSegments3D(
        header=path.header,
        segments=[
            LineSegment3D(
                start=Point(x=a.pose.position.x, y=a.pose.position.y, z=a.pose.position.z),
                end=Point(x=b.pose.position.x, y=b.pose.position.y, z=b.pose.position.z),
            )
            for a, b in zip(path.poses, list(path.poses)[1:], strict=False)
        ],
    )
    rr.log("world/path_edges", render_node_edges(LineSegments3D.decode(segments.encode())))
    rr.disconnect()
    assert output.stat().st_size > 0
    print(f"Rerun recording: {output}")
    for index, pose in enumerate(path.poses):
        p = pose.pose.position
        print(f"  waypoint {index}: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")


if __name__ == "__main__":
    main()

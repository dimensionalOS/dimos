# Copyright 2025-2026 Dimensional Inc.
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

"""Project synthetic generated depth into a world-frame detection pose."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.msgs.image import image_from_array
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC


def main() -> None:
    header = Header(frame_id="camera", stamp=Time(sec=1700000000, nanosec=123456789))
    depth = image_from_array(
        np.full((4, 4), 2000, dtype=np.uint16), encoding="16UC1", header=header
    )
    depth = Image.decode(depth.encode())
    detection = Detection2DBBox(
        bbox=(0, 0, 3, 3),
        track_id=1,
        class_id=0,
        confidence=1,
        name="target",
        ts=1700000000.0,
        image=depth,
    )
    camera = CameraInfo(width=4, height=4, k=[2, 0, 0, 0, 2, 0, 0, 0, 1])
    transform = TransformStamped(
        header=Header(frame_id="camera"),
        child_frame_id="world",
        transform=Transform(translation=Vector3(x=-10), rotation=Quaternion(w=1)),
    )
    result = Detection3DPC.from_depth(detection, depth, camera, transform, filters=[])
    assert result is not None
    pose = PoseStamped.decode(result.pose.encode())
    assert pose.header.stamp == header.stamp
    assert pose.header.frame_id == "world"
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (11.5, 1.5, 2)
    print("16 depth pixels at 2000 mm → 16 world-frame points")
    print("CDR centroid: (11.5, 1.5, 2.0) m; stamp=1700000000123456789 ns")
    print("PASS: generated depth → detection cloud → generated CDR pose")


if __name__ == "__main__":
    main()

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

"""Show generated CDR cloud filtering without ROS or a sensor."""

from typing import cast

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped
from dimos_generated.sensor_msgs.msg import CameraInfo, PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection3d.pointcloud_filters import (
    height_filter,
    radius_outlier,
)


def main() -> None:
    points = np.random.default_rng(42).normal(scale=0.05, size=(100, 3))
    points[:, 2] += 2
    points = np.vstack((points, [10, 10, -1]))
    cloud = pointcloud_from_xyz(points, header=Header(frame_id="world"))
    cloud = PointCloud2.decode(cloud.encode())
    transform = TransformStamped(transform=Transform(rotation=Quaternion(w=1)))
    # These geometric filters do not inspect the detection argument.
    detection = cast("Detection2DBBox", None)
    for name, operation in (("height", height_filter()), ("radius", radius_outlier())):
        result = operation(detection, cloud, CameraInfo(), transform)
        assert result is not None
        result = PointCloud2.decode(result.encode())
        assert result.width == 100 and result.header == cloud.header
        print(f"{name}: 101 input points → {result.width} retained points; source header preserved")
    print("PASS: generated clouds survive filtering and CDR round trips")


if __name__ == "__main__":
    main()

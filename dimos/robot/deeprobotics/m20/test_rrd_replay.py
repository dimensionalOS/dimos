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

import numpy as np
import pyarrow as pa

from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.robot.deeprobotics.m20.rrd_replay import (
    _image_from_batch,
    _odometry_from_batch,
    _pointcloud_from_batch,
)


def test_pointcloud_from_rerun_batch_preserves_points_frame_and_time() -> None:
    batch = pa.record_batch(
        [
            pa.array([1_500_000_000], type=pa.timestamp("ns")),
            pa.array(
                [[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]],
                type=pa.list_(pa.list_(pa.float32(), 3)),
            ),
        ],
        names=["log_time", "Points3D:positions"],
    )

    pointcloud = _pointcloud_from_batch(batch, 0)

    assert pointcloud.ts == 1.5
    assert pointcloud.frame_id == "map"
    np.testing.assert_array_equal(
        np.asarray(pointcloud.pointcloud.points),
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
    )


def test_odometry_from_rerun_batch_preserves_map_pose() -> None:
    batch = pa.record_batch(
        [
            pa.array([2_250_000_000], type=pa.timestamp("ns")),
            pa.array(
                [[[1.0, 2.0, 0.5]]],
                type=pa.list_(pa.list_(pa.float32(), 3)),
            ),
            pa.array(
                [[[0.0, 0.0, 0.5, 0.8660254]]],
                type=pa.list_(pa.list_(pa.float32(), 4)),
            ),
        ],
        names=[
            "log_time",
            "Transform3D:translation",
            "Transform3D:quaternion",
        ],
    )

    odometry = _odometry_from_batch(batch, 0)

    assert odometry.ts == 2.25
    assert odometry.frame_id == "map"
    assert odometry.child_frame_id == "base_link"
    np.testing.assert_allclose(odometry.position.to_numpy(), [1.0, 2.0, 0.5])
    np.testing.assert_allclose(odometry.orientation.to_numpy(), [0.0, 0.0, 0.5, 0.8660254])


def test_image_from_rerun_batch_preserves_bgr_pixels_frame_and_time() -> None:
    image_format_type = pa.list_(
        pa.struct(
            [
                ("width", pa.uint32()),
                ("height", pa.uint32()),
                ("pixel_format", pa.uint8()),
                ("color_model", pa.uint8()),
                ("channel_datatype", pa.uint8()),
            ]
        )
    )
    batch = pa.record_batch(
        [
            pa.array([3_500_000_000], type=pa.timestamp("ns")),
            pa.array(
                [[[1, 2, 3, 4, 5, 6]]],
                type=pa.list_(pa.list_(pa.uint8())),
            ),
            pa.array(
                [
                    [
                        {
                            "width": 2,
                            "height": 1,
                            "pixel_format": None,
                            "color_model": 4,
                            "channel_datatype": 6,
                        }
                    ]
                ],
                type=image_format_type,
            ),
        ],
        names=["log_time", "Image:buffer", "Image:format"],
    )

    image = _image_from_batch(batch, 0, "camera_optical")

    assert image.ts == 3.5
    assert image.frame_id == "camera_optical"
    assert image.format == ImageFormat.BGR
    np.testing.assert_array_equal(image.data, np.array([[[1, 2, 3], [4, 5, 6]]]))

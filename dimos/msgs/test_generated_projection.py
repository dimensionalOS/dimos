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

"""Stored image projection uses generated transforms and padded depth pixels."""

from dimos_generated.geometry_msgs.msg import (
    Point,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.memory.type.observation import Observation
from dimos.msgs.geometry import transform_matrix
from dimos.perception.detection.project import _world_to_optical, sees


def test_stored_pose_projection_inverts_world_camera_chain() -> None:
    obs = Observation(
        id=0, ts=0.0, data_type=Image, _data=Image(), pose_tuple=(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    )
    extrinsic = TransformStamped(
        header=Header(frame_id="base_link"),
        child_frame_id="camera_optical",
        transform=Transform(translation=Vector3(x=1), rotation=Quaternion(w=1)),
    )
    result = _world_to_optical(obs, "world", None, extrinsic, "camera_optical", 5.0)
    assert result is not None
    assert result.header.frame_id == "camera_optical"
    assert result.child_frame_id == "world"
    np.testing.assert_allclose(transform_matrix(result.transform) @ [3, 0, 2, 1], [0, 0, 2, 1])


def test_depth_occlusion_reads_big_endian_rows_and_millimeters() -> None:
    obs = Observation(
        id=0, ts=0.0, data_type=Image, _data=Image(), pose_tuple=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    )
    extrinsic = TransformStamped(
        header=Header(frame_id="base_link"),
        child_frame_id="camera_optical",
        transform=Transform(rotation=Quaternion(w=1)),
    )
    calibration = CameraInfo(width=2, height=2, k=[1, 0, 0, 0, 1, 0, 0, 0, 1])
    # Pixel (0, 1) = 1500mm; padding cannot be interpreted as pixels.
    depth = Image(
        width=2,
        height=2,
        step=6,
        encoding="16UC1",
        is_bigendian=1,
        data=[0, 0, 0, 0, 99, 99, 5, 220, 0, 0, 99, 99],
    )
    visible = sees(
        Point(x=0, y=1, z=1), calibration, base_to_optical=extrinsic, depth=lambda _: depth
    )
    hidden = sees(
        Point(x=0, y=2, z=2), calibration, base_to_optical=extrinsic, depth=lambda _: depth
    )
    assert visible(obs)
    assert not hidden(obs)


def test_timestamp_adapter_matches_generated_cloud_without_mutating_header() -> None:
    from dimos_generated.sensor_msgs.msg import PointCloud2
    from reactivex.subject import Subject

    from dimos.msgs.time import time_from_nanoseconds, to_seconds
    from dimos.types.timestamped import TimestampedData, align_timestamped

    source = PointCloud2(header=Header(stamp=time_from_nanoseconds(1700000000123456789)))
    primary = Subject()
    secondary = Subject()
    received = []
    disposable = align_timestamped(primary, secondary, match_tolerance=0.0).subscribe(
        received.append
    )
    timed = TimestampedData(source, to_seconds(source.header.stamp))
    secondary.on_next(timed)
    primary.on_next(TimestampedData("detection", timed.ts))
    assert len(received) == 1
    assert received[0][1].value is source
    assert source.header.stamp.nanosec == 123456789
    disposable.dispose()

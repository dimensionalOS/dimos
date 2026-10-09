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

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.time import time_from_seconds


def test_tfmessage_initialization() -> None:
    first = TransformStamped(
        header=Header(stamp=time_from_seconds(100), frame_id="world"),
        transform=Transform(
            translation=Vector3(x=1, y=2, z=3), rotation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
        ),
        child_frame_id="",
    )
    second = TransformStamped(
        header=Header(stamp=time_from_seconds(101), frame_id="map"),
        transform=Transform(
            translation=Vector3(x=4, y=5, z=6), rotation=Quaternion(z=0.707, w=0.707, x=0.0, y=0.0)
        ),
        child_frame_id="",
    )
    message = TFMessage(transforms=[first, second])
    assert len(message.transforms) == 2
    assert [cdr_encode(item) for item in message.transforms] == [
        cdr_encode(first),
        cdr_encode(second),
    ]


def test_tfmessage_empty() -> None:
    message = TFMessage(transforms=[])
    assert len(message.transforms) == 0
    assert list(message.transforms) == []


def test_tfmessage_add_transform() -> None:
    message = TFMessage(transforms=[])
    transform = TransformStamped(
        header=Header(stamp=time_from_seconds(200), frame_id="base"),
        transform=Transform(
            translation=Vector3(x=1, y=2, z=3), rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
        child_frame_id="",
    )
    message.transforms.append(transform)
    assert len(message.transforms) == 1
    assert cdr_encode(message.transforms[0]) == cdr_encode(transform)


def test_tfmessage_tree() -> None:
    message = TFMessage(transforms=[])
    edges = [
        ("world", "robot", (1, 2, 0.5)),
        ("robot", "camera", (0.1, 0, 1)),
        ("robot", "lidar", (0.2, 0, 0.2)),
        ("lidar", "lidar_scanner", (0.05, 0, 0)),
    ]
    for parent, child, (x, y, z) in edges:
        message.transforms.append(
            TransformStamped(
                header=Header(stamp=time_from_seconds(200), frame_id=parent),
                child_frame_id=child,
                transform=Transform(
                    translation=Vector3(x=x, y=y, z=z),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )
        )
    assert len(message.transforms) == 4
    assert [
        (
            t.header.frame_id,
            t.child_frame_id,
            (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
        )
        for t in cdr_decode(cdr_encode(message), TFMessage).transforms
    ] == edges


def test_tfmessage_cdr_encode_decode() -> None:
    first = TransformStamped(
        header=Header(stamp=time_from_seconds(123.456), frame_id="world"),
        child_frame_id="robot",
        transform=Transform(
            translation=Vector3(x=1, y=2, z=3), rotation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
        ),
    )
    second = TransformStamped(
        header=Header(stamp=time_from_seconds(124.567), frame_id="robot"),
        child_frame_id="target",
        transform=Transform(
            translation=Vector3(x=4, y=5, z=6), rotation=Quaternion(z=0.707, w=0.707, x=0.0, y=0.0)
        ),
    )
    message = TFMessage(transforms=[first, second])
    decoded = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(
        cdr_encode(message), TFMessage.__msgtype__
    )
    assert len(decoded.transforms) == 2
    first_result, second_result = decoded.transforms
    assert first_result.header.frame_id == "world"
    assert first_result.child_frame_id == "robot"
    assert (first_result.header.stamp.sec, first_result.header.stamp.nanosec) == (123, 456000000)
    assert (
        first_result.transform.translation.x,
        first_result.transform.translation.y,
        first_result.transform.translation.z,
    ) == (1, 2, 3)
    assert second_result.header.frame_id == "robot"
    assert second_result.child_frame_id == "target"
    assert (second_result.transform.rotation.z, second_result.transform.rotation.w) == (
        0.707,
        0.707,
    )

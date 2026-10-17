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

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest

from dimos.memory.codecs.jpeg import JpegCodec
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.mcap import McapStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

mcap_writer = pytest.importorskip("mcap.writer", reason="mcap not installed")


def test_lcm_channel_decodes_with_explicit_codec(tmp_path: Path) -> None:
    path = tmp_path / "recording.mcap"
    expected = Imu(
        ts=12.5,
        frame_id="imu_link",
        angular_velocity=Vector3(1.0, 2.0, 3.0),
    )
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        channel_id = writer.register_channel(
            topic="imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={
                "dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu",
                "dimos.observation_time": "publish_time",
            },
        )
        writer.add_message(
            channel_id=channel_id,
            log_time=13_000_000_000,
            publish_time=12_500_000_000,
            data=expected.lcm_encode(),
        )
        writer.add_message(
            channel_id=channel_id,
            log_time=14_000_000_000,
            publish_time=11_500_000_000,
            data=Imu(ts=11.5, frame_id="earlier").lcm_encode(),
        )
        writer.finish()

    with McapStore(path=str(path), codecs={"imu": LcmCodec(Imu)}) as store:
        assert store.list_streams() == ["imu"]
        observation: Observation[Imu] = store.stream("imu").order_by("ts").first()
        assert observation.ts == 11.5
        assert observation.data.frame_id == "earlier"

        latest_observation: Observation[Imu] = store.stream("imu").order_by("ts", desc=True).first()
        assert latest_observation.ts == 12.5
        assert latest_observation.data.lcm_encode() == expected.lcm_encode()


def test_wrapped_codec_decodes_with_explicit_codec(tmp_path: Path) -> None:
    path = tmp_path / "recording.mcap"
    expected = Imu(
        ts=12.5,
        frame_id="imu_link",
        angular_velocity=Vector3(1.0, 2.0, 3.0),
    )
    codec = Lz4Codec(LcmCodec(Imu))
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        channel_id = writer.register_channel(
            topic="imu",
            message_encoding="lz4+lcm",
            schema_id=0,
            metadata={
                "dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu",
                "dimos.observation_time": "publish_time",
            },
        )
        writer.add_message(
            channel_id=channel_id,
            log_time=13_000_000_000,
            publish_time=12_500_000_000,
            data=codec.encode(expected),
        )
        writer.finish()

    with McapStore(path=str(path), codecs={"imu": codec}) as store:
        observation: Observation[Imu] = store.stream("imu").first()
        assert observation.ts == 12.5
        assert observation.data.lcm_encode() == expected.lcm_encode()


# ros2msg definitions, each written once; a schema is its type plus the types it nests
_MSGDEFS = {
    "builtin_interfaces/Time": "int32 sec\nuint32 nanosec",
    "std_msgs/Header": "builtin_interfaces/Time stamp\nstring frame_id",
    "geometry_msgs/Vector3": "float64 x\nfloat64 y\nfloat64 z",
    "geometry_msgs/Point": "float64 x\nfloat64 y\nfloat64 z",
    "geometry_msgs/Quaternion": "float64 x\nfloat64 y\nfloat64 z\nfloat64 w",
    "geometry_msgs/Pose": "geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation",
    "geometry_msgs/PoseWithCovariance": "geometry_msgs/Pose pose\nfloat64[36] covariance",
    "geometry_msgs/Twist": "geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular",
    "geometry_msgs/TwistWithCovariance": "geometry_msgs/Twist twist\nfloat64[36] covariance",
    "geometry_msgs/Transform": "geometry_msgs/Vector3 translation\ngeometry_msgs/Quaternion rotation",
    "geometry_msgs/TransformStamped": "std_msgs/Header header\nstring child_frame_id\n"
    "geometry_msgs/Transform transform",
    "sensor_msgs/PointField": "string name\nuint32 offset\nuint8 datatype\nuint32 count",
    "sensor_msgs/PointCloud2": "std_msgs/Header header\nuint32 height\nuint32 width\n"
    "sensor_msgs/PointField[] fields\nbool is_bigendian\nuint32 point_step\nuint32 row_step\n"
    "uint8[] data\nbool is_dense",
    "sensor_msgs/CompressedImage": "std_msgs/Header header\nstring format\nuint8[] data",
    "tf2_msgs/TFMessage": "geometry_msgs/TransformStamped[] transforms",
    "std_msgs/UInt32": "uint32 data",
    "nav_msgs/Odometry": "std_msgs/Header header\nstring child_frame_id\n"
    "geometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist",
}


def _ros2_schema(name: str) -> str:
    """The ros2msg text of *name*: its definition, then every type it nests."""
    seen: list[str] = []

    def walk(t: str) -> None:
        for line in _MSGDEFS[t].splitlines():
            nested = line.split()[0].split("[")[0]
            if nested in _MSGDEFS and nested not in seen:
                seen.append(nested)
                walk(nested)

    walk(name)
    return _MSGDEFS[name] + "".join(f"\n{'=' * 80}\nMSG: {t}\n{_MSGDEFS[t]}" for t in seen)


def ros2_header(ts: float, frame: str) -> dict[str, Any]:
    return {"stamp": {"sec": int(ts), "nanosec": round(ts % 1 * 1e9)}, "frame_id": frame}


def ros2_cloud(ts: float, frame: str, *xyz: tuple[float, float, float]) -> dict[str, Any]:
    fields = [{"name": n, "offset": 4 * i, "datatype": 7, "count": 1} for i, n in enumerate("xyz")]
    return {
        "header": ros2_header(ts, frame),
        "height": 1,
        "width": len(xyz),
        "fields": fields,
        "is_bigendian": False,
        "point_step": 12,
        "row_step": 12 * len(xyz),
        "data": np.array(xyz, "<f4").tobytes(),
        "is_dense": True,
    }


def ros2_odom(ts: float, x: float, y: float) -> dict[str, Any]:
    zero = {"x": 0.0, "y": 0.0, "z": 0.0}
    pose = {"position": {"x": x, "y": y, "z": 0.3}, "orientation": {**zero, "w": 1.0}}
    return {
        "header": ros2_header(ts, "odom"),
        "child_frame_id": "base_link",
        "pose": {"pose": pose, "covariance": [0.0] * 36},
        "twist": {"twist": {"linear": zero, "angular": zero}, "covariance": [0.0] * 36},
    }


def ros2_tf(
    ts: float, parent: str, child: str, xyz: tuple[float, float, float], yaw: float
) -> dict[str, Any]:
    rotation = {"x": 0.0, "y": 0.0, "z": float(np.sin(yaw / 2)), "w": float(np.cos(yaw / 2))}
    translation = dict(zip("xyz", xyz, strict=True))
    return {
        "transforms": [
            {
                "header": ros2_header(ts, parent),
                "child_frame_id": child,
                "transform": {"translation": translation, "rotation": rotation},
            }
        ]
    }


def write_ros2_mcap(path: Path, messages: list[tuple[str, str, float, dict[str, Any]]]) -> None:
    """A ROS 2 recording: (topic, ros type, log time, message) rows, CDR-encoded."""
    ros2_writer = pytest.importorskip("mcap_ros2.writer", reason="mcap-ros2-support not installed")
    with path.open("wb") as output:
        writer = ros2_writer.Writer(output)
        schemas: dict[str, Any] = {}
        for topic, name, ts, msg in messages:
            if name not in schemas:
                package, kind = name.split("/")
                schemas[name] = writer.register_msgdef(f"{package}/msg/{kind}", _ros2_schema(name))
            writer.write_message(
                topic, schemas[name], msg, log_time=int(ts * 1e9), publish_time=int(ts * 1e9)
            )
        writer.finish()


def test_ros2_channels_decode_from_their_embedded_schema(tmp_path: Path) -> None:
    path = tmp_path / "ros2.mcap"
    jpeg = cv2.imencode(".jpg", np.full((8, 8, 3), 90, np.uint8))[1].tobytes()
    frame = {"header": ros2_header(3.0, "camera"), "format": "jpeg", "data": jpeg}
    write_ros2_mcap(
        path,
        [
            (
                "/tf",
                "tf2_msgs/TFMessage",
                1.0,
                ros2_tf(1.0, "odom", "lidar_link", (5, 0, 0.3), np.pi / 2),
            ),
            (
                "/livox/lidar",
                "sensor_msgs/PointCloud2",
                2.0,
                ros2_cloud(2.0, "lidar_link", (1, 2, 3), (4, 5, 6)),
            ),
            ("/camera/compressed", "sensor_msgs/CompressedImage", 3.0, frame),
        ],
    )

    with McapStore(path=str(path)) as store:
        assert store.list_streams() == ["camera_compressed", "livox_lidar", "tf"]
        cloud = store.streams.livox_lidar.first()
        assert cloud.ts == 2.0 and isinstance(cloud.data, PointCloud2)
        assert cloud.data.frame_id == "lidar_link"
        assert np.allclose(cloud.data.points_f32(), [[1, 2, 3], [4, 5, 6]])
        tf = store.streams.tf.first().data
        assert isinstance(tf, TFMessage)
        (t,) = tf.transforms
        assert (t.frame_id, t.child_frame_id) == ("odom", "lidar_link") and t.ts == 1.0
        assert np.allclose(
            [t.translation.x, t.translation.z, t.rotation.z], [5, 0.3, np.sin(np.pi / 4)]
        )
        image = store.streams.camera_compressed.first().data  # decoded, like a jpeg channel
        assert isinstance(image, Image) and image.data.shape == (8, 8, 3)


def test_ros2_unsigned_ints_keep_their_range(tmp_path: Path) -> None:
    path = tmp_path / "ros2.mcap"
    values = [0, 2**31 - 1, 2**31, 3_000_000_000, 2**32 - 1]
    write_ros2_mcap(
        path, [("/count", "std_msgs/UInt32", 1.0 + i, {"data": v}) for i, v in enumerate(values)]
    )
    with McapStore(path=str(path)) as store:
        assert [obs.data.data for obs in store.streams.count] == values


def test_ros2_topics_folding_to_one_name_stay_apart(tmp_path: Path) -> None:
    path = tmp_path / "ros2.mcap"
    write_ros2_mcap(
        path,
        [
            ("/cam/ir_left/count", "std_msgs/UInt32", 1.0, {"data": 1}),
            ("/cam/ir_left_count", "std_msgs/UInt32", 2.0, {"data": 2}),
            ("/cam/other", "std_msgs/UInt32", 3.0, {"data": 3}),
        ],
    )
    with McapStore(path=str(path)) as store:
        assert store.list_streams() == ["cam__ir_left__count", "cam__ir_left_count", "cam_other"]
        assert store.streams.cam__ir_left__count.first().data.data == 1
        assert store.streams.cam__ir_left_count.first().data.data == 2


def test_self_describing_jpeg_channel_decodes_without_a_codec_registry(tmp_path: Path) -> None:
    path = tmp_path / "recording.mcap"
    expected = Image(
        data=np.full((8, 8, 3), [20, 80, 140], dtype=np.uint8),
        format=ImageFormat.RGB,
        frame_id="camera",
        ts=12.5,
    )
    codec = JpegCodec()
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        channel_id = writer.register_channel(
            topic="color_image",
            message_encoding="jpeg",
            schema_id=0,
            metadata={
                "dimos.payload_type": "dimos.msgs.sensor_msgs.Image.Image",
                "dimos.observation_time": "publish_time",
            },
        )
        writer.add_message(
            channel_id=channel_id,
            log_time=13_000_000_000,
            publish_time=12_500_000_000,
            data=codec.encode(expected),
        )
        writer.finish()

    with McapStore(path=str(path)) as store:
        observation: Observation[Image] = store.stream("color_image").first()
        decoded = observation.data
        assert observation.ts == 12.5
        assert decoded.frame_id == "camera"
        assert decoded.format is ImageFormat.RGB
        assert decoded.data.shape == expected.data.shape
        assert np.mean(np.abs(decoded.data.astype(float) - expected.data.astype(float))) < 5


def test_lcm_metadata_does_not_import_payload_module(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "untrusted.mcap"
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        channel_id = writer.register_channel(
            topic="untrusted",
            message_encoding="lcm",
            schema_id=0,
            metadata={"dimos.payload_type": "untrusted_module.Payload"},
        )
        writer.add_message(
            channel_id=channel_id,
            log_time=1,
            publish_time=1,
            data=b"raw payload",
        )
        writer.finish()

    def fail_import(name: str) -> None:
        raise AssertionError(f"artifact metadata imported {name!r}")

    monkeypatch.setattr("dimos.memory.codecs.base.importlib.import_module", fail_import)
    with McapStore(path=str(path)) as store:
        assert store.stream("untrusted").first().data == b"raw payload"

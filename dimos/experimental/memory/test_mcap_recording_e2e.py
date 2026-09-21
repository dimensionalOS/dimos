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

"""Native encoding checked against the independent MCAP ROS 2 decoder."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
from typing import Any

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import numpy as np
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.experimental.memory.rust_recorder import mcap_codec
from dimos.memory.cli.dataset import open_store
from dimos.memory.cli.render import render_store
from dimos.memory.cli.summary import main as summarize
from dimos.memory.store.mcap_recording import McapRecordingStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as MessagePath
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

pytestmark = pytest.mark.self_hosted


@pytest.fixture(scope="module")
def recording(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, dict[str, Any]]:
    directory = tmp_path_factory.mktemp("native-mcap")
    pose = PoseStamped(
        ts=12.5,
        frame_id="map",
        position=Vector3(1, 2, 3),
        orientation=Quaternion(0.1, 0.2, 0.3, 0.9),
    )
    camera = CameraInfo(
        ts=12.5,
        frame_id="camera",
        height=8,
        width=8,
        distortion_model="plumb_bob",
        D=[0.1, 0.2],
        K=list(range(9)),
        P=list(range(12)),
        binning_x=2,
        binning_y=3,
    )
    camera.roi_x_offset, camera.roi_y_offset = 2, 3
    camera.roi_height, camera.roi_width, camera.roi_do_rectify = 4, 5, True
    messages: dict[str, Any] = {
        "imu": Imu(
            ts=12.5,
            frame_id="imu",
            orientation=Quaternion(0.1, 0.2, 0.3, 0.9),
            angular_velocity=Vector3(1, 2, 3),
            linear_acceleration=Vector3(4, 5, 6),
            orientation_covariance=list(range(9)),
            angular_velocity_covariance=list(range(10, 19)),
            linear_acceleration_covariance=list(range(20, 29)),
        ),
        "pose": pose,
        "odom": Odometry(
            ts=12.5,
            frame_id="map",
            child_frame_id="base",
            pose=PoseWithCovariance(Pose(position=Vector3(1, 2, 3)), list(range(36))),
            twist=TwistWithCovariance(
                Twist(Vector3(4, 5, 6), Vector3(7, 8, 9)), list(range(36, 72))
            ),
        ),
        "path": MessagePath(ts=12.5, frame_id="map", poses=[pose]),
        "camera_info": camera,
        "joints": JointState(
            ts=12.5,
            frame_id="base",
            name=["hip", "knee"],
            position=[0.1, 0.2],
            velocity=[3, 4],
            effort=[],
        ),
        "tf": TFMessage(
            Transform(ts=12.5, frame_id="map", child_frame_id="base", translation=Vector3(1, 2, 3)),
            Transform(
                ts=13.5, frame_id="base", child_frame_id="camera", translation=Vector3(4, 5, 6)
            ),
        ),
        "image": Image(
            np.full((8, 8, 3), [20, 80, 140], dtype=np.uint8), ImageFormat.RGB, "camera", 12.5
        ),
        "depth": Image(
            np.array([[1, 1000], [65535, 42]], dtype=np.uint16), ImageFormat.DEPTH16, "depth", 12.5
        ),
        "cloud": PointCloud2.from_numpy(
            np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32),
            "lidar",
            12.5,
            intensities=np.array([11, 22], dtype=np.float32),
        ),
    }
    messages["jpeg"] = messages["image"]
    messages["jpeg_mono"] = Image(
        np.full((8, 8), 128, dtype=np.uint8), ImageFormat.GRAY, "camera", 12.5
    )
    messages["float_depth"] = Image(
        np.array([[1.5, 100.25]], dtype=np.float32), ImageFormat.DEPTH, "depth", 12.5
    )
    segments_path = MessagePath(
        ts=12.5,
        frame_id="map",
        poses=[
            PoseStamped(
                ts=12.5,
                frame_id="map",
                position=Vector3(1, 2, 3),
                orientation=Quaternion(0, 0, 0, 0.75),
            ),
            PoseStamped(ts=12.5, frame_id="map", position=Vector3(4, 5, 6)),
        ],
    )
    messages["segments"] = LineSegments3D(
        ts=12.5, frame_id="map", segments=[[[1, 2, 3], [4, 5, 6]]], weights=[0.75]
    )
    streams = []
    for name, value in messages.items():
        streams.append(
            {
                "name": name,
                "port": name,
                "payload_type": f"{type(value).__module__}.{type(value).__qualname__}",
                "codec": mcap_codec(type(value), "jpeg" if name.startswith("jpeg") else None),
            }
        )
        data = segments_path.lcm_encode() if name == "segments" else value.lcm_encode()
        (directory / f"{name}.lcm").write_bytes(data)
    artifact = directory / "recording.mcap"
    (directory / "config.json").write_text(
        json.dumps(
            {
                "store": {"kind": "mcap", "path": str(artifact)},
                "encoding_threads": 2,
                "streams": streams,
            }
        )
    )
    subprocess.run(
        [
            "cargo",
            "test",
            "-p",
            "dimos-memory-recorder",
            "--lib",
            "write_interop_fixture",
            "--",
            "--ignored",
        ],
        cwd=DIMOS_PROJECT_ROOT,
        env={**os.environ, "DIMOS_MCAP_INTEROP_DIR": str(directory)},
        check=True,
        capture_output=True,
        timeout=300,
    )
    return artifact, messages


def test_independent_cdr_decode(recording: tuple[Path, dict[str, Any]]) -> None:
    artifact, _ = recording
    with artifact.open("rb") as source:
        reader = make_reader(source, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        assert summary is not None
        assert len(summary.schemas) == 11  # image and depth share a schema
        assert all(channel.schema_id != 0 for channel in summary.channels.values())
        assert all(chunk.compression == "zstd" for chunk in summary.chunk_indexes)
        decoded = {
            channel.topic: value
            for _, channel, _, value in reader.iter_decoded_messages(
                topics=[c.topic for c in summary.channels.values() if c.message_encoding == "cdr"]
            )
        }
    imu = decoded["imu"]
    assert (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) == (
        0.1,
        0.2,
        0.3,
        0.9,
    )
    assert list(imu.angular_velocity_covariance) == list(range(10, 19))
    assert imu.header.stamp.sec == 12 and imu.header.stamp.nanosec == 500_000_000
    odom = decoded["odom"]
    assert list(odom.pose.covariance) == list(range(36))
    assert list(odom.twist.covariance) == list(range(36, 72))
    assert odom.twist.twist.angular.z == 9
    assert decoded["camera_info"].roi.do_rectify
    assert list(decoded["camera_info"].p) == list(range(12))
    assert decoded["cloud"].fields[3].name == "intensity"
    assert decoded["image"].encoding == "rgb8"
    assert decoded["depth"].encoding == "16UC1"
    assert bytes(decoded["jpeg"].data).startswith(b"\xff\xd8")


def test_native_recording_mem2_roundtrip(recording: tuple[Path, dict[str, Any]]) -> None:
    artifact, expected = recording
    with open_store(artifact) as store:
        assert isinstance(store, McapRecordingStore)
        assert store.list_streams() == sorted(expected)
        for name in (
            "imu",
            "pose",
            "odom",
            "path",
            "camera_info",
            "joints",
            "image",
            "depth",
            "float_depth",
        ):
            observation = store.stream(name).first()
            assert observation.ts == 12.5
            assert observation.data.lcm_encode() == expected[name].lcm_encode(), name
        tf = list(store.stream("tf").order_by("ts"))
        assert [observation.ts for observation in tf] == [12.5, 13.5]
        assert [observation.data.transforms[0].child_frame_id for observation in tf] == [
            "base",
            "camera",
        ]
        cloud = store.stream("cloud").first().data
        np.testing.assert_array_equal(cloud.points(), expected["cloud"].points())
        np.testing.assert_array_equal(cloud.intensities_f32(), [11, 22])
        segments = store.stream("segments").first().data
        np.testing.assert_array_equal(segments.segments, expected["segments"].segments)
        np.testing.assert_array_equal(segments.weights, [0.75])
        mono = store.stream("jpeg_mono").first().data
        assert mono.format is ImageFormat.GRAY
        np.testing.assert_array_equal(mono.data, expected["jpeg_mono"].data)
        jpeg = store.stream("jpeg").first().data
        assert jpeg.frame_id == "camera" and jpeg.format is ImageFormat.RGB
        assert np.mean(np.abs(jpeg.data.astype(float) - expected["image"].data.astype(float))) < 5


def test_native_recording_summary(
    recording: tuple[Path, dict[str, Any]], capsys: pytest.CaptureFixture[str]
) -> None:
    summarize(str(recording[0]))
    output = capsys.readouterr().out
    assert 'Stream("imu"): 1 items' in output
    assert 'Stream("segments"): 1 items' in output
    assert 'Stream("tf"): 2 items' in output


def test_native_recording_rerun_export(
    recording: tuple[Path, dict[str, Any]], tmp_path: Path
) -> None:
    out = tmp_path / "recording.rrd"
    with open_store(recording[0]) as store:
        assert render_store(store, out=str(out), no_gui=True) == str(out)
    assert out.stat().st_size > 0

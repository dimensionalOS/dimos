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

"""Camera metadata uses the same generated header and frame chain as its images."""

from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.tf2_msgs.msg import TFMessage
import pytest

from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam


@pytest.fixture
def module():
    camera = CameraModule()
    camera.hardware = Webcam(frame_id_prefix="robot")
    try:
        yield camera
    finally:
        camera.stop()


def test_metadata_preserves_calibration_and_does_not_mutate_mount_template(module, mocker):
    info_publish = mocker.patch.object(module.camera_info, "publish")
    tf_publish = mocker.patch.object(module.tf, "publish")
    mocker.patch(
        "dimos.hardware.sensors.camera.module.time.time_ns", return_value=1700000000123456789
    )
    original = module.config.transform.encode()

    module.publish_metadata()

    info_publish.assert_called_once()
    tf_publish.assert_called_once()
    info = CameraInfo.decode(info_publish.call_args.args[0].encode())
    transforms = TFMessage.decode(tf_publish.call_args.args[0].encode()).transforms
    assert (info.header.stamp.sec, info.header.stamp.nanosec) == (1700000000, 123456789)
    assert len(transforms) == 2
    assert [(edge.header.frame_id, edge.child_frame_id) for edge in transforms] == [
        ("base_link", "camera_link"),
        ("camera_link", "robot/camera_optical"),
    ]
    assert all(edge.header.stamp == info.header.stamp for edge in transforms)
    assert module.config.transform.encode() == original

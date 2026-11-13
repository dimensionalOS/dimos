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

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.robot.galaxea.r1pro.spatial_sensors import R1SpatialSensors
from dimos.robot.galaxea.r1pro.stereo import disparity_depth


def test_metric_depth_and_invalid_disparity_holes():
    result = disparity_depth(np.array([[12, 6, 0, -1, float("nan"), 0.01]], dtype=np.float32), 12)
    np.testing.assert_array_equal(result, [[1, 2, 0, 0, 0, 0]])


@pytest.fixture
def sensors():
    module = R1SpatialSensors()
    yield module
    module.dispose()


def image_stamp(ts):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=int(ts), nanosec=round((ts - int(ts)) * 1e9))
        )
    )


def test_stereo_pairs_close_timestamps_and_never_reuses_or_restamps_old_frames(sensors, mocker):
    mocker.patch("dimos.robot.galaxea.r1pro.spatial_sensors.time.time", return_value=100)
    left = image_stamp(99.9)
    right = image_stamp(99.907)
    sensors._receive("a", left)
    sensors._receive("b", image_stamp(99.84))
    assert sensors._pair("a", "b") is None
    sensors._receive("b", right)
    assert sensors._pair("a", "b") == (left, right)
    assert sensors._pair("a", "b") is None
    sensors._receive("c", image_stamp(99))
    sensors._receive("d", image_stamp(99.007))
    assert sensors._pair("c", "d") is None


def test_camera_period_includes_processing_time(sensors, mocker):
    mocker.patch(
        "dimos.robot.galaxea.r1pro.spatial_sensors.time.monotonic", side_effect=[10.0, 10.15]
    )
    capture = mocker.patch.object(sensors, "capture", side_effect=sensors._stop.set)
    wait = mocker.patch.object(sensors._stop, "wait")
    sensors._run()
    capture.assert_called_once_with()
    assert wait.call_args.args[0] == pytest.approx(0.05)


@pytest.mark.asyncio
async def test_native_head_streams_pair_on_capture_time(sensors, mocker):
    mocker.patch("dimos.robot.galaxea.r1pro.spatial_sensors.time.time", return_value=100)
    left = CompressedImage(ts=99.9, data=b"left")
    right = CompressedImage(ts=99.901, data=b"right")
    await sensors.handle_head_left_color(left)
    await sensors.handle_head_right_color(right)
    assert sensors._pair("head_left", "head_right") == (left, right)
    await sensors.handle_head_left_color(CompressedImage(ts=99.91, data=b"next"))
    assert sensors._pair("head_left", "head_right") is None


def test_rgbd_output_shares_frame_calibration_and_capture_stamp_and_drops_expired_work(
    sensors, mocker
):
    now = mocker.patch("dimos.robot.galaxea.r1pro.spatial_sensors.time.time", return_value=100)
    ports = (sensors.color_image, sensors.depth_image, sensors.camera_info)
    outputs = [mocker.patch.object(port, "publish") for port in ports]
    color = np.zeros((24, 32, 3), dtype=np.uint8)
    depth = np.full((24, 32), 1.5, dtype=np.float32)
    info = CameraInfo.from_intrinsics(30, 30, 16, 12, 32, 24)
    assert sensors._publish(ports, color, depth, info, "optical", 99.9)
    messages = [output.call_args.args[0] for output in outputs]
    assert [(msg.frame_id, msg.ts) for msg in messages] == [("optical", 99.9)] * 3
    np.testing.assert_array_equal(messages[1].data, depth)
    now.return_value = 101
    assert not sensors._publish(ports, color, depth, info, "optical", 99.9)
    assert [output.call_count for output in outputs] == [1, 1, 1]

#!/usr/bin/env python3
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

from dataclasses import asdict

from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.hardware.sensors.camera.zed import compat as zed


def test_zed_import_and_calibration_access() -> None:
    """Test that zed module can be imported and calibrations accessed."""
    # Test that CameraInfo is accessible
    assert hasattr(zed, "CameraInfo")

    # Test snake_case access
    camera_info_snake = zed.CameraInfo.single_webcam
    assert isinstance(camera_info_snake, CameraInfo)
    assert camera_info_snake.width == 640
    assert camera_info_snake.height == 376
    assert camera_info_snake.distortion_model == "plumb_bob"

    # Test PascalCase access
    camera_info_pascal = zed.CameraInfo.SingleWebcam
    assert isinstance(camera_info_pascal, CameraInfo)
    assert camera_info_pascal.width == 640
    assert camera_info_pascal.height == 376

    # Verify both access methods return the same cached object
    assert camera_info_snake is camera_info_pascal

    print("✓ ZED import and calibration access test passed!")


def test_calibration_is_generated_cdr_with_default_optical_frame():
    value = zed.CameraInfo.SingleWebcam
    assert type(value) is CameraInfo
    assert value.header.frame_id == "camera_optical"
    assert value.k[0] == 379.45267
    np.testing.assert_equal(asdict(cdr_decode(cdr_encode(value), CameraInfo)), asdict(value))

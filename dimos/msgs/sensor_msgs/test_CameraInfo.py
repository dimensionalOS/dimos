#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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
import pytest

try:
    from sensor_msgs.msg import CameraInfo as ROSCameraInfo, RegionOfInterest as ROSRegionOfInterest
    from std_msgs.msg import Header as ROSHeader
except ImportError:
    ROSCameraInfo = None
    ROSRegionOfInterest = None
    ROSHeader = None

from dimos.msgs.sensor_msgs.CameraInfo import CalibrationProvider, CameraInfo
from dimos.utils.path_utils import get_project_root


def test_lcm_encode_decode() -> None:
    """Test LCM encode/decode preserves CameraInfo data."""
    print("Testing CameraInfo LCM encode/decode...")

    # Create test camera info with sample calibration data
    original = CameraInfo(
        height=480,
        width=640,
        distortion_model="plumb_bob",
        D=[-0.1, 0.05, 0.001, -0.002, 0.0],  # 5 distortion coefficients
        K=[
            500.0,
            0.0,
            320.0,  # fx, 0, cx
            0.0,
            500.0,
            240.0,  # 0, fy, cy
            0.0,
            0.0,
            1.0,
        ],  # 0, 0, 1
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[
            500.0,
            0.0,
            320.0,
            0.0,  # fx, 0, cx, Tx
            0.0,
            500.0,
            240.0,
            0.0,  # 0, fy, cy, Ty
            0.0,
            0.0,
            1.0,
            0.0,
        ],  # 0, 0, 1, 0
        binning_x=2,
        binning_y=2,
        frame_id="camera_optical_frame",
        ts=1234567890.123456,
    )

    # Set ROI
    original.roi_x_offset = 100
    original.roi_y_offset = 50
    original.roi_height = 200
    original.roi_width = 300
    original.roi_do_rectify = True

    # Encode and decode
    binary_msg = original.lcm_encode()
    decoded = CameraInfo.lcm_decode(binary_msg)

    # Check basic properties
    assert original.height == decoded.height, (
        f"Height mismatch: {original.height} vs {decoded.height}"
    )
    assert original.width == decoded.width, f"Width mismatch: {original.width} vs {decoded.width}"
    print(f"✓ Image dimensions preserved: {decoded.width}x{decoded.height}")

    assert original.distortion_model == decoded.distortion_model, (
        f"Distortion model mismatch: '{original.distortion_model}' vs '{decoded.distortion_model}'"
    )
    print(f"✓ Distortion model preserved: '{decoded.distortion_model}'")

    # Check distortion coefficients
    assert len(original.D) == len(decoded.D), (
        f"D length mismatch: {len(original.D)} vs {len(decoded.D)}"
    )
    np.testing.assert_allclose(
        original.D, decoded.D, rtol=1e-9, atol=1e-9, err_msg="Distortion coefficients don't match"
    )
    print(f"✓ Distortion coefficients preserved: {len(decoded.D)} coefficients")

    # Check camera matrices
    np.testing.assert_allclose(
        original.K, decoded.K, rtol=1e-9, atol=1e-9, err_msg="K matrix doesn't match"
    )
    print("✓ Intrinsic matrix K preserved")

    np.testing.assert_allclose(
        original.R, decoded.R, rtol=1e-9, atol=1e-9, err_msg="R matrix doesn't match"
    )
    print("✓ Rectification matrix R preserved")

    np.testing.assert_allclose(
        original.P, decoded.P, rtol=1e-9, atol=1e-9, err_msg="P matrix doesn't match"
    )
    print("✓ Projection matrix P preserved")

    # Check binning
    assert original.binning_x == decoded.binning_x, (
        f"Binning X mismatch: {original.binning_x} vs {decoded.binning_x}"
    )
    assert original.binning_y == decoded.binning_y, (
        f"Binning Y mismatch: {original.binning_y} vs {decoded.binning_y}"
    )
    print(f"✓ Binning preserved: {decoded.binning_x}x{decoded.binning_y}")

    # Check ROI
    assert original.roi_x_offset == decoded.roi_x_offset, "ROI x_offset mismatch"
    assert original.roi_y_offset == decoded.roi_y_offset, "ROI y_offset mismatch"
    assert original.roi_height == decoded.roi_height, "ROI height mismatch"
    assert original.roi_width == decoded.roi_width, "ROI width mismatch"
    assert original.roi_do_rectify == decoded.roi_do_rectify, "ROI do_rectify mismatch"
    print("✓ ROI preserved")

    # Check metadata
    assert original.frame_id == decoded.frame_id, (
        f"Frame ID mismatch: '{original.frame_id}' vs '{decoded.frame_id}'"
    )
    print(f"✓ Frame ID preserved: '{decoded.frame_id}'")

    assert abs(original.ts - decoded.ts) < 1e-6, (
        f"Timestamp mismatch: {original.ts} vs {decoded.ts}"
    )
    print(f"✓ Timestamp preserved: {decoded.ts}")

    print("✓ LCM encode/decode test passed - all properties preserved!")


def test_numpy_matrix_operations() -> None:
    """Test numpy matrix getter/setter operations."""
    print("\nTesting numpy matrix operations...")

    camera_info = CameraInfo()

    # Test K matrix
    K = np.array([[525.0, 0.0, 319.5], [0.0, 525.0, 239.5], [0.0, 0.0, 1.0]])
    camera_info.set_K_matrix(K)
    K_retrieved = camera_info.get_K_matrix()
    np.testing.assert_allclose(K, K_retrieved, rtol=1e-9, atol=1e-9)
    print("✓ K matrix setter/getter works")

    # Test P matrix
    P = np.array([[525.0, 0.0, 319.5, 0.0], [0.0, 525.0, 239.5, 0.0], [0.0, 0.0, 1.0, 0.0]])
    camera_info.set_P_matrix(P)
    P_retrieved = camera_info.get_P_matrix()
    np.testing.assert_allclose(P, P_retrieved, rtol=1e-9, atol=1e-9)
    print("✓ P matrix setter/getter works")

    # Test R matrix
    R = np.eye(3)
    camera_info.set_R_matrix(R)
    R_retrieved = camera_info.get_R_matrix()
    np.testing.assert_allclose(R, R_retrieved, rtol=1e-9, atol=1e-9)
    print("✓ R matrix setter/getter works")

    # Test D coefficients
    D = np.array([-0.2, 0.1, 0.001, -0.002, 0.05])
    camera_info.set_D_coeffs(D)
    D_retrieved = camera_info.get_D_coeffs()
    np.testing.assert_allclose(D, D_retrieved, rtol=1e-9, atol=1e-9)
    print("✓ D coefficients setter/getter works")

    print("✓ All numpy matrix operations passed!")


@pytest.mark.ros
def test_ros_conversion() -> None:
    """Test ROS message conversion preserves CameraInfo data."""
    print("\nTesting ROS CameraInfo conversion...")

    # Create test camera info
    original = CameraInfo(
        height=720,
        width=1280,
        distortion_model="rational_polynomial",
        D=[0.1, -0.2, 0.001, 0.002, -0.05, 0.01, -0.02, 0.003],  # 8 coefficients
        K=[600.0, 0.0, 640.0, 0.0, 600.0, 360.0, 0.0, 0.0, 1.0],
        R=[0.999, -0.01, 0.02, 0.01, 0.999, -0.01, -0.02, 0.01, 0.999],
        P=[
            600.0,
            0.0,
            640.0,
            -60.0,  # Stereo baseline of 0.1m
            0.0,
            600.0,
            360.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ],
        binning_x=1,
        binning_y=1,
        frame_id="left_camera_optical",
        ts=1234567890.987654,
    )

    # Set ROI
    original.roi_x_offset = 200
    original.roi_y_offset = 100
    original.roi_height = 400
    original.roi_width = 800
    original.roi_do_rectify = False

    # Test 1: Convert to ROS and back
    ros_msg = original.to_ros_msg()
    converted = CameraInfo.from_ros_msg(ros_msg)

    # Check all properties
    assert original.height == converted.height, (
        f"Height mismatch: {original.height} vs {converted.height}"
    )
    assert original.width == converted.width, (
        f"Width mismatch: {original.width} vs {converted.width}"
    )
    print(f"✓ Dimensions preserved: {converted.width}x{converted.height}")

    assert original.distortion_model == converted.distortion_model, (
        f"Distortion model mismatch: '{original.distortion_model}' vs '{converted.distortion_model}'"
    )
    print(f"✓ Distortion model preserved: '{converted.distortion_model}'")

    np.testing.assert_allclose(
        original.D,
        converted.D,
        rtol=1e-9,
        atol=1e-9,
        err_msg="D coefficients don't match after ROS conversion",
    )
    print(f"✓ Distortion coefficients preserved: {len(converted.D)} coefficients")

    np.testing.assert_allclose(
        original.K,
        converted.K,
        rtol=1e-9,
        atol=1e-9,
        err_msg="K matrix doesn't match after ROS conversion",
    )
    print("✓ K matrix preserved")

    np.testing.assert_allclose(
        original.R,
        converted.R,
        rtol=1e-9,
        atol=1e-9,
        err_msg="R matrix doesn't match after ROS conversion",
    )
    print("✓ R matrix preserved")

    np.testing.assert_allclose(
        original.P,
        converted.P,
        rtol=1e-9,
        atol=1e-9,
        err_msg="P matrix doesn't match after ROS conversion",
    )
    print("✓ P matrix preserved")

    assert original.binning_x == converted.binning_x, "Binning X mismatch"
    assert original.binning_y == converted.binning_y, "Binning Y mismatch"
    print(f"✓ Binning preserved: {converted.binning_x}x{converted.binning_y}")

    assert original.roi_x_offset == converted.roi_x_offset, "ROI x_offset mismatch"
    assert original.roi_y_offset == converted.roi_y_offset, "ROI y_offset mismatch"
    assert original.roi_height == converted.roi_height, "ROI height mismatch"
    assert original.roi_width == converted.roi_width, "ROI width mismatch"
    assert original.roi_do_rectify == converted.roi_do_rectify, "ROI do_rectify mismatch"
    print("✓ ROI preserved")

    assert original.frame_id == converted.frame_id, (
        f"Frame ID mismatch: '{original.frame_id}' vs '{converted.frame_id}'"
    )
    print(f"✓ Frame ID preserved: '{converted.frame_id}'")

    assert abs(original.ts - converted.ts) < 1e-6, (
        f"Timestamp mismatch: {original.ts} vs {converted.ts}"
    )
    print(f"✓ Timestamp preserved: {converted.ts}")

    # Test 2: Create ROS message directly and convert to DIMOS
    ros_msg2 = ROSCameraInfo()
    ros_msg2.header = ROSHeader()
    ros_msg2.header.frame_id = "test_camera"
    ros_msg2.header.stamp.sec = 1234567890
    ros_msg2.header.stamp.nanosec = 500000000

    ros_msg2.height = 1080
    ros_msg2.width = 1920
    ros_msg2.distortion_model = "plumb_bob"
    ros_msg2.d = [-0.3, 0.15, 0.0, 0.0, 0.0]
    ros_msg2.k = [1000.0, 0.0, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0]
    ros_msg2.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ros_msg2.p = [1000.0, 0.0, 960.0, 0.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    ros_msg2.binning_x = 4
    ros_msg2.binning_y = 4

    ros_msg2.roi = ROSRegionOfInterest()
    ros_msg2.roi.x_offset = 10
    ros_msg2.roi.y_offset = 20
    ros_msg2.roi.height = 100
    ros_msg2.roi.width = 200
    ros_msg2.roi.do_rectify = True

    # Convert to DIMOS
    dimos_info = CameraInfo.from_ros_msg(ros_msg2)

    assert dimos_info.height == 1080, (
        f"Height not preserved: expected 1080, got {dimos_info.height}"
    )
    assert dimos_info.width == 1920, f"Width not preserved: expected 1920, got {dimos_info.width}"
    assert dimos_info.frame_id == "test_camera", (
        f"Frame ID not preserved: expected 'test_camera', got '{dimos_info.frame_id}'"
    )
    assert dimos_info.distortion_model == "plumb_bob", "Distortion model not preserved"
    assert len(dimos_info.D) == 5, (
        f"Wrong number of distortion coefficients: expected 5, got {len(dimos_info.D)}"
    )
    print("✓ ROS to DIMOS conversion works correctly")

    # Test 3: Empty/minimal CameraInfo
    minimal = CameraInfo(frame_id="minimal_camera", ts=1234567890.0)
    minimal_ros = minimal.to_ros_msg()
    minimal_converted = CameraInfo.from_ros_msg(minimal_ros)

    assert minimal.frame_id == minimal_converted.frame_id, (
        "Minimal CameraInfo frame_id not preserved"
    )
    assert len(minimal_converted.D) == 0, "Minimal CameraInfo should have empty D"
    print("✓ Minimal CameraInfo handling works")

    print("\n✓ All ROS conversion tests passed!")


def test_equality() -> None:
    """Test CameraInfo equality comparison."""
    print("\nTesting CameraInfo equality...")

    info1 = CameraInfo(
        height=480,
        width=640,
        distortion_model="plumb_bob",
        D=[-0.1, 0.05, 0.0, 0.0, 0.0],
        frame_id="camera1",
    )

    info2 = CameraInfo(
        height=480,
        width=640,
        distortion_model="plumb_bob",
        D=[-0.1, 0.05, 0.0, 0.0, 0.0],
        frame_id="camera1",
    )

    info3 = CameraInfo(
        height=720,
        width=1280,  # Different resolution
        distortion_model="plumb_bob",
        D=[-0.1, 0.05, 0.0, 0.0, 0.0],
        frame_id="camera1",
    )

    assert info1 == info2, "Identical CameraInfo objects should be equal"
    assert info1 != info3, "Different CameraInfo objects should not be equal"
    assert info1 != "not_camera_info", "CameraInfo should not equal non-CameraInfo object"

    print("✓ Equality comparison works correctly")


def test_camera_info_from_yaml() -> None:
    """Test loading CameraInfo from YAML file."""

    # Get path to the single webcam YAML file
    yaml_path = (
        get_project_root()
        / "dimos"
        / "hardware"
        / "sensors"
        / "camera"
        / "zed"
        / "single_webcam.yaml"
    )

    # Load CameraInfo from YAML
    camera_info = CameraInfo.from_yaml(str(yaml_path))

    # Verify loaded values
    assert camera_info.width == 640
    assert camera_info.height == 376
    assert camera_info.distortion_model == "plumb_bob"
    assert camera_info.frame_id == "camera_optical"

    # Check camera matrix K
    K = camera_info.get_K_matrix()
    assert K.shape == (3, 3)
    assert np.isclose(K[0, 0], 379.45267)  # fx
    assert np.isclose(K[1, 1], 380.67871)  # fy
    assert np.isclose(K[0, 2], 302.43516)  # cx
    assert np.isclose(K[1, 2], 228.00954)  # cy

    # Check distortion coefficients
    D = camera_info.get_D_coeffs()
    assert len(D) == 5
    assert np.isclose(D[0], -0.309435)

    # Check projection matrix P
    P = camera_info.get_P_matrix()
    assert P.shape == (3, 4)
    assert np.isclose(P[0, 0], 291.12888)

    print("✓ CameraInfo loaded successfully from YAML file")


def test_calibration_provider() -> None:
    """Test CalibrationProvider lazy loading of YAML files."""
    # Get the directory containing calibration files (not the file itself)
    calibration_dir = get_project_root() / "dimos" / "hardware" / "sensors" / "camera" / "zed"

    # Create CalibrationProvider instance
    Calibrations = CalibrationProvider(calibration_dir)

    # Test lazy loading of single_webcam.yaml using snake_case
    camera_info = Calibrations.single_webcam
    assert isinstance(camera_info, CameraInfo)
    assert camera_info.width == 640
    assert camera_info.height == 376

    # Test PascalCase access to same calibration
    camera_info2 = Calibrations.SingleWebcam
    assert isinstance(camera_info2, CameraInfo)
    assert camera_info2.width == 640
    assert camera_info2.height == 376

    # Test caching - both access methods should return same object
    assert camera_info is camera_info2  # Same object reference

    # Test __dir__ lists available calibrations in both cases
    available = dir(Calibrations)
    assert "single_webcam" in available
    assert "SingleWebcam" in available

    print("✓ CalibrationProvider test passed with both naming conventions!")

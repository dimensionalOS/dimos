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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import CameraInfo, RegionOfInterest
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
from rosbags.typesys import Stores, get_typestore

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.msgs.camera_info import CalibrationProvider, camera_info_from_yaml, intrinsic_matrix
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds


def test_encode_decode() -> None:
    """Test CDR encode/decode preserves CameraInfo data."""
    print("Testing CameraInfo CDR encode/decode...")

    # Create test camera info with sample calibration data
    original = CameraInfo(
        height=480,
        width=640,
        distortion_model="plumb_bob",
        d=np.array([-0.1, 0.05, 0.001, -0.002, 0.0], dtype=np.float64),  # 5 distortion coefficients
        k=np.array(
            [
                500.0,
                0.0,
                320.0,  # fx, 0, cx
                0.0,
                500.0,
                240.0,  # 0, fy, cy
                0.0,
                0.0,
                1.0,
            ],
            dtype=np.float64,
        ),  # 0, 0, 1
        r=np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        p=np.array(
            [
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
            ],
            dtype=np.float64,
        ),  # 0, 0, 1, 0
        binning_x=2,
        binning_y=2,
        header=Header(
            frame_id="camera_optical_frame", stamp=time_from_nanoseconds(1234567890123456789)
        ),
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )

    # Set ROI
    original.roi.x_offset = 100
    original.roi.y_offset = 50
    original.roi.height = 200
    original.roi.width = 300
    original.roi.do_rectify = True

    # Encode and decode
    binary_msg = cdr_encode(original)
    decoded = cdr_decode(binary_msg, CameraInfo)
    independent = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(
        binary_msg, CameraInfo.__msgtype__
    )
    assert independent.header.stamp.nanosec == 123456789
    assert independent.roi.x_offset == 100
    np.testing.assert_array_equal(independent.k, original.k)

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
    assert len(original.d) == len(decoded.d), (
        f"D length mismatch: {len(original.d)} vs {len(decoded.d)}"
    )
    np.testing.assert_allclose(
        original.d, decoded.d, rtol=1e-9, atol=1e-9, err_msg="Distortion coefficients don't match"
    )
    print(f"✓ Distortion coefficients preserved: {len(decoded.d)} coefficients")

    # Check camera matrices
    np.testing.assert_allclose(
        original.k, decoded.k, rtol=1e-9, atol=1e-9, err_msg="K matrix doesn't match"
    )
    print("✓ Intrinsic matrix K preserved")

    np.testing.assert_allclose(
        original.r, decoded.r, rtol=1e-9, atol=1e-9, err_msg="R matrix doesn't match"
    )
    print("✓ Rectification matrix R preserved")

    np.testing.assert_allclose(
        original.p, decoded.p, rtol=1e-9, atol=1e-9, err_msg="P matrix doesn't match"
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
    assert original.roi.x_offset == decoded.roi.x_offset, "ROI x_offset mismatch"
    assert original.roi.y_offset == decoded.roi.y_offset, "ROI y_offset mismatch"
    assert original.roi.height == decoded.roi.height, "ROI height mismatch"
    assert original.roi.width == decoded.roi.width, "ROI width mismatch"
    assert original.roi.do_rectify == decoded.roi.do_rectify, "ROI do_rectify mismatch"
    print("✓ ROI preserved")

    # Check metadata
    assert original.header.frame_id == decoded.header.frame_id, (
        f"Frame ID mismatch: '{original.header.frame_id}' vs '{decoded.header.frame_id}'"
    )
    print(f"✓ Frame ID preserved: '{decoded.header.frame_id}'")

    assert to_nanoseconds(original.header.stamp) == to_nanoseconds(decoded.header.stamp), (
        f"Timestamp mismatch: {to_nanoseconds(original.header.stamp)} vs {to_nanoseconds(decoded.header.stamp)}"
    )
    print(f"✓ Timestamp preserved: {to_nanoseconds(decoded.header.stamp)}")

    print("✓ CDR encode/decode test passed - all properties preserved!")


def test_numpy_matrix_operations() -> None:
    """Test numpy matrix getter/setter operations."""
    print("\nTesting numpy matrix operations...")

    camera_info = CameraInfo(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        height=0,
        width=0,
        distortion_model="",
        d=np.array([], dtype=np.float64),
        k=np.zeros(9, dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )

    # Test K matrix
    K = np.array([[525.0, 0.0, 319.5], [0.0, 525.0, 239.5], [0.0, 0.0, 1.0]])
    camera_info.k = K.ravel()
    K_retrieved = intrinsic_matrix(camera_info)
    np.testing.assert_allclose(K, K_retrieved, rtol=1e-9, atol=1e-9)
    print("✓ K matrix setter/getter works")

    # Test P matrix
    P = np.array([[525.0, 0.0, 319.5, 0.0], [0.0, 525.0, 239.5, 0.0], [0.0, 0.0, 1.0, 0.0]])
    camera_info.p = P.ravel()
    P_retrieved = np.asarray(camera_info.p).reshape(3, 4)
    np.testing.assert_allclose(P, P_retrieved, rtol=1e-9, atol=1e-9)
    print("✓ P matrix setter/getter works")

    # Test R matrix
    R = np.eye(3)
    camera_info.r = R.ravel()
    R_retrieved = np.asarray(camera_info.r).reshape(3, 3)
    np.testing.assert_allclose(R, R_retrieved, rtol=1e-9, atol=1e-9)
    print("✓ R matrix setter/getter works")

    # Test D coefficients
    D = np.array([-0.2, 0.1, 0.001, -0.002, 0.05])
    camera_info.d = D
    D_retrieved = np.asarray(camera_info.d)
    np.testing.assert_allclose(D, D_retrieved, rtol=1e-9, atol=1e-9)
    print("✓ D coefficients setter/getter works")

    print("✓ All numpy matrix operations passed!")


def test_equality() -> None:
    """Test CameraInfo equality comparison."""
    print("\nTesting CameraInfo equality...")

    info1 = CameraInfo(
        height=480,
        width=640,
        distortion_model="plumb_bob",
        d=np.array([-0.1, 0.05, 0.0, 0.0, 0.0], dtype=np.float64),
        header=Header(frame_id="camera1", stamp=Time(sec=0, nanosec=0)),
        k=np.zeros(9, dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )

    info2 = CameraInfo(
        height=480,
        width=640,
        distortion_model="plumb_bob",
        d=np.array([-0.1, 0.05, 0.0, 0.0, 0.0], dtype=np.float64),
        header=Header(frame_id="camera1", stamp=Time(sec=0, nanosec=0)),
        k=np.zeros(9, dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )

    info3 = CameraInfo(
        height=720,
        width=1280,  # Different resolution
        distortion_model="plumb_bob",
        d=np.array([-0.1, 0.05, 0.0, 0.0, 0.0], dtype=np.float64),
        header=Header(frame_id="camera1", stamp=Time(sec=0, nanosec=0)),
        k=np.zeros(9, dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )

    np.testing.assert_equal(asdict(info1), asdict(info2))
    assert (info1.height, info1.width) != (info3.height, info3.width)
    assert info1 != "not_camera_info", "CameraInfo should not equal non-CameraInfo object"

    print("✓ Equality comparison works correctly")


def test_camera_info_from_yaml() -> None:
    """Test loading CameraInfo from YAML file."""

    # Get path to the single webcam YAML file
    yaml_path = (
        DIMOS_PROJECT_ROOT
        / "dimos"
        / "hardware"
        / "sensors"
        / "camera"
        / "zed"
        / "single_webcam.yaml"
    )

    # Load CameraInfo from YAML
    camera_info = camera_info_from_yaml(
        yaml_path, header=Header(frame_id="camera_optical", stamp=Time(sec=0, nanosec=0))
    )

    # Verify loaded values
    assert camera_info.width == 640
    assert camera_info.height == 376
    assert camera_info.distortion_model == "plumb_bob"
    assert camera_info.header.frame_id == "camera_optical"

    # Check camera matrix K
    K = intrinsic_matrix(camera_info)
    assert K.shape == (3, 3)
    assert np.isclose(K[0, 0], 379.45267)  # fx
    assert np.isclose(K[1, 1], 380.67871)  # fy
    assert np.isclose(K[0, 2], 302.43516)  # cx
    assert np.isclose(K[1, 2], 228.00954)  # cy

    # Check distortion coefficients
    D = np.asarray(camera_info.d)
    assert len(D) == 5
    assert np.isclose(D[0], -0.309435)

    # Check projection matrix P
    P = np.asarray(camera_info.p).reshape(3, 4)
    assert P.shape == (3, 4)
    assert np.isclose(P[0, 0], 291.12888)

    print("✓ CameraInfo loaded successfully from YAML file")


def test_calibration_provider() -> None:
    """Test CalibrationProvider lazy loading of YAML files."""
    # Get the directory containing calibration files (not the file itself)
    calibration_dir = DIMOS_PROJECT_ROOT / "dimos" / "hardware" / "sensors" / "camera" / "zed"

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

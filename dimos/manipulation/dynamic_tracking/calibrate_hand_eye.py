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

"""Eye-in-hand calibration tool for computing the EEF→camera transform.

Collects paired (EE pose, marker-in-camera pose) samples while the user
moves the arm to different configurations, then solves AX=XB using
OpenCV's calibrateHandEye().

Usage:
    python -m dimos.manipulation.dynamic_tracking.calibrate_hand_eye \\
        --arm-ip 192.168.1.210 \\
        --marker-size 0.015 \\
        --output calibration.json

    # The tool will:
    # 1. Connect to the arm and camera
    # 2. Prompt you to move the arm to different poses (marker must be visible)
    # 3. Press Enter to capture each sample (minimum 3, recommend 10+)
    # 4. Press 'q' to finish collecting and solve
    # 5. Save the EEF→camera transform to JSON
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Callable

import cv2
import numpy as np
from scipy.spatial.transform import Rotation  # type: ignore[import-untyped]


def collect_sample_interactive(
    arm_positions_getter: Callable[[], np.ndarray | None],
    camera_frame_getter: Callable[[], tuple[np.ndarray | None, np.ndarray, np.ndarray]],
    marker_size: float,
    aruco_dict_id: int = cv2.aruco.DICT_4X4_50,
) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
    """Interactively collect calibration samples.

    Returns:
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    R_gripper2base_list: list[np.ndarray] = []
    t_gripper2base_list: list[np.ndarray] = []
    R_target2cam_list: list[np.ndarray] = []
    t_target2cam_list: list[np.ndarray] = []

    print("\n=== Eye-in-Hand Calibration ===")
    print("Move the arm so the marker is visible from different angles.")
    print("Press ENTER to capture a sample, 'q' to finish.\n")

    sample_idx = 0
    while True:
        user_input = input(f"Sample {sample_idx + 1} — press Enter to capture, 'q' to finish: ")
        if user_input.strip().lower() == "q":
            break

        # Get EE pose from arm (4x4 homogeneous matrix)
        ee_pose = arm_positions_getter()
        if ee_pose is None:
            print("  Failed to read arm pose, try again.")
            continue

        # Get camera frame and detect marker
        frame, camera_matrix, dist_coeffs = camera_frame_getter()
        if frame is None:
            print("  Failed to read camera frame, try again.")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            print("  No markers detected, try again.")
            continue

        # Use first detected marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners[:1], marker_size, camera_matrix, dist_coeffs
        )
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        R_target, _ = cv2.Rodrigues(rvec)

        # EE pose decomposition
        R_ee = ee_pose[:3, :3]
        t_ee = ee_pose[:3, 3]

        R_gripper2base_list.append(R_ee)
        t_gripper2base_list.append(t_ee.reshape(3, 1))
        R_target2cam_list.append(R_target)
        t_target2cam_list.append(tvec.reshape(3, 1))

        sample_idx += 1
        print(
            f"  Captured sample {sample_idx}: marker at [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}]"
        )

    return R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list


def solve_hand_eye(
    R_gripper2base: list[np.ndarray],
    t_gripper2base: list[np.ndarray],
    R_target2cam: list[np.ndarray],
    t_target2cam: list[np.ndarray],
    method: int = cv2.CALIB_HAND_EYE_TSAI,
) -> tuple[np.ndarray, np.ndarray]:
    """Solve AX=XB for the camera-to-gripper transform.

    Args:
        R_gripper2base: List of 3x3 rotation matrices (EE in base frame)
        t_gripper2base: List of 3x1 translation vectors (EE in base frame)
        R_target2cam: List of 3x3 rotation matrices (marker in camera frame)
        t_target2cam: List of 3x1 translation vectors (marker in camera frame)
        method: OpenCV hand-eye method (default: Tsai)

    Returns:
        (R_cam2gripper, t_cam2gripper) — the 3x3 rotation and 3x1 translation
    """
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=method,
    )
    return R_cam2gripper, t_cam2gripper


def save_calibration(
    R: np.ndarray,
    t: np.ndarray,
    output_path: str | Path,
    method: str = "tsai",
    num_samples: int = 0,
) -> None:
    """Save calibration result as JSON.

    The JSON contains translation (xyz) and rotation (quaternion xyzw),
    suitable for use as base_transform in RealSenseCamera config.
    """
    quat = Rotation.from_matrix(R).as_quat()  # [x, y, z, w]
    result = {
        "transform": {
            "translation": {
                "x": float(t[0, 0]),
                "y": float(t[1, 0]),
                "z": float(t[2, 0]),
            },
            "rotation": {
                "x": float(quat[0]),
                "y": float(quat[1]),
                "z": float(quat[2]),
                "w": float(quat[3]),
            },
        },
        "metadata": {
            "method": method,
            "num_samples": num_samples,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "description": "EEF (gripper) to camera transform for eye-in-hand setup",
        },
    }
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + "\n")
    print(f"\nCalibration saved to {path}")
    print(f"  Translation: [{t[0, 0]:.6f}, {t[1, 0]:.6f}, {t[2, 0]:.6f}]")
    print(f"  Quaternion (xyzw): [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")


def load_calibration(path: str | Path) -> tuple[list[float], list[float]]:
    """Load calibration JSON and return (translation_xyz, quaternion_xyzw)."""
    data = json.loads(Path(path).read_text())
    t = data["transform"]["translation"]
    r = data["transform"]["rotation"]
    return [t["x"], t["y"], t["z"]], [r["x"], r["y"], r["z"], r["w"]]


def calibration_to_transform(path: str | Path) -> tuple[Any, Any]:
    """Load calibration and return (Vector3, Quaternion) for blueprint use.

    Example:
        from dimos.manipulation.dynamic_tracking.calibrate_hand_eye import calibration_to_transform
        translation, rotation = calibration_to_transform("calibration.json")
        # Use in RealSenseCamera blueprint:
        #   base_transform=Transform(translation=translation, rotation=rotation)
    """
    from dimos.msgs.geometry_msgs import Quaternion, Vector3

    xyz, xyzw = load_calibration(path)
    return Vector3(*xyz), Quaternion(*xyzw)


def _reprojection_error(
    R_cam2gripper: np.ndarray,
    t_cam2gripper: np.ndarray,
    R_gripper2base: list[np.ndarray],
    t_gripper2base: list[np.ndarray],
    R_target2cam: list[np.ndarray],
    t_target2cam: list[np.ndarray],
) -> float:
    """Compute mean reprojection error across all sample pairs."""
    errors = []
    n = len(R_gripper2base)
    for i in range(n):
        for j in range(i + 1, n):
            # Expected: base_T_target should be consistent across all samples
            T_base_cam_i = np.eye(4)
            T_base_cam_i[:3, :3] = R_gripper2base[i] @ R_cam2gripper
            T_base_cam_i[:3, 3] = (R_gripper2base[i] @ t_cam2gripper + t_gripper2base[i]).flatten()

            T_base_target_i = np.eye(4)
            T_base_target_i[:3, :3] = T_base_cam_i[:3, :3] @ R_target2cam[i]
            T_base_target_i[:3, 3] = (
                T_base_cam_i[:3, :3] @ t_target2cam[i] + T_base_cam_i[:3, 3:]
            ).flatten()

            T_base_cam_j = np.eye(4)
            T_base_cam_j[:3, :3] = R_gripper2base[j] @ R_cam2gripper
            T_base_cam_j[:3, 3] = (R_gripper2base[j] @ t_cam2gripper + t_gripper2base[j]).flatten()

            T_base_target_j = np.eye(4)
            T_base_target_j[:3, :3] = T_base_cam_j[:3, :3] @ R_target2cam[j]
            T_base_target_j[:3, 3] = (
                T_base_cam_j[:3, :3] @ t_target2cam[j] + T_base_cam_j[:3, 3:]
            ).flatten()

            pos_err = np.linalg.norm(T_base_target_i[:3, 3] - T_base_target_j[:3, 3])
            errors.append(pos_err)

    return float(np.mean(errors)) if errors else 0.0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Eye-in-hand calibration: compute EEF→camera transform"
    )
    parser.add_argument("--arm-ip", type=str, default="192.168.1.210", help="XArm IP address")
    parser.add_argument("--arm-dof", type=int, default=6, help="Arm degrees of freedom")
    parser.add_argument("--marker-size", type=float, default=0.015, help="ArUco marker size (m)")
    parser.add_argument(
        "--aruco-dict", type=int, default=cv2.aruco.DICT_4X4_50, help="ArUco dict ID"
    )
    parser.add_argument("--output", type=str, default="calibration.json", help="Output JSON path")
    parser.add_argument(
        "--method",
        type=str,
        default="tsai",
        choices=["tsai", "park", "horaud", "andreff", "daniilidis"],
        help="Hand-eye calibration method",
    )
    args = parser.parse_args()

    method_map = {
        "tsai": cv2.CALIB_HAND_EYE_TSAI,
        "park": cv2.CALIB_HAND_EYE_PARK,
        "horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    # --- Connect to arm ---
    try:
        from dimos.hardware.manipulators.xarm import XArmAdapter

        adapter = XArmAdapter(address=args.arm_ip, dof=args.arm_dof)
        if not adapter.connect():
            print("Failed to connect to arm")
            return 1
        print(f"Connected to XArm at {args.arm_ip}")
    except ImportError:
        print("XArm adapter not available. Install xArm SDK.")
        return 1

    # --- Connect to camera ---
    try:
        import pyrealsense2 as rs  # type: ignore[import-not-found]

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 15)
        profile = pipeline.start(config)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_stream.get_intrinsics()
        camera_matrix = np.array(
            [[intrinsics.fx, 0, intrinsics.ppx], [0, intrinsics.fy, intrinsics.ppy], [0, 0, 1]],
            dtype=np.float32,
        )
        dist_coeffs = (
            np.array(intrinsics.coeffs, dtype=np.float32)
            if intrinsics.coeffs
            else np.zeros(5, dtype=np.float32)
        )
        print("RealSense camera connected")
    except ImportError:
        print("pyrealsense2 not available. Install Intel RealSense SDK.")
        return 1

    def get_ee_pose() -> np.ndarray | None:
        """Read current EE pose as 4x4 homogeneous matrix."""
        try:
            pose = adapter.read_cartesian_position()
            if pose is None:
                return None
            R = Rotation.from_euler("xyz", [pose["roll"], pose["pitch"], pose["yaw"]]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [pose["x"], pose["y"], pose["z"]]
            return T
        except Exception as e:
            print(f"Failed to read EE pose: {e}")
            return None

    def get_camera_frame() -> tuple[np.ndarray | None, np.ndarray, np.ndarray]:
        """Capture a camera frame."""
        try:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None, camera_matrix, dist_coeffs
            return np.asanyarray(color_frame.get_data()), camera_matrix, dist_coeffs
        except Exception:
            return None, camera_matrix, dist_coeffs

    # --- Collect samples ---
    R_g2b, t_g2b, R_t2c, t_t2c = collect_sample_interactive(
        get_ee_pose, get_camera_frame, args.marker_size, args.aruco_dict
    )

    if len(R_g2b) < 3:
        print(f"\nNeed at least 3 samples, got {len(R_g2b)}. Aborting.")
        pipeline.stop()
        adapter.disconnect()
        return 1

    # --- Solve ---
    print(f"\nSolving hand-eye calibration with {len(R_g2b)} samples ({args.method})...")
    R_cam2gripper, t_cam2gripper = solve_hand_eye(
        R_g2b, t_g2b, R_t2c, t_t2c, method=method_map[args.method]
    )

    error = _reprojection_error(R_cam2gripper, t_cam2gripper, R_g2b, t_g2b, R_t2c, t_t2c)
    print(f"Mean pairwise target consistency error: {error * 1000:.2f} mm")

    save_calibration(R_cam2gripper, t_cam2gripper, args.output, args.method, len(R_g2b))

    # Cleanup
    pipeline.stop()
    adapter.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())

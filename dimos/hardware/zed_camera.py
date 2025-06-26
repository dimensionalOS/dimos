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
import cv2
import open3d as o3d
from typing import Optional, Tuple, Dict, Any
import logging

try:
    import pyzed.sl as sl
except ImportError:
    sl = None
    logging.warning("ZED SDK not found. Please install pyzed to use ZED camera functionality.")

from dimos.hardware.stereo_camera import StereoCamera

logger = logging.getLogger(__name__)


class ZEDCamera(StereoCamera):
    """ZED Camera capture node with neural depth processing."""

    def __init__(
        self,
        camera_id: int = 0,
        resolution: sl.RESOLUTION = sl.RESOLUTION.HD720,
        depth_mode: sl.DEPTH_MODE = sl.DEPTH_MODE.NEURAL,
        fps: int = 30,
        **kwargs,
    ):
        """
        Initialize ZED Camera.

        Args:
            camera_id: Camera ID (0 for first ZED)
            resolution: ZED camera resolution
            depth_mode: Depth computation mode
            fps: Camera frame rate (default: 30)
        """
        if sl is None:
            raise ImportError("ZED SDK not installed. Please install pyzed package.")

        super().__init__(**kwargs)

        self.camera_id = camera_id
        self.resolution = resolution
        self.depth_mode = depth_mode
        self.fps = fps

        # Initialize ZED camera
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = resolution
        self.init_params.depth_mode = depth_mode
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.camera_fps = fps

        # Set camera ID using the correct parameter name
        if hasattr(self.init_params, "set_from_camera_id"):
            self.init_params.set_from_camera_id(camera_id)
        elif hasattr(self.init_params, "input"):
            self.init_params.input.set_from_camera_id(camera_id)

        # Use enable_fill_mode instead of SENSING_MODE.STANDARD
        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.enable_fill_mode = True  # False = STANDARD mode, True = FILL mode

        # Image containers
        self.image_left = sl.Mat()
        self.image_right = sl.Mat()
        self.depth_map = sl.Mat()
        self.point_cloud = sl.Mat()
        self.confidence_map = sl.Mat()

        self.is_opened = False

    def open(self) -> bool:
        """Open the ZED camera."""
        try:
            err = self.zed.open(self.init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                logger.error(f"Failed to open ZED camera: {err}")
                return False

            self.is_opened = True
            logger.info("ZED camera opened successfully")

            # Get camera information
            info = self.zed.get_camera_information()
            logger.info(f"ZED Camera Model: {info.camera_model}")
            logger.info(f"Serial Number: {info.serial_number}")
            logger.info(f"Firmware: {info.camera_configuration.firmware_version}")

            return True

        except Exception as e:
            logger.error(f"Error opening ZED camera: {e}")
            return False

    def close(self):
        """Close the ZED camera."""
        if self.is_opened:
            self.zed.close()
            self.is_opened = False
            logger.info("ZED camera closed")

    def capture_frame(
        self,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Capture a frame from ZED camera.

        Returns:
            Tuple of (left_image, right_image, depth_map) as numpy arrays
        """
        if not self.is_opened:
            logger.error("ZED camera not opened")
            return None, None, None

        try:
            # Grab frame
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image
                self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
                left_img = self.image_left.get_data()[:, :, :3]  # Remove alpha channel

                # Retrieve right image
                self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)
                right_img = self.image_right.get_data()[:, :, :3]  # Remove alpha channel

                # Retrieve depth map
                self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
                depth = self.depth_map.get_data()

                return left_img, right_img, depth
            else:
                logger.warning("Failed to grab frame from ZED camera")
                return None, None, None

        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return None, None, None

    def capture_pointcloud(self) -> Optional[o3d.geometry.PointCloud]:
        """
        Capture point cloud from ZED camera.

        Returns:
            Open3D point cloud with XYZ coordinates and RGB colors
        """
        if not self.is_opened:
            logger.error("ZED camera not opened")
            return None

        try:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve point cloud with RGBA data
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
                point_cloud_data = self.point_cloud.get_data()

                # Convert to numpy array format
                height, width = point_cloud_data.shape[:2]
                points = point_cloud_data.reshape(-1, 4)

                # Extract XYZ coordinates
                xyz = points[:, :3]

                # Extract and unpack RGBA color data from 4th channel
                rgba_packed = points[:, 3].view(np.uint32)

                # Unpack RGBA: each 32-bit value contains 4 bytes (R, G, B, A)
                colors_rgba = np.zeros((len(rgba_packed), 4), dtype=np.uint8)
                colors_rgba[:, 0] = rgba_packed & 0xFF  # R
                colors_rgba[:, 1] = (rgba_packed >> 8) & 0xFF  # G
                colors_rgba[:, 2] = (rgba_packed >> 16) & 0xFF  # B
                colors_rgba[:, 3] = (rgba_packed >> 24) & 0xFF  # A

                # Extract RGB (ignore alpha) and normalize to [0, 1]
                colors_rgb = colors_rgba[:, :3].astype(np.float64) / 255.0

                # Filter out invalid points (NaN or inf)
                valid = np.isfinite(xyz).all(axis=1)
                valid_xyz = xyz[valid]
                valid_colors = colors_rgb[valid]

                # Create Open3D point cloud
                pcd = o3d.geometry.PointCloud()

                if len(valid_xyz) > 0:
                    pcd.points = o3d.utility.Vector3dVector(valid_xyz)
                    pcd.colors = o3d.utility.Vector3dVector(valid_colors)

                return pcd
            else:
                logger.warning("Failed to grab frame for point cloud")
                return None

        except Exception as e:
            logger.error(f"Error capturing point cloud: {e}")
            return None

    def get_camera_info(self) -> Dict[str, Any]:
        """Get ZED camera information and calibration parameters."""
        if not self.is_opened:
            return {}

        try:
            info = self.zed.get_camera_information()
            calibration = info.camera_configuration.calibration_parameters

            # In ZED SDK 4.0+, the baseline calculation has changed
            # Try to get baseline from the stereo parameters
            try:
                # Method 1: Try to get from stereo parameters if available
                if hasattr(calibration, "getCameraBaseline"):
                    baseline = calibration.getCameraBaseline()
                else:
                    # Method 2: Calculate from left and right camera positions
                    # The baseline is the distance between left and right cameras
                    left_cam = calibration.left_cam
                    right_cam = calibration.right_cam

                    # Try different ways to get baseline in SDK 4.0+
                    if hasattr(info.camera_configuration, "calibration_parameters_raw"):
                        # Use raw calibration if available
                        raw_calib = info.camera_configuration.calibration_parameters_raw
                        if hasattr(raw_calib, "T"):
                            baseline = abs(raw_calib.T[0])
                        else:
                            baseline = 0.12  # Default ZED-M baseline approximation
                    else:
                        # Use default baseline for ZED-M
                        baseline = 0.12  # ZED-M baseline is approximately 120mm
            except:
                baseline = 0.12  # Fallback to approximate ZED-M baseline

            return {
                "model": str(info.camera_model),
                "serial_number": info.serial_number,
                "firmware": info.camera_configuration.firmware_version,
                "resolution": {
                    "width": info.camera_configuration.resolution.width,
                    "height": info.camera_configuration.resolution.height,
                },
                "fps": info.camera_configuration.fps,
                "left_cam": {
                    "fx": calibration.left_cam.fx,
                    "fy": calibration.left_cam.fy,
                    "cx": calibration.left_cam.cx,
                    "cy": calibration.left_cam.cy,
                    "k1": calibration.left_cam.disto[0],
                    "k2": calibration.left_cam.disto[1],
                    "p1": calibration.left_cam.disto[2],
                    "p2": calibration.left_cam.disto[3],
                    "k3": calibration.left_cam.disto[4],
                },
                "right_cam": {
                    "fx": calibration.right_cam.fx,
                    "fy": calibration.right_cam.fy,
                    "cx": calibration.right_cam.cx,
                    "cy": calibration.right_cam.cy,
                    "k1": calibration.right_cam.disto[0],
                    "k2": calibration.right_cam.disto[1],
                    "p1": calibration.right_cam.disto[2],
                    "p2": calibration.right_cam.disto[3],
                    "k3": calibration.right_cam.disto[4],
                },
                "baseline": baseline,
            }
        except Exception as e:
            logger.error(f"Error getting camera info: {e}")
            return {}

    def calculate_intrinsics(self):
        """Calculate camera intrinsics from ZED calibration."""
        info = self.get_camera_info()
        if not info:
            return super().calculate_intrinsics()

        left_cam = info.get("left_cam", {})
        resolution = info.get("resolution", {})

        return {
            "focal_length_x": left_cam.get("fx", 0),
            "focal_length_y": left_cam.get("fy", 0),
            "principal_point_x": left_cam.get("cx", 0),
            "principal_point_y": left_cam.get("cy", 0),
            "baseline": info.get("baseline", 0),
            "resolution_width": resolution.get("width", 0),
            "resolution_height": resolution.get("height", 0),
        }

    def __enter__(self):
        """Context manager entry."""
        if not self.open():
            raise RuntimeError("Failed to open ZED camera")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()

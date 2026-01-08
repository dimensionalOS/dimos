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

"""
Perception Module

Provides interface for camera data retrieval and external grasp planning services.
"""

import base64
from dataclasses import dataclass, field
import json
import logging
import threading
import time
from typing import Any, Literal, Optional

import cv2
import numpy as np
import reactivex as rx
from reactivex import operators as ops
import requests
from scipy.spatial.transform import Rotation as R

from dimos.core import Module, rpc
from dimos.core.module import ModuleConfig
from dimos.hardware.sensors.camera.depthai.camera import DepthAI, DepthAIConfig
from dimos.hardware.sensors.camera.spec import StereoCameraHardware
from dimos.msgs.sensor_msgs import CameraInfo, Image

logger = logging.getLogger(__name__)


@dataclass
class PerceptionModuleConfig(ModuleConfig):
    grasp_service_url: str = "http://localhost:8080/grasp"
    calibration_file: str = "/home/ruthwik/Documents/dimos/calibration_result.json"
    # DepthAI camera hardware configuration
    fps: int = 30
    rgb_resolution: Literal["1080p", "4k", "720p"] = "1080p"
    mono_resolution: Literal["400p", "720p", "800p"] = "720p"


class PerceptionModule(Module):
    """Module for perception and grasp planning.

    Directly connects to DepthAI camera hardware to get RGB and depth images,
    and exposes RPC methods for grasp planning using the live camera feed.
    """

    rpc_calls: list[str] = []

    default_config = PerceptionModuleConfig

    # Camera hardware - directly connected DepthAI camera
    hardware: StereoCameraHardware[DepthAIConfig] | None = None

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        # Thread-safe storage for latest data
        self._lock = threading.Lock()
        self._last_color: np.ndarray | None = None
        self._last_depth: np.ndarray | None = None
        self._last_camera_K: list[list[float]] | None = None

        # Subscription tracking for camera streams
        self._rgb_sub = None
        self._depth_sub = None

        # Load Calibration
        self.T_base_camera: np.ndarray | None = None
        try:
            with open(self.config.calibration_file) as f:
                calib_data = json.load(f)
                self.T_base_camera = np.array(calib_data["T_base_camera"])
                logger.info(f"Loaded T_base_camera from {self.config.calibration_file}")
        except Exception as e:
            logger.error(f"Failed to load calibration file: {e}")

    @rpc
    def start(self) -> None:
        """Start the perception module and connect directly to DepthAI camera."""
        # Check if already started
        if self._rgb_sub is not None:
            logger.info("PerceptionModule already started")
            return

        # Initialize DepthAI camera hardware
        try:
            # Pass config fields directly as keyword arguments
            self.hardware = DepthAI(
                fps=self.config.fps,
                rgb_resolution=self.config.rgb_resolution,
                mono_resolution=self.config.mono_resolution,
            )
            logger.info("Initialized DepthAI camera hardware")

            # Start the camera hardware to begin emitting images
            self.hardware.start()
            logger.info("Started DepthAI camera hardware")
        except Exception as e:
            logger.error(f"Failed to initialize or start DepthAI camera: {e}")
            return

        # Subscribe to camera streams
        try:
            self._rgb_sub = self.hardware.image_stream().subscribe(self._on_hardware_image)
            self._disposables.add(self._rgb_sub)
            logger.info("Subscribed to DepthAI color_image stream")

            self._depth_sub = self.hardware.depth_stream().subscribe(self._on_hardware_depth)
            self._disposables.add(self._depth_sub)
            logger.info("Subscribed to DepthAI depth_image stream")
        except Exception as e:
            logger.error(f"Failed to subscribe to camera streams: {e}")
            return

        # Extract camera info from hardware (after start, as it's initialized during start)
        try:
            if hasattr(self.hardware, "camera_info"):
                info = self.hardware.camera_info
                if hasattr(info, "K") and len(info.K) == 9:
                    with self._lock:
                        self._last_camera_K = [
                            [info.K[0], info.K[1], info.K[2]],
                            [info.K[3], info.K[4], info.K[5]],
                            [info.K[6], info.K[7], info.K[8]],
                        ]
                    logger.info("Extracted camera matrix K from hardware")
        except Exception as e:
            logger.warning(f"Failed to extract camera info: {e}")

        logger.info(f"PerceptionModule started with planner: {self.config.grasp_service_url}")

    @rpc
    def stop(self) -> None:
        """Stop the perception module and camera hardware."""
        # Dispose subscriptions
        if self._rgb_sub:
            self._rgb_sub.dispose()
            self._rgb_sub = None
        if self._depth_sub:
            self._depth_sub.dispose()
            self._depth_sub = None

        # Stop camera hardware
        if self.hardware and hasattr(self.hardware, "stop"):
            try:
                self.hardware.stop()
                logger.info("Stopped DepthAI camera hardware")
            except Exception as e:
                logger.warning(f"Error stopping camera hardware: {e}")

        super().stop()

    def _on_hardware_image(self, data: Any) -> None:
        """Handle incoming RGB image data from DepthAI camera."""
        try:
            # Extract numpy array from Image message
            if hasattr(data, "data"):
                # Image message has .data property that returns numpy array
                img_data = data.data
                if isinstance(img_data, np.ndarray):
                    with self._lock:
                        # Make a copy to avoid reference issues
                        self._last_color = img_data.copy()
            elif isinstance(data, np.ndarray):
                with self._lock:
                    self._last_color = data.copy()
        except Exception as e:
            logger.error(f"Error processing color image: {e}")

    def _on_hardware_depth(self, data: Any) -> None:
        """Handle incoming depth data from DepthAI camera."""
        try:
            # Extract numpy array from Image message
            if hasattr(data, "data"):
                # Image message has .data property that returns numpy array
                depth_data = data.data
                if isinstance(depth_data, np.ndarray):
                    with self._lock:
                        # Make a copy to avoid reference issues
                        self._last_depth = depth_data.copy()
            elif isinstance(data, np.ndarray):
                with self._lock:
                    self._last_depth = data.copy()
        except Exception as e:
            logger.error(f"Error processing depth image: {e}")

    @rpc
    def get_images(self) -> dict[str, Any]:
        """Get latest camera data.

        Returns image metadata (shape, dtype) instead of full arrays to avoid
        RPC serialization timeouts with large images.
        """
        with self._lock:
            rgb_shape = self._last_color.shape if self._last_color is not None else None
            rgb_dtype = str(self._last_color.dtype) if self._last_color is not None else None
            depth_shape = self._last_depth.shape if self._last_depth is not None else None
            depth_dtype = str(self._last_depth.dtype) if self._last_depth is not None else None
            K = list(self._last_camera_K) if self._last_camera_K is not None else None

        if rgb_shape is None:
            logger.warning("get_images: No RGB image available.")

        return {
            "rgb_shape": rgb_shape,
            "rgb_dtype": rgb_dtype,
            "depth_shape": depth_shape,
            "depth_dtype": depth_dtype,
            "K": K,
            "has_rgb": rgb_shape is not None,
            "has_depth": depth_shape is not None,
        }

    @rpc
    def get_grasp_pose(
        self,
        label: str,
        bbox: list[int] | None = None,
        use_box_prompt: bool = False,
        filter_collisions: bool = True,
        gripper_type: str = "robotiq_2f_140",
    ) -> dict[str, Any]:
        """Send request to grasp planner planner with current images and return best grasp in BASE frame.

        Returns:
            Dict containing best grasp pose or error.
            Structure: {"transform": [x, y, z, roll, pitch, yaw], "score": float, ...}
        """
        if self.T_base_camera is None:
            err = "Calibration T_base_camera not loaded"
            logger.error(err)
            return {"error": err}

        # 1. Capture Data
        with self._lock:
            if self._last_color is None:
                return {"error": "No RGB image"}
            if self._last_depth is None:
                return {"error": "No Depth image"}
            if self._last_camera_K is None:
                # Late check for K from hardware
                if self.hardware and hasattr(self.hardware, "camera_info"):
                    info = self.hardware.camera_info
                    if hasattr(info, "K") and len(info.K) == 9:
                        self._last_camera_K = [
                            [info.K[0], info.K[1], info.K[2]],
                            [info.K[3], info.K[4], info.K[5]],
                            [info.K[6], info.K[7], info.K[8]],
                        ]
            if self._last_camera_K is None:
                return {"error": "No camera matrix K"}

            rgb_img = self._last_color.copy()
            depth_img = self._last_depth.copy()
            K = self._last_camera_K

        # 2. Encode
        try:
            # RGB: encode as JPEG
            success_rgb, encoded_rgb = cv2.imencode(".jpg", rgb_img)
            if not success_rgb:
                return {"error": "RGB encode failed"}
            rgb_b64 = base64.b64encode(encoded_rgb).decode("utf-8")

            # Depth: encode as raw float32 bytes (meters, not PNG)
            # Server expects raw float32 bytes, not PNG-encoded data
            # DepthAI provides depth as uint16 in millimeters, convert to float32 in meters
            if depth_img.dtype == np.uint16:
                # Convert from uint16 millimeters to float32 meters
                depth_float32 = depth_img.astype(np.float32) / 1000.0
            elif depth_img.dtype == np.float32:
                # Already float32, but check if it's in meters or millimeters
                # If max value > 100, assume it's in millimeters and convert
                if depth_img.max() > 100.0:
                    depth_float32 = depth_img / 1000.0
                else:
                    depth_float32 = depth_img
            elif depth_img.dtype == np.float64:
                # Convert float64 to float32, and check units
                if depth_img.max() > 100.0:
                    depth_float32 = (depth_img / 1000.0).astype(np.float32)
                else:
                    depth_float32 = depth_img.astype(np.float32)
            else:
                # Unknown type, try to convert to float32
                depth_float32 = depth_img.astype(np.float32)

            # Encode depth as raw float32 bytes (not PNG)
            depth_b64 = base64.b64encode(depth_float32.tobytes()).decode("utf-8")
        except Exception as e:
            return {"error": f"Encoding error: {e}"}

        # 3. Payload
        # Always use [0, 0, 0, 0] for bbox (not using bounding boxes)
        payload = {
            "image_rgb_b64": rgb_b64,
            "depth_b64": depth_b64,
            "K": K,
            "label": label,
            "bbox": [0, 0, 0, 0],  # Always [0, 0, 0, 0] - not using bounding boxes
            "use_box_prompt": use_box_prompt,
            "filter_collisions": filter_collisions,
            "gripper_type": gripper_type,
        }

        # 5. Request
        try:
            # Increased timeout to 120 seconds for grasp planning with large images
            resp = requests.post(self.config.grasp_service_url, json=payload, timeout=120.0)
            resp.raise_for_status()
            data = resp.json()
        except requests.exceptions.HTTPError as e:
            # Include response details for HTTP errors
            try:
                error_detail = resp.text[:500]  # First 500 chars of response
                return {
                    "error": f"Service request failed: {e} (Status: {resp.status_code}, Response: {error_detail})"
                }
            except:
                return {"error": f"Service request failed: {e} (Status: {resp.status_code})"}
        except Exception as e:
            return {"error": f"Service request failed: {e}"}

        # 6. Process Response
        if "grasps" not in data or not data["grasps"]:
            return {"error": "No grasps returned"}

        # Find best grasp
        best_grasp = max(data["grasps"], key=lambda g: g.get("score", -1))

        # Transform Logic
        try:
            # T_camera_grasp (4x4)
            T_cg_flat = best_grasp.get("transform")
            if not T_cg_flat or len(T_cg_flat) != 16:
                return {"error": "Invalid grasp transform format"}

            T_cg = np.array(T_cg_flat).reshape(4, 4)

            # T_base_grasp = T_base_camera * T_camera_grasp
            T_bg = self.T_base_camera @ T_cg

            # Extract xyz
            x, y, z = T_bg[0, 3], T_bg[1, 3], T_bg[2, 3]

            # Extract RPY
            rot_mat = T_bg[:3, :3]
            r = R.from_matrix(rot_mat)
            roll, pitch, yaw = r.as_euler("xyz", degrees=False)

            return {
                "transform": [x, y, z, roll, pitch, yaw],
                "score": best_grasp.get("score"),
                "gripper_type": data.get("gripper_type"),
            }

        except Exception as e:
            logger.error(f"Transformation error: {e}")
            return {"error": f"Transformation error: {e}"}


# Expose blueprint
perception_module = PerceptionModule.blueprint

__all__ = ["PerceptionModule", "perception_module"]

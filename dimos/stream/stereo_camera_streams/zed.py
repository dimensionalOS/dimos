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

import logging
import time
from typing import Optional, Dict

from reactivex import Observable, create
from reactivex import operators as ops

from dimos.hardware.zed_camera import ZEDCamera

try:
    import pyzed.sl as sl
except ImportError:
    sl = None
    logging.warning("ZED SDK not found. Please install pyzed to use ZED camera functionality.")

logger = logging.getLogger(__name__)


class ZEDCameraStream:
    """ZED Camera stream that provides RGB and depth data as observables."""

    def __init__(
        self,
        camera_id: int = 0,
        resolution: Optional["sl.RESOLUTION"] = None,
        depth_mode: Optional["sl.DEPTH_MODE"] = None,
        fps: int = 30,
    ):
        """
        Initialize ZED camera stream.

        Args:
            camera_id: Camera ID (0 for first ZED)
            resolution: ZED camera resolution (defaults to HD720)
            depth_mode: Depth computation mode (defaults to NEURAL)
            fps: Camera frame rate (default: 30)
        """
        if sl is None:
            raise ImportError("ZED SDK not installed. Please install pyzed package.")

        self.camera_id = camera_id
        self.fps = fps

        # Set default values if not provided
        if resolution is None:
            resolution = sl.RESOLUTION.HD720
        if depth_mode is None:
            depth_mode = sl.DEPTH_MODE.NEURAL

        self.resolution = resolution
        self.depth_mode = depth_mode

        # Initialize ZED camera
        self.zed_camera = ZEDCamera(
            camera_id=camera_id, resolution=resolution, depth_mode=depth_mode, fps=fps
        )

        self.is_opened = False

    def _initialize_camera(self) -> None:
        """Initialize the ZED camera if not already initialized."""
        if not self.is_opened:
            if not self.zed_camera.open():
                raise RuntimeError(f"Failed to open ZED camera {self.camera_id}")
            self.is_opened = True
            logger.info(f"ZED camera {self.camera_id} opened successfully")

    def create_stream(self) -> Observable:
        """
        Create an observable stream of RGB and depth frames.

        Returns:
            Observable: An observable emitting dictionaries with 'rgb' and 'depth' keys.
        """

        def emit_frames(observer, scheduler):
            try:
                # Initialize camera
                if not self.is_opened:
                    self._initialize_camera()

                while True:
                    # Capture frame directly
                    left_img, right_img, depth_img = self.zed_camera.capture_frame()

                    if left_img is not None and depth_img is not None:
                        frame_data = {
                            "rgb": left_img,
                            "depth": depth_img,
                            "right": right_img,
                            "timestamp": time.time(),
                        }

                        observer.on_next(frame_data)

            except Exception as e:
                logger.error(f"Error during ZED frame emission: {e}")
                observer.on_error(e)
            finally:
                # Clean up resources
                self._cleanup_camera()
                observer.on_completed()

        return create(emit_frames).pipe(
            ops.share(),  # Share the stream among multiple subscribers
        )

    def get_camera_info(self) -> Dict[str, float]:
        """
        Get ZED camera intrinsics (fx, fy, cx, cy).

        Returns:
            Dictionary containing camera intrinsics: fx, fy, cx, cy
        """
        if not self.is_opened:
            self._initialize_camera()

        try:
            camera_info = self.zed_camera.get_camera_info()
            left_cam = camera_info.get("left_cam", {})

            return {
                "fx": left_cam.get("fx", 0.0),
                "fy": left_cam.get("fy", 0.0),
                "cx": left_cam.get("cx", 0.0),
                "cy": left_cam.get("cy", 0.0),
            }
        except Exception as e:
            logger.error(f"Error getting camera info: {e}")
            return {}

    def _cleanup_camera(self) -> None:
        """Clean up camera resources."""
        if self.is_opened:
            self.zed_camera.close()
            self.is_opened = False
            logger.info("ZED camera resources cleaned up")

    def cleanup(self) -> None:
        """Clean up all resources."""
        self._cleanup_camera()

    def __enter__(self):
        """Context manager entry."""
        self._initialize_camera()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self._cleanup_camera()

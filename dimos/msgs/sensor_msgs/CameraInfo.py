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

from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

# Import LCM types
from dimos_lcm.sensor_msgs import CameraInfo as LCMCameraInfo
from dimos_lcm.std_msgs.Header import Header

# Import ROS types
try:
    from sensor_msgs.msg import CameraInfo as ROSCameraInfo
    from std_msgs.msg import Header as ROSHeader
    from sensor_msgs.msg import RegionOfInterest as ROSRegionOfInterest

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

from dimos.types.timestamped import Timestamped


class CameraInfo(Timestamped):
    """Camera calibration information message."""

    msg_name = "sensor_msgs.CameraInfo"

    def __init__(
        self,
        height: int = 0,
        width: int = 0,
        distortion_model: str = "",
        D: Optional[List[float]] = None,
        K: Optional[List[float]] = None,
        R: Optional[List[float]] = None,
        P: Optional[List[float]] = None,
        binning_x: int = 0,
        binning_y: int = 0,
        frame_id: str = "",
        ts: Optional[float] = None,
    ):
        """Initialize CameraInfo.

        Args:
            height: Image height
            width: Image width
            distortion_model: Name of distortion model (e.g., "plumb_bob")
            D: Distortion coefficients
            K: 3x3 intrinsic camera matrix
            R: 3x3 rectification matrix
            P: 3x4 projection matrix
            binning_x: Horizontal binning
            binning_y: Vertical binning
            frame_id: Frame ID
            ts: Timestamp
        """
        self.ts = ts if ts is not None else time.time()
        self.frame_id = frame_id
        self.height = height
        self.width = width
        self.distortion_model = distortion_model

        # Initialize distortion coefficients
        self.D = D if D is not None else []

        # Initialize 3x3 intrinsic camera matrix (row-major)
        self.K = K if K is not None else [0.0] * 9

        # Initialize 3x3 rectification matrix (row-major)
        self.R = R if R is not None else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Initialize 3x4 projection matrix (row-major)
        self.P = P if P is not None else [0.0] * 12

        self.binning_x = binning_x
        self.binning_y = binning_y

        # Region of interest (not used in basic implementation)
        self.roi_x_offset = 0
        self.roi_y_offset = 0
        self.roi_height = 0
        self.roi_width = 0
        self.roi_do_rectify = False

    def get_K_matrix(self) -> np.ndarray:
        """Get intrinsic matrix as numpy array."""
        return np.array(self.K, dtype=np.float64).reshape(3, 3)

    def get_P_matrix(self) -> np.ndarray:
        """Get projection matrix as numpy array."""
        return np.array(self.P, dtype=np.float64).reshape(3, 4)

    def get_R_matrix(self) -> np.ndarray:
        """Get rectification matrix as numpy array."""
        return np.array(self.R, dtype=np.float64).reshape(3, 3)

    def get_D_coeffs(self) -> np.ndarray:
        """Get distortion coefficients as numpy array."""
        return np.array(self.D, dtype=np.float64)

    def set_K_matrix(self, K: np.ndarray):
        """Set intrinsic matrix from numpy array."""
        if K.shape != (3, 3):
            raise ValueError(f"K matrix must be 3x3, got {K.shape}")
        self.K = K.flatten().tolist()

    def set_P_matrix(self, P: np.ndarray):
        """Set projection matrix from numpy array."""
        if P.shape != (3, 4):
            raise ValueError(f"P matrix must be 3x4, got {P.shape}")
        self.P = P.flatten().tolist()

    def set_R_matrix(self, R: np.ndarray):
        """Set rectification matrix from numpy array."""
        if R.shape != (3, 3):
            raise ValueError(f"R matrix must be 3x3, got {R.shape}")
        self.R = R.flatten().tolist()

    def set_D_coeffs(self, D: np.ndarray):
        """Set distortion coefficients from numpy array."""
        self.D = D.flatten().tolist()

    def lcm_encode(self) -> bytes:
        """Convert to LCM CameraInfo message."""
        msg = LCMCameraInfo()

        # Header
        msg.header = Header()
        msg.header.seq = 0
        msg.header.frame_id = self.frame_id
        msg.header.stamp.sec = int(self.ts)
        msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)

        # Image dimensions
        msg.height = self.height
        msg.width = self.width

        # Distortion model
        msg.distortion_model = self.distortion_model

        # Distortion coefficients
        msg.D_length = len(self.D)
        msg.D = self.D

        # Camera matrices (all stored as row-major)
        msg.K = self.K
        msg.R = self.R
        msg.P = self.P

        # Binning
        msg.binning_x = self.binning_x
        msg.binning_y = self.binning_y

        # ROI
        msg.roi.x_offset = self.roi_x_offset
        msg.roi.y_offset = self.roi_y_offset
        msg.roi.height = self.roi_height
        msg.roi.width = self.roi_width
        msg.roi.do_rectify = self.roi_do_rectify

        return msg.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes) -> "CameraInfo":
        """Decode from LCM CameraInfo bytes."""
        msg = LCMCameraInfo.lcm_decode(data)

        # Extract timestamp
        ts = msg.header.stamp.sec + msg.header.stamp.nsec / 1e9 if hasattr(msg, "header") else None

        camera_info = cls(
            height=msg.height,
            width=msg.width,
            distortion_model=msg.distortion_model,
            D=list(msg.D) if msg.D_length > 0 else [],
            K=list(msg.K),
            R=list(msg.R),
            P=list(msg.P),
            binning_x=msg.binning_x,
            binning_y=msg.binning_y,
            frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
            ts=ts,
        )

        # Set ROI if present
        if hasattr(msg, "roi"):
            camera_info.roi_x_offset = msg.roi.x_offset
            camera_info.roi_y_offset = msg.roi.y_offset
            camera_info.roi_height = msg.roi.height
            camera_info.roi_width = msg.roi.width
            camera_info.roi_do_rectify = msg.roi.do_rectify

        return camera_info

    @classmethod
    def from_ros_msg(cls, ros_msg: "ROSCameraInfo") -> "CameraInfo":
        """Create CameraInfo from ROS sensor_msgs/CameraInfo message.

        Args:
            ros_msg: ROS CameraInfo message

        Returns:
            CameraInfo instance
        """
        if not ROS_AVAILABLE:
            raise ImportError("ROS packages not available. Cannot convert from ROS message.")

        # Extract timestamp
        ts = ros_msg.header.stamp.sec + ros_msg.header.stamp.nanosec / 1e9

        camera_info = cls(
            height=ros_msg.height,
            width=ros_msg.width,
            distortion_model=ros_msg.distortion_model,
            D=list(ros_msg.d),
            K=list(ros_msg.k),
            R=list(ros_msg.r),
            P=list(ros_msg.p),
            binning_x=ros_msg.binning_x,
            binning_y=ros_msg.binning_y,
            frame_id=ros_msg.header.frame_id,
            ts=ts,
        )

        # Set ROI
        camera_info.roi_x_offset = ros_msg.roi.x_offset
        camera_info.roi_y_offset = ros_msg.roi.y_offset
        camera_info.roi_height = ros_msg.roi.height
        camera_info.roi_width = ros_msg.roi.width
        camera_info.roi_do_rectify = ros_msg.roi.do_rectify

        return camera_info

    def to_ros_msg(self) -> "ROSCameraInfo":
        """Convert to ROS sensor_msgs/CameraInfo message.

        Returns:
            ROS CameraInfo message
        """
        if not ROS_AVAILABLE:
            raise ImportError("ROS packages not available. Cannot convert to ROS message.")

        ros_msg = ROSCameraInfo()

        # Set header
        ros_msg.header = ROSHeader()
        ros_msg.header.frame_id = self.frame_id
        ros_msg.header.stamp.sec = int(self.ts)
        ros_msg.header.stamp.nanosec = int((self.ts - int(self.ts)) * 1e9)

        # Image dimensions
        ros_msg.height = self.height
        ros_msg.width = self.width

        # Distortion model and coefficients
        ros_msg.distortion_model = self.distortion_model
        ros_msg.d = self.D

        # Camera matrices (all row-major)
        ros_msg.k = self.K
        ros_msg.r = self.R
        ros_msg.p = self.P

        # Binning
        ros_msg.binning_x = self.binning_x
        ros_msg.binning_y = self.binning_y

        # ROI
        ros_msg.roi = ROSRegionOfInterest()
        ros_msg.roi.x_offset = self.roi_x_offset
        ros_msg.roi.y_offset = self.roi_y_offset
        ros_msg.roi.height = self.roi_height
        ros_msg.roi.width = self.roi_width
        ros_msg.roi.do_rectify = self.roi_do_rectify

        return ros_msg

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"CameraInfo(height={self.height}, width={self.width}, "
            f"distortion_model='{self.distortion_model}', "
            f"frame_id='{self.frame_id}', ts={self.ts})"
        )

    def __str__(self) -> str:
        """Human-readable string."""
        return (
            f"CameraInfo:\n"
            f"  Resolution: {self.width}x{self.height}\n"
            f"  Distortion model: {self.distortion_model}\n"
            f"  Frame ID: {self.frame_id}\n"
            f"  Binning: {self.binning_x}x{self.binning_y}"
        )

    def __eq__(self, other) -> bool:
        """Check if two CameraInfo messages are equal."""
        if not isinstance(other, CameraInfo):
            return False

        return (
            self.height == other.height
            and self.width == other.width
            and self.distortion_model == other.distortion_model
            and self.D == other.D
            and self.K == other.K
            and self.R == other.R
            and self.P == other.P
            and self.binning_x == other.binning_x
            and self.binning_y == other.binning_y
            and self.frame_id == other.frame_id
        )

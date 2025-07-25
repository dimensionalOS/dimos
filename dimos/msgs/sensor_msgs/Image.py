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

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

import cv2
import numpy as np

# Import LCM types
from dimos_lcm.sensor_msgs.Image import Image as LCMImage
from dimos_lcm.std_msgs.Header import Header

from dimos.types.timestamped import Timestamped


class ImageFormat(Enum):
    """Supported image formats."""

    BGR = "bgr8"
    RGB = "rgb8"
    RGBA = "rgba8"
    BGRA = "bgra8"
    GRAY = "mono8"
    GRAY16 = "mono16"


@dataclass
class Image(Timestamped):
    """Standardized image type with LCM integration."""

    msg_name = "sensor_msgs.Image"
    data: np.ndarray
    format: ImageFormat = field(default=ImageFormat.BGR)
    frame_id: str = field(default="")
    ts: float = field(default_factory=time.time)

    def __post_init__(self):
        """Validate image data and format."""
        if self.data is None:
            raise ValueError("Image data cannot be None")

        if not isinstance(self.data, np.ndarray):
            raise ValueError("Image data must be a numpy array")

        if len(self.data.shape) < 2:
            raise ValueError("Image data must be at least 2D")

        # Ensure data is contiguous for efficient operations
        if not self.data.flags["C_CONTIGUOUS"]:
            self.data = np.ascontiguousarray(self.data)

    @property
    def height(self) -> int:
        """Get image height."""
        return self.data.shape[0]

    @property
    def width(self) -> int:
        """Get image width."""
        return self.data.shape[1]

    @property
    def channels(self) -> int:
        """Get number of channels."""
        if len(self.data.shape) == 2:
            return 1
        elif len(self.data.shape) == 3:
            return self.data.shape[2]
        else:
            raise ValueError("Invalid image dimensions")

    @property
    def shape(self) -> Tuple[int, ...]:
        """Get image shape."""
        return self.data.shape

    @property
    def dtype(self) -> np.dtype:
        """Get image data type."""
        return self.data.dtype

    def copy(self) -> "Image":
        """Create a deep copy of the image."""
        return self.__class__(
            data=self.data.copy(),
            format=self.format,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    @classmethod
    def from_opencv(
        cls, cv_image: np.ndarray, format: ImageFormat = ImageFormat.BGR, **kwargs
    ) -> "Image":
        """Create Image from OpenCV image array."""
        return cls(data=cv_image, format=format, **kwargs)

    @classmethod
    def from_numpy(
        cls, np_image: np.ndarray, format: ImageFormat = ImageFormat.BGR, **kwargs
    ) -> "Image":
        """Create Image from numpy array."""
        return cls(data=np_image, format=format, **kwargs)

    @classmethod
    def from_file(cls, filepath: str, format: ImageFormat = ImageFormat.BGR) -> "Image":
        """Load image from file."""
        # OpenCV loads as BGR by default
        cv_image = cv2.imread(filepath, cv2.IMREAD_UNCHANGED)
        if cv_image is None:
            raise ValueError(f"Could not load image from {filepath}")

        # Detect format based on channels
        if len(cv_image.shape) == 2:
            detected_format = ImageFormat.GRAY
        elif cv_image.shape[2] == 3:
            detected_format = ImageFormat.BGR  # OpenCV default
        elif cv_image.shape[2] == 4:
            detected_format = ImageFormat.BGRA
        else:
            detected_format = format

        return cls(data=cv_image, format=detected_format)

    def to_opencv(self) -> np.ndarray:
        """Convert to OpenCV-compatible array (BGR format)."""
        if self.format == ImageFormat.BGR:
            return self.data
        elif self.format == ImageFormat.RGB:
            return cv2.cvtColor(self.data, cv2.COLOR_RGB2BGR)
        elif self.format == ImageFormat.RGBA:
            return cv2.cvtColor(self.data, cv2.COLOR_RGBA2BGR)
        elif self.format == ImageFormat.BGRA:
            return cv2.cvtColor(self.data, cv2.COLOR_BGRA2BGR)
        elif self.format == ImageFormat.GRAY:
            return self.data
        elif self.format == ImageFormat.GRAY16:
            return self.data
        else:
            raise ValueError(f"Unsupported format conversion: {self.format}")

    def to_rgb(self) -> "Image":
        """Convert image to RGB format."""
        if self.format == ImageFormat.RGB:
            return self.copy()
        elif self.format == ImageFormat.BGR:
            rgb_data = cv2.cvtColor(self.data, cv2.COLOR_BGR2RGB)
        elif self.format == ImageFormat.RGBA:
            return self.copy()  # Already RGB with alpha
        elif self.format == ImageFormat.BGRA:
            rgb_data = cv2.cvtColor(self.data, cv2.COLOR_BGRA2RGBA)
        elif self.format == ImageFormat.GRAY:
            rgb_data = cv2.cvtColor(self.data, cv2.COLOR_GRAY2RGB)
        elif self.format == ImageFormat.GRAY16:
            # Convert 16-bit grayscale to 8-bit then to RGB
            gray8 = (self.data / 256).astype(np.uint8)
            rgb_data = cv2.cvtColor(gray8, cv2.COLOR_GRAY2RGB)
        else:
            raise ValueError(f"Unsupported format conversion from {self.format} to RGB")

        return self.__class__(
            data=rgb_data,
            format=ImageFormat.RGB if self.format != ImageFormat.BGRA else ImageFormat.RGBA,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def to_bgr(self) -> "Image":
        """Convert image to BGR format."""
        if self.format == ImageFormat.BGR:
            return self.copy()
        elif self.format == ImageFormat.RGB:
            bgr_data = cv2.cvtColor(self.data, cv2.COLOR_RGB2BGR)
        elif self.format == ImageFormat.RGBA:
            bgr_data = cv2.cvtColor(self.data, cv2.COLOR_RGBA2BGR)
        elif self.format == ImageFormat.BGRA:
            bgr_data = cv2.cvtColor(self.data, cv2.COLOR_BGRA2BGR)
        elif self.format == ImageFormat.GRAY:
            bgr_data = cv2.cvtColor(self.data, cv2.COLOR_GRAY2BGR)
        elif self.format == ImageFormat.GRAY16:
            # Convert 16-bit grayscale to 8-bit then to BGR
            gray8 = (self.data / 256).astype(np.uint8)
            bgr_data = cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(f"Unsupported format conversion from {self.format} to BGR")

        return self.__class__(
            data=bgr_data,
            format=ImageFormat.BGR,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def to_grayscale(self) -> "Image":
        """Convert image to grayscale."""
        if self.format == ImageFormat.GRAY:
            return self.copy()
        elif self.format == ImageFormat.GRAY16:
            return self.copy()
        elif self.format == ImageFormat.BGR:
            gray_data = cv2.cvtColor(self.data, cv2.COLOR_BGR2GRAY)
        elif self.format == ImageFormat.RGB:
            gray_data = cv2.cvtColor(self.data, cv2.COLOR_RGB2GRAY)
        elif self.format == ImageFormat.RGBA:
            gray_data = cv2.cvtColor(self.data, cv2.COLOR_RGBA2GRAY)
        elif self.format == ImageFormat.BGRA:
            gray_data = cv2.cvtColor(self.data, cv2.COLOR_BGRA2GRAY)
        else:
            raise ValueError(f"Unsupported format conversion from {self.format} to grayscale")

        return self.__class__(
            data=gray_data,
            format=ImageFormat.GRAY,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def resize(self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR) -> "Image":
        """Resize the image to the specified dimensions."""
        resized_data = cv2.resize(self.data, (width, height), interpolation=interpolation)

        return self.__class__(
            data=resized_data,
            format=self.format,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def crop(self, x: int, y: int, width: int, height: int) -> "Image":
        """Crop the image to the specified region."""
        # Ensure crop region is within image bounds
        x = max(0, min(x, self.width))
        y = max(0, min(y, self.height))
        x2 = min(x + width, self.width)
        y2 = min(y + height, self.height)

        cropped_data = self.data[y:y2, x:x2]

        return self.__class__(
            data=cropped_data,
            format=self.format,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def save(self, filepath: str) -> bool:
        """Save image to file."""
        # Convert to OpenCV format for saving
        cv_image = self.to_opencv()
        return cv2.imwrite(filepath, cv_image)

    def lcm_encode(self, frame_id: Optional[str] = None) -> LCMImage:
        """Convert to LCM Image message."""
        msg = LCMImage()

        # Header
        msg.header = Header()
        msg.header.seq = 0  # Initialize sequence number
        msg.header.frame_id = frame_id or self.frame_id

        # Set timestamp properly as Time object
        if self.ts is not None:
            msg.header.stamp.sec = int(self.ts)
            msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)
        else:
            current_time = time.time()
            msg.header.stamp.sec = int(current_time)
            msg.header.stamp.nsec = int((current_time - int(current_time)) * 1e9)

        # Image properties
        msg.height = self.height
        msg.width = self.width
        msg.encoding = self.format.value
        msg.is_bigendian = False  # Use little endian
        msg.step = self._get_row_step()

        # Image data
        image_bytes = self.data.tobytes()
        msg.data_length = len(image_bytes)
        msg.data = image_bytes

        return msg.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs) -> "Image":
        """Create Image from LCM Image message."""
        # Parse encoding to determine format and data type
        msg = LCMImage.lcm_decode(data)
        format_info = cls._parse_encoding(msg.encoding)

        # Convert bytes back to numpy array
        data = np.frombuffer(msg.data, dtype=format_info["dtype"])

        # Reshape to image dimensions
        if format_info["channels"] == 1:
            data = data.reshape((msg.height, msg.width))
        else:
            data = data.reshape((msg.height, msg.width, format_info["channels"]))

        return cls(
            data=data,
            format=format_info["format"],
            frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
            if hasattr(msg, "header") and msg.header.stamp.sec > 0
            else time.time(),
            **kwargs,
        )

    def _get_row_step(self) -> int:
        """Calculate row step (bytes per row)."""
        bytes_per_pixel = self._get_bytes_per_pixel()
        return self.width * bytes_per_pixel

    def _get_bytes_per_pixel(self) -> int:
        """Calculate bytes per pixel based on format and data type."""
        bytes_per_element = self.data.dtype.itemsize
        return self.channels * bytes_per_element

    @staticmethod
    def _parse_encoding(encoding: str) -> dict:
        """Parse LCM image encoding string to determine format and data type."""
        encoding_map = {
            "mono8": {"format": ImageFormat.GRAY, "dtype": np.uint8, "channels": 1},
            "mono16": {"format": ImageFormat.GRAY16, "dtype": np.uint16, "channels": 1},
            "rgb8": {"format": ImageFormat.RGB, "dtype": np.uint8, "channels": 3},
            "rgba8": {"format": ImageFormat.RGBA, "dtype": np.uint8, "channels": 4},
            "bgr8": {"format": ImageFormat.BGR, "dtype": np.uint8, "channels": 3},
            "bgra8": {"format": ImageFormat.BGRA, "dtype": np.uint8, "channels": 4},
        }

        if encoding not in encoding_map:
            raise ValueError(f"Unsupported encoding: {encoding}")

        return encoding_map[encoding]

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"Image(shape={self.shape}, format={self.format.value}, "
            f"dtype={self.dtype}, frame_id='{self.frame_id}', ts={self.ts})"
        )

    def __eq__(self, other) -> bool:
        """Check equality with another Image."""
        if not isinstance(other, Image):
            return False

        return (
            np.array_equal(self.data, other.data)
            and self.format == other.format
            and self.frame_id == other.frame_id
            and abs(self.ts - other.ts) < 1e-6
        )

    def __len__(self) -> int:
        """Return total number of pixels."""
        return self.height * self.width

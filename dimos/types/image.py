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

import base64
import struct
from typing import Optional, Tuple, Union, Any
import numpy as np
from PIL import Image as PILImage
import cv2
from datetime import datetime


# Image format constants
class ImageFormat:
    """Standard image format constants."""

    GRAYSCALE = "grayscale"
    RGB = "rgb"
    BGR = "bgr"
    RGBA = "rgba"
    BGRA = "bgra"


# Data type mappings for serialization
DTYPE2STR = {
    np.uint8: "u8",
    np.uint16: "u16",
    np.float32: "f32",
    np.float64: "f64",
    np.int8: "i8",
    np.int16: "i16",
    np.int32: "i32",
}

STR2DTYPE = {v: k for k, v in DTYPE2STR.items()}


def encode_image_array(arr: np.ndarray) -> dict:
    """Encode numpy array for serialization."""
    arr_c = np.ascontiguousarray(arr)
    payload = arr_c.tobytes()
    b64 = base64.b64encode(payload).decode("ascii")

    return {
        "shape": arr_c.shape,
        "dtype": DTYPE2STR[arr_c.dtype.type],
        "data": b64,
    }


def decode_image_array(encoded: dict) -> np.ndarray:
    """Decode numpy array from serialization."""
    payload = base64.b64decode(encoded["data"].encode("ascii"))
    dtype = STR2DTYPE[encoded["dtype"]]
    arr = np.frombuffer(payload, dtype=dtype)
    return arr.reshape(encoded["shape"])


class Image:
    """A wrapper around numpy arrays for image data with intuitive operations."""

    def __init__(
        self,
        data: Union[np.ndarray, str, PILImage.Image],
        format: str = ImageFormat.RGB,
        timestamp: Optional[datetime] = None,
        metadata: Optional[dict] = None,
    ):
        """Initialize Image from numpy array, file path, or PIL Image.

        Args:
            data: Image data as numpy array, file path string, or PIL Image
            format: Image format (rgb, bgr, grayscale, etc.)
            timestamp: Optional timestamp for the image
            metadata: Optional metadata dictionary
        """
        if isinstance(data, str):
            # Load from file path
            self._data = self._load_from_path(data)
            # Detect format from loaded data
            if len(self._data.shape) == 2:
                format = ImageFormat.GRAYSCALE
            elif self._data.shape[2] == 3:
                format = ImageFormat.RGB  # PIL loads as RGB by default
            elif self._data.shape[2] == 4:
                format = ImageFormat.RGBA
        elif isinstance(data, PILImage.Image):
            # Convert PIL Image to numpy
            self._data = np.array(data)
            # Detect format from PIL mode
            if data.mode == "L":
                format = ImageFormat.GRAYSCALE
            elif data.mode == "RGB":
                format = ImageFormat.RGB
            elif data.mode == "RGBA":
                format = ImageFormat.RGBA
        else:
            # Assume numpy array
            self._data = np.asarray(data)

        self.format = format
        self.timestamp = timestamp or datetime.now()
        self.metadata = metadata or {}

        # Validate data shape
        if len(self._data.shape) not in [2, 3]:
            raise ValueError(f"Image data must be 2D or 3D array, got shape {self._data.shape}")

    def _load_from_path(self, path: str) -> np.ndarray:
        """Load image from file path."""
        try:
            # Try with PIL first (supports more formats)
            pil_img = PILImage.open(path)
            return np.array(pil_img)
        except Exception:
            # Fallback to OpenCV
            img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if img is None:
                raise ValueError(f"Could not load image from {path}")
            return img

    @property
    def data(self) -> np.ndarray:
        """Get the underlying numpy array."""
        return self._data

    @property
    def shape(self) -> Tuple[int, ...]:
        """Get image shape (height, width) or (height, width, channels)."""
        return self._data.shape

    @property
    def height(self) -> int:
        """Get image height."""
        return self._data.shape[0]

    @property
    def width(self) -> int:
        """Get image width."""
        return self._data.shape[1]

    @property
    def channels(self) -> int:
        """Get number of channels."""
        return self._data.shape[2] if len(self._data.shape) == 3 else 1

    @property
    def dtype(self) -> np.dtype:
        """Get data type of the image."""
        return self._data.dtype

    @property
    def size(self) -> Tuple[int, int]:
        """Get image size as (width, height) tuple."""
        return (self.width, self.height)

    def serialize(self) -> dict:
        """Serialize the Image instance to a dictionary."""
        return {
            "type": "image",
            "data": encode_image_array(self._data),
            "format": self.format,
            "timestamp": self.timestamp.isoformat(),
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Image":
        """Create Image from serialized dictionary."""
        img_data = decode_image_array(data["data"])
        timestamp = datetime.fromisoformat(data["timestamp"])

        return cls(
            data=img_data,
            format=data["format"],
            timestamp=timestamp,
            metadata=data.get("metadata", {}),
        )

    def to_zenoh_binary(self) -> bytes:
        """High-performance binary serialization for Zenoh."""
        # Create header with image metadata
        format_bytes = self.format.encode("utf-8")[:16].ljust(16, b"\0")  # 16 bytes
        timestamp_us = int(self.timestamp.timestamp() * 1_000_000)  # microseconds

        # Pack header: format(16), timestamp(8), height(4), width(4), channels(4), dtype(1), reserved(3)
        header = struct.pack(
            "!16sQIIIB3x",
            format_bytes,
            timestamp_us,
            self.height,
            self.width,
            self.channels,
            list(DTYPE2STR.keys()).index(self._data.dtype.type),
        )

        # Ensure data is contiguous for efficient transfer
        img_bytes = np.ascontiguousarray(self._data).tobytes()

        return header + img_bytes

    @classmethod
    def from_zenoh_binary(cls, data: Union[bytes, Any]) -> "Image":
        """Reconstruct Image from binary Zenoh data.

        Args:
            data: Binary data from Zenoh (can be bytes or ZBytes)

        Returns:
            Image instance reconstructed from binary data
        """
        # Handle ZBytes from Zenoh automatically
        if hasattr(data, "to_bytes"):
            # Zenoh ZBytes object
            data_bytes = data.to_bytes()
        elif hasattr(data, "__bytes__"):
            # Object that can be converted to bytes
            data_bytes = bytes(data)
        else:
            # Assume it's already bytes
            data_bytes = data

        # Unpack header (40 bytes total)
        header_size = 40
        if len(data_bytes) < header_size:
            raise ValueError("Invalid binary data: too short for header")

        format_bytes, timestamp_us, height, width, channels, dtype_idx = struct.unpack(
            "!16sQIIIB3x", data_bytes[:header_size]
        )

        # Decode format string
        format_str = format_bytes.rstrip(b"\0").decode("utf-8")

        # Decode timestamp
        timestamp = datetime.fromtimestamp(timestamp_us / 1_000_000)

        # Decode dtype
        dtype = list(DTYPE2STR.keys())[dtype_idx]

        # Reconstruct image data
        img_bytes = data_bytes[header_size:]
        if channels == 1:
            shape = (height, width)
        else:
            shape = (height, width, channels)

        img_data = np.frombuffer(img_bytes, dtype=dtype).reshape(shape)

        return cls(
            data=img_data,
            format=format_str,
            timestamp=timestamp,
        )

    def convert_format(self, target_format: str) -> "Image":
        """Convert image to different format."""
        if self.format == target_format:
            return self.copy()

        converted_data = self._data.copy()

        # Handle common conversions
        if self.format == ImageFormat.BGR and target_format == ImageFormat.RGB:
            converted_data = cv2.cvtColor(converted_data, cv2.COLOR_BGR2RGB)
        elif self.format == ImageFormat.RGB and target_format == ImageFormat.BGR:
            converted_data = cv2.cvtColor(converted_data, cv2.COLOR_RGB2BGR)
        elif (
            self.format in [ImageFormat.RGB, ImageFormat.BGR]
            and target_format == ImageFormat.GRAYSCALE
        ):
            if self.format == ImageFormat.RGB:
                converted_data = cv2.cvtColor(converted_data, cv2.COLOR_RGB2GRAY)
            else:
                converted_data = cv2.cvtColor(converted_data, cv2.COLOR_BGR2GRAY)
        elif self.format == ImageFormat.GRAYSCALE and target_format == ImageFormat.RGB:
            converted_data = cv2.cvtColor(converted_data, cv2.COLOR_GRAY2RGB)
        elif self.format == ImageFormat.GRAYSCALE and target_format == ImageFormat.BGR:
            converted_data = cv2.cvtColor(converted_data, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(f"Conversion from {self.format} to {target_format} not supported")

        return Image(
            data=converted_data,
            format=target_format,
            timestamp=self.timestamp,
            metadata=self.metadata.copy(),
        )

    def resize(self, size: Tuple[int, int], interpolation: int = cv2.INTER_LINEAR) -> "Image":
        """Resize image to new size."""
        resized_data = cv2.resize(self._data, size, interpolation=interpolation)

        return Image(
            data=resized_data,
            format=self.format,
            timestamp=self.timestamp,
            metadata=self.metadata.copy(),
        )

    def crop(self, x: int, y: int, width: int, height: int) -> "Image":
        """Crop image to specified region."""
        cropped_data = self._data[y : y + height, x : x + width]

        return Image(
            data=cropped_data,
            format=self.format,
            timestamp=self.timestamp,
            metadata=self.metadata.copy(),
        )

    def copy(self) -> "Image":
        """Create a deep copy of the image."""
        return Image(
            data=self._data.copy(),
            format=self.format,
            timestamp=self.timestamp,
            metadata=self.metadata.copy(),
        )

    def save(self, path: str, quality: int = 95) -> None:
        """Save image to file."""
        if self.format == ImageFormat.BGR:
            # Convert BGR to RGB for saving
            rgb_data = cv2.cvtColor(self._data, cv2.COLOR_BGR2RGB)
            pil_img = PILImage.fromarray(rgb_data)
        elif self.format == ImageFormat.GRAYSCALE:
            pil_img = PILImage.fromarray(self._data, mode="L")
        else:
            pil_img = PILImage.fromarray(self._data)

        if path.lower().endswith((".jpg", ".jpeg")):
            pil_img.save(path, "JPEG", quality=quality)
        else:
            pil_img.save(path)

    def to_pil(self) -> PILImage.Image:
        """Convert to PIL Image."""
        if self.format == ImageFormat.BGR:
            # Convert BGR to RGB for PIL
            rgb_data = cv2.cvtColor(self._data, cv2.COLOR_BGR2RGB)
            return PILImage.fromarray(rgb_data)
        elif self.format == ImageFormat.GRAYSCALE:
            return PILImage.fromarray(self._data, mode="L")
        else:
            return PILImage.fromarray(self._data)

    def to_opencv(self) -> np.ndarray:
        """Convert to OpenCV format (BGR)."""
        if self.format == ImageFormat.RGB:
            return cv2.cvtColor(self._data, cv2.COLOR_RGB2BGR)
        else:
            return self._data.copy()

    @classmethod
    def from_opencv(cls, cv_img: np.ndarray, format: str = ImageFormat.BGR) -> "Image":
        """Create Image from OpenCV array."""
        return cls(data=cv_img, format=format)

    @classmethod
    def create_empty(
        cls, width: int, height: int, channels: int = 3, dtype: np.dtype = np.uint8
    ) -> "Image":
        """Create an empty image with specified dimensions."""
        if channels == 1:
            shape = (height, width)
            format = ImageFormat.GRAYSCALE
        elif channels == 3:
            shape = (height, width, channels)
            format = ImageFormat.RGB
        elif channels == 4:
            shape = (height, width, channels)
            format = ImageFormat.RGBA
        else:
            raise ValueError(f"Unsupported number of channels: {channels}")

        return cls(
            data=np.zeros(shape, dtype=dtype),
            format=format,
        )

    def __repr__(self) -> str:
        return f"Image({self.width}x{self.height}, {self.channels}ch, {self.format}, {self.dtype})"

    def __str__(self) -> str:
        return (
            f"📷 Image {self.width}x{self.height} "
            f"({self.channels}ch {self.format}) "
            f"@{self.timestamp.strftime('%H:%M:%S')}"
        )

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

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import TYPE_CHECKING, Any, Literal

from dimos_lcm.sensor_msgs.CompressedImage import CompressedImage as LCMCompressedImage
from dimos_lcm.std_msgs.Header import Header

from dimos.types.timestamped import Timestamped, to_human_readable

# cv2/turbojpeg/Image are imported lazily so this module stays cheap for
# byte-level consumers that never encode or decode pixels.
if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image

CompressionFormat = Literal["jpeg", "png", "jxl"]


@dataclass
class CompressedImage(Timestamped):
    """Compressed image bytes (JPEG/PNG/JPEG-XL) — ROS sensor_msgs/CompressedImage."""

    msg_name = "sensor_msgs.CompressedImage"

    data: bytes = b""
    format: str = "jpeg"
    frame_id: str = ""
    ts: float = field(default_factory=time.time)

    def __str__(self) -> str:
        return (
            f"CompressedImage(format={self.format}, bytes={len(self.data)}, "
            f"ts={to_human_readable(self.ts)})"
        )

    @classmethod
    def from_image(
        cls,
        image: Image,
        format: CompressionFormat = "jpeg",
        quality: int = 75,
        max_width: int | None = None,
        effort: int | None = None,
    ) -> CompressedImage:
        """Encode a raw Image.

        JPEG rejects 16-bit/depth formats; PNG rejects float depth. JXL takes
        everything: lossless for 16-bit/float data (depth must survive the wire
        exactly), `quality` for uint8. `effort` (jxl only, 1-9) trades encode
        cpu for size; defaults are pinned for camera-rate streaming.
        """
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        if not isinstance(image, Image):
            raise TypeError(f"from_image expects Image, got {type(image).__name__}")
        if max_width is not None:
            image, _ = image.resize_to_fit(max_width, max_width)
        if format == "jpeg":
            if image.format in (ImageFormat.GRAY16, ImageFormat.DEPTH, ImageFormat.DEPTH16):
                raise ValueError(f"JPEG cannot encode {image.format.value}; use format='png'")
            data = image.to_jpeg_bytes(quality=quality)
        elif format == "png":
            import cv2

            if image.format in (ImageFormat.DEPTH, ImageFormat.DEPTH16):
                raise ValueError(f"PNG cannot encode {image.format.value}")
            arr = image.data if image.channels == 1 else image.to_bgr().data
            ok, buf = cv2.imencode(".png", arr)
            if not ok:
                raise ValueError("PNG encoding failed")
            data = buf.tobytes()
        elif format == "jxl":
            import imagecodecs
            import numpy as np

            # zero-copy passthrough unless channel order actually needs fixing
            if image.channels == 1 or image.format is ImageFormat.RGB:
                arr = image.data
            else:
                arr = image.to_rgb().data
            if arr.dtype not in (np.uint8, np.uint16, np.float32):
                raise ValueError(f"JXL cannot encode dtype {arr.dtype}")
            arr = np.ascontiguousarray(arr)
            # libjxl's default effort=7 costs 83-300ms per 720p frame; these
            # pins keep camera-rate (13ms lossy / 3ms lossless) for ~5% size
            if arr.dtype == np.uint8:
                data = bytes(imagecodecs.jpegxl_encode(arr, level=quality, effort=effort or 3))
            else:
                data = bytes(imagecodecs.jpegxl_encode(arr, lossless=True, effort=effort or 1))
        else:
            raise ValueError(f"unsupported format {format!r}")
        return cls(data=data, format=format, frame_id=image.frame_id, ts=image.ts)

    def decode(self) -> Image:
        """Decompress to a raw Image; ts/frame_id preserved."""
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        if self.format.startswith("jpeg"):
            from turbojpeg import TJPF_RGB, TurboJPEG

            arr = TurboJPEG().decode(self.data, pixel_format=TJPF_RGB)
            fmt = ImageFormat.RGB
        elif self.format.startswith("png"):
            import cv2
            import numpy as np

            arr = cv2.imdecode(np.frombuffer(self.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            if arr is None:
                raise ValueError("PNG decoding failed")
            if arr.ndim == 2:
                fmt = ImageFormat.GRAY16 if arr.dtype == np.uint16 else ImageFormat.GRAY
            elif arr.shape[2] == 4:
                fmt = ImageFormat.BGRA
            else:
                fmt = ImageFormat.BGR
        elif self.format.startswith("jxl"):
            import imagecodecs
            import numpy as np

            arr = imagecodecs.jpegxl_decode(self.data)
            if arr.ndim == 2:
                fmt = {np.float32: ImageFormat.DEPTH, np.uint16: ImageFormat.GRAY16}.get(
                    arr.dtype.type, ImageFormat.GRAY
                )
            else:
                fmt = ImageFormat.RGBA if arr.shape[2] == 4 else ImageFormat.RGB
        else:
            raise ValueError(f"unsupported format {self.format!r}")
        return Image(data=arr, format=fmt, frame_id=self.frame_id, ts=self.ts)

    def lcm_encode(self, frame_id: str | None = None) -> bytes:
        msg = LCMCompressedImage()
        msg.header = Header()
        msg.header.seq = 0
        msg.header.frame_id = frame_id or self.frame_id
        msg.header.stamp.sec = int(self.ts)
        msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)
        msg.format = self.format
        msg.data = self.data
        msg.data_length = len(self.data)
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: Any) -> CompressedImage:
        msg = LCMCompressedImage.lcm_decode(data)
        return cls(
            data=bytes(msg.data),
            format=msg.format,
            frame_id=msg.header.frame_id,
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
        )

    def to_rerun(self) -> Any:
        import rerun as rr

        if self.format.startswith(("jpeg", "png")):
            media_type = "image/jpeg" if self.format.startswith("jpeg") else "image/png"
            return rr.EncodedImage(contents=self.data, media_type=media_type)
        # formats rerun's EncodedImage can't decode: send pixels instead
        return self.decode().to_rerun()

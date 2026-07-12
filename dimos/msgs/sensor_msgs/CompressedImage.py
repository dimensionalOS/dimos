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
    import rerun as rr

    from dimos.msgs.sensor_msgs.Image import Image

CompressionFormat = Literal["jpeg", "png"]


@dataclass
class CompressedImage(Timestamped):
    """Compressed image bytes (JPEG/PNG) — ROS sensor_msgs/CompressedImage."""

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
    ) -> CompressedImage:
        """Encode a raw Image. JPEG rejects 16-bit/depth formats; PNG rejects float depth."""
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
        else:
            raise ValueError(f"unsupported format {format!r}")
        return cls(data=data, format=format, frame_id=image.frame_id, ts=image.ts)

    def decode(self) -> Image:
        """Decompress to a raw Image; ts/frame_id preserved.

        Memoized: consumers that keep the latest frame and decode it several
        times (or never) pay for at most one decode per message.
        """
        cached: Image | None = getattr(self, "_decoded", None)
        if cached is not None:
            return cached
        img = self._decode()
        self._decoded = img
        return img

    def __getstate__(self) -> dict[str, Any]:
        # never pickle the decode cache — it would put raw pixels on the wire
        return {k: v for k, v in self.__dict__.items() if k != "_decoded"}

    def _decode(self) -> Image:
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
            else:
                fmt = ImageFormat.BGR
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

    def agent_encode(self) -> list[dict[str, Any]]:
        """VLM message content — the wire bytes go straight to the model, no re-encode."""
        import base64

        media = "image/jpeg" if self.format.startswith("jpeg") else "image/png"
        b64 = base64.b64encode(self.data).decode()
        return [{"type": "image_url", "image_url": {"url": f"data:{media};base64,{b64}"}}]

    def to_rerun(self) -> rr.EncodedImage:
        import rerun as rr

        media_type = "image/jpeg" if self.format.startswith("jpeg") else "image/png"
        return rr.EncodedImage(contents=self.data, media_type=media_type)

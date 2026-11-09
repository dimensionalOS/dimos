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

from __future__ import annotations

import io

from dimos_lcm.sensor_msgs.Image import Image as LCMImage
import numpy as np
from turbojpeg import TJPF_RGB, TurboJPEG

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header


class JpegCodec:
    """Read historical JPEG-in-LCM images as generated RGB Image messages."""

    def __init__(self, quality: int = 50) -> None:
        self._quality = quality
        self._decoder = TurboJPEG()

    @property
    def payload_type(self) -> type[Image]:
        return Image

    def encode(self, value: Image) -> bytes:
        raise TypeError("Historical JPEG codecs are read-only; use CDR for new recordings")

    def decode(self, data: bytes) -> Image:
        stream = io.BytesIO(data)
        old = LCMImage.lcm_decode(stream)
        if stream.tell() != len(data) or old.encoding != "jpeg":
            raise ValueError("Expected one historical JPEG Image envelope")
        pixels = self._decoder.decode(old.data, pixel_format=TJPF_RGB)
        return Image(
            header=Header(Time(old.header.stamp.sec, old.header.stamp.nsec), old.header.frame_id),
            height=pixels.shape[0],
            width=pixels.shape[1],
            encoding="rgb8",
            is_bigendian=0,
            step=pixels.shape[1] * 3,
            data=np.frombuffer(pixels.tobytes(), dtype=np.uint8).copy(),
        )

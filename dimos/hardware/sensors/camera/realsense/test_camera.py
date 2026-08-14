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

import numpy as np

from dimos.hardware.sensors.camera.realsense.camera import RealSenseCameraConfig
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def test_color_compression_is_opt_in() -> None:
    # Every color consumer in the repo takes In[Image] and none takes
    # In[CompressedImage], so raw stays the default until a decode path exists.
    assert RealSenseCameraConfig().compress_color is False


def test_compressed_color_logs_to_rerun_without_decoding() -> None:
    rng = np.random.default_rng(0)
    raw = Image(
        data=rng.integers(0, 255, (48, 64, 3), dtype=np.uint8),
        format=ImageFormat.RGB,
        frame_id="camera_color_optical_frame",
    )

    compressed = CompressedImage.from_image(raw, quality=RealSenseCameraConfig().jpeg_quality)

    # The bridge hands JPEG bytes straight to the viewer; decoding them on the
    # robot would spend Jetson CPU and re-inflate the stream.
    assert type(compressed.to_rerun()).__name__ == "EncodedImage"

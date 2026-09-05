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

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image


def letterbox_image(image: Image, width: int, height: int) -> Image:
    """Create a fixed-size preview while preserving the image's aspect ratio.

    DimOS sends color images to Rerun as JPEG ``EncodedImage`` values. Detection
    crops naturally change dimensions between frames, but reusing one Rerun
    entity for those varying dimensions triggers ``Detected change of video
    encoding properties over time`` and can panic the Viewer. Letterboxing keeps
    the encoded dimensions stable without stretching the crop.
    """
    if width <= 0 or height <= 0:
        raise ValueError("letterbox dimensions must be positive")

    scale = min(width / image.width, height / image.height)
    resized_width = max(1, min(width, round(image.width * scale)))
    resized_height = max(1, min(height, round(image.height * scale)))
    resized = image.resize(resized_width, resized_height)

    output_shape = (height, width, *resized.data.shape[2:])
    output = np.zeros(output_shape, dtype=resized.data.dtype)
    x_offset = (width - resized_width) // 2
    y_offset = (height - resized_height) // 2
    output[
        y_offset : y_offset + resized_height,
        x_offset : x_offset + resized_width,
        ...,
    ] = resized.data
    return Image(data=output, format=image.format, frame_id=image.frame_id, ts=image.ts)

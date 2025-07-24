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

"""Test the Qwen image query functionality."""

import os
import cv2
import numpy as np
from PIL import Image
from dimos.models.qwen.video_query import query_single_frame


def test_qwen_image_query():
    """Test querying Qwen with a single image."""
    # Skip if no API key
    if not os.getenv("ALIBABA_API_KEY"):
        print("ALIBABA_API_KEY not set")
        return

    # Load test image
    image_path = os.path.join(os.getcwd(), "assets", "test_spatial_memory", "frame_038.jpg")
    pil_image = Image.open(image_path)

    # Convert PIL image to numpy array in RGB format
    image_array = np.array(pil_image)
    if image_array.shape[-1] == 3:
        # Ensure it's in RGB format (PIL loads as RGB by default)
        image = image_array
    else:
        # Handle grayscale images
        image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2RGB)

    # Test basic object detection query
    response = query_single_frame(
        image=image,
        query="What objects do you see in this image? Return as a comma-separated list.",
    )
    print(response)

    # Test coordinate query
    response = query_single_frame(
        image=image,
        query="Return the center coordinates of any person in the image as a tuple (x,y)",
    )
    print(response)


if __name__ == "__main__":
    test_qwen_image_query()

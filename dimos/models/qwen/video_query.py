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

"""Utility functions for one-off video frame queries using Qwen model."""

import json

import numpy as np

from dimos.models.qwen.bbox import BBox
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.sensor_msgs.Image import Image


def query_single_frame(
    image: np.ndarray,
    query: str = "Return the center coordinates of the fridge handle as a tuple (x,y)",
    api_key: str | None = None,
    model_name: str = "qwen2.5-vl-72b-instruct",
) -> str:
    """Process a single numpy image array with Qwen model.

    Args:
        image: A numpy array image to process, shape (H, W, 3)
        query: The query to ask about the image
        api_key: Alibaba API key. If None, falls back to the ALIBABA_API_KEY env var
        model_name: The Qwen model to use. Defaults to qwen2.5-vl-72b-instruct

    Returns:
        str: The model's response

    Example:
        ```python
        import cv2
        image = cv2.imread("image.jpg")
        response = query_single_frame(image, "Return the center of the object as (x,y)")
        print(response)
        ```
    """
    model = QwenVlModel(model_name=model_name, api_key=api_key)
    # Wrap with the default BGR tag so Image.to_base64()'s to_bgr() is a no-op and the array
    # reaches Qwen's JPEG encoder unchanged, matching the prior cv2.imencode(".jpg", frame).
    return model.query(Image.from_numpy(image), query)


def get_bbox_from_qwen_frame(frame, object_name: str | None = None) -> BBox | None:  # type: ignore[no-untyped-def]
    """Get bounding box coordinates from Qwen for a specific object using a single frame.

    Args:
        frame: A single image frame (numpy array)
        object_name: Optional name of object to detect

    Returns:
        BBox: Bounding box as (x1, y1, x2, y2) or None if no detection
    """
    # Ensure frame is numpy array
    if not isinstance(frame, np.ndarray):
        raise ValueError("Frame must be a numpy array")

    prompt = (
        f"Look at this image and find the {object_name if object_name else 'most prominent object'}. "
        "Return ONLY a JSON object with format: {'name': 'object_name', 'bbox': [x1, y1, x2, y2]} "
        "where x1,y1 is the top-left and x2,y2 is the bottom-right corner of the bounding box. If not found, return None."
    )

    response = query_single_frame(frame, prompt)

    try:
        # Extract JSON from response
        start_idx = response.find("{")
        end_idx = response.rfind("}") + 1
        if start_idx >= 0 and end_idx > start_idx:
            json_str = response[start_idx:end_idx]
            result = json.loads(json_str)

            # Extract and validate bbox
            if "bbox" in result and len(result["bbox"]) == 4:
                return tuple(result["bbox"])  # Convert list to tuple
    except Exception as e:
        print(f"Error parsing Qwen response: {e}")
        print(f"Raw response: {response}")

    return None

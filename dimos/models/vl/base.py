import json
import re
from abc import ABC, abstractmethod
from typing import Union

import numpy as np

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.type import Detection2DBBox, ImageDetections2D
from dimos.utils.decorators import retry


def extract_json(response: str) -> Union[dict, list]:
    """Extract JSON from potentially messy LLM response.

    Tries multiple strategies:
    1. Parse the entire response as JSON
    2. Find and parse JSON arrays in the response
    3. Find and parse JSON objects in the response

    Args:
        response: Raw text response that may contain JSON

    Returns:
        Parsed JSON object (dict or list)

    Raises:
        json.JSONDecodeError: If no valid JSON can be extracted
    """
    # First try to parse the whole response as JSON
    try:
        return json.loads(response)
    except json.JSONDecodeError:
        pass

    # If that fails, try to extract JSON from the messy response
    # Look for JSON arrays or objects in the text

    # Pattern to match JSON arrays (including nested arrays/objects)
    # This finds the outermost [...] structure
    array_pattern = r'\[(?:[^\[\]]*|\[(?:[^\[\]]*|\[[^\[\]]*\])*\])*\]'

    # Pattern to match JSON objects
    object_pattern = r'\{(?:[^{}]*|\{(?:[^{}]*|\{[^{}]*\})*\})*\}'

    # Try to find JSON arrays first (most common for detections)
    matches = re.findall(array_pattern, response, re.DOTALL)
    for match in matches:
        try:
            parsed = json.loads(match)
            # For detection arrays, we expect a list
            if isinstance(parsed, list):
                return parsed
        except json.JSONDecodeError:
            continue

    # Try JSON objects if no arrays found
    matches = re.findall(object_pattern, response, re.DOTALL)
    for match in matches:
        try:
            return json.loads(match)
        except json.JSONDecodeError:
            continue

    # If nothing worked, raise an error with the original response
    raise json.JSONDecodeError(
        f"Could not extract valid JSON from response: {response[:200]}...",
        response, 0
    )


class VlModel(ABC):
    @abstractmethod
    def query(self, image: Image | np.ndarray, query: str) -> str: ...

    @retry(max_retries=2, on_exception=json.JSONDecodeError, delay=0.0)
    def query_json(self, image: Image, query: str) -> dict:
        response = self.query(image, query)
        return extract_json(response)

    def query_detections(self, image: Image, query: str) -> ImageDetections2D:
        full_query = f"""show me bounding boxes in pixels for this query: `{query}`

        format should be:
        `[
        [label, x1, y1, x2, y2]
        ...
        ]`

        (etc, multiple matches are possible)

        If there's no match return `[]`. Label is whatever you think is appropriate
        Only respond with the coordinates, no other text."""

        image_detections = ImageDetections2D(image)
        try:
            coords = self.query_json(image, full_query)
        except Exception:
            return image_detections

        img_height, img_width = image.shape[:2] if image.shape else (float("inf"), float("inf"))

        for track_id, detection_list in enumerate(coords):
            if len(detection_list) != 5:
                continue

            name = detection_list[0]

            # Convert to floats with error handling
            try:
                bbox = list(map(float, detection_list[1:]))
            except (ValueError, TypeError):
                print(
                    f"Warning: Invalid bbox coordinates for detection '{name}': {detection_list[1:]}"
                )
                continue

            # Validate bounding box
            x1, y1, x2, y2 = bbox

            # Check if coordinates are valid
            if x2 <= x1 or y2 <= y1:
                print(
                    f"Warning: Invalid bbox dimensions for '{name}': x1={x1}, y1={y1}, x2={x2}, y2={y2}"
                )
                continue

            # Clamp to image bounds if we have image dimensions
            if image.shape:
                x1 = max(0, min(x1, img_width))
                y1 = max(0, min(y1, img_height))
                x2 = max(0, min(x2, img_width))
                y2 = max(0, min(y2, img_height))
                bbox = [x1, y1, x2, y2]

            image_detections.detections.append(
                Detection2DBBox(
                    bbox=bbox,
                    track_id=track_id,
                    class_id=-100,  # Using -100 to indicate VLModel-generated detection
                    confidence=1.0,
                    name=name,
                    ts=image.ts,
                    image=image,
                )
            )
        return image_detections

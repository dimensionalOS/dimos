import json
from abc import ABC, abstractmethod

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import Detection2DBBox, ImageDetections2D
from dimos.perception.detection.type.detection2d import Detection
from dimos.utils.decorators import retry
from dimos.utils.llm_utils import extract_json


def vlm_detection_to_yolo(vlm_detection: list, track_id: int) -> Detection | None:
    """Convert a single VLM detection [label, x1, y1, x2, y2] to Detection tuple.

    Args:
        vlm_detection: Single detection list containing [label, x1, y1, x2, y2]
        track_id: Track ID to assign to this detection

    Returns:
        Detection tuple (bbox, track_id, class_id, confidence, name) or None if invalid
    """
    if len(vlm_detection) != 5:
        return None

    name = str(vlm_detection[0])
    try:
        bbox = tuple(map(float, vlm_detection[1:]))
        # Use -1 for class_id since VLM doesn't provide it
        # confidence defaults to 1.0 for VLM
        return (bbox, track_id, -1, 1.0, name)
    except (ValueError, TypeError):
        return None


class VlModel(ABC):
    @abstractmethod
    def query(self, image: Image, query: str) -> str: ...

    # requery once if JSON parsing fails
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
            detection_tuples = self.query_json(image, full_query)
        except Exception:
            return image_detections

        for track_id, detection_tuple in enumerate(detection_tuples):
            detection = vlm_detection_to_yolo(detection_tuple, track_id)
            if detection is None:
                continue
            detection2d = Detection2DBBox.from_detection(detection, ts=image.ts, image=image)
            if detection2d.is_valid():
                image_detections.detections.append(detection2d)

        return image_detections

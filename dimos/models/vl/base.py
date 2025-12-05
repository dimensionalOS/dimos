from abc import ABC, abstractmethod
import json
import logging

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import Detection2DBBox, ImageDetections2D
from dimos.utils.data import get_data
from dimos.utils.decorators import retry
from dimos.utils.llm_utils import extract_json

logger = logging.getLogger(__name__)


def vlm_detection_to_detection2d(
    vlm_detection: list, track_id: int, image: Image  # type: ignore[type-arg]
) -> Detection2DBBox | None:
    """Convert a single VLM detection [label, x1, y1, x2, y2] to Detection2DBBox.

    Args:
        vlm_detection: Single detection list containing [label, x1, y1, x2, y2]
        track_id: Track ID to assign to this detection
        image: Source image for the detection

    Returns:
        Detection2DBBox instance or None if invalid
    """
    # Validate list structure
    if not isinstance(vlm_detection, list):
        logger.debug(f"VLM detection is not a list: {type(vlm_detection)}")
        return None

    if len(vlm_detection) != 5:
        logger.debug(
            f"Invalid VLM detection length: {len(vlm_detection)}, expected 5. Got: {vlm_detection}"
        )
        return None

    # Extract label
    name = str(vlm_detection[0])

    # Validate and convert coordinates
    try:
        coords = [float(x) for x in vlm_detection[1:]]
    except (ValueError, TypeError) as e:
        logger.debug(f"Invalid VLM detection coordinates: {vlm_detection[1:]}. Error: {e}")
        return None

    bbox = tuple(coords)

    # Use -1 for class_id since VLM doesn't provide it
    # confidence defaults to 1.0 for VLM
    return Detection2DBBox(
        bbox=bbox,  # type: ignore[arg-type]
        track_id=track_id,
        class_id=-1,
        confidence=1.0,
        name=name,
        ts=image.ts,
        image=image,
    )


class VlModel(ABC):
    @abstractmethod
    def query(self, image: Image, query: str, **kwargs) -> str: ...  # type: ignore[no-untyped-def]

    def warmup(self) -> None:
        try:
            image = Image.from_file(get_data("cafe-smol.jpg")).to_rgb()  # type: ignore[arg-type]
            self._model.detect(image, "person", settings={"max_objects": 1})  # type: ignore[attr-defined]
        except Exception:
            pass

    # requery once if JSON parsing fails
    @retry(max_retries=2, on_exception=json.JSONDecodeError, delay=0.0)  # type: ignore[misc, untyped-decorator]
    def query_json(self, image: Image, query: str) -> dict:  # type: ignore[type-arg]
        response = self.query(image, query)
        return extract_json(response)  # type: ignore[return-value]

    def query_detections(self, image: Image, query: str, **kwargs) -> ImageDetections2D:  # type: ignore[no-untyped-def]
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
            detection2d = vlm_detection_to_detection2d(detection_tuple, track_id, image)
            if detection2d is not None and detection2d.is_valid():
                image_detections.detections.append(detection2d)

        return image_detections

from abc import ABC, abstractmethod
from dataclasses import dataclass
import json
import logging
import warnings

from dimos.core.resource import Resource
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import Detection2DBBox, Detection2DPoint, ImageDetections2D
from dimos.protocol.service import Configurable  # type: ignore[attr-defined]
from dimos.utils.data import get_data
from dimos.utils.decorators import retry
from dimos.utils.llm_utils import extract_json

logger = logging.getLogger(__name__)


class Captioner(ABC):
    """Interface for models that can generate image captions."""

    @abstractmethod
    def caption(self, image: Image) -> str:
        """Generate a text description of the image.

        Args:
            image: Input image to caption

        Returns:
            Text description of the image
        """
        ...

    def caption_batch(self, *images: Image) -> list[str]:
        """Generate captions for multiple images.

        Default implementation calls caption() for each image.
        Subclasses may override for more efficient batching.

        Args:
            images: Input images to caption

        Returns:
            List of text descriptions
        """
        return [self.caption(img) for img in images]


# Type alias for VLM detection format: [label, x1, y1, x2, y2]
VlmDetection = tuple[str, float, float, float, float]


def vlm_detection_to_detection2d(
    vlm_detection: VlmDetection | list[str | float],
    track_id: int,
    image: Image,
) -> Detection2DBBox | None:
    """Convert a single VLM detection [label, x1, y1, x2, y2] to Detection2DBBox.

    Args:
        vlm_detection: Single detection tuple/list containing [label, x1, y1, x2, y2]
        track_id: Track ID to assign to this detection
        image: Source image for the detection

    Returns:
        Detection2DBBox instance or None if invalid
    """
    # Validate list/tuple structure
    if not isinstance(vlm_detection, (list, tuple)):
        logger.debug(f"VLM detection is not a list/tuple: {type(vlm_detection)}")
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
        coords = [float(vlm_detection[i]) for i in range(1, 5)]
    except (ValueError, TypeError) as e:
        logger.debug(f"Invalid VLM detection coordinates: {vlm_detection[1:]}. Error: {e}")
        return None

    bbox = (coords[0], coords[1], coords[2], coords[3])

    # Use -1 for class_id since VLM doesn't provide it
    # confidence defaults to 1.0 for VLM
    return Detection2DBBox(
        bbox=bbox,
        track_id=track_id,
        class_id=-1,
        confidence=1.0,
        name=name,
        ts=image.ts,
        image=image,
    )


# Type alias for VLM point format: [label, x, y]
VlmPoint = tuple[str, float, float]


def vlm_point_to_detection2d_point(
    vlm_point: VlmPoint | list[str | float],
    track_id: int,
    image: Image,
) -> Detection2DPoint | None:
    """Convert a single VLM point [label, x, y] to Detection2DPoint.

    Args:
        vlm_point: Single point tuple/list containing [label, x, y]
        track_id: Track ID to assign to this detection
        image: Source image for the detection

    Returns:
        Detection2DPoint instance or None if invalid
    """
    # Validate list/tuple structure
    if not isinstance(vlm_point, (list, tuple)):
        logger.debug(f"VLM point is not a list/tuple: {type(vlm_point)}")
        return None

    if len(vlm_point) != 3:
        logger.debug(f"Invalid VLM point length: {len(vlm_point)}, expected 3. Got: {vlm_point}")
        return None

    # Extract label
    name = str(vlm_point[0])

    # Validate and convert coordinates
    try:
        x = float(vlm_point[1])
        y = float(vlm_point[2])
    except (ValueError, TypeError) as e:
        logger.debug(f"Invalid VLM point coordinates: {vlm_point[1:]}. Error: {e}")
        return None

    return Detection2DPoint(
        x=x,
        y=y,
        name=name,
        ts=image.ts,
        image=image,
        track_id=track_id,
    )


@dataclass
class VlModelConfig:
    """Configuration for VlModel."""

    auto_resize: tuple[int, int] | None = None
    """Optional (width, height) tuple. If set, images are resized to fit."""


class VlModel(Captioner, Resource, Configurable[VlModelConfig]):
    """Vision-language model that can answer questions about images.

    Inherits from Captioner, providing a default caption() implementation
    that uses query() with a standard captioning prompt.

    Implements Resource interface for lifecycle management.
    """

    default_config = VlModelConfig
    config: VlModelConfig

    def _prepare_image(self, image: Image) -> tuple[Image, float]:
        """Prepare image for inference, applying any configured transformations.

        Returns:
            Tuple of (prepared_image, scale_factor). Scale factor is 1.0 if no resize.
        """
        if self.config.auto_resize is not None:
            max_w, max_h = self.config.auto_resize
            return image.resize_to_fit(max_w, max_h)
        return image, 1.0

    @abstractmethod
    def query(self, image: Image, query: str, **kwargs) -> str: ...  # type: ignore[no-untyped-def]

    def query_batch(self, images: list[Image], query: str, **kwargs) -> list[str]:  # type: ignore[no-untyped-def]
        """Query multiple images with the same question.

        Default implementation calls query() for each image sequentially.
        Subclasses may override for more efficient batched inference.

        Args:
            images: List of input images
            query: Question to ask about each image

        Returns:
            List of responses, one per image
        """
        warnings.warn(
            f"{self.__class__.__name__}.query_batch() is using default sequential implementation. "
            "Override for efficient batched inference.",
            stacklevel=2,
        )
        return [self.query(image, query, **kwargs) for image in images]

    def query_multi(self, image: Image, queries: list[str], **kwargs) -> list[str]:  # type: ignore[no-untyped-def]
        """Query a single image with multiple different questions.

        Default implementation calls query() for each question sequentially.
        Subclasses may override for more efficient inference (e.g., by
        encoding the image once and reusing it for all queries).

        Args:
            image: Input image
            queries: List of questions to ask about the image

        Returns:
            List of responses, one per query
        """
        warnings.warn(
            f"{self.__class__.__name__}.query_multi() is using default sequential implementation. "
            "Override for efficient batched inference.",
            stacklevel=2,
        )
        return [self.query(image, q, **kwargs) for q in queries]

    def caption(self, image: Image) -> str:
        """Generate a caption by querying the VLM with a standard prompt."""
        return self.query(image, "Describe this image concisely.")

    def start(self) -> None:
        """Start the model by running a simple query (Resource interface)."""
        try:
            image = Image.from_file(get_data("cafe-smol.jpg")).to_rgb()
            self.query(image, "What is this?")
        except Exception:
            pass

    # requery once if JSON parsing fails
    @retry(max_retries=2, on_exception=json.JSONDecodeError, delay=0.0)  # type: ignore[misc, untyped-decorator]
    def query_json(self, image: Image, query: str) -> dict:  # type: ignore[type-arg]
        response = self.query(image, query)
        return extract_json(response)  # type: ignore[return-value]

    def query_detections(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
        full_query = f"""show me bounding boxes in pixels for this query: `{query}`

        format should be:
        ```json
        [
           ["label1", x1, y1, x2, y2]
           ["label2", x1, y1, x2, y2]
        ...
        ]`

        (etc, multiple matches are possible)

        If there's no match return `[]`. Label is whatever you think is appropriate
        Only respond with JSON, no other text.
        """

        image_detections = ImageDetections2D(image)

        # Get scaled image and scale factor for coordinate rescaling
        scaled_image, scale = self._prepare_image(image)

        try:
            detection_tuples = self.query_json(scaled_image, full_query)
        except Exception:
            return image_detections

        for track_id, detection_tuple in enumerate(detection_tuples):
            # Scale coordinates back to original image size if resized
            if (
                scale != 1.0
                and isinstance(detection_tuple, (list, tuple))
                and len(detection_tuple) == 5
            ):
                detection_tuple = [
                    detection_tuple[0],  # label
                    detection_tuple[1] / scale,  # x1
                    detection_tuple[2] / scale,  # y1
                    detection_tuple[3] / scale,  # x2
                    detection_tuple[4] / scale,  # y2
                ]
            detection2d = vlm_detection_to_detection2d(detection_tuple, track_id, image)
            if detection2d is not None and detection2d.is_valid():
                image_detections.detections.append(detection2d)

        return image_detections

    def query_points(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DPoint]:
        """Query the VLM for point locations matching the query.

        Args:
            image: Input image to query
            query: Description of what points to find (e.g., "center of the red ball")

        Returns:
            ImageDetections2D containing Detection2DPoint instances
        """
        full_query = f"""Show me point coordinates in pixels for this query: `{query}`

        The format should be:
        ```json
        [
           ["label 1", x, y],
           ["label 2", x, y],
        ...
        ]

        If there's no match return `[]`. Label is whatever you think is appropriate.
        Only respond with the JSON, no other text.
        """

        image_detections: ImageDetections2D[Detection2DPoint] = ImageDetections2D(image)

        # Get scaled image and scale factor for coordinate rescaling
        scaled_image, scale = self._prepare_image(image)

        try:
            point_tuples = self.query_json(scaled_image, full_query)
        except Exception:
            return image_detections

        for track_id, point_tuple in enumerate(point_tuples):
            # Scale coordinates back to original image size if resized
            if scale != 1.0 and isinstance(point_tuple, (list, tuple)) and len(point_tuple) == 3:
                point_tuple = [
                    point_tuple[0],  # label
                    point_tuple[1] / scale,  # x
                    point_tuple[2] / scale,  # y
                ]
            point2d = vlm_point_to_detection2d_point(point_tuple, track_id, image)
            if point2d is not None and point2d.is_valid():
                image_detections.detections.append(point2d)

        return image_detections

from functools import cached_property
import os
import warnings

import moondream as md  # type: ignore[import-untyped]
import numpy as np
from PIL import Image as PILImage

from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import Detection2DBBox, ImageDetections2D


class MoondreamHostedVlModel(VlModel):
    _api_key: str | None

    def __init__(self, api_key: str | None = None) -> None:
        self._api_key = api_key

    @cached_property
    def _client(self) -> md.vl:
        api_key = self._api_key or os.getenv("MOONDREAM_API_KEY")
        if not api_key:
            raise ValueError(
                "Moondream API key must be provided or set in MOONDREAM_API_KEY environment variable"
            )
        return md.vl(api_key=api_key)

    def _to_pil_image(self, image: Image | np.ndarray) -> PILImage.Image:  # type: ignore[type-arg]
        if isinstance(image, np.ndarray):
            warnings.warn(
                "MoondreamHostedVlModel should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=3,
            )
            image = Image.from_numpy(image)

        rgb_image = image.to_rgb()
        return PILImage.fromarray(rgb_image.data)

    def query(self, image: Image | np.ndarray, query: str, **kwargs) -> str:  # type: ignore[no-untyped-def, type-arg]
        pil_image = self._to_pil_image(image)

        result = self._client.query(pil_image, query)
        return result.get("answer", str(result))  # type: ignore[no-any-return]

    def caption(self, image: Image | np.ndarray, length: str = "normal") -> str:  # type: ignore[type-arg]
        """Generate a caption for the image.

        Args:
            image: Input image
            length: Caption length ("normal", "short", "long")
        """
        pil_image = self._to_pil_image(image)
        result = self._client.caption(pil_image, length=length)
        return result.get("caption", str(result))  # type: ignore[no-any-return]

    def query_detections(self, image: Image, query: str, **kwargs) -> ImageDetections2D[Detection2DBBox]:  # type: ignore[no-untyped-def]
        """Detect objects using Moondream's hosted detect method.

        Args:
            image: Input image
            query: Object query (e.g., "person", "car")
            max_objects: Maximum number of objects to detect (not directly supported by hosted API args in docs,
                         but we handle the output)

        Returns:
            ImageDetections2D containing detected bounding boxes
        """
        pil_image = self._to_pil_image(image)

        # API docs: detect(image, object) -> {"objects": [...]}
        result = self._client.detect(pil_image, query)
        objects = result.get("objects", [])

        # Convert to ImageDetections2D
        image_detections = ImageDetections2D(image)
        height, width = image.height, image.width

        for track_id, obj in enumerate(objects):
            # Expected format from docs: Region with x_min, y_min, x_max, y_max
            # Assuming normalized coordinates as per local model and standard VLM behavior
            x_min_norm = obj.get("x_min", 0.0)
            y_min_norm = obj.get("y_min", 0.0)
            x_max_norm = obj.get("x_max", 1.0)
            y_max_norm = obj.get("y_max", 1.0)

            x1 = x_min_norm * width
            y1 = y_min_norm * height
            x2 = x_max_norm * width
            y2 = y_max_norm * height

            bbox = (x1, y1, x2, y2)

            detection = Detection2DBBox(
                bbox=bbox,
                track_id=track_id,
                class_id=-1,
                confidence=1.0,
                name=query,
                ts=image.ts,
                image=image,
            )

            if detection.is_valid():
                image_detections.detections.append(detection)

        return image_detections

    def point(self, image: Image, query: str) -> list[tuple[float, float]]:
        """Get coordinates of specific objects in an image.

        Args:
            image: Input image
            query: Object query

        Returns:
            List of (x, y) pixel coordinates
        """
        pil_image = self._to_pil_image(image)
        result = self._client.point(pil_image, query)
        points = result.get("points", [])

        pixel_points = []
        height, width = image.height, image.width

        for p in points:
            x_norm = p.get("x", 0.0)
            y_norm = p.get("y", 0.0)
            pixel_points.append((x_norm * width, y_norm * height))

        return pixel_points

    def stop(self) -> None:
        pass


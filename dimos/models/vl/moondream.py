from dataclasses import dataclass
from functools import cached_property
from typing import Any
import warnings

import numpy as np
from PIL import Image as PILImage
import torch
from transformers import AutoModelForCausalLM  # type: ignore[import-untyped]

from dimos.models.base import HuggingFaceModel, HuggingFaceModelConfig
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import Detection2DBBox, Detection2DPoint, ImageDetections2D

# Moondream works well with 512x512 max
MOONDREAM_DEFAULT_AUTO_RESIZE = (512, 512)


@dataclass
class MoondreamConfig(HuggingFaceModelConfig):
    """Configuration for MoondreamVlModel."""

    model_name: str = "vikhyatk/moondream2"
    dtype: torch.dtype = torch.bfloat16
    auto_resize: tuple[int, int] | None = MOONDREAM_DEFAULT_AUTO_RESIZE


class MoondreamVlModel(HuggingFaceModel, VlModel):
    _model_class = AutoModelForCausalLM
    default_config = MoondreamConfig  # type: ignore[assignment]
    config: MoondreamConfig  # type: ignore[assignment]

    @cached_property
    def _model(self) -> AutoModelForCausalLM:
        """Load model with compile() for optimization."""
        model = AutoModelForCausalLM.from_pretrained(
            self.config.model_name,
            trust_remote_code=self.config.trust_remote_code,
            torch_dtype=self.config.dtype,
        ).to(self.config.device)
        model.compile()
        return model

    def _to_pil(self, image: Image | np.ndarray[Any, Any]) -> PILImage.Image:
        """Convert dimos Image or numpy array to PIL Image, applying auto_resize."""
        if isinstance(image, np.ndarray):
            warnings.warn(
                "MoondreamVlModel should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=2,
            )
            image = Image.from_numpy(image)

        image, _ = self._prepare_image(image)
        rgb_image = image.to_rgb()
        return PILImage.fromarray(rgb_image.data)

    def query(self, image: Image | np.ndarray, query: str, **kwargs) -> str:  # type: ignore[no-untyped-def, type-arg]
        pil_image = self._to_pil(image)

        # Query the model
        result = self._model.query(image=pil_image, question=query, reasoning=False)

        # Handle both dict and string responses
        if isinstance(result, dict):
            return result.get("answer", str(result))  # type: ignore[no-any-return]

        return str(result)

    def query_batch(self, images: list[Image], query: str, **kwargs) -> list[str]:  # type: ignore[no-untyped-def]
        """Query multiple images with the same question.

        Note: moondream2's batch_answer is not truly batched - it processes
        images sequentially. No speedup over sequential calls.

        Args:
            images: List of input images
            query: Question to ask about each image

        Returns:
            List of responses, one per image
        """
        warnings.warn(
            "MoondreamVlModel.query_batch() uses moondream's batch_answer which is not "
            "truly batched - images are processed sequentially with no speedup.",
            stacklevel=2,
        )
        if not images:
            return []

        pil_images = [self._to_pil(img) for img in images]
        prompts = [query] * len(images)
        result: list[str] = self._model.batch_answer(pil_images, prompts)
        return result

    def query_multi(self, image: Image, queries: list[str], **kwargs) -> list[str]:  # type: ignore[no-untyped-def]
        """Query a single image with multiple different questions.

        Optimized implementation that encodes the image once and reuses
        the encoded representation for all queries.

        Args:
            image: Input image
            queries: List of questions to ask about the image

        Returns:
            List of responses, one per query
        """
        if not queries:
            return []

        # Encode image once
        pil_image = self._to_pil(image)
        encoded_image = self._model.encode_image(pil_image)

        # Query with each question, reusing the encoded image
        results = []
        for query in queries:
            result = self._model.query(image=encoded_image, question=query, reasoning=False)
            if isinstance(result, dict):
                results.append(result.get("answer", str(result)))
            else:
                results.append(str(result))

        return results

    def query_detections(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
        """Detect objects using Moondream's native detect method.

        Args:
            image: Input image
            query: Object query (e.g., "person", "car")
            max_objects: Maximum number of objects to detect

        Returns:
            ImageDetections2D containing detected bounding boxes
        """
        pil_image = self._to_pil(image)

        settings = {"max_objects": kwargs.get("max_objects", 5)}
        result = self._model.detect(pil_image, query, settings=settings)

        # Convert to ImageDetections2D
        image_detections = ImageDetections2D(image)

        # Get image dimensions for converting normalized coords to pixels
        height, width = image.height, image.width

        for track_id, obj in enumerate(result.get("objects", [])):
            # Convert normalized coordinates (0-1) to pixel coordinates
            x_min_norm = obj["x_min"]
            y_min_norm = obj["y_min"]
            x_max_norm = obj["x_max"]
            y_max_norm = obj["y_max"]

            x1 = x_min_norm * width
            y1 = y_min_norm * height
            x2 = x_max_norm * width
            y2 = y_max_norm * height

            bbox = (x1, y1, x2, y2)

            detection = Detection2DBBox(
                bbox=bbox,
                track_id=track_id,
                class_id=-1,  # Moondream doesn't provide class IDs
                confidence=1.0,  # Moondream doesn't provide confidence scores
                name=query,  # Use the query as the object name
                ts=image.ts,
                image=image,
            )

            if detection.is_valid():
                image_detections.detections.append(detection)

        return image_detections

    def query_points(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DPoint]:
        """Detect point locations using Moondream's native point method.

        Args:
            image: Input image
            query: Object query (e.g., "person's head", "center of the ball")

        Returns:
            ImageDetections2D containing detected points
        """
        pil_image = self._to_pil(image)

        result = self._model.point(pil_image, query)

        # Convert to ImageDetections2D
        image_detections: ImageDetections2D[Detection2DPoint] = ImageDetections2D(image)

        # Get image dimensions for converting normalized coords to pixels
        height, width = image.height, image.width

        for track_id, point in enumerate(result.get("points", [])):
            # Convert normalized coordinates (0-1) to pixel coordinates
            x = point["x"] * width
            y = point["y"] * height

            detection = Detection2DPoint(
                x=x,
                y=y,
                name=query,
                ts=image.ts,
                image=image,
                track_id=track_id,
            )

            if detection.is_valid():
                image_detections.detections.append(detection)

        return image_detections

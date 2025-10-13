import warnings
from functools import cached_property
from typing import Optional

import numpy as np
import torch
from PIL import Image as PILImage
from transformers import AutoModelForCausalLM

from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import Detection2DBBox, ImageDetections2D


class MoondreamVlModel(VlModel):
    _model_name: str
    _device: str
    _dtype: torch.dtype

    def __init__(
        self,
        model_name: str = "vikhyatk/moondream2",
        device: Optional[str] = None,
        dtype: torch.dtype = torch.bfloat16,
    ):
        self._model_name = model_name
        self._device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self._dtype = dtype

    @cached_property
    def _model(self) -> AutoModelForCausalLM:
        model = AutoModelForCausalLM.from_pretrained(
            self._model_name,
            trust_remote_code=True,
            torch_dtype=self._dtype,
        )
        model = model.to(self._device)
        model.compile()

        return model

    def query(self, image: Image | np.ndarray, query: str, **kwargs) -> str:
        if isinstance(image, np.ndarray):
            warnings.warn(
                "MoondreamVlModel.query should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=2,
            )
            image = Image.from_numpy(image)

        # Convert dimos Image to PIL Image
        # dimos Image stores data in RGB/BGR format, convert to RGB for PIL
        rgb_image = image.to_rgb()
        pil_image = PILImage.fromarray(rgb_image.data)

        # Query the model
        result = self._model.query(image=pil_image, question=query, reasoning=False)

        # Handle both dict and string responses
        if isinstance(result, dict):
            return result.get("answer", str(result))

        return str(result)

    def query_detections(self, image: Image, query: str, **kwargs) -> ImageDetections2D:
        """Detect objects using Moondream's native detect method.

        Args:
            image: Input image
            query: Object query (e.g., "person", "car")
            max_objects: Maximum number of objects to detect

        Returns:
            ImageDetections2D containing detected bounding boxes
        """
        pil_image = PILImage.fromarray(image.data)

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

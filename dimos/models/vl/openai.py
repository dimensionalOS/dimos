from dataclasses import dataclass
from functools import cached_property
import os
from typing import Any

import numpy as np
from openai import OpenAI

from dimos.models.vl.base import VlModel, VlModelConfig
from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class OpenAIVlModelConfig(VlModelConfig):
    model_name: str = "gpt-4o-mini"
    api_key: str | None = None


class OpenAIVlModel(VlModel):
    default_config = OpenAIVlModelConfig
    config: OpenAIVlModelConfig

    def is_set_up(self) -> None:
        """
        Verify that OpenAI API key is configured.
        """
        api_key = self.config.api_key or os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError(
                "OpenAI API key must be provided or set in OPENAI_API_KEY environment variable"
            )

    def __getstate__(self) -> dict[str, Any]:
        """Exclude unpicklable attributes when serializing."""
        state = super().__getstate__()
        # _client is already removed by base class, but ensure it's gone
        state.pop("_client", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        """Restore object from pickled state and reload API key if needed."""
        super().__setstate__(state)

        # Reload API key from environment if config doesn't have it
        # This is important when unpickling on Dask workers where env vars may differ
        if not self.config.api_key:
            api_key = os.getenv("OPENAI_API_KEY")
            if api_key:
                self.config.api_key = api_key

        # Verify setup (will raise ValueError if API key is still missing)
        self.is_set_up()

    @cached_property
    def _client(self) -> OpenAI:
        api_key = self.config.api_key or os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError(
                "OpenAI API key must be provided or set in OPENAI_API_KEY environment variable"
            )

        return OpenAI(api_key=api_key)

    def query(self, image: Image | np.ndarray, query: str, response_format: dict | None = None, **kwargs) -> str:  # type: ignore[override, type-arg, no-untyped-def]
        if isinstance(image, np.ndarray):
            import warnings

            warnings.warn(
                "OpenAIVlModel.query should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=2,
            )

            image = Image.from_numpy(image)

        # Apply auto_resize if configured
        image, _ = self._prepare_image(image)

        img_base64 = image.to_base64()

        api_kwargs: dict[str, Any] = {
            "model": self.config.model_name,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/png;base64,{img_base64}"},
                        },
                        {"type": "text", "text": query},
                    ],
                }
            ],
        }

        if response_format:
            api_kwargs["response_format"] = response_format

        response = self._client.chat.completions.create(**api_kwargs)

        return response.choices[0].message.content  # type: ignore[return-value]

    def query_multi_images(
        self, images: list[Image], query: str, response_format: dict | None = None, **kwargs
    ) -> str:  # type: ignore[no-untyped-def, override]
        """Query VLM with multiple images (for temporal/multi-view reasoning).

        Args:
            images: List of images to analyze together
            query: Question about all images
            response_format: Optional response format for structured output
                - {"type": "json_object"} for JSON mode
                - {"type": "json_schema", "json_schema": {...}} for schema enforcement

        Returns:
            Response from the model
        """
        if not images:
            raise ValueError("Must provide at least one image")

        # Build content with multiple images
        content: list[dict] = []  # type: ignore[type-arg]

        # Add all images first
        for img in images:
            # Apply auto_resize if configured
            prepared_img, _ = self._prepare_image(img)
            img_base64 = prepared_img.to_base64()
            content.append(
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/png;base64,{img_base64}"},
                }
            )

        # Add query text last
        content.append({"type": "text", "text": query})

        # Build messages
        messages = [{"role": "user", "content": content}]

        # Call API with optional response_format
        api_kwargs: dict[str, Any] = {"model": self.config.model_name, "messages": messages}
        if response_format:
            api_kwargs["response_format"] = response_format

        response = self._client.chat.completions.create(**api_kwargs)

        return response.choices[0].message.content  # type: ignore[return-value]

    def stop(self) -> None:
        """Release the OpenAI client."""
        if "_client" in self.__dict__:
            del self.__dict__["_client"]


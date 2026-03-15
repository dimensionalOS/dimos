# Copyright 2025-2026 Dimensional Inc.
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

from enum import Enum
from functools import cached_property

from PIL import Image as PILImage
import torch
from transformers import AutoModelForCausalLM, AutoProcessor  # type: ignore[import-untyped]

from dimos.models.base import HuggingFaceModel
from dimos.models.vl.base import Captioner
from dimos.msgs.sensor_msgs.Image import Image


class CaptionDetail(Enum):
    """Florence-2 caption detail level."""

    BRIEF = "<CAPTION>"
    NORMAL = "<DETAILED_CAPTION>"
    DETAILED = "<MORE_DETAILED_CAPTION>"


class Florence2Model(HuggingFaceModel, Captioner):
    """Florence-2 captioning model from Microsoft.

    A lightweight, fast captioning model optimized for generating image descriptions
    without requiring a text prompt. Supports multiple caption detail levels.
    """

    _model_class = AutoModelForCausalLM

    def __init__(
        self,
        model_name: str = "microsoft/Florence-2-base",
        detail: CaptionDetail = CaptionDetail.NORMAL,
        **kwargs: object,
    ) -> None:
        """Initialize Florence-2 model.

        Args:
            model_name: HuggingFace model name. Options:
                - "microsoft/Florence-2-base" (~0.2B, fastest)
                - "microsoft/Florence-2-large" (~0.8B, better quality)
            detail: Caption detail level
            **kwargs: Additional config options (device, dtype, warmup, etc.)
        """
        super().__init__(model_name=model_name, **kwargs)
        self._task_prompt = detail.value

    @cached_property
    def _processor(self) -> AutoProcessor:
        return AutoProcessor.from_pretrained(
            self.config.model_name, trust_remote_code=self.config.trust_remote_code
        )

    _STRIP_PREFIXES = ("The image shows ", "The image is a ", "A ")

    @staticmethod
    def _clean_caption(text: str) -> str:
        for prefix in Florence2Model._STRIP_PREFIXES:
            if text.startswith(prefix):
                return text[len(prefix) :]
        return text

    def caption(self, image: Image) -> str:
        """Generate a caption for the image.

        Returns:
            Text description of the image
        """
        task_prompt = self._task_prompt

        # Convert to PIL
        pil_image = PILImage.fromarray(image.to_rgb().data)

        # Process inputs
        inputs = self._processor(text=task_prompt, images=pil_image, return_tensors="pt")
        inputs = self._move_inputs_to_device(inputs)

        # Generate
        with torch.inference_mode():
            generated_ids = self._model.generate(
                **inputs,
                max_new_tokens=256,
                num_beams=3,
                do_sample=False,
            )

        # Decode
        generated_text = self._processor.batch_decode(generated_ids, skip_special_tokens=False)[0]

        # Parse output - Florence returns structured output
        parsed = self._processor.post_process_generation(
            generated_text, task=task_prompt, image_size=pil_image.size
        )

        # Extract caption from parsed output
        caption: str = parsed.get(task_prompt, generated_text)
        return self._clean_caption(caption.strip())

    def caption_batch(self, *images: Image) -> list[str]:
        """Generate captions for multiple images efficiently.

        Returns:
            List of text descriptions
        """
        if not images:
            return []

        task_prompt = self._task_prompt

        # Convert all to PIL
        pil_images = [PILImage.fromarray(img.to_rgb().data) for img in images]

        # Process batch
        inputs = self._processor(
            text=[task_prompt] * len(images), images=pil_images, return_tensors="pt", padding=True
        )
        inputs = self._move_inputs_to_device(inputs)

        # Generate
        with torch.inference_mode():
            generated_ids = self._model.generate(
                **inputs,
                max_new_tokens=256,
                num_beams=3,
                do_sample=False,
            )

        # Decode all
        generated_texts = self._processor.batch_decode(generated_ids, skip_special_tokens=True)

        # Parse outputs
        captions = []
        for text, pil_img in zip(generated_texts, pil_images, strict=True):
            parsed = self._processor.post_process_generation(
                text, task=task_prompt, image_size=pil_img.size
            )
            captions.append(self._clean_caption(parsed.get(task_prompt, text).strip()))

        return captions

    def start(self) -> None:
        """Start the model with a dummy forward pass."""
        # Load model and processor via base class
        super().start()

        # Run a small inference
        dummy = PILImage.new("RGB", (224, 224), color="gray")
        inputs = self._processor(text="<CAPTION>", images=dummy, return_tensors="pt")
        inputs = self._move_inputs_to_device(inputs)

        with torch.inference_mode():
            self._model.generate(**inputs, max_new_tokens=10)

    def stop(self) -> None:
        """Release model and free GPU memory."""
        # Clean up processor cached property
        if "_processor" in self.__dict__:
            del self.__dict__["_processor"]
        # Call parent which handles _model cleanup
        super().stop()

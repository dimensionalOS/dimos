# Copyright 2025 Dimensional Inc.
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

"""
Image embedding module for converting images to vector embeddings.

This module provides a class for generating vector embeddings from images
using pre-trained models like CLIP, ResNet, etc.
"""

import base64
import io
import os
import sys

import cv2
import numpy as np
from PIL import Image

from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ImageEmbeddingProvider:
    """
    A provider for generating vector embeddings from images.

    This class uses pre-trained models to convert images into vector embeddings
    that can be stored in a vector database and used for similarity search.
    """

    def __init__(self, model_name: str = "clip", dimensions: int = 512) -> None:
        """
        Initialize the image embedding provider.

        Args:
            model_name: Name of the embedding model to use ("clip", "resnet", etc.)
            dimensions: Dimensions of the embedding vectors
        """
        self.model_name = model_name
        self.dimensions = dimensions
        self.model = None
        self.processor = None
        self.model_path = None

        self._initialize_model()  # type: ignore[no-untyped-call]

        logger.info(f"ImageEmbeddingProvider initialized with model {model_name}")

    def _initialize_model(self):  # type: ignore[no-untyped-def]
        """Initialize the specified embedding model."""
        try:
            import onnxruntime as ort  # type: ignore[import-untyped]
            import torch
            from transformers import (  # type: ignore[import-untyped]
                AutoFeatureExtractor,
                AutoModel,
                CLIPProcessor,
            )

            if self.model_name == "clip":
                model_id = get_data("models_clip") / "model.onnx"
                self.model_path = str(model_id)  # type: ignore[assignment]  # Store for pickling
                processor_id = "openai/clip-vit-base-patch32"

                providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
                if sys.platform == "darwin":
                    # 2025-11-17 12:36:47.877215 [W:onnxruntime:, helper.cc:82 IsInputSupported] CoreML does not support input dim > 16384. Input:text_model.embeddings.token_embedding.weight, shape: {49408,512}
                    # 2025-11-17 12:36:47.878496 [W:onnxruntime:, coreml_execution_provider.cc:107 GetCapability] CoreMLExecutionProvider::GetCapability, number of partitions supported by CoreML: 88 number of nodes in the graph: 1504 number of nodes supported by CoreML: 933
                    providers = ["CoreMLExecutionProvider"] + [
                        each for each in providers if each != "CUDAExecutionProvider"
                    ]

                self.model = ort.InferenceSession(str(model_id), providers=providers)

                actual_providers = self.model.get_providers()  # type: ignore[attr-defined]
                self.processor = CLIPProcessor.from_pretrained(processor_id)
                logger.info(f"Loaded CLIP model: {model_id} with providers: {actual_providers}")
            elif self.model_name == "resnet":
                model_id = "microsoft/resnet-50"  # type: ignore[assignment]
                self.model = AutoModel.from_pretrained(model_id)
                self.processor = AutoFeatureExtractor.from_pretrained(model_id)
                logger.info(f"Loaded ResNet model: {model_id}")
            else:
                raise ValueError(f"Unsupported model: {self.model_name}")
        except ImportError as e:
            logger.error(f"Failed to import required modules: {e}")
            logger.error("Please install with: pip install transformers torch")
            # Initialize with dummy model for type checking
            self.model = None
            self.processor = None
            raise

    def get_embedding(self, image: np.ndarray | str | bytes) -> np.ndarray:  # type: ignore[type-arg]
        """
        Generate an embedding vector for the provided image.

        Args:
            image: The image to embed, can be a numpy array (OpenCV format),
                  a file path, or a base64-encoded string

        Returns:
            A numpy array containing the embedding vector
        """
        if self.model is None or self.processor is None:
            logger.error("Model not initialized. Using fallback random embedding.")
            return np.random.randn(self.dimensions).astype(np.float32)

        pil_image = self._prepare_image(image)

        try:
            import torch

            if self.model_name == "clip":
                inputs = self.processor(images=pil_image, return_tensors="np")

                with torch.no_grad():
                    ort_inputs = {
                        inp.name: inputs[inp.name]
                        for inp in self.model.get_inputs()
                        if inp.name in inputs
                    }

                    # If required, add dummy text inputs
                    input_names = [i.name for i in self.model.get_inputs()]
                    batch_size = inputs["pixel_values"].shape[0]
                    if "input_ids" in input_names:
                        ort_inputs["input_ids"] = np.zeros((batch_size, 1), dtype=np.int64)
                    if "attention_mask" in input_names:
                        ort_inputs["attention_mask"] = np.ones((batch_size, 1), dtype=np.int64)

                    # Run inference
                    ort_outputs = self.model.run(None, ort_inputs)

                    # Look up correct output name
                    output_names = [o.name for o in self.model.get_outputs()]
                    if "image_embeds" in output_names:
                        image_embedding = ort_outputs[output_names.index("image_embeds")]
                    else:
                        raise RuntimeError(f"No 'image_embeds' found in outputs: {output_names}")

                embedding = image_embedding / np.linalg.norm(image_embedding, axis=1, keepdims=True)
                embedding = embedding[0]

            elif self.model_name == "resnet":
                inputs = self.processor(images=pil_image, return_tensors="pt")

                with torch.no_grad():
                    outputs = self.model(**inputs)

                # Get the [CLS] token embedding
                embedding = outputs.last_hidden_state[:, 0, :].numpy()[0]
            else:
                logger.warning(f"Unsupported model: {self.model_name}. Using random embedding.")
                embedding = np.random.randn(self.dimensions).astype(np.float32)

            # Normalize and ensure correct dimensions
            embedding = embedding / np.linalg.norm(embedding)

            logger.debug(f"Generated embedding with shape {embedding.shape}")
            return embedding

        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            return np.random.randn(self.dimensions).astype(np.float32)

    def get_text_embedding(self, text: str) -> np.ndarray:  # type: ignore[type-arg]
        """
        Generate an embedding vector for the provided text.

        Args:
            text: The text to embed

        Returns:
            A numpy array containing the embedding vector
        """
        if self.model is None or self.processor is None:
            logger.error("Model not initialized. Using fallback random embedding.")
            return np.random.randn(self.dimensions).astype(np.float32)

        if self.model_name != "clip":
            logger.warning(
                f"Text embeddings are only supported with CLIP model, not {self.model_name}. Using random embedding."
            )
            return np.random.randn(self.dimensions).astype(np.float32)

        try:
            import torch

            inputs = self.processor(text=[text], return_tensors="np", padding=True)

            with torch.no_grad():
                # Prepare ONNX input dict (handle only what's needed)
                ort_inputs = {
                    inp.name: inputs[inp.name]
                    for inp in self.model.get_inputs()
                    if inp.name in inputs
                }
                # Determine which inputs are expected by the ONNX model
                input_names = [i.name for i in self.model.get_inputs()]
                batch_size = inputs["input_ids"].shape[0]  # pulled from text input

                # If the model expects pixel_values (i.e., fused model), add dummy vision input
                if "pixel_values" in input_names:
                    ort_inputs["pixel_values"] = np.zeros(
                        (batch_size, 3, 224, 224), dtype=np.float32
                    )

                # Run inference
                ort_outputs = self.model.run(None, ort_inputs)

                # Determine correct output (usually 'last_hidden_state' or 'text_embeds')
                output_names = [o.name for o in self.model.get_outputs()]
                if "text_embeds" in output_names:
                    text_embedding = ort_outputs[output_names.index("text_embeds")]
                else:
                    text_embedding = ort_outputs[0]  # fallback to first output

                # Normalize
                text_embedding = text_embedding / np.linalg.norm(
                    text_embedding, axis=1, keepdims=True
                )
                text_embedding = text_embedding[0]  # shape: (512,)

            logger.debug(
                f"Generated text embedding with shape {text_embedding.shape} for text: '{text}'"
            )
            return text_embedding

        except Exception as e:
            logger.error(f"Error generating text embedding: {e}")
            return np.random.randn(self.dimensions).astype(np.float32)

    def _prepare_image(self, image: np.ndarray | str | bytes) -> Image.Image:  # type: ignore[type-arg]
        """
        Convert the input image to PIL format required by the models.

        Args:
            image: Input image in various formats

        Returns:
            PIL Image object
        """
        if isinstance(image, np.ndarray):
            if len(image.shape) == 3 and image.shape[2] == 3:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                image_rgb = image

            return Image.fromarray(image_rgb)

        elif isinstance(image, str):
            if os.path.isfile(image):
                return Image.open(image)
            else:
                try:
                    image_data = base64.b64decode(image)
                    return Image.open(io.BytesIO(image_data))
                except Exception as e:
                    logger.error(f"Failed to decode image string: {e}")
                    raise ValueError("Invalid image string format")

        elif isinstance(image, bytes):
            return Image.open(io.BytesIO(image))

        else:
            raise ValueError(f"Unsupported image format: {type(image)}")

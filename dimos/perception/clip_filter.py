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

"""
CLIP-based frame filtering for selecting diverse frames from video windows.

Adapted from videorag/clip_filter.py - uses CLIP embeddings to select the most
visually diverse frames from a window, reducing VLM costs while maintaining coverage.
"""

import logging
from typing import Any

from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Optional CLIP imports
try:
    import clip
    from PIL import Image as PILImage
    import torch

    CLIP_AVAILABLE = True
except ImportError:
    CLIP_AVAILABLE = False
    logger.warning(
        "CLIP not available. Install with: pip install torch torchvision openai-clip. "
        "Frame filtering will fall back to simple sampling."
    )


class CLIPFrameFilter:
    """Filter video frames using CLIP embeddings to select diverse frames."""

    def __init__(self, model_name: str = "ViT-B/32", device: str | None = None):
        """
        Initialize CLIP frame filter.

        Args:
            model_name: CLIP model name (e.g., "ViT-B/32", "ViT-L/14")
            device: Device to use ("cuda", "cpu", or None for auto-detect)
        """
        if not CLIP_AVAILABLE:
            raise ImportError(
                "CLIP is not available. Install with: pip install torch torchvision openai-clip"
            )

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        logger.info(f"Loading CLIP model {model_name} on {self.device}")
        self.model, self.preprocess = clip.load(model_name, device=self.device)
        logger.info("CLIP model loaded successfully")

    def _image_to_pil(self, image: Image) -> PILImage.Image:
        """Convert dimos Image to PIL Image."""
        # Get numpy array from dimos Image
        img_array = image.data  # Assumes Image has .data attribute with numpy array

        # Convert to PIL
        return PILImage.fromarray(img_array)

    def _encode_images(self, images: list[Image]) -> torch.Tensor:
        """Encode images using CLIP.

        Args:
            images: List of dimos Images

        Returns:
            Tensor of normalized CLIP embeddings, shape (N, embedding_dim)
        """
        # Convert to PIL and preprocess
        pil_images = [self._image_to_pil(img) for img in images]
        preprocessed = [self.preprocess(img) for img in pil_images]

        # Stack and encode
        image_tensor = torch.stack(preprocessed).to(self.device)
        with torch.no_grad():
            embeddings = self.model.encode_image(image_tensor)
            # Normalize embeddings
            embeddings = embeddings / embeddings.norm(dim=-1, keepdim=True)

        return embeddings

    def select_diverse_frames(self, frames: list[Any], max_frames: int = 3) -> list[Any]:
        """
        Select diverse frames using greedy farthest-point sampling.

        This selects frames that are maximally different from each other in CLIP
        embedding space, ensuring good visual coverage of the window.

        Algorithm:
        1. Always include first frame (temporal anchor)
        2. Iteratively select frame most different from already-selected frames
        3. Continue until we have max_frames frames

        Args:
            frames: List of Frame objects with .image attribute
            max_frames: Maximum number of frames to select

        Returns:
            List of selected Frame objects (subset of input frames)
        """
        if len(frames) <= max_frames:
            return frames

        # Extract images from frames
        images = [f.image for f in frames]

        # Encode all images
        embeddings = self._encode_images(images)

        # Greedy farthest-point sampling
        selected_indices = [0]  # Always include first frame
        remaining_indices = list(range(1, len(frames)))

        while len(selected_indices) < max_frames and remaining_indices:
            selected_embs = embeddings[selected_indices]
            remaining_embs = embeddings[remaining_indices]

            # Compute similarities between remaining and selected
            # Shape: (num_remaining, num_selected)
            similarities = remaining_embs @ selected_embs.T

            # For each remaining frame, find its max similarity to any selected frame
            # Shape: (num_remaining,)
            max_similarities = similarities.max(dim=1)[0]

            # Select frame with minimum max similarity (most different from all selected)
            best_idx = max_similarities.argmin().item()

            selected_indices.append(remaining_indices[best_idx])
            remaining_indices.pop(best_idx)

        # Return frames in temporal order (sorted by index)
        return [frames[i] for i in sorted(selected_indices)]

    def close(self) -> None:
        """Clean up CLIP model."""
        if hasattr(self, "model"):
            del self.model
        if hasattr(self, "preprocess"):
            del self.preprocess


def select_diverse_frames_simple(frames: list[Any], max_frames: int = 3) -> list[Any]:
    """
    Fallback frame selection when CLIP is not available.

    Uses simple uniform sampling across the window.

    Args:
        frames: List of Frame objects
        max_frames: Maximum number of frames to select

    Returns:
        List of selected Frame objects
    """
    if len(frames) <= max_frames:
        return frames

    # Sample uniformly across window
    indices = [int(i * len(frames) / max_frames) for i in range(max_frames)]
    return [frames[i] for i in indices]


__all__ = ["CLIP_AVAILABLE", "CLIPFrameFilter", "select_diverse_frames_simple"]

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

"""CLIP-based frame filtering for selecting diverse frames from video windows."""

from typing import Any

import numpy as np

from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

try:
    import torch  # type: ignore

    from dimos.models.embedding.clip import CLIPModel  # type: ignore

    CLIP_AVAILABLE = True
except ImportError as e:
    CLIP_AVAILABLE = False
    logger.info(f"CLIP unavailable ({e}), using simple frame sampling")


def _get_image_data(image: Image) -> np.ndarray[Any, Any]:
    """Extract numpy array from Image."""
    if not hasattr(image, "data"):
        raise AttributeError(f"Image missing .data attribute: {type(image)}")
    return image.data


if CLIP_AVAILABLE:

    class CLIPFrameFilter:
        """Filter video frames using CLIP embeddings for diversity."""

        def __init__(self, model_name: str = "ViT-B/32", device: str | None = None):
            if not CLIP_AVAILABLE:
                raise ImportError("CLIP not available. Install transformers[torch].")

            resolved_name = (
                "openai/clip-vit-base-patch32" if model_name == "ViT-B/32" else model_name
            )
            if device is None:
                self._model = CLIPModel(model_name=resolved_name)
            else:
                self._model = CLIPModel(model_name=resolved_name, device=device)
            logger.info(f"Loading CLIP {resolved_name} on {self._model.device}")

        def _encode_images(self, images: list[Image]) -> "torch.Tensor":
            """Encode images using CLIP."""
            embeddings = self._model.embed(*images)
            if not isinstance(embeddings, list):
                embeddings = [embeddings]
            vectors = [e.to_torch(self._model.device) for e in embeddings]
            return torch.stack(vectors)

        def select_diverse_frames(self, frames: list[Any], max_frames: int = 3) -> list[Any]:
            """Select diverse frames using greedy farthest-point sampling in CLIP space."""
            if len(frames) <= max_frames:
                return frames

            embeddings = self._encode_images([f.image for f in frames])

            # Greedy farthest-point sampling
            selected_indices = [0]  # Always include first frame
            remaining_indices = list(range(1, len(frames)))

            while len(selected_indices) < max_frames and remaining_indices:
                # Compute similarities: (num_remaining, num_selected)
                similarities = embeddings[remaining_indices] @ embeddings[selected_indices].T
                # Find max similarity for each remaining frame
                max_similarities = similarities.max(dim=1)[0]
                # Select frame most different from all selected
                best_idx = int(max_similarities.argmin().item())

                selected_indices.append(remaining_indices[best_idx])
                remaining_indices.pop(best_idx)

            return [frames[i] for i in sorted(selected_indices)]

        def close(self) -> None:
            """Clean up CLIP model."""
            if hasattr(self, "_model"):
                self._model.stop()
                del self._model


def select_diverse_frames_simple(frames: list[Any], max_frames: int = 3) -> list[Any]:
    """Fallback frame selection: uniform sampling across window."""
    if len(frames) <= max_frames:
        return frames
    indices = [int(i * len(frames) / max_frames) for i in range(max_frames)]
    return [frames[i] for i in indices]


def adaptive_keyframes(
    frames: list[Any],
    min_frames: int = 3,
    max_frames: int = 5,
    change_threshold: float = 15.0,
) -> list[Any]:
    """Select frames based on visual change, adaptive count."""
    if len(frames) <= min_frames:
        return frames

    # Compute frame-to-frame differences
    try:
        diffs = [
            np.abs(
                _get_image_data(frames[i].image).astype(float)
                - _get_image_data(frames[i - 1].image).astype(float)
            ).mean()
            for i in range(1, len(frames))
        ]
    except (AttributeError, ValueError) as e:
        logger.warning(f"Failed to compute frame diffs: {e}. Falling back to uniform sampling.")
        return select_diverse_frames_simple(frames, max_frames)

    total_motion = sum(diffs)
    n_frames = int(np.clip(total_motion / change_threshold, min_frames, max_frames))

    # Always include first and last
    keyframe_indices = {0, len(frames) - 1}

    # Add peaks in diff signal
    for i in range(1, len(diffs) - 1):
        if (
            diffs[i] > diffs[i - 1]
            and diffs[i] > diffs[i + 1]
            and diffs[i] > change_threshold * 0.5
        ):
            keyframe_indices.add(i + 1)

    # Adjust count
    if len(keyframe_indices) > n_frames:
        # Keep first, last, and highest-diff peaks
        middle = [i for i in keyframe_indices if i not in (0, len(frames) - 1)]
        middle_by_diff = sorted(middle, key=lambda i: diffs[i - 1], reverse=True)
        keyframe_indices = {0, len(frames) - 1, *middle_by_diff[: n_frames - 2]}
    elif len(keyframe_indices) < n_frames:
        # Fill uniformly from remaining
        needed = n_frames - len(keyframe_indices)
        candidates = sorted(set(range(len(frames))) - keyframe_indices)
        if candidates:
            step = max(1, len(candidates) // (needed + 1))
            keyframe_indices.update(candidates[::step][:needed])

    return [frames[i] for i in sorted(keyframe_indices)]


__all__ = [
    "CLIP_AVAILABLE",
    "adaptive_keyframes",
    "select_diverse_frames_simple",
]

if CLIP_AVAILABLE:
    __all__.append("CLIPFrameFilter")

"""Point-cloud selection helpers for private VQA primitives."""

from __future__ import annotations

import numpy as np

from dimos.benchmark.vqa.contracts import CalibratedFrame
from dimos.benchmark.vqa.generation.primitives.projection import project_visible_points


def points_in_mask(frame: CalibratedFrame, mask: np.ndarray) -> np.ndarray:
    """Return nearest visible camera points covered by one foreground mask."""
    if mask.shape != (frame.image.height, frame.image.width):
        raise ValueError("segmentation mask dimensions must match the image")
    projected = project_visible_points(frame)
    return np.asarray(
        [
            point
            for point, (x, y) in zip(projected.camera_points, projected.pixels, strict=True)
            if mask[y, x] > 0
        ],
        dtype=np.float64,
    )

"""Pure frame-difference fallback for subject-motion detection.

When the primary tracker (EdgeTAM) drops the subject -- common when a fast
mover blurs or briefly leaves the mask -- this finds the largest *moving* region
between two consecutive frames, so the monitor can cheaply re-seed the tracker
on it instead of escalating straight to the (slow, networked) VLM reacquire.

Pure: numpy + OpenCV only, no DimOS, no models -- unit tested with synthetic
frames. Operates on BGR uint8 arrays (``Image.to_opencv()``).

Assumes a roughly static camera: it returns the largest moving blob over the
whole frame, so under camera ego-motion (e.g. a wandering robot) the background
can dominate. Gate it on robot motion state -- or disable it via the monitor's
``motion_fallback`` config -- when the camera itself is moving.
"""

from __future__ import annotations

import cv2
import numpy as np

Bbox = tuple[float, float, float, float]


def detect_motion_bbox(
    prev_bgr: np.ndarray,
    curr_bgr: np.ndarray,
    diff_threshold: int = 25,
    min_area_fraction: float = 0.0008,
) -> Bbox | None:
    """Bounding box of the largest moving region between two frames.

    Args:
        prev_bgr: Previous frame, BGR uint8 ``(H, W, 3)``.
        curr_bgr: Current frame, BGR uint8 ``(H, W, 3)``; must match ``prev``.
        diff_threshold: Per-pixel grayscale difference counted as motion.
        min_area_fraction: Reject moving blobs smaller than this fraction of
            the image (suppresses sensor noise).

    Returns:
        ``(x1, y1, x2, y2)`` of the largest moving blob, or None if nothing
        moved enough or the frames are unusable.
    """
    if prev_bgr.shape != curr_bgr.shape or prev_bgr.ndim != 3:
        return None
    height, width = curr_bgr.shape[:2]
    prev_gray = cv2.cvtColor(prev_bgr, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(curr_bgr, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(prev_gray, curr_gray)
    _, mask = cv2.threshold(diff, diff_threshold, 255, cv2.THRESH_BINARY)
    # Dilate so the disjoint halves of one moving subject merge into one blob.
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=2)

    count, _labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count <= 1:  # only the background component
        return None
    # Largest non-background component (index 0 is the background).
    areas = stats[1:, cv2.CC_STAT_AREA]
    best = 1 + int(np.argmax(areas))
    if stats[best, cv2.CC_STAT_AREA] < min_area_fraction * width * height:
        return None
    x = stats[best, cv2.CC_STAT_LEFT]
    y = stats[best, cv2.CC_STAT_TOP]
    w = stats[best, cv2.CC_STAT_WIDTH]
    h = stats[best, cv2.CC_STAT_HEIGHT]
    return (float(x), float(y), float(x + w), float(y + h))

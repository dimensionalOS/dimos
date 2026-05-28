"""Tests for the pure frame-difference subject-motion fallback."""

import cv2
import numpy as np

from pawtrack.motion_fallback import detect_motion_bbox


def _frame(cx):
    img = np.full((200, 320, 3), 200, np.uint8)
    cv2.circle(img, (cx, 100), 25, (0, 0, 255), -1)
    return img


def test_detects_a_moving_blob():
    bbox = detect_motion_bbox(_frame(80), _frame(220))
    assert bbox is not None
    x1, y1, x2, y2 = bbox
    # The moving region sits in the marker's vertical band and is marker-sized.
    assert y1 < 100 < y2
    assert (x2 - x1) > 20 and (y2 - y1) > 20


def test_no_motion_returns_none():
    frame = _frame(150)
    assert detect_motion_bbox(frame, frame.copy()) is None


def test_mismatched_shapes_returns_none():
    a = np.zeros((10, 10, 3), np.uint8)
    b = np.zeros((20, 20, 3), np.uint8)
    assert detect_motion_bbox(a, b) is None

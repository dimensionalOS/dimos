from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class VisionObstruction:
    blocked: bool
    contour_area: float
    bbox: tuple[int, int, int, int] | None


class CenterLaneVisionDetector:
    def __init__(
        self,
        roi_x_frac: tuple[float, float] = (0.28, 0.72),
        roi_y_frac: tuple[float, float] = (0.42, 0.95),
        min_contour_area: float = 2200.0,
    ) -> None:
        self._roi_x_frac = roi_x_frac
        self._roi_y_frac = roi_y_frac
        self._min_contour_area = min_contour_area

    def detect(self, frame: np.ndarray) -> VisionObstruction:
        if frame is None or frame.size == 0:
            return VisionObstruction(False, 0.0, None)

        h, w = frame.shape[:2]
        x1 = int(w * self._roi_x_frac[0])
        x2 = int(w * self._roi_x_frac[1])
        y1 = int(h * self._roi_y_frac[0])
        y2 = int(h * self._roi_y_frac[1])
        if x2 <= x1 or y2 <= y1:
            return VisionObstruction(False, 0.0, None)

        roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 140)
        kernel = np.ones((5, 5), np.uint8)
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_area = 0.0
        best_bbox: tuple[int, int, int, int] | None = None
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self._min_contour_area:
                continue
            rx, ry, rw, rh = cv2.boundingRect(contour)
            # Bias toward closer/lower-center obstacles and ignore thin floor reflections.
            if rh < 24 or rw < 24:
                continue
            if (ry + rh) < int((y2 - y1) * 0.58):
                continue
            if area > best_area:
                best_area = area
                best_bbox = (x1 + rx, y1 + ry, x1 + rx + rw, y1 + ry + rh)

        return VisionObstruction(best_bbox is not None, best_area, best_bbox)

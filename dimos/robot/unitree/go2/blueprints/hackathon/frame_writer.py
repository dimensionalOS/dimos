"""Shared annotated frame writer for dashboard visualization.

Modules call write_annotated_frame() to push an OpenCV-annotated JPEG to
/tmp/go2_hackathon_frame.jpg, which the dashboard MJPEG stream reads.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

FRAME_PATH  = Path("/tmp/go2_hackathon_frame.jpg")
STATE_PATH  = Path("/tmp/go2_hackathon_state.json")

# Threat level colors: BGR
_COLORS = {
    "aggressive": (0,   0,   220),   # red
    "cautious":   (0,  160,  255),   # orange
    "curious":    (0,  200,   80),   # green
    "tracking":   (255, 200,   0),   # cyan
    "approach":   (255,  80, 200),   # purple
    "unknown":    (180, 180, 180),   # grey
}

_FONT      = cv2.FONT_HERSHEY_SIMPLEX
_FONT_SCALE = 0.5
_THICKNESS  = 1


def _threat_label(threat: float | None) -> tuple[str, tuple]:
    if threat is None:
        return "tracking", _COLORS["tracking"]
    if threat >= 0.7:
        return f"THREAT {threat:.0%}", _COLORS["aggressive"]
    if threat >= 0.4:
        return f"cautious {threat:.0%}", _COLORS["cautious"]
    return f"curious {threat:.0%}", _COLORS["curious"]


def write_annotated_frame(
    image_bgr: np.ndarray,
    detections: list[dict],   # [{"bbox":(x1,y1,x2,y2), "track_id":int, "threat":float|None, "label":str}]
    overlay_text: str = "",
    quality: int = 75,
) -> None:
    """Draw bboxes + labels on frame and write to shared JPEG path."""
    frame = image_bgr.copy()
    h, w = frame.shape[:2]

    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det["bbox"]]
        tid   = det.get("track_id", -1)
        threat = det.get("threat")
        label_text, color = _threat_label(threat)
        custom = det.get("label")
        if custom:
            label_text = custom

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        tag = f"#{tid} {label_text}"
        (tw, th), _ = cv2.getTextSize(tag, _FONT, _FONT_SCALE, _THICKNESS)
        ty = max(y1 - 6, th + 4)
        cv2.rectangle(frame, (x1, ty - th - 4), (x1 + tw + 4, ty + 2), color, -1)
        cv2.putText(frame, tag, (x1 + 2, ty), _FONT, _FONT_SCALE, (255, 255, 255), _THICKNESS, cv2.LINE_AA)

    # Overlay text (state label) in top-left
    if overlay_text:
        lines = overlay_text.split("\n")
        y0 = 22
        for line in lines:
            (lw, lh), _ = cv2.getTextSize(line, _FONT, 0.6, 2)
            cv2.rectangle(frame, (8, y0 - lh - 4), (14 + lw, y0 + 4), (30, 30, 30), -1)
            cv2.putText(frame, line, (10, y0), _FONT, 0.6, (0, 255, 180), 2, cv2.LINE_AA)
            y0 += lh + 10

    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if ok:
        FRAME_PATH.write_bytes(buf.tobytes())


def write_state(state: dict[str, Any]) -> None:
    """Write current module state as JSON for dashboard polling."""
    state["ts"] = time.time()
    STATE_PATH.write_text(json.dumps(state))


def read_state() -> dict[str, Any]:
    try:
        return json.loads(STATE_PATH.read_text())
    except Exception:
        return {}

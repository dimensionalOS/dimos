from __future__ import annotations

import asyncio
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from demo_app.types import AlertEvent


class CaptureBuffer:
    def __init__(self, buffer_seconds: int, fps: int, output_dir: Path):
        self._buffer: deque[tuple[float, np.ndarray]] = deque(maxlen=buffer_seconds * fps)
        self._fps = fps
        self._output_dir = output_dir
        self._output_dir.mkdir(parents=True, exist_ok=True)

    def push(self, frame: np.ndarray, timestamp: float) -> None:
        self._buffer.append((timestamp, frame.copy()))

    async def snapshot(self, event: AlertEvent) -> Path:
        return await asyncio.to_thread(self._encode_snapshot, event)

    async def snapshot_and_clip(self, event: AlertEvent, duration_sec: float) -> tuple[Path, Path]:
        return await asyncio.to_thread(self._encode, event, duration_sec)

    def _encode_snapshot(self, event: AlertEvent) -> Path:
        ts_label = int(event.timestamp * 1000)
        jpg_path = self._output_dir / f"obstruction_{ts_label}.jpg"
        annotated = event.frame.copy()
        for idx, line in enumerate([
            "AISLE OBSTRUCTION DETECTED",
            f"dir={event.obstruction_direction or 'center'} "
            f"dist={event.obstruction_distance_m:.2f}m" if event.obstruction_distance_m is not None else "dist=unknown",
            f"points={event.obstruction_point_count}",
        ]):
            y = 28 + idx * 28
            cv2.putText(
                annotated,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )

        detections = event.evidence_detections or []
        if detections:
            for det in detections:
                x1, y1, x2, y2 = det.bbox
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
                label = f"{det.class_name} {det.confidence:.2f}"
                cv2.putText(
                    annotated,
                    label,
                    (x1, max(90, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )
        elif event.bbox is not None:
            x1, y1, x2, y2 = event.bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imwrite(str(jpg_path), annotated)
        return jpg_path

    def _encode(self, event: AlertEvent, duration_sec: float) -> tuple[Path, Path]:
        jpg_path = self._encode_snapshot(event)
        ts_label = int(event.timestamp * 1000)
        mp4_path = self._output_dir / f"anomaly_{ts_label}.mp4"
        annotated = event.frame.copy()

        half = duration_sec / 2.0
        window = [
            (ts, frame)
            for ts, frame in list(self._buffer)
            if event.timestamp - half <= ts <= event.timestamp + half
        ]
        if not window:
            fallback = list(self._buffer)[-int(duration_sec * self._fps):]
            window = fallback or [(event.timestamp, annotated)]

        h, w = window[0][1].shape[:2]
        writer = cv2.VideoWriter(
            str(mp4_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            self._fps,
            (w, h),
        )
        for _, frame in window:
            writer.write(frame)
        writer.release()
        return jpg_path, mp4_path

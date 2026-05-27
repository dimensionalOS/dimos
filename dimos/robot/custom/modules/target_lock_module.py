from __future__ import annotations

import json
import threading
import time
from typing import Any, Literal

from dimos_lcm.std_msgs import String
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray

_DEFAULT_FRAME_ID = "camera_optical"

LockState = Literal["unselected", "locked", "searching", "lost"]


class TargetLockConfig(ModuleConfig):
    search_timeout_sec: float = 3.0
    reacquire_by_class: bool = True


class TargetLockModule(Module):
    """Maintain a stable single-target lock from selected_bbox + detections."""

    config: TargetLockConfig

    detections: In[Detection2DArray]
    selected_bbox: In[Detection2DArray]

    locked_bbox: Out[Detection2DArray]
    lock_status: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._state: LockState = "unselected"
        self._target_id: str | None = None
        self._target_class_id: str | None = None
        self._last_center: tuple[float, float] | None = None
        self._last_seen_at: float | None = None
        self._last_header: Header | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.selected_bbox.subscribe(self._on_selected_bbox)))
        self.register_disposable(Disposable(self.detections.subscribe(self._on_detections)))
        self._publish_status(force=True)

    @rpc
    def clear_lock(self) -> str:
        with self._lock:
            self._reset_lock_state()
            header = self._last_header
        self.locked_bbox.publish(self._empty_detection_array(header))
        self._set_state("unselected")
        return "target lock cleared"

    @rpc
    def get_lock_state(self) -> dict[str, Any]:
        with self._lock:
            return {
                "state": self._state,
                "target_id": self._target_id,
                "target_class_id": self._target_class_id,
                "last_seen_at": self._last_seen_at,
            }

    def _on_selected_bbox(self, selected_bbox: Detection2DArray) -> None:
        with self._lock:
            self._last_header = selected_bbox.header

        detection = self._extract_single_detection(selected_bbox)
        if detection is None:
            with self._lock:
                self._reset_lock_state()
            self.locked_bbox.publish(self._empty_detection_array(selected_bbox.header))
            self._set_state("unselected")
            return

        now = time.monotonic()
        center = self._bbox_center(detection)
        class_id = self._detection_class_id(detection)

        with self._lock:
            self._target_id = self._detection_id(detection, fallback_index=0)
            self._target_class_id = class_id
            self._last_center = center
            self._last_seen_at = now

        self.locked_bbox.publish(self._single_detection_array(detection, selected_bbox.header))
        self._set_state("locked")

    def _on_detections(self, detections: Detection2DArray) -> None:
        with self._lock:
            self._last_header = detections.header
            target_id = self._target_id
            target_class_id = self._target_class_id
            last_center = self._last_center
            last_seen_at = self._last_seen_at

        if target_id is None:
            self.locked_bbox.publish(self._empty_detection_array(detections.header))
            self._set_state("unselected")
            return

        matched = self._find_by_id(detections, target_id)
        if matched is not None:
            self._update_lock_from_detection(matched, time.monotonic())
            self.locked_bbox.publish(self._single_detection_array(matched, detections.header))
            self._set_state("locked")
            return

        now = time.monotonic()
        if last_seen_at is None:
            self.locked_bbox.publish(self._empty_detection_array(detections.header))
            self._set_state("searching")
            return

        if now - last_seen_at > max(self.config.search_timeout_sec, 0.0):
            self.locked_bbox.publish(self._empty_detection_array(detections.header))
            self._set_state("lost")
            return

        reacquired = self._reacquire_candidate(detections, target_class_id, last_center)
        if reacquired is None:
            self.locked_bbox.publish(self._empty_detection_array(detections.header))
            self._set_state("searching")
            return

        self._update_lock_from_detection(reacquired, now)
        self.locked_bbox.publish(self._single_detection_array(reacquired, detections.header))
        self._set_state("locked")

    def _set_state(self, state: LockState) -> None:
        with self._lock:
            old_state = self._state
            self._state = state
        self._publish_status(force=old_state != state)

    def _publish_status(self, force: bool = False) -> None:
        with self._lock:
            state = self._state
            payload = {
                "state": state,
                "target_id": self._target_id,
                "target_class_id": self._target_class_id,
            }
        if force or state in ("unselected", "lost"):
            self.lock_status.publish(String(json.dumps(payload, ensure_ascii=True)))

    def _update_lock_from_detection(self, detection: Any, now: float) -> None:
        with self._lock:
            self._target_id = self._detection_id(detection, fallback_index=0)
            self._target_class_id = self._detection_class_id(detection)
            self._last_center = self._bbox_center(detection)
            self._last_seen_at = now

    def _reset_lock_state(self) -> None:
        self._target_id = None
        self._target_class_id = None
        self._last_center = None
        self._last_seen_at = None

    def _reacquire_candidate(
        self,
        detections: Detection2DArray,
        target_class_id: str | None,
        last_center: tuple[float, float] | None,
    ) -> Any | None:
        candidates: list[Any] = list(detections.detections)
        if not candidates:
            return None

        if self.config.reacquire_by_class and target_class_id is not None:
            class_filtered = [d for d in candidates if self._detection_class_id(d) == target_class_id]
            if class_filtered:
                candidates = class_filtered

        if not candidates:
            return None

        if last_center is None:
            return candidates[0]

        return min(
            candidates,
            key=lambda detection: self._center_distance_sq(last_center, self._bbox_center(detection)),
        )

    def _find_by_id(self, detections: Detection2DArray, target_id: str) -> Any | None:
        for index, detection in enumerate(detections.detections):
            if self._detection_id(detection, fallback_index=index) == target_id:
                return detection
        return None

    @staticmethod
    def _extract_single_detection(detections: Detection2DArray | None) -> Any | None:
        if detections is None or not detections.detections:
            return None
        return detections.detections[0]

    @staticmethod
    def _single_detection_array(detection: Any, header: Header | None) -> Detection2DArray:
        safe_header = header if header is not None else Header(time.time(), _DEFAULT_FRAME_ID)
        return Detection2DArray(detections_length=1, header=safe_header, detections=[detection])

    @staticmethod
    def _empty_detection_array(header: Header | None) -> Detection2DArray:
        safe_header = header if header is not None else Header(time.time(), _DEFAULT_FRAME_ID)
        return Detection2DArray(detections_length=0, header=safe_header, detections=[])

    @staticmethod
    def _detection_id(detection: Any, fallback_index: int) -> str:
        detection_id = getattr(detection, "id", "")
        return str(detection_id) if detection_id else str(fallback_index)

    @staticmethod
    def _detection_class_id(detection: Any) -> str | None:
        results = getattr(detection, "results", [])
        if not results:
            return None
        hypothesis = results[0].hypothesis
        class_id = getattr(hypothesis, "class_id", None)
        return str(class_id) if class_id is not None else None

    @staticmethod
    def _bbox_center(detection: Any) -> tuple[float, float]:
        center = detection.bbox.center.position
        return float(center.x), float(center.y)

    @staticmethod
    def _center_distance_sq(a: tuple[float, float], b: tuple[float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx * dx + dy * dy


__all__ = [
    "TargetLockConfig",
    "TargetLockModule",
]
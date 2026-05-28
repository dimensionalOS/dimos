from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from dimos.experimental.dogops.config_loader import DEFAULT_SITE, load_site_config
from dimos.experimental.dogops.detector import DetectedTag, DogOpsTagDetector
from dimos.experimental.dogops.models import Observation

try:  # pragma: no cover - exercised only inside a full DimOS checkout.
    from reactivex.disposable import Disposable as RxDisposable
except ModuleNotFoundError:  # pragma: no cover - local project-pack fallback.
    RxDisposable = None

try:  # pragma: no cover - exercised only inside a full DimOS checkout.
    from dimos.core.module import Module
except ModuleNotFoundError:

    class Module:
        def __init__(self, **_: object) -> None:
            pass

        @classmethod
        def blueprint(cls, **kwargs: object) -> dict[str, object]:
            return {"module": cls.__name__, "kwargs": kwargs}

        def start(self) -> None:
            pass

        def stop(self) -> None:
            pass


try:  # pragma: no cover - exercised only inside a full DimOS checkout.
    from dimos.core.stream import In
    from dimos.msgs.sensor_msgs.Image import Image
except ModuleNotFoundError:  # pragma: no cover - local project-pack fallback.

    class In:
        def __class_getitem__(cls, _: object) -> type[In]:
            return cls

    Image = Any  # type: ignore[misc, assignment]


class DogOpsObservationModule(Module):
    color_image: In[Image]

    def __init__(self, *, site_path: str | Path = DEFAULT_SITE, **_: object) -> None:
        super().__init__(**_)
        self.site_path = Path(site_path)
        self.site = load_site_config(self.site_path)
        self.detector = DogOpsTagDetector(self.site)
        self._latest_image: object | None = None
        self._latest_image_received_at: float | None = None

    def start(self) -> None:  # pragma: no cover - lifecycle exercised in full DimOS.
        super().start()
        color_image = getattr(self, "color_image", None)
        if color_image is not None and hasattr(color_image, "subscribe"):
            subscription = color_image.subscribe(self.ingest_camera_image)
            _register_subscription(self, subscription)

    def ingest_camera_image(self, image: object) -> None:
        self._latest_image = image
        self._latest_image_received_at = time.time()

    def observe_simulated_tags(
        self,
        tag_ids: list[int],
        *,
        zone_id: str,
        run_id: str = "simulated",
    ) -> list[Observation]:
        return [
            self._observation_for_detection(detection, zone_id=zone_id, run_id=run_id)
            for detection in self.detector.detect_simulated(tag_ids)
        ]

    def observe_image(
        self,
        image_path: str | Path,
        *,
        zone_id: str,
        run_id: str = "image",
    ) -> list[Observation]:
        return [
            self._observation_for_detection(detection, zone_id=zone_id, run_id=run_id)
            for detection in self.detector.detect_image(image_path)
        ]

    def observe_latest_image(
        self,
        *,
        zone_id: str,
        run_id: str = "camera",
    ) -> list[Observation]:
        if self._latest_image is None:
            return []
        return [
            self._observation_for_detection(detection, zone_id=zone_id, run_id=run_id)
            for detection in self.detector.detect_dimos_image(self._latest_image)
        ]

    def image_stream_status(self) -> dict[str, object]:
        if self._latest_image is not None and self._latest_image_received_at is not None:
            shape = getattr(self._latest_image, "shape", None)
            data = getattr(self._latest_image, "data", None)
            if shape is None and data is not None:
                shape = getattr(data, "shape", None)
            return {
                "ok": True,
                "mode": "latest_frame",
                "frame_age_s": round(time.time() - self._latest_image_received_at, 3),
                "frame_id": getattr(self._latest_image, "frame_id", None),
                "shape": [int(item) for item in shape] if shape is not None else None,
            }
        return {
            "ok": False,
            "mode": "not_subscribed",
            "fallback": "use observe_simulated_tags or observe_image",
        }

    @staticmethod
    def _observation_for_detection(
        detection: DetectedTag,
        *,
        zone_id: str,
        run_id: str,
    ) -> Observation:
        return Observation(
            id=f"TAG-{detection.tag_id}",
            ts=time.time(),
            run_id=run_id,
            entity_id=detection.entity_id,
            tag_id=detection.tag_id,
            zone_id=zone_id,
            facts={
                "entity_kind": detection.entity_kind or "unknown",
                "detection_source": detection.source,
                "frame_id": detection.frame_id or "",
                "center_px": _format_point(detection.center_px),
                "area_px": round(detection.area_px, 3) if detection.area_px is not None else 0.0,
            },
            confidence=detection.confidence,
            source=detection.source,
        )


def _format_point(point: tuple[float, float] | None) -> str:
    if point is None:
        return ""
    return f"{point[0]:.1f},{point[1]:.1f}"


def _register_subscription(owner: object, subscription: object) -> None:
    register = getattr(owner, "register_disposable", None)
    if not callable(register):
        return
    if callable(subscription) and RxDisposable is not None:
        register(RxDisposable(subscription))
        return
    register(subscription)

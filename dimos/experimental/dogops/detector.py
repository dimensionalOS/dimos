from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from dimos.experimental.dogops.config_loader import DEFAULT_SITE, load_site_config
from dimos.experimental.dogops.models import SiteConfig
from dimos.experimental.dogops.tag_registry import DogOpsTagRegistry, TagRegistration


@dataclass(frozen=True)
class DetectedTag:
    tag_id: int
    entity_id: str | None
    entity_kind: str | None
    zone_id: str | None
    corners: tuple[tuple[float, float], ...] = ()
    center_px: tuple[float, float] | None = None
    area_px: float | None = None
    frame_id: str | None = None
    confidence: float = 1.0
    source: str = "simulation"


class DogOpsTagDetector:
    def __init__(self, site: SiteConfig | None = None) -> None:
        self.site = site or load_site_config(DEFAULT_SITE)
        self.registry = DogOpsTagRegistry(self.site)

    def detect_simulated(self, tag_ids: list[int], *, source: str = "simulation") -> list[DetectedTag]:
        return [self._detected_from_registration(tag_id, source=source) for tag_id in tag_ids]

    def detect_image(self, image_path: str | Path) -> list[DetectedTag]:
        cv2 = _import_cv2()
        image = cv2.imread(str(image_path))
        if image is None:
            raise ValueError(f"failed to read image: {image_path}")
        return self.detect_array(image, source="cv2.imread")

    def detect_dimos_image(self, image: Any) -> list[DetectedTag]:
        array, frame_id = _opencv_array_from_image(image)
        return self.detect_array(array, source="dimos.color_image", frame_id=frame_id)

    def detect_array(
        self,
        image: Any,
        *,
        source: str = "cv2.aruco",
        frame_id: str | None = None,
    ) -> list[DetectedTag]:
        cv2 = _import_cv2()
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        parameters = aruco.DetectorParameters()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(dictionary, parameters)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
        if ids is None:
            return []
        detections: list[DetectedTag] = []
        for index, raw_id in enumerate(ids.flatten().tolist()):
            tag_corners = tuple(
                (float(point[0]), float(point[1])) for point in corners[index].reshape(-1, 2)
            )
            detections.append(
                self._detected_from_registration(
                    int(raw_id),
                    corners=tag_corners,
                    source=source,
                    frame_id=frame_id,
                )
            )
        return detections

    def _detected_from_registration(
        self,
        tag_id: int,
        *,
        corners: tuple[tuple[float, float], ...] = (),
        frame_id: str | None = None,
        source: str,
    ) -> DetectedTag:
        registration = self.registry.get(tag_id)
        return _detected_tag(
            tag_id,
            registration,
            corners=corners,
            frame_id=frame_id,
            source=source,
        )


def _detected_tag(
    tag_id: int,
    registration: TagRegistration | None,
    *,
    corners: tuple[tuple[float, float], ...],
    frame_id: str | None,
    source: str,
) -> DetectedTag:
    center_px = _corner_center(corners)
    area_px = _corner_area(corners)
    return DetectedTag(
        tag_id=tag_id,
        entity_id=registration.entity_id if registration else None,
        entity_kind=registration.entity_kind if registration else None,
        zone_id=registration.zone_id if registration else None,
        corners=corners,
        center_px=center_px,
        area_px=area_px,
        frame_id=frame_id,
        confidence=1.0,
        source=source,
    )


def _opencv_array_from_image(image: Any) -> tuple[Any, str | None]:
    frame_id = getattr(image, "frame_id", None)
    if hasattr(image, "to_opencv"):
        return image.to_opencv(), frame_id
    data = getattr(image, "data", None)
    if data is not None and hasattr(data, "shape"):
        return data, frame_id
    return image, frame_id


def _corner_center(corners: tuple[tuple[float, float], ...]) -> tuple[float, float] | None:
    if not corners:
        return None
    x = sum(point[0] for point in corners) / len(corners)
    y = sum(point[1] for point in corners) / len(corners)
    return (x, y)


def _corner_area(corners: tuple[tuple[float, float], ...]) -> float | None:
    if len(corners) < 3:
        return None
    area = 0.0
    for index, point in enumerate(corners):
        next_point = corners[(index + 1) % len(corners)]
        area += point[0] * next_point[1] - next_point[0] * point[1]
    return abs(area) / 2.0


def _import_cv2() -> Any:
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "OpenCV aruco support is not installed. Install the optional vision extra: "
            "uv run --extra vision ..."
        ) from exc
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("Installed OpenCV package does not include cv2.aruco")
    return cv2

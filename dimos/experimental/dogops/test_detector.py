from __future__ import annotations

import pytest

from dimos.experimental.dogops.detector import DogOpsTagDetector
from dimos.experimental.dogops.tag_registry import DogOpsTagRegistry


class _ImageLike:
    def __init__(self, image, *, frame_id: str = "camera_optical") -> None:
        self._image = image
        self.frame_id = frame_id

    def to_opencv(self):
        return self._image


def test_tag_registry_resolves_demo_entities(dogops_site) -> None:
    registry = DogOpsTagRegistry(dogops_site)

    assert registry.require(104).entity_id == "PKG-104"
    assert registry.require(41).entity_id == "COOLING_1"
    assert registry.require(70).entity_id == "PORTAL_1"


def test_detector_resolves_simulated_tags(dogops_site) -> None:
    detector = DogOpsTagDetector(dogops_site)

    detections = detector.detect_simulated([20, 101, 102, 999])
    by_tag = {detection.tag_id: detection for detection in detections}

    assert by_tag[20].entity_id == "INBOUND_DOCK"
    assert by_tag[101].entity_id == "PKG-101"
    assert by_tag[999].entity_id is None
    assert by_tag[999].entity_kind is None


def test_detector_raises_clear_error_without_opencv(dogops_site) -> None:
    cv2 = pytest.importorskip("cv2")
    if hasattr(cv2, "aruco"):
        pytest.skip("OpenCV aruco is available")

    detector = DogOpsTagDetector(dogops_site)
    with pytest.raises(RuntimeError, match="aruco"):
        detector.detect_array(None)


def test_detector_reads_generated_apriltag_with_opencv(dogops_site) -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    if not hasattr(cv2, "aruco"):
        pytest.skip("OpenCV aruco is unavailable")
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(dictionary, 104, 240)
    else:
        marker = aruco.drawMarker(dictionary, 104, 240)
    canvas = np.full((320, 320), 255, dtype=marker.dtype)
    canvas[40:280, 40:280] = marker

    detector = DogOpsTagDetector(dogops_site)
    detections = detector.detect_array(canvas)

    assert detections[0].tag_id == 104
    assert detections[0].entity_id == "PKG-104"


def test_detector_reads_dimos_image_like_frame_with_opencv(dogops_site) -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    if not hasattr(cv2, "aruco"):
        pytest.skip("OpenCV aruco is unavailable")
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(dictionary, 104, 240)
    else:
        marker = aruco.drawMarker(dictionary, 104, 240)
    canvas = np.full((320, 320), 255, dtype=marker.dtype)
    canvas[40:280, 40:280] = marker

    detector = DogOpsTagDetector(dogops_site)
    detections = detector.detect_dimos_image(_ImageLike(canvas))

    assert detections[0].tag_id == 104
    assert detections[0].entity_id == "PKG-104"
    assert detections[0].frame_id == "camera_optical"
    assert detections[0].center_px == (159.5, 159.5)
    assert detections[0].area_px is not None

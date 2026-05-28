from collections.abc import Iterator
from contextlib import contextmanager

import pytest

from dimos.experimental.dogops.observation_module import DogOpsObservationModule


class _ImageLike:
    def __init__(self, image, *, frame_id: str = "camera_optical") -> None:
        self._image = image
        self.frame_id = frame_id

    def to_opencv(self):
        return self._image


@contextmanager
def _observation_module() -> Iterator[DogOpsObservationModule]:
    module = DogOpsObservationModule()
    try:
        yield module
    finally:
        module.stop()


def test_observation_module_builds_observations_from_simulated_tags() -> None:
    with _observation_module() as module:
        observations = module.observe_simulated_tags([20, 101, 102], zone_id="INBOUND_DOCK")

    assert [obs.tag_id for obs in observations] == [20, 101, 102]
    assert observations[0].entity_id == "INBOUND_DOCK"
    assert observations[1].entity_id == "PKG-101"
    assert observations[1].facts["detection_source"] == "simulation"


def test_observation_module_reports_stream_fallback() -> None:
    with _observation_module() as module:
        status = module.image_stream_status()

    assert status["ok"] is False
    assert status["mode"] == "not_subscribed"


def test_observation_module_scans_latest_camera_frame() -> None:
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

    with _observation_module() as module:
        module.ingest_camera_image(_ImageLike(canvas))
        status = module.image_stream_status()
        observations = module.observe_latest_image(zone_id="QA_HOLD")

    assert status["ok"] is True
    assert status["mode"] == "latest_frame"
    assert observations[0].tag_id == 104
    assert observations[0].entity_id == "PKG-104"
    assert observations[0].facts["detection_source"] == "dimos.color_image"
    assert observations[0].facts["frame_id"] == "camera_optical"

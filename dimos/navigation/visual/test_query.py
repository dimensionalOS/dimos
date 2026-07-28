from __future__ import annotations

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.visual.query import get_object_bbox_from_image


class _RecordingVlModel:
    def __init__(self, response: str) -> None:
        self.response = response
        self.prompt = ""

    def query(self, image: Image, query: str) -> str:
        self.prompt = query
        return self.response


def test_bbox_query_requests_strict_normalized_json_and_returns_pixels() -> None:
    image = Image.from_numpy(np.zeros((480, 640, 3), dtype=np.uint8))
    model = _RecordingVlModel(
        '{"name":"center person","bbox":[250,100,750,900]}'
    )

    bbox = get_object_bbox_from_image(model, image, "the center person")  # type: ignore[arg-type]

    assert bbox == (160.0, 48.0, 480.0, 432.0)
    assert "normalized 0-1000 coordinate scale" in model.prompt
    assert "strict JSON with double quotes" in model.prompt


def test_bbox_query_rejects_non_json_and_missing_results() -> None:
    image = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8))

    assert (
        get_object_bbox_from_image(
            _RecordingVlModel("{'bbox': [1, 2, 3, 4]}"),  # type: ignore[arg-type]
            image,
            "person",
        )
        is None
    )


def test_bbox_query_rejects_invalid_normalized_coordinates() -> None:
    image = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8))

    for response in (
        '{"bbox":[-1,0,500,500]}',
        '{"bbox":[0,0,1001,500]}',
        '{"bbox":[500,0,100,500]}',
        '{"bbox":[0,500,500,100]}',
    ):
        assert (
            get_object_bbox_from_image(  # type: ignore[arg-type]
                _RecordingVlModel(response),
                image,
                "person",
            )
            is None
        )
    assert (
        get_object_bbox_from_image(
            _RecordingVlModel("null"),  # type: ignore[arg-type]
            image,
            "person",
        )
        is None
    )

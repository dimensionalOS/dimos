# Copyright 2026 Dimensional Inc.

from dimos.benchmark.vqa.contracts import GroundedObject
from dimos.benchmark.vqa.generation.primitives.selection import select_nearest_object


def test_select_nearest_object_optionally_restricts_to_image_side() -> None:
    objects = [
        GroundedObject("left", "chair", 3, 2.0, "left"),
        GroundedObject("right", "chair", 3, 1.0, "right"),
    ]

    assert select_nearest_object(objects).id == "right"
    assert select_nearest_object(objects, "left").id == "left"
    assert select_nearest_object(objects, "center") is None

# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for image-plane relation derivation."""

from math import inf, nextafter

import pytest

from dimos.benchmark.spatiotemporal import relations
from dimos.benchmark.spatiotemporal.models import BoundingBox2D, ObjectObservation, SpatialPredicate
from dimos.benchmark.spatiotemporal.relations import derive_left_of


def _observation(object_id: str, label: str, box: BoundingBox2D) -> ObjectObservation:
    return ObjectObservation(
        episode_id="episode_1",
        frame_id=12,
        timestamp_s=0.5,
        object_id=object_id,
        label=label,
        box=box,
        confidence=0.9,
    )


def _transform_x(
    observation: ObjectObservation,
    *,
    offset: float = 0.0,
    mirror: bool = False,
) -> ObjectObservation:
    box = observation.box
    x_min, x_max = (1.0 - box.x_max, 1.0 - box.x_min) if mirror else (box.x_min, box.x_max)
    return observation.model_copy(
        update={"box": box.model_copy(update={"x_min": x_min + offset, "x_max": x_max + offset})}
    )


def test_accepts_left_of_relation_only_above_strict_margin() -> None:
    mug = _observation(
        "mug_1",
        "mug",
        BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.5),
    )
    laptop = _observation(
        "laptop_1",
        "laptop",
        BoundingBox2D(x_min=0.5, y_min=0.2, x_max=0.8, y_max=0.6),
    )

    relation = derive_left_of(mug, laptop, margin=0.1)

    assert relation is not None
    assert relation.subject is mug
    assert relation.predicate is SpatialPredicate.LEFT_OF
    assert relation.object is laptop
    assert relation.margin == 0.1
    assert derive_left_of(mug, laptop, margin=0.2 - 1e-12) is not None
    assert derive_left_of(mug, laptop, margin=0.2) is None
    assert derive_left_of(mug, laptop, margin=0.2 + 1e-12) is None


def test_equality_at_margin_remains_ambiguous_after_translation() -> None:
    touching_margin = (
        _observation(
            "mug_1",
            "mug",
            BoundingBox2D(x_min=0.0, y_min=0.2, x_max=0.125, y_max=0.5),
        ),
        _observation(
            "laptop_1",
            "laptop",
            BoundingBox2D(x_min=0.25, y_min=0.2, x_max=0.5, y_max=0.6),
        ),
    )
    translated = (
        _transform_x(touching_margin[0], offset=0.125),
        _transform_x(touching_margin[1], offset=0.125),
    )

    assert derive_left_of(*touching_margin, margin=0.125) is None
    assert derive_left_of(*translated, margin=0.125) is None


def test_accepted_relation_remains_accepted_after_translation() -> None:
    separated = (
        _observation(
            "mug_1",
            "mug",
            BoundingBox2D(x_min=0.0, y_min=0.2, x_max=0.125, y_max=0.5),
        ),
        _observation(
            "laptop_1",
            "laptop",
            BoundingBox2D(x_min=0.375, y_min=0.2, x_max=0.5, y_max=0.6),
        ),
    )
    translated = (
        _transform_x(separated[0], offset=0.125),
        _transform_x(separated[1], offset=0.125),
    )

    assert derive_left_of(*separated, margin=0.125) is not None
    assert derive_left_of(*translated, margin=0.125) is not None


def test_next_representable_separation_above_margin_is_accepted() -> None:
    subject = _observation(
        "subject_1",
        "subject",
        BoundingBox2D(x_min=0.0, y_min=0.2, x_max=0.0625, y_max=0.5),
    )
    object_ = _observation(
        "object_1",
        "object",
        BoundingBox2D(
            x_min=nextafter(0.1875, inf),
            y_min=0.2,
            x_max=0.5,
            y_max=0.6,
        ),
    )

    assert derive_left_of(subject, object_, margin=0.125) is not None


@pytest.mark.parametrize(
    ("subject_box", "object_box"),
    [
        (
            BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.6, y_max=0.5),
            BoundingBox2D(x_min=0.4, y_min=0.2, x_max=0.8, y_max=0.6),
        ),
        (
            BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.9, y_max=0.7),
            BoundingBox2D(x_min=0.3, y_min=0.3, x_max=0.6, y_max=0.5),
        ),
    ],
)
def test_overlap_and_containment_are_ambiguous(
    subject_box: BoundingBox2D,
    object_box: BoundingBox2D,
) -> None:
    subject = _observation("subject_1", "subject", subject_box)
    object_ = _observation("object_1", "object", object_box)

    assert relations.derive_left_of(subject, object_, margin=0.0) is None
    assert relations.derive_right_of(subject, object_, margin=0.0) is None


def test_horizontal_mirror_swaps_left_and_right() -> None:
    mug = _observation(
        "mug_1",
        "mug",
        BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.5),
    )
    laptop = _observation(
        "laptop_1",
        "laptop",
        BoundingBox2D(x_min=0.6, y_min=0.2, x_max=0.8, y_max=0.6),
    )

    original = derive_left_of(mug, laptop, margin=0.1)
    mirrored = relations.derive_right_of(
        _transform_x(mug, mirror=True),
        _transform_x(laptop, mirror=True),
        margin=0.1,
    )

    assert original is not None
    assert mirrored is not None
    assert mirrored.subject.object_id == original.subject.object_id
    assert mirrored.object.object_id == original.object.object_id


def test_right_of_is_left_of_with_arguments_swapped() -> None:
    mug = _observation(
        "mug_1",
        "mug",
        BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.5),
    )
    laptop = _observation(
        "laptop_1",
        "laptop",
        BoundingBox2D(x_min=0.5, y_min=0.2, x_max=0.8, y_max=0.6),
    )

    relation = relations.derive_right_of(laptop, mug, margin=0.1)

    assert relation is not None
    assert relation.subject is laptop
    assert relation.predicate is SpatialPredicate.RIGHT_OF
    assert relation.object is mug
    assert relations.derive_right_of(mug, laptop, margin=0.1) is None


def test_vertical_predicates_are_strict_inverses() -> None:
    lamp = _observation(
        "lamp_1",
        "lamp",
        BoundingBox2D(x_min=0.2, y_min=0.1, x_max=0.5, y_max=0.3),
    )
    table = _observation(
        "table_1",
        "table",
        BoundingBox2D(x_min=0.1, y_min=0.6, x_max=0.9, y_max=0.8),
    )

    above = relations.derive_above(lamp, table, margin=0.2)
    below = relations.derive_below(table, lamp, margin=0.2)

    assert above is not None
    assert above.predicate is SpatialPredicate.ABOVE
    assert below is not None
    assert below.predicate is SpatialPredicate.BELOW
    assert relations.derive_above(table, lamp, margin=0.2) is None
    assert relations.derive_below(lamp, table, margin=0.2) is None
    assert relations.derive_above(lamp, table, margin=0.3) is None
    assert relations.derive_below(table, lamp, margin=0.3) is None


def test_rejects_invalid_margin_and_cross_sample_comparisons() -> None:
    mug = _observation(
        "mug_1",
        "mug",
        BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.5),
    )
    laptop = _observation(
        "laptop_1",
        "laptop",
        BoundingBox2D(x_min=0.5, y_min=0.2, x_max=0.8, y_max=0.6),
    )

    for invalid_margin in (-0.1, 1.1, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="margin"):
            derive_left_of(mug, laptop, margin=invalid_margin)

    for update in (
        {"episode_id": "episode_2"},
        {"frame_id": 13},
        {"timestamp_s": 0.6},
    ):
        other_sample = laptop.model_copy(update=update)
        with pytest.raises(ValueError, match="same sample"):
            derive_left_of(mug, other_sample, margin=0.1)


def test_same_object_identity_never_produces_a_relation() -> None:
    left = _observation(
        "mug_1",
        "mug",
        BoundingBox2D(x_min=0.1, y_min=0.2, x_max=0.3, y_max=0.5),
    )
    right = _observation(
        "mug_1",
        "mug",
        BoundingBox2D(x_min=0.5, y_min=0.2, x_max=0.8, y_max=0.6),
    )

    assert derive_left_of(left, right, margin=0.1) is None

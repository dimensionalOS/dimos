# Copyright 2026 Dimensional Inc.
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

"""What several models agreeing buys, and what it refuses."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.mapping.hyperspace.frames import Frame, Hit
from dimos.mapping.hyperspace.heat import HeatConfig, blobs, boxes_in, places, summed_heat

CAMERA = "camera_optical"
# Looking down +z from the origin, so a ray times a depth is already a world point.
LOOKING_ALONG_Z = np.eye(4)


def hit(member: str, cell: int, grid: tuple[int, int], score: float, depth: float) -> Hit:
    rows, cols = grid
    row, col = divmod(cell, cols)
    # A ray through the cell centre of a 90-degree-ish lens, which keeps the numbers
    # readable: the middle of the grid looks straight ahead.
    return Hit(
        member=member,
        frame=CAMERA,
        ts=10.0,
        cell=cell,
        grid=grid,
        ray=((col + 0.5) / cols - 0.5, (row + 0.5) / rows - 0.5),
        depth=depth,
        score=score,
    )


def frame_of(*hits: Hit) -> Frame:
    return Frame(frame=CAMERA, ts=10.0, hits=list(hits))


def test_a_coarse_model_covers_the_block_it_stands_for() -> None:
    """Laying a 2x2 grid on a 4x4 one repeats cells rather than interpolating.

    A coarse model saying "warm here" means the whole of its cell is warm. Spreading a
    gradient across it would invent agreement that the model never expressed.
    """
    coarse = hit("small", cell=0, grid=(2, 2), score=1.0, depth=2.0)
    fine = hit("big", cell=0, grid=(4, 4), score=0.5, depth=2.0)
    total, _, contributors = summed_heat(frame_of(coarse, fine))
    assert total.shape == (4, 4), "the finest grid is the common one"
    assert contributors[0] == {"small", "big"}, "both models voted for the cell they share"
    assert contributors[1] == {"small"}, "only the coarse one covers its neighbour"
    assert total[0, 0] == pytest.approx(1.5), "both models, added"
    assert total[0, 1] == pytest.approx(1.0), "the coarse cell covers its whole block"
    assert total[1, 1] == pytest.approx(1.0)
    assert total[2, 2] == pytest.approx(0.0), "and nothing outside it"


def test_one_model_alone_does_not_make_a_box() -> None:
    """The whole point: a single model's hot floor is what put boxes on floors."""
    alone = frame_of(
        hit("small", cell=5, grid=(4, 4), score=1.0, depth=2.0),
        hit("small", cell=6, grid=(4, 4), score=1.0, depth=2.0),
    )
    assert boxes_in(alone, LOOKING_ALONG_Z) == []

    agreed = frame_of(
        hit("small", cell=5, grid=(4, 4), score=1.0, depth=2.0),
        hit("small", cell=6, grid=(4, 4), score=1.0, depth=2.0),
        hit("big", cell=5, grid=(4, 4), score=1.0, depth=2.0),
        hit("big", cell=6, grid=(4, 4), score=1.0, depth=2.0),
    )
    found = boxes_in(agreed, LOOKING_ALONG_Z)
    assert len(found) == 1 and found[0].agreement == 2


def test_asking_for_three_models_when_two_were_searched_still_answers() -> None:
    """The demand is clamped to what is on offer, or two models would answer nothing."""
    two = frame_of(
        hit("small", cell=5, grid=(4, 4), score=1.0, depth=2.0),
        hit("big", cell=5, grid=(4, 4), score=1.0, depth=2.0),
        hit("small", cell=6, grid=(4, 4), score=1.0, depth=2.0),
        hit("big", cell=6, grid=(4, 4), score=1.0, depth=2.0),
    )
    assert boxes_in(two, LOOKING_ALONG_Z, config=HeatConfig(min_members=3)) != []


def test_two_blobs_in_one_frame_are_two_boxes() -> None:
    """Opposite corners of the grid are not one thing, however hot both are."""
    frame = frame_of(
        hit("small", cell=0, grid=(4, 4), score=1.0, depth=2.0),
        hit("big", cell=0, grid=(4, 4), score=1.0, depth=2.0),
        hit("small", cell=1, grid=(4, 4), score=1.0, depth=2.0),
        hit("big", cell=1, grid=(4, 4), score=1.0, depth=2.0),
        hit("small", cell=14, grid=(4, 4), score=1.0, depth=3.0),
        hit("big", cell=14, grid=(4, 4), score=1.0, depth=3.0),
        hit("small", cell=15, grid=(4, 4), score=1.0, depth=3.0),
        hit("big", cell=15, grid=(4, 4), score=1.0, depth=3.0),
    )
    found = boxes_in(frame, LOOKING_ALONG_Z)
    assert len(found) == 2
    assert {round(box.near_m) for box in found} == {2, 3}


def test_the_box_spans_the_depths_its_cells_read() -> None:
    """A box is the volume the models point at, not a point inside it."""
    frame = frame_of(
        hit("small", cell=5, grid=(4, 4), score=1.0, depth=2.0),
        hit("big", cell=5, grid=(4, 4), score=1.0, depth=2.0),
        hit("small", cell=6, grid=(4, 4), score=1.0, depth=2.4),
        hit("big", cell=6, grid=(4, 4), score=1.0, depth=2.4),
    )
    found = boxes_in(frame, LOOKING_ALONG_Z)
    assert len(found) == 1
    box = found[0]
    assert box.near_m == pytest.approx(2.0) and box.far_m == pytest.approx(2.4)
    assert box.extent[2] == pytest.approx(0.4, abs=1e-6), "as deep as the readings span"


def test_a_wall_behind_the_thing_does_not_stretch_the_box() -> None:
    """A blob can straddle a thing and what is visible past it; the thing is nearer."""
    frame = frame_of(
        hit("small", cell=5, grid=(4, 4), score=1.0, depth=2.0),
        hit("big", cell=5, grid=(4, 4), score=1.0, depth=2.0),
        hit("small", cell=6, grid=(4, 4), score=1.0, depth=2.2),
        hit("big", cell=6, grid=(4, 4), score=1.0, depth=2.2),
        hit("small", cell=7, grid=(4, 4), score=1.0, depth=9.0),
        hit("big", cell=7, grid=(4, 4), score=1.0, depth=9.0),
    )
    found = boxes_in(frame, HeatConfig().depth_band_m and LOOKING_ALONG_Z)
    assert len(found) == 1
    assert found[0].far_m == pytest.approx(2.2), "the nine-metre cell is not part of it"


def test_an_absolute_floor_stops_an_empty_frame_yielding_a_blob() -> None:
    """Sixty percent of a peak is still sixty percent when the peak is noise."""
    faint = frame_of(
        hit("small", cell=5, grid=(4, 4), score=0.003, depth=2.0),
        hit("big", cell=5, grid=(4, 4), score=0.003, depth=2.0),
        hit("small", cell=6, grid=(4, 4), score=0.003, depth=2.0),
        hit("big", cell=6, grid=(4, 4), score=0.003, depth=2.0),
    )
    assert boxes_in(faint, LOOKING_ALONG_Z) == [], "nothing here clears the floor"
    assert boxes_in(faint, LOOKING_ALONG_Z, config=HeatConfig(floor=0.0)) != [], (
        "and without the floor it would have answered"
    )


def test_blobs_are_eight_connected() -> None:
    """Diagonally touching cells are one blob, which is how a thin object reads."""
    mask = np.zeros((3, 3), dtype=bool)
    mask[0, 0] = mask[1, 1] = mask[2, 2] = True
    assert len(list(blobs(mask))) == 1
    mask[1, 1] = False
    assert len(list(blobs(mask))) == 2


def test_places_group_across_frames_and_rank_by_heat() -> None:
    """The same thing seen twice is one place, and the hotter look is the one to use."""
    from dimos.mapping.hyperspace.heat import HeatBox

    def box(ts: float, centre: tuple[float, float, float], heat: float) -> HeatBox:
        return HeatBox(
            camera_frame=CAMERA,
            ts=ts,
            centre=centre,
            extent=(0.3, 0.3, 0.3),
            heat=heat,
            cells=4,
            members=("big", "small"),
            near_m=2.0,
            far_m=2.3,
        )

    found = places(
        [
            box(10.0, (0.0, 0.0, 2.0), heat=1.0),
            box(11.0, (0.2, 0.0, 2.0), heat=2.0),
            box(12.0, (9.0, 0.0, 2.0), heat=1.5),
            # No radius decided any of this: 0.3 m boxes 0.2 m apart share volume and
            # ones nine metres apart do not.
        ]
    )
    assert len(found) == 2, "two places"
    assert found[0].heat == 2.0, "hottest place first"
    assert found[0].best.ts == 11.0, "and the hottest frame of it is the one to detect in"
    assert found[0].frames == 2


def _box(centre, extent, heat_value, *, frame="cam", ts=0.0):
    from dimos.mapping.hyperspace.heat import HeatBox

    return HeatBox(
        camera_frame=frame,
        ts=ts,
        centre=tuple(float(value) for value in centre),
        extent=tuple(float(value) for value in extent),
        heat=heat_value,
        cells=1,
        members=("a", "b"),
        near_m=1.0,
        far_m=2.0,
    )


def test_overlap_answers_exactly_what_the_numpy_version_did() -> None:
    """The fast path has to be the same predicate, not a nearly-the-same one.

    It was six little numpy arrays per call and 0.24 s of a 0.44 s grouping pass on
    bike; plain floats with an early exit per axis is 37x faster over the same boxes.
    Faster and slightly different would be a silent change to which frames a detector
    is shown, so this checks the answer rather than the clock.
    """
    import numpy as np

    from dimos.mapping.hyperspace.heat import overlap

    def as_numpy_did(one, other, pad, fraction=0.0):
        size = max(max(one.extent), max(other.extent))
        room = pad + fraction * size
        here = np.array(one.centre) - np.array(one.extent) / 2 - room
        here_to = np.array(one.centre) + np.array(one.extent) / 2 + room
        there = np.array(other.centre) - np.array(other.extent) / 2
        there_to = np.array(other.centre) + np.array(other.extent) / 2
        return bool(np.all(here <= there_to) and np.all(there <= here_to))

    rng = np.random.default_rng(4)
    boxes = [
        _box(rng.normal(size=3) * 2, np.abs(rng.normal(size=3)) * 0.5 + 0.05, float(rng.random()))
        for _ in range(60)
    ]
    for pad, fraction in [(0.0, 0.0), (0.25, 0.0), (0.0, 0.5), (0.25, 0.5)]:
        for one in boxes:
            for other in boxes:
                assert overlap(one, other, pad, fraction) == as_numpy_did(
                    one, other, pad, fraction
                ), (one, other, pad, fraction)


def test_a_place_is_anchored_on_its_hottest_box_without_recomputing_it() -> None:
    """`places` compares against `place.boxes[0]`, and that has to BE the hottest.

    The boxes arrive in descending heat, so the first one a place is opened with is its
    hottest and stays so -- which is why the anchor can be read instead of `max`-ed over
    every box on every comparison. If that ordering ever changes this is the test that
    notices, because the grouping would quietly start anchoring on the wrong box.
    """
    from dimos.mapping.hyperspace.heat import HeatConfig, places

    config = HeatConfig()
    near = [_box((0.0, 0.0, 0.0), (0.2, 0.2, 0.2), heat) for heat in (0.1, 0.9, 0.5)]
    far = [_box((50.0, 0.0, 0.0), (0.2, 0.2, 0.2), 0.7)]

    found = places([*near, *far], config=config)
    assert len(found) == 2
    for place in found:
        assert place.boxes[0] is place.best, "the anchor has to be the hottest box"
        assert place.boxes[0].heat == max(box.heat for box in place.boxes)
    assert [place.heat for place in found] == [0.9, 0.7], "places come back hottest first"

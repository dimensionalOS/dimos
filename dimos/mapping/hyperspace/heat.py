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

"""Where several models agree there is something, as a box rather than a point.

One model's hot cells are not the object. On sf_office a cell is about 11 cm of scene
at 2.3 m, so a 40 cm cone is four cells -- and a single model came back with a hundred
hot cells, a tenth of the frame, most of them the floor under and in front of it.

Models of different sizes and resolutions do not hallucinate the same floor. Summing
their heat over one grid and keeping only what stands out of the sum is the cheapest
form of that agreement, and it needs no resampling: the grids are laid on the finest of
them by repeating cells, which is exact where interpolation would invent values.

What comes out is an *embedding box* per blob per frame -- the segment's rays between
the nearest and furthest depth its cells read. Those boxes say where to send the
detector first, and which frame to send it, which is the part a single hot patch was
never going to answer well.
"""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace.frames import Frame, Hit
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


@dataclass
class HeatBox:
    """One blob of agreement in one frame, placed in the world.

    `heat` is the peak of the summed heat inside the blob, which is what ranks frames
    against each other: the frame where the models agree hardest is the one worth
    showing a detector.
    """

    camera_frame: str
    ts: float
    centre: tuple[float, float, float]
    extent: tuple[float, float, float]
    heat: float
    cells: int
    members: tuple[str, ...]
    near_m: float
    far_m: float

    @property
    def agreement(self) -> int:
        """How many models put heat in this blob."""
        return len(self.members)


@dataclass
class HeatConfig:
    """Everything the agreement step can be turned by."""

    # Of the summed heat's peak in a frame: what stands out of the sum. Measured on
    # sf_office against the two cones that exist -- at 0.6 the blob is the hot core
    # only, 12 cm of a 40 cm cone, and one cone came back as five places; 0.3 gives a
    # 24 cm blob and one. Lower admits more of the frame without growing the thing.
    relative: float = 0.2
    # And an absolute floor under it, because a fraction of a peak is still a fraction
    # when the peak is noise -- without this every frame yields a blob of whatever it
    # has most of.
    floor: float = 0.02
    # Blobs smaller than this are single hot cells, which is what the summing is meant
    # to be robust to.
    min_cells: int = 2
    # A blob has to have heat from at least this many models, clamped to how many were
    # actually searched. Measured on sf_office: demanding all three rather than two of
    # three cut the places from 18 to 3 with the cones untouched -- the ones it removes
    # are exactly the places one model invented on its own.
    min_members: int = 3
    # Cells whose depth is this far from the blob's nearest reading are a different
    # surface seen through the same blob, and do not stretch the box.
    depth_band_m: float = 0.6
    # Boxes that overlap are one place, and this pads them before asking. A box is a
    # volume, so overlap answers "the same thing" without a radius to pick -- which is
    # the point of boxes over points: a metre is nothing across a room and everything
    # inside a shelf, and no single number is right for both.
    pad_m: float = 0.1
    # And most of the padding is a fraction of the box itself, because what survives
    # the cut is the hot core of a thing rather than its outline: a 40 cm cone comes
    # back as a 12 cm blob, and two such blobs on one cone miss each other by more than
    # either is wide. Scaling with the box keeps that from becoming a fixed radius by
    # another name -- a small thing stays small.
    pad_fraction: float = 3.0


def summed_heat(
    frame: Frame, *, members: Sequence[str] | None = None
) -> tuple[NDArray[np.float32], dict[int, Hit], dict[int, set[str]]]:
    """Every model's heat for one frame, laid on the finest grid and added up.

    Returns the summed grid, the best hit behind each of its cells -- which carries the
    ray and the depth the box is built from -- and which models put heat in each cell.

    Those two are kept apart on purpose: one cell has one ray and one depth, but it can
    have heat from every model, and counting agreement off the single winning hit would
    say one model every time.

    Grids are laid on the finest one by repeating cells rather than interpolating: a
    coarse model saying "warm here" means the whole of its cell is warm, and inventing
    a gradient across it would be inventing agreement.
    """
    hits_by_member: dict[str, list[Hit]] = {}
    for hit in frame.hits:
        if members is not None and hit.member not in members:
            continue
        hits_by_member.setdefault(hit.member, []).append(hit)
    if not hits_by_member:
        return np.zeros((0, 0), dtype=np.float32), {}, {}

    shapes = {member: hits[0].grid for member, hits in hits_by_member.items()}
    rows = max(shape[0] for shape in shapes.values())
    cols = max(shape[1] for shape in shapes.values())

    total = np.zeros((rows, cols), dtype=np.float32)
    best: dict[int, Hit] = {}
    contributors: dict[int, set[str]] = {}
    for member, hits in hits_by_member.items():
        source_rows, source_cols = shapes[member]
        for hit in hits:
            # Nearest cell, which for a whole-number ratio is exactly cell repeat.
            row, col = divmod(hit.cell, source_cols)
            row_from = row * rows // source_rows
            row_to = ((row + 1) * rows + source_rows - 1) // source_rows
            col_from = col * cols // source_cols
            col_to = ((col + 1) * cols + source_cols - 1) // source_cols
            total[row_from:row_to, col_from:col_to] += hit.score
            placeable = bool(np.isfinite(hit.depth) and hit.depth > 0)
            for target_row in range(row_from, row_to):
                for target_col in range(col_from, col_to):
                    index = target_row * cols + target_col
                    contributors.setdefault(index, set()).add(member)
                    if not placeable:
                        continue
                    held = best.get(index)
                    # The finest model's hit wins: it is the one whose ray and depth
                    # actually belong to this cell rather than to a block of them.
                    if held is None or source_cols > held.grid[1]:
                        best[index] = hit
                    elif source_cols == held.grid[1] and hit.score > held.score:
                        best[index] = hit
    return total, best, contributors


def blobs(mask: NDArray[np.bool_]) -> Iterator[list[int]]:
    """Connected runs of `True`, eight-connected, as flat cell indices."""
    rows, cols = mask.shape
    seen = np.zeros_like(mask)
    for start in np.flatnonzero(mask):
        if seen.flat[start]:
            continue
        stack = [int(start)]
        seen.flat[start] = True
        found: list[int] = []
        while stack:
            index = stack.pop()
            found.append(index)
            row, col = divmod(index, cols)
            for down in (-1, 0, 1):
                for right in (-1, 0, 1):
                    near_row, near_col = row + down, col + right
                    if not (0 <= near_row < rows and 0 <= near_col < cols):
                        continue
                    near = near_row * cols + near_col
                    if mask.flat[near] and not seen.flat[near]:
                        seen.flat[near] = True
                        stack.append(near)
        yield found


def boxes_in(
    frame: Frame,
    pose: NDArray[np.floating],
    *,
    config: HeatConfig | None = None,
    members: Sequence[str] | None = None,
) -> list[HeatBox]:
    """The embedding boxes of one frame: agreement, segmented and placed.

    A blob's box is its cells' rays carried out between the nearest and furthest depth
    those cells read, so it is the volume the models are pointing at rather than a
    point somewhere inside it.
    """
    config = config or HeatConfig()
    total, best, contributors = summed_heat(frame, members=members)
    if not total.size:
        return []
    # Asking for three models when two were searched would answer nothing at all, so
    # the demand is clamped to what is on offer -- but never below two, because one
    # model agreeing with itself is the thing this step exists to refuse.
    searched = len({hit.member for hit in frame.hits}) if members is None else len(set(members))
    if searched < 2:
        return []
    wanted = min(config.min_members, searched)
    peak = float(total.max())
    cut = max(peak * config.relative, config.floor)
    mask = total >= cut

    found: list[HeatBox] = []
    for cells in blobs(mask):
        usable = [(index, best[index]) for index in cells if index in best]
        if len(usable) < config.min_cells:
            continue
        voted: set[str] = set()
        for index in cells:
            voted |= contributors.get(index, set())
        if len(voted) < wanted:
            continue
        depths = np.array([hit.depth for _, hit in usable])
        near = float(depths.min())
        # A blob can straddle a thing and the wall behind it; the near surface is the
        # thing, and readings far past it are not part of it.
        kept = [
            (index, hit)
            for (index, hit), depth in zip(usable, depths, strict=True)
            if depth - near <= config.depth_band_m
        ]
        if len(kept) < config.min_cells:
            continue
        far = float(max(hit.depth for _, hit in kept))
        corners = []
        for _, hit in kept:
            for depth in (near, far):
                corners.append([hit.ray[0] * depth, hit.ray[1] * depth, depth])
        local = np.array(corners)
        world = (pose[:3, :3] @ local.T).T + pose[:3, 3]
        low, high = world.min(axis=0), world.max(axis=0)
        found.append(
            HeatBox(
                camera_frame=frame.frame,
                ts=frame.ts,
                centre=(
                    float((low[0] + high[0]) / 2),
                    float((low[1] + high[1]) / 2),
                    float((low[2] + high[2]) / 2),
                ),
                extent=(
                    float(max(0.05, high[0] - low[0])),
                    float(max(0.05, high[1] - low[1])),
                    float(max(0.05, high[2] - low[2])),
                ),
                heat=float(max(total.flat[index] for index, _ in kept)),
                cells=len(kept),
                members=tuple(sorted(voted)),
                near_m=near,
                far_m=far,
            )
        )
    return found


@dataclass
class HeatPlace:
    """Embedding boxes from several frames that are the same place."""

    boxes: list[HeatBox] = field(default_factory=list)

    @property
    def best(self) -> HeatBox:
        """The frame where the models agreed hardest -- the one to show a detector."""
        return max(self.boxes, key=lambda box: box.heat)

    @property
    def heat(self) -> float:
        return self.best.heat

    @property
    def centre(self) -> NDArray[np.float64]:
        weights = np.array([box.heat for box in self.boxes], dtype=float)
        centres = np.array([box.centre for box in self.boxes], dtype=float)
        return (centres * weights[:, None]).sum(axis=0) / weights.sum()

    @property
    def frames(self) -> int:
        return len({(box.camera_frame, box.ts) for box in self.boxes})


def overlap(one: HeatBox, other: HeatBox, pad: float, fraction: float = 0.0) -> bool:
    """Do these two boxes share any volume, once padded?

    The reason to carry boxes at all. Two looks at one cone overlap however the camera
    moved between them, and a cone and the chair a metre away do not, so nobody has to
    choose a distance that is right both across a room and inside a shelf.

    Most of the padding scales with the boxes, because a blob is the hot core of a
    thing and not its outline. A fixed pad big enough to join two cores of a cone would
    be a radius again; one that grows with the box is not.
    """
    # Plain floats, not numpy. These are three numbers a side and the call is made tens
    # of thousands of times per query -- building six little arrays for each was 0.24 s
    # of a 0.44 s grouping pass, measured on bike's 800 boxes and 180 places. An axis
    # that misses also gets to stop early, which an `np.all` over the whole vector
    # cannot do.
    one_centre, one_extent = one.centre, one.extent
    other_centre, other_extent = other.centre, other.extent
    size = max(max(one_extent), max(other_extent))
    room = pad + fraction * size
    for axis in range(3):
        half, other_half = one_extent[axis] / 2, other_extent[axis] / 2
        if one_centre[axis] - half - room > other_centre[axis] + other_half:
            return False
        if other_centre[axis] - other_half > one_centre[axis] + half + room:
            return False
    return True


def places(boxes: Sequence[HeatBox], *, config: HeatConfig | None = None) -> list[HeatPlace]:
    """Group embedding boxes across frames into places, hottest first.

    Grouping is around the hottest box of a place rather than single-link, so a row of
    boxes along a shelf cannot chain into one place the length of the aisle.
    """
    config = config or HeatConfig()
    found: list[HeatPlace] = []
    for box in sorted(boxes, key=lambda box: -box.heat):
        for place in found:
            # `place.boxes[0]` IS `place.best` here, and cheaply: the boxes arrive in
            # descending heat, so the first one a place was opened with is its hottest
            # and stays so. The property recomputed a `max` over every box of the place
            # on every comparison, which on bike was 135,705 of them.
            if overlap(place.boxes[0], box, config.pad_m, config.pad_fraction):
                place.boxes.append(box)
                break
        else:
            found.append(HeatPlace(boxes=[box]))
    return sorted(found, key=lambda place: -place.heat)


def places_of(
    frames: Sequence[Frame],
    poses: dict[tuple[str, float], Any],
    *,
    config: HeatConfig | None = None,
    members: Sequence[str] | None = None,
) -> list[HeatPlace]:
    """The whole step: frames in, places to look at out, hottest first."""
    config = config or HeatConfig()
    found: list[HeatBox] = []
    for frame in frames:
        pose = poses.get((frame.frame, frame.ts))
        if pose is None:
            continue
        found.extend(boxes_in(frame, np.asarray(pose), config=config, members=members))
    return places(found, config=config)

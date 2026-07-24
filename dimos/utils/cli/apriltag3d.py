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

"""3D-printable AprilTag/ArUco generator for multicolor (AMS/MMU) printers.

Emits two solids per tag that share one coordinate frame and meet flush at the top
face: `base` (light) is the plate minus the dark cells, `marker` (dark) is only the
top `marker_mm` of the tag's black cells. Printed together the top surface is flat —
no relief, so no shadows at grazing light angles.

Sizes match the PDF generator: the tag's outer black border edge measures `size_mm`,
which is the value to pass as `tag_size` to pose estimation.
"""

from __future__ import annotations

from pathlib import Path
from typing import NamedTuple
from xml.sax.saxutils import escape
import zipfile

import numpy as np
import trimesh

from dimos.utils.cli.apriltag import _FAMILIES, cell_matrix, display_color, row_runs

# Cutting solids overshoot the plate by this much so coplanar faces never meet in a
# boolean; the exported solids stay at their exact nominal dimensions.
_EPS = 0.01

# Plastic around a mounting hole. Generous on purpose: a thin ring around the hole slices
# into a couple of perimeters that peel off the bed.
_HOLE_PAD_MM = 4.0

# Corner rounding on the plate, capped against the plate so small tags aren't all corner.
_CORNER_R_MM = 4.0

# Facets per mounting-hole cylinder.
_HOLE_SEGMENTS = 48

# Diagonally adjacent cells would otherwise meet at a single point, which is non-manifold
# and makes slicers report a broken mesh. Growing every cell by this much turns each such
# contact into a real overlap. It has to stay well clear of the float32 an STL stores —
# at plate-scale coordinates a 1um overlap yields boolean vertices that round together on
# reload — while staying far below anything a 0.4mm nozzle can resolve.
_CELL_GROW = 1e-2

# Classic 5x7 cell font, 5 column bytes per glyph, bit 0 = top row. Used for the
# engraving on the back, so no font dependency and every stroke is one cell wide.
_FONT_5X7: dict[str, tuple[int, int, int, int, int]] = {
    " ": (0x00, 0x00, 0x00, 0x00, 0x00),
    "-": (0x08, 0x08, 0x08, 0x08, 0x08),
    ".": (0x00, 0x60, 0x60, 0x00, 0x00),
    "_": (0x40, 0x40, 0x40, 0x40, 0x40),
    "0": (0x3E, 0x51, 0x49, 0x45, 0x3E),
    "1": (0x00, 0x42, 0x7F, 0x40, 0x00),
    "2": (0x42, 0x61, 0x51, 0x49, 0x46),
    "3": (0x21, 0x41, 0x45, 0x4B, 0x31),
    "4": (0x18, 0x14, 0x12, 0x7F, 0x10),
    "5": (0x27, 0x45, 0x45, 0x45, 0x39),
    "6": (0x3C, 0x4A, 0x49, 0x49, 0x30),
    "7": (0x01, 0x71, 0x09, 0x05, 0x03),
    "8": (0x36, 0x49, 0x49, 0x49, 0x36),
    "9": (0x06, 0x49, 0x49, 0x29, 0x1E),
    "A": (0x7E, 0x11, 0x11, 0x11, 0x7E),
    "B": (0x7F, 0x49, 0x49, 0x49, 0x36),
    "C": (0x3E, 0x41, 0x41, 0x41, 0x22),
    "D": (0x7F, 0x41, 0x41, 0x22, 0x1C),
    "E": (0x7F, 0x49, 0x49, 0x49, 0x41),
    "F": (0x7F, 0x09, 0x09, 0x09, 0x01),
    "G": (0x3E, 0x41, 0x49, 0x49, 0x7A),
    "H": (0x7F, 0x08, 0x08, 0x08, 0x7F),
    "I": (0x00, 0x41, 0x7F, 0x41, 0x00),
    "J": (0x20, 0x40, 0x41, 0x3F, 0x01),
    "K": (0x7F, 0x08, 0x14, 0x22, 0x41),
    "L": (0x7F, 0x40, 0x40, 0x40, 0x40),
    "M": (0x7F, 0x02, 0x0C, 0x02, 0x7F),
    "N": (0x7F, 0x04, 0x08, 0x10, 0x7F),
    "O": (0x3E, 0x41, 0x41, 0x41, 0x3E),
    "P": (0x7F, 0x09, 0x09, 0x09, 0x06),
    "Q": (0x3E, 0x41, 0x51, 0x21, 0x5E),
    "R": (0x7F, 0x09, 0x19, 0x29, 0x46),
    "S": (0x46, 0x49, 0x49, 0x49, 0x31),
    "T": (0x01, 0x01, 0x7F, 0x01, 0x01),
    "U": (0x3F, 0x40, 0x40, 0x40, 0x3F),
    "V": (0x1F, 0x20, 0x40, 0x20, 0x1F),
    "W": (0x3F, 0x40, 0x38, 0x40, 0x3F),
    "X": (0x63, 0x14, 0x08, 0x14, 0x63),
    "Y": (0x07, 0x08, 0x70, 0x08, 0x07),
    "Z": (0x61, 0x51, 0x49, 0x45, 0x43),
}

_GLYPH_W = 5
_GLYPH_H = 7
_GLYPH_GAP = 2
_LINE_GAP = 2

# The font's glyphs are one cell thick, and its diagonals step a cell at a time — printed
# straight that is a knife-edge stroke made of near-detached cubes. Each glyph is instead
# resampled this many cells per font pixel and then grown by _FONT_DILATE cells, which
# fattens every stroke to _STROKE_CELLS and turns each diagonal step into a real overlap.
# Growth is at the resampled scale, so counters (the holes in 0, 8, A) stay open.
_FONT_SCALE = 4
_FONT_DILATE = 1
_STROKE_CELLS = _FONT_SCALE + 2 * _FONT_DILATE


class TagParts(NamedTuple):
    """One tag's solids, all in a shared coordinate frame."""

    base: trimesh.Trimesh
    marker: trimesh.Trimesh
    text: trimesh.Trimesh | None
    frame: trimesh.Trimesh | None = None


def _at(x: float, y: float, z: float) -> np.ndarray:
    """Homogeneous transform translating to (x, y, z)."""
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _at_along_y(x: float, y: float, z: float) -> np.ndarray:
    """Transform placing a Z-axis primitive so its axis runs along Y, centered at (x, y, z)."""
    return np.array([[1, 0, 0, x], [0, 0, -1, y], [0, 1, 0, z], [0, 0, 0, 1]], dtype=float)


def _box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> trimesh.Trimesh:
    """Axis-aligned box spanning the given half-open ranges."""
    return trimesh.creation.box(  # type: ignore[no-any-return]
        extents=(x1 - x0, y1 - y0, z1 - z0),
        transform=_at((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
    )


def _union(parts: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    """Boolean-union the parts into one watertight solid, pairwise.

    Unioning all of them in one call leaves a handful of duplicate vertices where many
    boxes meet — pinch points that survive as a watertight mesh in memory but weld into
    non-manifold bowties once an STL round-trip rounds them together. Folding one box at a
    time into a single growing solid avoids them, where merging disjoint groups pairwise
    does not; it costs well under a second for the few hundred boxes a tag needs.
    """
    solid = parts[0]
    for part in parts[1:]:
        solid = trimesh.boolean.union([solid, part])
    return solid


def _grown_box(
    x0: float, x1: float, y0: float, y1: float, z0: float, z1: float, bound: float | None
) -> trimesh.Trimesh:
    """Cell box grown by _CELL_GROW in XY, clamped to ±bound so outer dimensions stay exact."""
    x0, x1 = x0 - _CELL_GROW, x1 + _CELL_GROW
    y0, y1 = y0 - _CELL_GROW, y1 + _CELL_GROW
    if bound is not None:
        x0, x1 = max(x0, -bound), min(x1, bound)
        y0, y1 = max(y0, -bound), min(y1, bound)
    return _box(x0, x1, y0, y1, z0, z1)


def grid_rects(grid: list[list[int]]) -> list[tuple[int, int, int, int]]:
    """Merge a cell grid's set cells into (col0, col1, row0, row1) rectangles.

    Runs are merged horizontally and then stacked vertically wherever consecutive rows
    share a run, so a tag's black border or a glyph's vertical stroke becomes one box
    instead of one per row. Fewer boxes means a faster union and fewer seams in it.
    """
    rects: list[tuple[int, int, int, int]] = []
    open_runs: dict[tuple[int, int], int] = {}
    for r, row in enumerate(grid):
        runs = set(row_runs(row))
        for key in [k for k in open_runs if k not in runs]:
            rects.append((key[0], key[1], open_runs.pop(key), r))
        for key in runs:
            open_runs.setdefault(key, r)
    rects.extend((s, e, r0, len(grid)) for (s, e), r0 in open_runs.items())
    return sorted(rects)


def marker_solid(family: str, tag_id: int, size_mm: float, z0: float, z1: float) -> trimesh.Trimesh:
    """The tag's black cells as one solid, centered on the origin in XY."""
    cells = cell_matrix(family, tag_id)
    n = len(cells)
    cell = size_mm / n
    half = size_mm / 2
    # cv2 row 0 is the top row; +Y is up when the plate is viewed from +Z.
    boxes = [
        _grown_box(
            -half + c0 * cell,
            -half + c1 * cell,
            half - r1 * cell,
            half - r0 * cell,
            z0,
            z1,
            bound=half,
        )
        for c0, c1, r0, r1 in grid_rects(cells)
    ]
    if not boxes:
        raise ValueError(f"{family} id {tag_id} has no black cells")
    return _union(boxes)


def _text_cells(lines: list[str]) -> tuple[list[list[int]], int, int]:
    """Rasterize text lines to a fattened cell grid; returns (rows, width, height) in cells."""
    width = max(len(line) * (_GLYPH_W + _GLYPH_GAP) - _GLYPH_GAP for line in lines)
    height = len(lines) * (_GLYPH_H + _LINE_GAP) - _LINE_GAP
    pixels = np.zeros((height, width), dtype=bool)
    for li, line in enumerate(lines):
        y0 = li * (_GLYPH_H + _LINE_GAP)
        x_pad = (width - (len(line) * (_GLYPH_W + _GLYPH_GAP) - _GLYPH_GAP)) // 2
        for ci, ch in enumerate(line):
            cols = _FONT_5X7.get(ch, _FONT_5X7[" "])
            x0 = x_pad + ci * (_GLYPH_W + _GLYPH_GAP)
            for dx, bits in enumerate(cols):
                for dy in range(_GLYPH_H):
                    if bits >> dy & 1:
                        pixels[y0 + dy, x0 + dx] = True

    up = np.repeat(np.repeat(pixels, _FONT_SCALE, axis=0), _FONT_SCALE, axis=1)
    rows, cols_n = up.shape
    pad = _FONT_DILATE
    grown = np.zeros((rows + 2 * pad, cols_n + 2 * pad), dtype=bool)
    # Square structuring element: growing diagonally too is what welds a diagonal step's
    # corner contact into an overlap.
    for dy in range(2 * pad + 1):
        for dx in range(2 * pad + 1):
            grown[dy : dy + rows, dx : dx + cols_n] |= up
    return grown.astype(int).tolist(), grown.shape[1], grown.shape[0]


def text_solid(
    lines: list[str],
    plate_w: float,
    plate_h: float,
    depth: float,
    min_stroke_mm: float,
    *,
    z0: float = 0.0,
) -> trimesh.Trimesh | None:
    """Mirrored text as a solid on the plate bottom; None if the strokes can't be printed.

    Mirrored in X so the engraving reads correctly when the plate is viewed from below.
    Pass `z0 = -_EPS` for the solid that cuts the pocket, `0.0` for the inlay filling it.
    """
    grid, cells_w, cells_h = _text_cells(lines)
    # Fill most of the plate: the text scales with it, and the wider the strokes the
    # easier they are to lay down. Width is the binding constraint for a typical family
    # name, so the margin left here is what actually sets the stroke.
    cell = min(plate_w * 0.9 / cells_w, plate_h * 0.55 / cells_h)
    # A cell is a fraction of a stroke now, so printability is about the stroke it builds.
    if cell * _STROKE_CELLS < min_stroke_mm:
        return None
    x0 = cells_w * cell / 2
    y0 = cells_h * cell / 2
    # Grid row 0 is the top row; mirror in X so column c spans
    # [x0 - (c+1)*cell, x0 - c*cell] and the engraving reads from below.
    boxes = [
        _grown_box(
            x0 - c1 * cell,
            x0 - c0 * cell,
            y0 - r1 * cell,
            y0 - r0 * cell,
            z0,
            depth,
            bound=None,
        )
        for c0, c1, r0, r1 in grid_rects(grid)
    ]
    return _union(boxes) if boxes else None


def hole_pad_mm(hole_dia_mm: float) -> float:
    """Plastic around a mounting hole; also how far the side flange reaches past the plate."""
    return hole_dia_mm / 2 + _HOLE_PAD_MM


def hole_spacing_mm(plate_mm: float, hole_dia_mm: float) -> float:
    """Distance between the two holes on one side, spread as wide as the plate allows."""
    return plate_mm - 2 * hole_pad_mm(hole_dia_mm)


def plate_footprint_mm(plate_mm: float, hole_dia_mm: float, holes: bool) -> tuple[float, float]:
    """Outer (width, height) of the printed plate — wider than the tag when it has flanges."""
    if not holes:
        return plate_mm, plate_mm
    return plate_mm + 4 * hole_pad_mm(hole_dia_mm), plate_mm


def _rounded_plate(width: float, height: float, thickness: float) -> trimesh.Trimesh:
    """A plate with rounded corners, as two crossed boxes plus a cylinder per corner."""
    radius = min(_CORNER_R_MM, min(width, height) / 8)
    half_w, half_h = width / 2, height / 2
    parts = [
        _box(-half_w + radius, half_w - radius, -half_h, half_h, 0.0, thickness),
        _box(-half_w, half_w, -half_h + radius, half_h - radius, 0.0, thickness),
    ]
    parts += [
        trimesh.creation.cylinder(
            radius=radius,
            height=thickness,
            sections=_HOLE_SEGMENTS,
            transform=_at(sx * (half_w - radius), sy * (half_h - radius), thickness / 2),
        )
        for sx in (-1, 1)
        for sy in (-1, 1)
    ]
    return _union(parts)


def _hole_cuts(plate: float, thickness: float, diameter: float) -> list[trimesh.Trimesh]:
    """The four mounting holes drilled through the side flanges.

    The plate simply runs on past the tag on both sides and the holes go through it, rather
    than growing shaped ears: a plain rectangle has no narrow neck or round tab to peel off
    the bed. It also keeps the tag's quiet zone uniformly light — a hole punched through the
    margin instead would show the background right where quad corner refinement is most
    sensitive.

    Two holes per side, spread to the plate corners: a single screw per side would leave the
    tag free to pivot about that screw.
    """
    pad = hole_pad_mm(diameter)
    spacing = hole_spacing_mm(plate, diameter)
    if spacing <= 2 * pad:
        raise ValueError(
            f"a {plate:.1f} mm plate is too small to carry two ⌀{diameter:g} mm holes per "
            f"side; raise --size-mm or --margin-cells, lower --hole-dia-mm, or --no-holes"
        )
    return [
        trimesh.creation.cylinder(
            radius=diameter / 2,
            height=thickness + 2 * _EPS,
            sections=_HOLE_SEGMENTS,
            transform=_at(sx * (plate / 2 + pad), sy * spacing / 2, thickness / 2),
        )
        for sx in (-1, 1)
        for sy in (-1, 1)
    ]


def _prism_yz(points: list[tuple[float, float]], x0: float, x1: float) -> trimesh.Trimesh:
    """A filled YZ-plane triangle extruded along X."""
    solid = trimesh.creation.extrude_triangulation(
        vertices=np.array(points, dtype=float), faces=np.array([[0, 1, 2]]), height=x1 - x0
    )
    # extrude_triangulation lays the polygon in XY and extrudes along Z; cycle the axes so
    # the triangle lands in YZ with the extrusion running along X.
    cycle = np.eye(4)
    cycle[:3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
    solid.apply_transform(cycle)
    solid.apply_translation((x0, 0.0, 0.0))
    return solid


def _prism_xz(points: list[tuple[float, float]], y0: float, y1: float) -> trimesh.Trimesh:
    """A filled XZ-plane triangle extruded along Y."""
    solid = trimesh.creation.extrude_triangulation(
        vertices=np.array(points, dtype=float), faces=np.array([[0, 1, 2]]), height=y1 - y0
    )
    # Swap the extrusion axis into Y, keeping the triangle's own axes as X and Z.
    swap = np.eye(4)
    swap[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, 1, 0]]
    solid.apply_transform(swap)
    solid.apply_translation((0.0, y0, 0.0))
    return solid


def _strut_yz(
    p0: tuple[float, float],
    p1: tuple[float, float],
    x0: float,
    x1: float,
    thickness: float,
    margin: float = 0.5,
) -> trimesh.Trimesh:
    """A square-section bar between two YZ points, each end sunk into what it lands on.

    The end faces are square to the bar, so a tilted bar whose end stops on the surface it
    joins leaves one corner of that face sticking out. Each end is therefore pushed in
    along the bar's own axis by just enough to bury that corner — derived from the slope,
    so it holds at any leg height, and never so far that the far corner breaks out the
    other side.
    """
    (y0, z0), (y1, z1) = p0, p1
    dy, dz = y1 - y0, z1 - z0
    length = float(np.hypot(dy, dz))
    half = thickness / 2
    # Burying a corner offset by half across the bar costs half * |perpendicular run| /
    # |run along the surface| of extra length at that end.
    sink0 = half * abs(dz) / abs(dy) + margin
    sink1 = half * abs(dy) / abs(dz) + margin
    # Tilt about X so the box's long axis (local Z) points along the strut.
    angle = float(np.arctan2(-dy / length, dz / length))
    cos_a, sin_a = float(np.cos(angle)), float(np.sin(angle))
    rotate = np.array(
        [[1, 0, 0, 0], [0, cos_a, -sin_a, 0], [0, sin_a, cos_a, 0], [0, 0, 0, 1]], dtype=float
    )
    # p0 is at local -Z, p1 at +Z; sink each end further out along that axis.
    bar = _box(x0, x1, -half, half, -(length / 2 + sink0), length / 2 + sink1)
    bar.apply_transform(_at(0.0, (y0 + y1) / 2, (z0 + z1) / 2) @ rotate)
    return bar


# The tag drops into the frame from behind: the pocket is this much deeper and wider than
# the plate, and the front lip laps this far over its edge — less if the quiet zone can't
# spare it, but never less than _FRAME_RABBET_MIN_MM or there is nothing holding the tag.
_FRAME_SLACK_MM = 0.3
_FRAME_FIT_MM = 0.4
_FRAME_RABBET_MM = 3.0
_FRAME_RABBET_MIN_MM = 1.5


def _ring(
    outer_w: float, outer_h: float, inner_w: float, inner_h: float, z0: float, z1: float
) -> list[trimesh.Trimesh]:
    """A rectangular ring, as the four bars that make it up."""
    ox, oy, ix, iy = outer_w / 2, outer_h / 2, inner_w / 2, inner_h / 2
    return [
        _box(-ox, ox, iy, oy, z0, z1),
        _box(-ox, ox, -oy, -iy, z0, z1),
        _box(-ox, -ix, -iy, iy, z0, z1),
        _box(ix, ox, -iy, iy, z0, z1),
    ]


def _stud(x: float, y: float, radius: float, z0: float, z1: float) -> trimesh.Trimesh:
    """A round boss standing proud of the frame face — printable, no overhang."""
    return trimesh.creation.cylinder(  # type: ignore[no-any-return]
        radius=radius,
        height=z1 - z0,
        sections=_HOLE_SEGMENTS,
        transform=_at(x, y, (z0 + z1) / 2),
    )


def _bead_run(
    half_w: float, half_h: float, spacing: float, inset: float
) -> list[tuple[float, float]]:
    """Evenly spaced points along a rectangle's perimeter, leaving the corners free."""
    points: list[tuple[float, float]] = []
    for length, place in (
        (2 * half_w, lambda t: [(t, half_h), (t, -half_h)]),
        (2 * half_h, lambda t: [(half_w, t), (-half_w, t)]),
    ):
        span = length - 2 * inset
        count = int(span // spacing)
        if count < 1:
            continue
        step = span / count
        for i in range(count + 1):
            points.extend(place(-span / 2 + i * step))
    return points


def frame_solid(
    plate_mm: float,
    tag_thickness_mm: float,
    *,
    width_mm: float = 0.0,
    face_mm: float = 3.0,
    rabbet_mm: float = _FRAME_RABBET_MM,
) -> trimesh.Trimesh:
    """An ornamental picture frame the tag sits in, as a separate part to print in its own
    material.

    Modelled face-up in the tag's own coordinate frame, so the tag drops straight into the
    pocket at z=0 and the front lip laps over its edge — meaning the 3MF shows the pair
    assembled, and the frame still prints flat on its back with every ornament facing up.

    The moulding is a raised rim at each edge of the face with a run of beads on the land
    between them, and a two-tier rosette on each corner.
    """
    pocket = plate_mm + 2 * _FRAME_FIT_MM
    lip = plate_mm - 2 * rabbet_mm
    band = width_mm if width_mm > 0 else max(12.0, 0.22 * plate_mm)
    outer = plate_mm + 2 * band
    if lip <= 0:
        raise ValueError(f"a {plate_mm:.1f} mm plate is too small to frame")

    back = tag_thickness_mm + _FRAME_SLACK_MM
    face = back + face_mm
    # Everything on the face is proportioned off the band, so the frame scales with the tag.
    rim = band * 0.30
    rim_h = band * 0.14
    bead_r = band * 0.15
    rosette_r = band * 0.24

    parts = [
        # Pocket wall, then the front face whose inner edge laps over the tag.
        *_ring(outer, outer, pocket, pocket, 0.0, back),
        *_ring(outer, outer, lip, lip, back, face),
        # A raised rim along the outer and inner edges of the face.
        *_ring(outer, outer, outer - 2 * rim, outer - 2 * rim, face - _EPS, face + rim_h),
        *_ring(lip + 2 * rim, lip + 2 * rim, lip, lip, face - _EPS, face + rim_h * 0.7),
    ]
    # Beads along the land between the rims, and a rosette on each corner.
    land = (outer - 2 * rim + lip + 2 * rim) / 4
    parts += [
        _stud(x, y, bead_r, face - _EPS, face + rim_h * 0.9)
        for x, y in _bead_run(land, land, bead_r * 3.2, rosette_r * 2.2)
    ]
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(_stud(sx * land, sy * land, rosette_r, face - _EPS, face + rim_h))
            parts.append(_stud(sx * land, sy * land, rosette_r * 0.6, face, face + rim_h * 1.8))
    return _union(parts)


def leg_solid(
    height_mm: float,
    hole_spacing_mm: float,
    *,
    hole_dia_mm: float = 3.4,
    thickness_mm: float = 6.0,
    foot_thickness_mm: float = 4.0,
    brace: bool = True,
    outrigger: int = 1,
) -> trimesh.Trimesh:
    """A T-footed leg that bolts to one side's pair of tag holes, `height_mm` off the floor.

    Modelled standing on the floor at z=0 with the tag's mounting face at y=0. The two
    screw holes sit on the Y axis at z = height_mm ± hole_spacing_mm/2, matching the hole
    pair on one side of the plate, so the tag's center lands at `height_mm`. Two screws
    per leg pin the tag against pivoting; two identical legs carry a tag.

    The leg is a flat plate of constant thickness in X, so it prints lying on its side.
    The only thing off that plane is the outrigger against sideways tipping, which sticks
    out on one side only (`outrigger` = +1 or -1) and so reads as a raised boss on top of
    the plate rather than an overhang. Mirror it for the other leg: one points left, one
    right, and the pair braces the tag both ways.
    """
    if height_mm <= 0:
        raise ValueError(f"leg height must be positive; got {height_mm}")
    if hole_spacing_mm <= 0:
        raise ValueError(f"hole spacing must be positive; got {hole_spacing_mm}")
    radius = hole_pad_mm(hole_dia_mm)
    half_span = hole_spacing_mm / 2
    lowest = height_mm - half_span - radius
    if lowest < foot_thickness_mm:
        raise ValueError(
            f"leg height {height_mm:g} mm puts the lower hole into the foot; "
            f"raise --legs above {half_span + radius + foot_thickness_mm:g} mm"
        )
    width = 2 * radius
    top = height_mm + half_span
    # The whole foot scales with height: a taller leg needs a proportionally longer lever
    # against tipping and a thicker base to stay stiff.
    depth = max(45.0, 0.40 * height_mm)
    # A short toe the other way, so the foot is a T in profile and the stand can't rock
    # forward onto the tag hanging off the front face.
    toe = max(15.0, 0.08 * height_mm)
    foot_thickness_mm = max(foot_thickness_mm, 0.015 * height_mm)

    parts = [
        # Column, capped with a round boss on the upper hole so it mirrors the ear.
        _box(-width / 2, width / 2, 0.0, thickness_mm, 0.0, top),
        trimesh.creation.cylinder(
            radius=radius,
            height=thickness_mm,
            sections=_HOLE_SEGMENTS,
            transform=_at_along_y(0.0, thickness_mm / 2, top),
        ),
        # Foot: a long tail backward plus a short toe forward, the same width as the column
        # so the whole leg stays planar and prints lying on its side.
        _box(-width / 2, width / 2, -toe, depth, 0.0, foot_thickness_mm),
        # The toe is short enough to brace with a solid triangle rather than a strut.
        _prism_yz(
            [
                (_EPS, foot_thickness_mm - _EPS),
                (-toe, foot_thickness_mm - _EPS),
                (_EPS, foot_thickness_mm - _EPS + toe),
            ],
            -width / 2,
            width / 2,
        ),
    ]
    if outrigger:
        # Sideways outrigger: as deep in Y as the column it grows out of, and braced with
        # the same solid triangle as the toe — just turned into the XZ plane.
        side = float(np.sign(outrigger))
        span = max(35.0, 0.18 * height_mm)
        # Round the tip on the arm's own half-width, the way the column's top is rounded on
        # its half-width, and pull the centre in so the arm still reaches exactly `span`.
        tip = side * (span - thickness_mm / 2)
        root = side * (width / 2 - _EPS)
        x_in, x_out = sorted([root, tip])
        parts += [
            _box(x_in, x_out, 0.0, thickness_mm, 0.0, foot_thickness_mm),
            trimesh.creation.cylinder(
                radius=thickness_mm / 2,
                height=foot_thickness_mm,
                sections=_HOLE_SEGMENTS,
                transform=_at(tip, thickness_mm / 2, foot_thickness_mm / 2),
            ),
            # The same 45° triangle as the toe, just turned into XZ; the arm then runs on
            # flat at foot thickness for the rest of the span.
            _prism_xz(
                [
                    (root, foot_thickness_mm - _EPS),
                    (root + side * toe, foot_thickness_mm - _EPS),
                    (root, foot_thickness_mm - _EPS + toe),
                ],
                0.0,
                thickness_mm,
            ),
        ]
    if brace:
        # A diagonal strut up the back of the column — the tipping axis the T foot can't
        # stiffen. Both ends scale with the leg, so a taller leg gets a longer, shallower
        # strut reaching further down the foot stem.
        #
        # It must stop below the lower screw hole: a strut landing across it would bury the
        # screw. Its buried top end rises past the attach point by an amount that depends
        # on the slope, so solve for an attach height whose actual top clears the hole.
        ceiling = height_mm - half_span - radius - 1.0
        half_t = thickness_mm / 2
        attach_z = foot_thickness_mm + 0.62 * (top - foot_thickness_mm)
        for _ in range(4):
            reach_y = min(depth - 10.0, thickness_mm + 0.75 * (attach_z - foot_thickness_mm))
            run, rise = reach_y - thickness_mm, attach_z - foot_thickness_mm
            if run <= 0 or rise <= 0:
                break
            length = float(np.hypot(run, rise))
            sink = half_t * rise / run + 0.5
            overshoot = (sink * rise + half_t * run) / length
            if attach_z + overshoot <= ceiling:
                break
            attach_z = ceiling - overshoot
        reach_y = min(depth - 10.0, thickness_mm + 0.75 * (attach_z - foot_thickness_mm))
        if attach_z - foot_thickness_mm > 5.0 and reach_y - thickness_mm > 5.0:
            parts.append(
                _strut_yz(
                    (thickness_mm, attach_z),
                    (reach_y, foot_thickness_mm),
                    -width / 2,
                    width / 2,
                    thickness_mm,
                )
            )

    holes = [
        trimesh.creation.cylinder(
            radius=hole_dia_mm / 2,
            height=thickness_mm + 2 * _EPS,
            sections=_HOLE_SEGMENTS,
            transform=_at_along_y(0.0, thickness_mm / 2, height_mm + sz * half_span),
        )
        for sz in (-1, 1)
    ]
    return trimesh.boolean.difference([_union(parts), *holes])  # type: ignore[no-any-return]


def build_tag_meshes(
    family: str,
    tag_id: int,
    *,
    size_mm: float = 50.0,
    thickness_mm: float = 3.0,
    marker_mm: float = 0.8,
    margin_cells: float = 1.0,
    holes: bool = True,
    hole_dia_mm: float = 3.4,
    back_text: bool = True,
    text_depth_mm: float = 0.6,
    text_inlay: bool = True,
    frame: bool = False,
    frame_width_mm: float = 0.0,
    min_stroke_mm: float = 0.45,
) -> TagParts:
    """Build one tag's solids, all sharing one coordinate frame."""
    family = family.lower()
    if family not in _FAMILIES:
        raise ValueError(f"unsupported family: {family}; choose from {sorted(_FAMILIES)}")
    if size_mm <= 0:
        raise ValueError(f"size_mm must be positive; got {size_mm}")
    if not 0 < marker_mm < thickness_mm:
        raise ValueError(f"marker_mm must be in (0, {thickness_mm}); got {marker_mm}")
    if margin_cells < 0:
        raise ValueError(f"margin_cells must be non-negative; got {margin_cells}")
    if back_text and text_depth_mm >= thickness_mm - marker_mm:
        raise ValueError(
            f"text_depth_mm {text_depth_mm} would cut into the marker layer; "
            f"keep it under {thickness_mm - marker_mm:g} mm"
        )

    n = _FAMILIES[family][2]
    margin = margin_cells * size_mm / n
    plate = size_mm + 2 * margin
    z_swap = thickness_mm - marker_mm

    marker = marker_solid(family, tag_id, size_mm, z_swap, thickness_mm)
    cuts = [marker_solid(family, tag_id, size_mm, z_swap - _EPS, thickness_mm + _EPS)]
    if holes:
        cuts.extend(_hole_cuts(plate, thickness_mm, hole_dia_mm))

    text: trimesh.Trimesh | None = None
    if back_text:
        # The size is the black-border edge — the number to hand pose estimation as
        # tag_size — so a tag found in a drawer can be identified without measuring.
        lines = [family.upper(), f"ID {tag_id}", f"{size_mm:g}MM"]
        # The cut overshoots below the plate; the inlay stops flush with the bottom face,
        # so it fills the pocket exactly and the two never share a coplanar face.
        engraving = text_solid(lines, plate, plate, text_depth_mm, min_stroke_mm, z0=-_EPS)
        if engraving is not None:
            cuts.append(engraving)
            if text_inlay:
                text = text_solid(lines, plate, plate, text_depth_mm, min_stroke_mm, z0=0.0)

    footprint = plate_footprint_mm(plate, hole_dia_mm, holes)
    body = _rounded_plate(footprint[0], footprint[1], thickness_mm)
    base = trimesh.boolean.difference([body, *cuts])
    ornament = None
    if frame:
        # The lip is opaque and a different color, so it eats into the quiet zone. Give it
        # only what the margin can spare beyond the one cell the detector needs.
        spare = margin - size_mm / n
        if spare < _FRAME_RABBET_MIN_MM:
            raise ValueError(
                f"a frame needs {_FRAME_RABBET_MIN_MM:g} mm of margin beyond the tag's "
                f"one-cell quiet zone to grip; this plate spares {spare:.1f} mm — "
                f"raise --margin-cells (2 works for most tags)"
            )
        ornament = frame_solid(
            plate,
            thickness_mm,
            width_mm=frame_width_mm,
            rabbet_mm=min(_FRAME_RABBET_MM, spare),
        )
    return TagParts(base=base, marker=marker, text=text, frame=ornament)


_CONTENT_TYPES = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
    '<Default Extension="rels" '
    'ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
    '<Default Extension="model" '
    'ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
    "</Types>\n"
)

_RELS = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
    '<Relationship Id="rel0" Target="/3D/3dmodel.model" '
    'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
    "</Relationships>\n"
)


def _mesh_xml(mesh: trimesh.Trimesh) -> str:
    # Enough digits to keep _CELL_GROW-scale differences distinct; rounding them together
    # would weld separate vertices back into the non-manifold points they were built to avoid.
    verts = "".join(
        f'<vertex x="{v[0]:.9g}" y="{v[1]:.9g}" z="{v[2]:.9g}"/>' for v in mesh.vertices
    )
    tris = "".join(f'<triangle v1="{f[0]}" v2="{f[1]}" v3="{f[2]}"/>' for f in mesh.faces)
    return f"<mesh><vertices>{verts}</vertices><triangles>{tris}</triangles></mesh>"


def write_3mf(path: Path, parts: list[tuple[str, trimesh.Trimesh, str]]) -> Path:
    """Write a 3MF holding each (name, mesh, '#RRGGBB') part as a separately colored object."""
    materials = "".join(
        f'<base name="{escape(name)}" displaycolor="{display_color(color)}"/>'
        for name, _mesh, color in parts
    )
    objects = "".join(
        f'<object id="{i + 2}" type="model" pid="1" pindex="{i}" name="{escape(name)}">'
        f"{_mesh_xml(mesh)}</object>"
        for i, (name, mesh, _color) in enumerate(parts)
    )
    items = "".join(f'<item objectid="{i + 2}"/>' for i in range(len(parts)))
    model = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
        f'<resources><basematerials id="1">{materials}</basematerials>{objects}</resources>'
        f"<build>{items}</build></model>\n"
    )
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", _CONTENT_TYPES)
        z.writestr("_rels/.rels", _RELS)
        z.writestr("3D/3dmodel.model", model)
    return path


def generate_3d(
    ids: list[int],
    out_dir: Path,
    *,
    family: str = "tag36h11",
    size_mm: float = 50.0,
    thickness_mm: float = 3.0,
    marker_mm: float = 0.8,
    margin_cells: float = 1.0,
    holes: bool = True,
    hole_dia_mm: float = 3.4,
    back_text: bool = True,
    text_inlay: bool = True,
    frame: bool = False,
    frame_width_mm: float = 0.0,
    legs_mm: float = 0.0,
    leg_thickness_mm: float = 6.0,
    leg_brace: bool = True,
    base_color: str = "#F5F5F5",
    marker_color: str = "#141414",
    text_color: str | None = None,
    frame_color: str = "#8A6A3B",
) -> list[Path]:
    """Write an STL per solid plus one colored 3MF per tag into `out_dir`."""
    if not ids:
        raise ValueError("no IDs to render")
    # Legs bolt to the ears, so asking for legs asks for the ears that carry them.
    holes = holes or legs_mm > 0
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    written: list[Path] = []
    for tag_id in ids:
        tag = build_tag_meshes(
            family,
            tag_id,
            size_mm=size_mm,
            thickness_mm=thickness_mm,
            marker_mm=marker_mm,
            margin_cells=margin_cells,
            holes=holes,
            hole_dia_mm=hole_dia_mm,
            back_text=back_text,
            text_inlay=text_inlay,
            frame=frame,
            frame_width_mm=frame_width_mm,
        )
        stem = f"{family.lower()}_{tag_id:03d}"
        parts = [("base", tag.base, base_color), ("marker", tag.marker, marker_color)]
        if tag.text is not None:
            parts.append(("text", tag.text, text_color or marker_color))
        if tag.frame is not None:
            parts.append(("frame", tag.frame, frame_color))
        for name, mesh, _color in parts:
            path = out_dir / f"{stem}_{name}.stl"
            mesh.export(path)
            written.append(path)
        written.append(write_3mf(out_dir / f"{stem}.3mf", parts))

    if legs_mm > 0:
        # A mirrored pair: each leg's outrigger points outward, and the hole spacing
        # follows the plate the tag actually generated.
        spacing = hole_spacing_mm(plate_size_mm(family, size_mm, margin_cells), hole_dia_mm)
        for name, side in (("left", -1), ("right", 1)):
            leg = leg_solid(
                legs_mm,
                spacing,
                hole_dia_mm=hole_dia_mm,
                thickness_mm=leg_thickness_mm,
                brace=leg_brace,
                outrigger=side,
            )
            leg_path = out_dir / f"leg_{legs_mm:g}mm_{name}.stl"
            leg.export(leg_path)
            written.append(leg_path)
    return written


def plate_size_mm(family: str, size_mm: float, margin_cells: float) -> float:
    """Outer plate edge length for the given tag size and quiet-zone margin."""
    n = _FAMILIES[family.lower()][2]
    return size_mm + 2 * margin_cells * size_mm / n


__all__ = [
    "TagParts",
    "build_tag_meshes",
    "generate_3d",
    "hole_pad_mm",
    "hole_spacing_mm",
    "leg_solid",
    "marker_solid",
    "plate_footprint_mm",
    "plate_size_mm",
    "text_solid",
    "write_3mf",
]

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

from pathlib import Path
import zipfile

import numpy as np
import pytest
import trimesh

from dimos.utils.cli.apriltag import cell_matrix
from dimos.utils.cli.apriltag3d import (
    TagParts,
    build_tag_meshes,
    generate_3d,
    grid_rects,
    hole_pad_mm,
    hole_spacing_mm,
    leg_solid,
    marker_solid,
    plate_size_mm,
    text_solid,
)

SIZE_MM = 50.0
PLATE_MM = plate_size_mm("tag36h11", SIZE_MM, 1.0)
HOLE_DIA_MM = 3.4
SPACING_MM = hole_spacing_mm(PLATE_MM, HOLE_DIA_MM)


# Booleans are the slow part here, so the file shares one built tag and one written set.
@pytest.fixture(scope="module")
def tag() -> TagParts:
    return build_tag_meshes("tag36h11", 0, size_mm=SIZE_MM)


@pytest.fixture(scope="module")
def written(tmp_path_factory: pytest.TempPathFactory) -> list[Path]:
    out = tmp_path_factory.mktemp("tags")
    return generate_3d([0], out, size_mm=SIZE_MM, legs_mm=250.0, marker_color="#101010")


def test_grid_rects_stacks_runs_that_repeat_down_rows() -> None:
    assert grid_rects([[1, 1, 0], [1, 1, 0], [0, 1, 0]]) == [(0, 2, 0, 2), (1, 2, 2, 3)]


def test_marker_is_the_same_tag_the_pdf_draws() -> None:
    """Orientation is the one thing a flip or rotation would silently break."""
    cells = cell_matrix("tag36h11", 7)
    n = len(cells)
    cell, half = SIZE_MM / n, SIZE_MM / 2
    centers = np.array(
        [
            [-half + (c + 0.5) * cell, half - (r + 0.5) * cell, 2.6]
            for r in range(n)
            for c in range(n)
        ]
    )
    expected = np.array([bool(cells[r][c]) for r in range(n) for c in range(n)])
    marker = marker_solid("tag36h11", 7, SIZE_MM, 2.2, 3.0)
    assert np.array_equal(marker.contains(centers), expected)


def test_the_two_filaments_meet_flush_and_never_share_space(tag: TagParts) -> None:
    assert tag.base.is_watertight and tag.marker.is_watertight
    # Both end at the same z, so the printed face is flat rather than embossed.
    assert tag.base.bounds[1][2] == pytest.approx(3.0)
    assert tag.marker.bounds[1][2] == pytest.approx(3.0)
    assert trimesh.boolean.intersection([tag.base, tag.marker]).volume < 1e-6


def test_text_inlay_exactly_refills_the_pocket_it_cut(tag: TagParts) -> None:
    assert tag.text is not None
    assert tag.text.is_watertight
    # Flush with the bottom face, so it prints as the first layers.
    assert tag.text.bounds[0][2] == pytest.approx(0.0)
    solid = build_tag_meshes("tag36h11", 0, size_mm=SIZE_MM, back_text=False).base
    assert tag.base.volume + tag.text.volume == pytest.approx(solid.volume, rel=1e-6)


def test_flange_holes_are_open_and_clear_of_the_quiet_zone(tag: TagParts) -> None:
    cx = PLATE_MM / 2 + hole_pad_mm(HOLE_DIA_MM)
    cy = SPACING_MM / 2
    holes = np.array([[sx * cx, sy * cy, 1.5] for sx in (-1, 1) for sy in (-1, 1)])
    assert not tag.base.contains(holes).any()
    # The flanges widen the plate in X only; the tag's own margin stays solid plastic.
    assert tag.base.bounds[1][1] == pytest.approx(PLATE_MM / 2)
    corner = PLATE_MM / 2 - 0.5
    corners = np.array([[sx * corner, sy * corner, 1.5] for sx in (-1, 1) for sy in (-1, 1)])
    assert tag.base.contains(corners).all()


def test_no_holes_leaves_a_bare_square_plate() -> None:
    base = build_tag_meshes("tag36h11", 0, size_mm=SIZE_MM, holes=False).base
    assert base.bounds[1][0] == pytest.approx(PLATE_MM / 2)
    assert base.bounds[1][1] == pytest.approx(PLATE_MM / 2)


def test_back_text_is_mirrored_so_it_reads_from_below() -> None:
    solid = text_solid(["L"], 40.0, 40.0, 0.6, 0.1)
    assert solid is not None
    # 'L' is a stem with a foot to its right; mirrored, the stem sits at max X.
    lo, hi = solid.bounds
    stroke = (hi[0] - lo[0]) / 5
    assert solid.contains(np.array([[hi[0] - stroke / 2, 0.0, 0.3]]))[0]
    assert solid.contains(np.array([[lo[0] + stroke / 2, lo[1] + stroke / 2, 0.3]]))[0]
    assert not solid.contains(np.array([[lo[0] + stroke / 2, hi[1] - 0.1, 0.3]]))[0]


def test_text_too_fine_to_print_is_dropped_rather_than_generated() -> None:
    assert text_solid(["APRILTAG", "ID 0"], 6.0, 6.0, 0.6, 0.45) is None


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"family": "tag99h99"}, "unsupported family"),
        ({"thickness_mm": 1.0, "marker_mm": 1.0}, "marker_mm must be in"),
        ({"thickness_mm": 1.0, "marker_mm": 0.8}, "cut into the marker layer"),
        ({"size_mm": 12.0}, "too small to carry two"),
    ],
)
def test_unbuildable_configurations_are_rejected(kwargs: dict, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        build_tag_meshes(**{"family": "tag36h11", "tag_id": 0, **kwargs})


def test_legs_bolt_to_the_holes_the_plate_actually_has() -> None:
    leg = leg_solid(250.0, SPACING_MM)
    assert leg.is_watertight
    # Both holes open end to end — the brace must never land across the lower one.
    for z in (250.0 - SPACING_MM / 2, 250.0 + SPACING_MM / 2):
        assert not leg.contains(np.array([[0.0, y, z] for y in (0.2, 3.0, 5.8)])).any()
    # Planar but for the outrigger, so it prints lying on its side.
    assert leg.bounds[0][0] == pytest.approx(-hole_pad_mm(HOLE_DIA_MM))


def test_generate_3d_writes_a_file_set_that_reloads_watertight(written: list[Path]) -> None:
    assert sorted(p.name for p in written) == [
        "leg_250mm_left.stl",
        "leg_250mm_right.stl",
        "tag36h11_000.3mf",
        "tag36h11_000_base.stl",
        "tag36h11_000_marker.stl",
        "tag36h11_000_text.stl",
    ]
    # STL stores float32, and near-coincident vertices used to weld into bowties on reload.
    for path in (p for p in written if p.suffix == ".stl"):
        assert trimesh.load(path).is_watertight, path.name


def test_3mf_carries_every_part_with_its_own_color(written: list[Path]) -> None:
    archive_path = next(p for p in written if p.suffix == ".3mf")
    with zipfile.ZipFile(archive_path) as archive:
        model = archive.read("3D/3dmodel.model").decode()
    assert '<base name="marker" displaycolor="#101010FF"/>' in model
    assert model.count("<object ") == 3
    assert model.count("<item ") == 3


def test_generate_3d_rejects_empty_ids(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="no IDs"):
        generate_3d([], tmp_path)


@pytest.mark.parametrize("family", ["tag25h9", "aruco_4x4_50"])
def test_other_families_build(family: str) -> None:
    parts = build_tag_meshes(family, 1, size_mm=60.0)
    assert parts.base.is_watertight and parts.marker.is_watertight

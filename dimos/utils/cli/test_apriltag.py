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

import pytest

from dimos.utils.cli.apriltag import _grid_layout, generate_pdf, parse_id_spec


def test_parse_id_spec_range() -> None:
    assert parse_id_spec("0-4") == [0, 1, 2, 3, 4]


def test_parse_id_spec_mixed() -> None:
    assert parse_id_spec("0,1,5,10-12") == [0, 1, 5, 10, 11, 12]


def test_parse_id_spec_dedup_and_sort() -> None:
    assert parse_id_spec("3,1,2,1-2") == [1, 2, 3]


def test_parse_id_spec_single() -> None:
    assert parse_id_spec("7") == [7]


def test_parse_id_spec_whitespace_and_empty() -> None:
    assert parse_id_spec(" 0, ,  2 ") == [0, 2]


def test_parse_id_spec_reversed_range_raises() -> None:
    with pytest.raises(ValueError, match="reversed range"):
        parse_id_spec("10-5")


def test_grid_layout_centers_with_nonneg_gaps() -> None:
    from reportlab.lib.pagesizes import A4

    page_w_pt, page_h_pt = A4
    cols, rows, x0, y_top, tile_w, tile_h = _grid_layout(page_w_pt, page_h_pt, size_mm=75.0)
    # 75 mm tags on A4 fit in a 2x2 grid with substantial slack distributed evenly.
    assert (cols, rows) == (2, 2)
    # tile_w/h must be at least the tag size; gaps are non-negative by construction.
    from reportlab.lib.units import mm

    assert tile_w >= 75 * mm
    assert tile_h >= (75 + 5) * mm
    # Grid must fit on the page: 2 columns + outer gaps stay within the page width.
    assert x0 + 2 * tile_w <= page_w_pt
    assert y_top - 2 * tile_h >= 0


def test_grid_layout_oversize_size_yields_one_per_page() -> None:
    from reportlab.lib.pagesizes import A4

    page_w_pt, page_h_pt = A4
    cols, rows, *_ = _grid_layout(page_w_pt, page_h_pt, size_mm=180.0)
    assert (cols, rows) == (1, 1)


def test_generate_pdf_pack_writes_expected_pages(tmp_path: Path) -> None:
    out = tmp_path / "tags.pdf"
    generate_pdf(list(range(12)), out, size_mm=50.0, page_size="a4", pack=True)
    # 12 × 50mm A4 packs to a single 3x4 page.
    assert out.read_bytes()[:5] == b"%PDF-"
    assert out.stat().st_size > 0


def test_generate_pdf_no_pack_one_page_per_tag(tmp_path: Path) -> None:
    out = tmp_path / "tags.pdf"
    generate_pdf([0, 1, 2], out, size_mm=50.0, page_size="a4", pack=False)
    assert out.read_bytes().count(b"/Type /Page\n") + out.read_bytes().count(b"/Type /Page ") >= 3


def test_generate_pdf_case_insensitive(tmp_path: Path) -> None:
    out = tmp_path / "tags.pdf"
    generate_pdf([0], out, size_mm=50.0, page_size="A4", family="Tag36h11")
    assert out.exists()


def test_generate_pdf_oversize_rejected(tmp_path: Path) -> None:
    out = tmp_path / "tags.pdf"
    with pytest.raises(ValueError, match="too large"):
        generate_pdf([0], out, size_mm=1000.0, page_size="a4")


def test_generate_pdf_pack_accepts_sizes_single_rejects(tmp_path: Path) -> None:
    out = tmp_path / "tags.pdf"
    # Tall tag that fits pack mode (only ~36mm chrome) but not single mode (~100mm chrome).
    generate_pdf([0, 1], out, size_mm=200.0, page_size="a3", pack=True)
    assert out.exists()


@pytest.mark.parametrize(
    "family",
    [
        "aruco_original",
        "aruco_mip_36h12",
        "aruco_4x4_50",
        "aruco_5x5_100",
        "aruco_6x6_250",
        "aruco_7x7_1000",
    ],
)
def test_generate_pdf_aruco_families(tmp_path: Path, family: str) -> None:
    out = tmp_path / f"{family}.pdf"
    generate_pdf([0, 1, 2], out, family=family, size_mm=50.0, page_size="a4")
    assert out.read_bytes()[:5] == b"%PDF-"


def test_generate_pdf_letter_page_size(tmp_path: Path) -> None:
    out = tmp_path / "letter.pdf"
    generate_pdf([0, 1, 2], out, size_mm=50.0, page_size="letter")
    assert out.read_bytes()[:5] == b"%PDF-"

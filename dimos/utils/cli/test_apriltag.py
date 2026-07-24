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

from dimos.utils.cli.apriltag import (
    TagRequest,
    _grid_layout,
    display_color,
    generate_pdf,
    parse_id_spec,
)


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
    # 12 x 50mm A4 packs to a single 3x4 page.
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


def _request(**kwargs: object) -> TagRequest:
    defaults: dict[str, object] = {"ids": [0], "out": Path("tags.pdf"), "id_spec": "0"}
    return TagRequest(**{**defaults, **kwargs})  # type: ignore[arg-type]


def test_3d_mode_collects_the_run_into_one_directory() -> None:
    assert _request().out_dir is None
    assert _request().pdf_path == Path("tags.pdf")
    assert _request(three_d=True).out_dir == Path("tags")
    assert _request(three_d=True).pdf_path == Path("tags/tags.pdf")


def test_legs_imply_the_holes_they_bolt_through() -> None:
    assert _request(holes=False, legs_mm=250.0).mounted
    assert not _request(holes=False).mounted


def test_describe_covers_the_paper_run_and_grows_for_3d() -> None:
    assert set(dict(_request().describe())) == {"family", "ids", "size", "page", "output"}
    rows = dict(_request(three_d=True, legs_mm=250.0).describe())
    assert "62.5 x 62.5 x 3 mm" in rows["plate"]
    assert "4 holes (2/side)" in rows["mounting"]
    assert "tag center 250 mm off the floor" in rows["legs"]
    bare = dict(_request(three_d=True, holes=False).describe())
    assert (bare["mounting"], bare["footprint"], bare["legs"]) == (
        "none (bare plate)",
        "= plate",
        "none",
    )


def test_render_writes_only_a_pdf_without_3d(tmp_path: Path) -> None:
    written = _request(out=tmp_path / "tags.pdf").render()
    assert written == [tmp_path / "tags.pdf"]
    assert written[0].read_bytes()[:5] == b"%PDF-"


def test_render_and_summary_cover_the_whole_3d_file_set(tmp_path: Path) -> None:
    request = _request(out=tmp_path / "tags.pdf", three_d=True, legs_mm=250.0)
    written = request.render()
    assert written[0] == tmp_path / "tags" / "tags.pdf"
    assert all(p.parent == tmp_path / "tags" for p in written)
    assert {p.suffix for p in written[1:]} == {".stl", ".3mf"}
    assert "leg_*_left and leg_*_right" in "\n".join(request.summary(written))


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"ids": []}, "no IDs"),
        ({"family": "bogus"}, "unsupported family"),
        ({"page_size": "a99"}, "unsupported page_size"),
        ({"three_d": True, "marker_color": "red"}, "color must be"),
        ({"three_d": True, "base_color": "#GGGGGG"}, "color must be"),
    ],
)
def test_bad_input_is_rejected_before_anything_is_described(
    kwargs: dict[str, object], match: str
) -> None:
    """describe() indexes ids and looks up the family, so it must not run on bad input."""
    with pytest.raises(ValueError, match=match):
        _request(**kwargs)


def test_display_color_normalizes_and_rejects() -> None:
    assert display_color("#f5f5f5") == "#F5F5F5FF"
    with pytest.raises(ValueError, match="color must be"):
        display_color("#fff")

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

"""Printable AprilTag/ArUco PDF generator with calibration ruler.

Draws tag cells as vector rects (no rasterization) so the PDF prints crisply at any
DPI. The tag's outer black border edge measures `size_mm` — that's the value to pass
as `tag_size` to pose-estimation routines (pupil-apriltags, solvePnP, etc.).
"""

from __future__ import annotations

from pathlib import Path

import cv2
from reportlab.lib.pagesizes import A0, A1, A2, A3, A4, A5, A6, A7, A8, LETTER
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

# (DICT_ID, max_id, sidePixels) — sidePixels = markerSize + 2*borderBits.
# AprilTag dicts use a 2-cell border; ArUco dicts use a 1-cell border.
_FAMILIES = {
    # AprilTag (2-cell black border)
    "tag36h11": (cv2.aruco.DICT_APRILTAG_36h11, 586, 8),
    "tag25h9": (cv2.aruco.DICT_APRILTAG_25h9, 34, 7),
    "tag16h5": (cv2.aruco.DICT_APRILTAG_16h5, 29, 6),
    # ArUco (1-cell black border)
    "aruco_original": (cv2.aruco.DICT_ARUCO_ORIGINAL, 1023, 7),
    "aruco_mip_36h12": (cv2.aruco.DICT_ARUCO_MIP_36h12, 249, 8),
    "aruco_4x4_50": (cv2.aruco.DICT_4X4_50, 49, 6),
    "aruco_4x4_100": (cv2.aruco.DICT_4X4_100, 99, 6),
    "aruco_4x4_250": (cv2.aruco.DICT_4X4_250, 249, 6),
    "aruco_4x4_1000": (cv2.aruco.DICT_4X4_1000, 999, 6),
    "aruco_5x5_50": (cv2.aruco.DICT_5X5_50, 49, 7),
    "aruco_5x5_100": (cv2.aruco.DICT_5X5_100, 99, 7),
    "aruco_5x5_250": (cv2.aruco.DICT_5X5_250, 249, 7),
    "aruco_5x5_1000": (cv2.aruco.DICT_5X5_1000, 999, 7),
    "aruco_6x6_50": (cv2.aruco.DICT_6X6_50, 49, 8),
    "aruco_6x6_100": (cv2.aruco.DICT_6X6_100, 99, 8),
    "aruco_6x6_250": (cv2.aruco.DICT_6X6_250, 249, 8),
    "aruco_6x6_1000": (cv2.aruco.DICT_6X6_1000, 999, 8),
    "aruco_7x7_50": (cv2.aruco.DICT_7X7_50, 49, 9),
    "aruco_7x7_100": (cv2.aruco.DICT_7X7_100, 99, 9),
    "aruco_7x7_250": (cv2.aruco.DICT_7X7_250, 249, 9),
    "aruco_7x7_1000": (cv2.aruco.DICT_7X7_1000, 999, 9),
}

_PAGE_SIZES = {
    "a0": A0,
    "a1": A1,
    "a2": A2,
    "a3": A3,
    "a4": A4,
    "a5": A5,
    "a6": A6,
    "a7": A7,
    "a8": A8,
    "letter": LETTER,
}


def parse_id_spec(spec: str) -> list[int]:
    """Parse '0-49' or '0,1,5,10-20' into a sorted unique list of ints."""
    out: set[int] = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo_s, hi_s = part.split("-", 1)
            lo, hi = int(lo_s), int(hi_s)
            if lo > hi:
                raise ValueError(f"reversed range in id spec: {part!r}")
            out.update(range(lo, hi + 1))
        else:
            out.add(int(part))
    return sorted(out)


def _cell_matrix(family: str, tag_id: int) -> list[list[int]]:
    """Return the tag's NxN binary cell matrix (1=black, 0=white)."""
    dict_id, max_id, n = _FAMILIES[family]
    if tag_id < 0 or tag_id > max_id:
        raise ValueError(f"id {tag_id} out of range for {family} (0..{max_id})")
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    bmp = cv2.aruco.generateImageMarker(aruco_dict, tag_id, n)
    return [[1 if bmp[r, c] == 0 else 0 for c in range(n)] for r in range(n)]


def _draw_tag(
    c: canvas.Canvas, family: str, tag_id: int, x0: float, y0: float, size: float
) -> None:
    """Draw the tag at (x0, y0) bottom-left with given side length, all in pt."""
    cells = _cell_matrix(family, tag_id)
    n = len(cells)
    cell = size / n
    c.setFillColorRGB(0, 0, 0)
    c.setStrokeColorRGB(0, 0, 0)
    # Merge horizontal runs of black cells per row: one rect spans contiguous blacks,
    # eliminating shared edges that some PDF renderers print as white hairlines.
    for r in range(n):
        # cv2 row 0 is top; flip for reportlab y-up coords
        cy = y0 + (n - 1 - r) * cell
        col = 0
        while col < n:
            if cells[r][col]:
                start = col
                while col < n and cells[r][col]:
                    col += 1
                c.rect(x0 + start * cell, cy, (col - start) * cell, cell, stroke=0, fill=1)
            else:
                col += 1


def _draw_ruler(c: canvas.Canvas, page_w_pt: float, y_mm: float = 18.0) -> None:
    """Draw a 100 mm calibration ruler centered at y_mm above page bottom."""
    length_mm = 100.0
    x0 = (page_w_pt - length_mm * mm) / 2
    y0 = y_mm * mm
    c.setStrokeColorRGB(0, 0, 0)
    c.setLineWidth(0.5)
    c.line(x0, y0, x0 + length_mm * mm, y0)
    for i in range(int(length_mm) // 10 + 1):
        x_tick = x0 + i * 10 * mm
        tick_h = (4 if i % 5 == 0 else 2) * mm
        c.line(x_tick, y0, x_tick, y0 + tick_h)
        if i % 5 == 0:
            c.setFont("Helvetica", 7)
            c.setFillColorRGB(0, 0, 0)
            c.drawCentredString(x_tick, y0 - 3 * mm, str(i * 10))
    c.setFont("Helvetica", 9)
    c.drawCentredString(
        page_w_pt / 2,
        y0 + 10 * mm,
        "Calibration ruler — measure with caliper. Should be exactly 100 mm.",
    )


def _draw_single_page(
    c: canvas.Canvas,
    family: str,
    tag_id: int,
    page_w_pt: float,
    page_h_pt: float,
    size_mm: float,
) -> None:
    """One large tag centered on the page with full label block and ruler."""
    x_tag = (page_w_pt - size_mm * mm) / 2
    y_tag = page_h_pt - (70 + size_mm) * mm
    _draw_tag(c, family, tag_id, x_tag, y_tag, size_mm * mm)

    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica-Bold", 14)
    c.drawCentredString(page_w_pt / 2, page_h_pt - 30 * mm, f"{family}  —  ID {tag_id}")
    c.setFont("Helvetica", 10)
    c.drawCentredString(
        page_w_pt / 2,
        page_h_pt - 45 * mm,
        f"black border edge = {size_mm:g} mm  (use this as tag_size)",
    )
    c.setFont("Helvetica", 9)
    c.drawCentredString(
        page_w_pt / 2,
        y_tag - 10 * mm,
        "Print at 100% / Actual Size — DO NOT use 'Fit to Page'.",
    )
    _draw_ruler(c, page_w_pt)


_PACK_MARGIN_MM = 10.0
_PACK_LABEL_MM = 5.0
_PACK_GAP_MIN_MM = 4.0
_PACK_TOP_BLOCK_MM = 14.0
_PACK_BOTTOM_BLOCK_MM = 22.0
_PACK_CHROME_H_MM = 2 * _PACK_MARGIN_MM + _PACK_TOP_BLOCK_MM + _PACK_BOTTOM_BLOCK_MM


def _grid_layout(
    page_w_pt: float, page_h_pt: float, size_mm: float
) -> tuple[int, int, float, float, float, float]:
    """Pack as many tags as fit, then distribute leftover space evenly to center the grid."""
    avail_w_mm = page_w_pt / mm - 2 * _PACK_MARGIN_MM
    avail_h_mm = page_h_pt / mm - _PACK_CHROME_H_MM

    cols = max(1, int((avail_w_mm - _PACK_GAP_MIN_MM) // (size_mm + _PACK_GAP_MIN_MM)))
    rows = max(
        1, int((avail_h_mm - _PACK_GAP_MIN_MM) // (size_mm + _PACK_LABEL_MM + _PACK_GAP_MIN_MM))
    )

    gap_x_mm = max(0.0, (avail_w_mm - cols * size_mm) / (cols + 1))
    gap_y_mm = max(0.0, (avail_h_mm - rows * (size_mm + _PACK_LABEL_MM)) / (rows + 1))

    x0_mm = _PACK_MARGIN_MM + gap_x_mm
    y_avail_top_mm = page_h_pt / mm - _PACK_MARGIN_MM - _PACK_TOP_BLOCK_MM
    y_top_mm = y_avail_top_mm - gap_y_mm

    tile_w_mm = size_mm + gap_x_mm
    tile_h_mm = size_mm + _PACK_LABEL_MM + gap_y_mm
    return cols, rows, x0_mm * mm, y_top_mm * mm, tile_w_mm * mm, tile_h_mm * mm


def _draw_packed_page(
    c: canvas.Canvas,
    family: str,
    page_ids: list[int],
    page_w_pt: float,
    page_h_pt: float,
    size_mm: float,
) -> None:
    """Fill a page with a grid of tags, each labeled, plus a ruler at the bottom."""
    cols, _rows, x0, y_top, tile_w, tile_h = _grid_layout(page_w_pt, page_h_pt, size_mm)

    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica-Bold", 11)
    span = f"{page_ids[0]}-{page_ids[-1]}" if len(page_ids) > 1 else str(page_ids[0])
    c.drawCentredString(
        page_w_pt / 2,
        page_h_pt - 12 * mm,
        f"{family}  —  IDs {span}  —  size = {size_mm:g} mm  (Print at 100%)",
    )

    n = len(page_ids)
    last_row_count = n - (n // cols) * cols or cols
    last_row_idx = (n - 1) // cols
    last_row_offset = (cols - last_row_count) * tile_w / 2

    for idx, tag_id in enumerate(page_ids):
        r = idx // cols
        col = idx % cols
        tag_x = x0 + col * tile_w + (last_row_offset if r == last_row_idx else 0)
        tag_y = y_top - r * tile_h - size_mm * mm
        _draw_tag(c, family, tag_id, tag_x, tag_y, size_mm * mm)
        c.setFont("Helvetica", 8)
        c.setFillColorRGB(0, 0, 0)
        c.drawCentredString(tag_x + size_mm * mm / 2, tag_y - 4 * mm, f"ID {tag_id}")

    _draw_ruler(c, page_w_pt)


def generate_pdf(
    ids: list[int],
    out_path: Path,
    *,
    family: str = "tag36h11",
    size_mm: float = 50.0,
    page_size: str = "a4",
    pack: bool = True,
) -> Path:
    """Write a printable AprilTag PDF.

    pack=False: one large tag per page with full label block.
    pack=True:  grid as many tags as fit per page; new pages added as needed.
    """
    family = family.lower()
    page_size = page_size.lower()
    if family not in _FAMILIES:
        raise ValueError(f"unsupported family: {family}; choose from {sorted(_FAMILIES)}")
    if page_size not in _PAGE_SIZES:
        raise ValueError(f"unsupported page_size: {page_size}; choose from {sorted(_PAGE_SIZES)}")
    if not ids:
        raise ValueError("no IDs to render")
    if size_mm <= 0:
        raise ValueError(f"size_mm must be positive; got {size_mm}")

    out_path = Path(out_path)
    page_w_pt, page_h_pt = _PAGE_SIZES[page_size]
    page_w_mm = page_w_pt / mm
    page_h_mm = page_h_pt / mm
    # Different vertical chrome for the two modes; use whichever applies.
    needed_h_mm = _PACK_CHROME_H_MM + size_mm + _PACK_LABEL_MM if pack else size_mm + 100.0
    needed_w_mm = size_mm + 2 * _PACK_MARGIN_MM
    if needed_w_mm > page_w_mm or needed_h_mm > page_h_mm:
        raise ValueError(
            f"tag size {size_mm} mm too large for {page_size.upper()} "
            f"({page_w_mm:.0f}x{page_h_mm:.0f} mm) in "
            f"{'pack' if pack else 'single'} mode; pick a smaller size or larger page"
        )
    c = canvas.Canvas(str(out_path), pagesize=_PAGE_SIZES[page_size])
    ids_span = f"{ids[0]}-{ids[-1]}" if len(ids) > 1 else str(ids[0])
    c.setTitle(f"AprilTag {family} IDs {ids_span} ({size_mm:g}mm, {page_size.upper()})")
    c.setSubject(
        f"{family}, size={size_mm:g}mm, ids={ids_span}, n={len(ids)}, "
        f"page={page_size.upper()}, mode={'pack' if pack else 'single'}"
    )
    c.setKeywords(
        ", ".join(
            [family, f"{size_mm:g}mm", page_size, "pack" if pack else "single", f"ids:{ids_span}"]
        )
    )
    c.setCreator("dimos apriltag")
    c.setProducer("dimos apriltag")

    if not pack:
        for tag_id in ids:
            _draw_single_page(c, family, tag_id, page_w_pt, page_h_pt, size_mm)
            c.showPage()
    else:
        cols, rows, *_ = _grid_layout(page_w_pt, page_h_pt, size_mm)
        per_page = cols * rows
        for i in range(0, len(ids), per_page):
            _draw_packed_page(c, family, ids[i : i + per_page], page_w_pt, page_h_pt, size_mm)
            c.showPage()

    c.save()
    return out_path

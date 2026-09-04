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

"""The apriltag command; the implementation lives in dimos.cli.apriltag."""

from __future__ import annotations

from pathlib import Path

import typer


def apriltag(
    out: Path = typer.Option(Path("apriltags.pdf"), "--out", "-o", help="Output PDF path"),
    ids: str = typer.Option("0-11", "--ids", help="ID spec, e.g. '0-49' or '0,1,5,10-20'"),
    size_mm: float = typer.Option(
        50.0, "--size-mm", "-s", help="Tag black-border edge size in mm (typical: 50 or 100)"
    ),
    page_size: str = typer.Option(
        "a4", "--page-size", "-p", help="Page size: a0..a8 (ISO A series) or letter"
    ),
    pack: bool = typer.Option(
        True, "--pack/--no-pack", help="Pack as many tags per page as fit (vs one per page)"
    ),
    family: str = typer.Option(
        "tag36h11",
        "--family",
        help=(
            "Tag family: AprilTag (tag36h11, tag25h9, tag16h5) or "
            "ArUco (aruco_original, aruco_mip_36h12, aruco_{4x4,5x5,6x6,7x7}_{50,100,250,1000})"
        ),
    ),
    three_d: bool = typer.Option(
        False,
        "--3d/--no-3d",
        help="Also emit 3D-printable STL pairs + colored 3MF per tag (into a directory)",
    ),
    thickness_mm: float = typer.Option(3.0, "--thickness-mm", help="[3d] Plate thickness in mm"),
    marker_mm: float = typer.Option(
        0.8, "--marker-mm", help="[3d] Depth of the dark top layer in mm (filament swap height)"
    ),
    margin_cells: float = typer.Option(
        1.0,
        "--margin-cells",
        help="[3d] Light quiet-zone border around the tag, in tag cells",
    ),
    holes: bool = typer.Option(
        True, "--holes/--no-holes", help="[3d] Corner mounting holes through the plate"
    ),
    hole_dia_mm: float = typer.Option(
        3.4, "--hole-dia-mm", help="[3d] Mounting hole diameter in mm (3.4 = M3 clearance)"
    ),
    back_text: bool = typer.Option(
        True, "--back-text/--no-back-text", help="[3d] Engrave family + ID on the back"
    ),
    text_inlay: bool = typer.Option(
        True,
        "--text-inlay/--no-text-inlay",
        help="[3d] Fill the back engraving with a colored solid (off = bare engraving)",
    ),
    legs: float = typer.Option(
        0.0,
        "--legs",
        metavar="HEIGHT_MM",
        help=(
            "[3d] Also generate a flat-printable T-footed leg. HEIGHT_MM is floor to TAG CENTER "
            "(the tag's pose origin). Implies --holes"
        ),
    ),
    leg_thickness_mm: float = typer.Option(
        6.0, "--leg-thickness-mm", help="[3d] Leg column thickness in mm"
    ),
    leg_brace: bool = typer.Option(
        True, "--leg-brace/--no-leg-brace", help="[3d] Gusset up the back of the leg column"
    ),
) -> None:
    """Generate a printable AprilTag/ArUco PDF, optionally with 3D-printable STLs."""
    from dimos.cli.apriltag import TagRequest, parse_id_spec

    try:
        request = TagRequest(
            ids=parse_id_spec(ids),
            out=out,
            id_spec=ids,
            family=family,
            size_mm=size_mm,
            page_size=page_size,
            pack=pack,
            three_d=three_d,
            thickness_mm=thickness_mm,
            marker_mm=marker_mm,
            margin_cells=margin_cells,
            holes=holes,
            hole_dia_mm=hole_dia_mm,
            back_text=back_text,
            text_inlay=text_inlay,
            legs_mm=legs,
            leg_thickness_mm=leg_thickness_mm,
            leg_brace=leg_brace,
        )
        typer.echo("apriltag")
        for key, value in request.describe():
            typer.echo(f"  {key:<10} {value}")
        typer.echo("")
        written = request.render()
    except ValueError as exc:
        raise typer.BadParameter(str(exc)) from exc

    for line in request.summary(written):
        typer.echo(line)

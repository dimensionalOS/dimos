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

"""The `dimos urdf-to-usd` command; the conversion lives in dimos.robot.assets.usd."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Annotated, cast

import typer

if TYPE_CHECKING:
    from dimos.robot.assets.usd import MeshMode


def urdf_to_usd(
    source: Annotated[Path, typer.Argument(help="A .urdf or .urdf.xacro file.")],
    out: Annotated[
        Path | None,
        typer.Option("-o", "--out", help="Where to write the layer. Default: stdout."),
    ] = None,
    package: Annotated[
        list[str] | None,
        typer.Option(
            "-p",
            "--package",
            help="A ROS package root for package:// URIs, as name=path. Repeatable.",
        ),
    ] = None,
    xacro_arg: Annotated[
        list[str] | None,
        typer.Option("-x", "--xacro-arg", help="A xacro argument, as name=value. Repeatable."),
    ] = None,
    mesh_mode: Annotated[
        str,
        typer.Option(
            "--mesh-mode",
            help="inline bakes mesh triangles into the layer; reference keeps the file paths.",
        ),
    ] = "inline",
    no_collision: Annotated[
        bool, typer.Option("--no-collision", help="Leave <collision> geometry out.")
    ] = False,
) -> None:
    """Convert a URDF (or xacro) robot description to a USD layer."""
    # Imported here: usd-core is in the scene-cooking extra and has no linux-aarch64 wheel, so
    # `dimos --help` on a Jetson must not pull it in.
    from dimos.robot.assets.model import RobotModel
    from dimos.robot.assets.usd import urdf_to_usd as convert

    if mesh_mode not in {"inline", "reference"}:
        raise typer.BadParameter("--mesh-mode must be 'inline' or 'reference'")
    model = RobotModel.from_file(
        source,
        package_paths=_pairs(package, "--package"),
        xacro_args=_pairs(xacro_arg, "--xacro-arg"),
    )
    usd = convert(
        model.load(),
        mesh_mode=cast("MeshMode", mesh_mode),
        include_collision=not no_collision,
    )
    if out is None:
        typer.echo(usd)
        return
    out.write_text(usd)
    typer.echo(f"wrote {out} ({len(usd)} bytes)", err=True)


def _pairs(values: list[str] | None, option: str) -> dict[str, str]:
    pairs = {}
    for value in values or []:
        name, _, rest = value.partition("=")
        if not name or not rest:
            raise typer.BadParameter(f"{option} takes name=value, got {value!r}")
        pairs[name] = rest
    return pairs

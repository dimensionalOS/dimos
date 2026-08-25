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

"""Render Blueprint stream connections as a Graphviz SVG."""

import os
from pathlib import Path
import subprocess
import sys

import typer

from dimos.core.introspection.blueprint.dot import render_svg
from dimos.robot import get_all_blueprints


class GraphRenderError(RuntimeError):
    """Raised when the isolated Graphviz renderer cannot produce the SVG."""


def _render_graph(blueprint_name: str, output: Path) -> None:
    blueprint = get_all_blueprints.get_by_name(blueprint_name)
    render_svg(blueprint, str(output))


def _render_in_subprocess(blueprint_name: str, output: Path) -> None:
    """Import and render a Blueprint without opening its runtime transports."""
    env = os.environ.copy()
    env["LCM_DEFAULT_URL"] = "memq://"
    env["viewer"] = "none"
    env.setdefault("PYTHONWARNINGS", "ignore")

    result = subprocess.run(
        [sys.executable, "-m", __name__, blueprint_name, str(output)],
        env=env,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise GraphRenderError(detail or "Blueprint graph renderer exited without an error message")


def graph(
    blueprint: str = typer.Argument(..., help="Blueprint or module name to visualize."),
    output: Path | None = typer.Option(
        None,
        "--output",
        "-o",
        help="SVG output path. Defaults to <blueprint>.svg.",
    ),
) -> None:
    """Render a Blueprint's active stream connections as a Graphviz SVG."""
    output_path = (output or Path(f"{blueprint}.svg")).expanduser().resolve()
    if output_path.suffix.lower() != ".svg":
        raise typer.BadParameter("output path must end in .svg", param_hint="--output")

    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        _render_in_subprocess(blueprint, output_path)
    except (GraphRenderError, OSError) as exc:
        typer.echo(typer.style(str(exc), fg=typer.colors.RED), err=True)
        raise typer.Exit(1) from exc

    typer.echo(f"Blueprint graph written to {output_path}")


def _worker_main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: python -m dimos.cli.commands.graph BLUEPRINT OUTPUT.svg")

    try:
        _render_graph(sys.argv[1], Path(sys.argv[2]))
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    _worker_main()

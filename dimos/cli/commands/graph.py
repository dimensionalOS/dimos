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

"""Render Blueprint relationships as a Graphviz SVG."""

import os
from pathlib import Path
import subprocess
import sys

import typer

from dimos.core.introspection.blueprint.dot import render_svg
from dimos.robot import get_all_blueprints


class GraphRenderError(RuntimeError):
    """Raised when the isolated Graphviz renderer cannot produce the SVG."""


def _render_graph(blueprint_name: str, output: Path, *, include_rpc: bool) -> None:
    blueprint = get_all_blueprints.get_by_name(blueprint_name)
    render_svg(blueprint, str(output), include_rpc=include_rpc)


def _render_in_subprocess(blueprint_name: str, output: Path, *, include_rpc: bool) -> None:
    """Import and render a Blueprint without opening its runtime transports."""
    env = os.environ.copy()
    env["LCM_DEFAULT_URL"] = "memq://"
    env["viewer"] = "none"
    env.setdefault("PYTHONWARNINGS", "ignore")

    command = [sys.executable, "-m", __name__, blueprint_name, str(output)]
    if include_rpc:
        command.append("--rpc")

    result = subprocess.run(
        command,
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
    rpc: bool = typer.Option(
        False,
        "--rpc",
        help="Include RPC contracts and relationships.",
    ),
) -> None:
    """Render a Blueprint's relationships as a Graphviz SVG."""
    output_path = (output or Path(f"{blueprint}.svg")).expanduser().resolve()
    if output_path.suffix.lower() != ".svg":
        raise typer.BadParameter("output path must end in .svg", param_hint="--output")

    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        _render_in_subprocess(blueprint, output_path, include_rpc=rpc)
    except (GraphRenderError, OSError) as exc:
        typer.echo(typer.style(str(exc), fg=typer.colors.RED), err=True)
        raise typer.Exit(1) from exc

    typer.echo(f"Blueprint graph written to {output_path}")


def _worker_main() -> None:
    args = sys.argv[1:]
    include_rpc = args[-1:] == ["--rpc"]
    if include_rpc:
        args.pop()
    if len(args) != 2:
        raise SystemExit("usage: python -m dimos.cli.commands.graph BLUEPRINT OUTPUT.svg [--rpc]")

    try:
        _render_graph(args[0], Path(args[1]), include_rpc=include_rpc)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    _worker_main()

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

"""`dimos dataprep` commands; the implementation lives in dimos.imitation.dataprep.cli."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import typer

from dimos.imitation.dataprep.cli import build, inspect

dataprep_app = typer.Typer(help="Build and inspect learning datasets from recordings")


@dataprep_app.command("build")
def dataprep_build(
    profile: str = typer.Option(..., "--profile", help="DataPrep profile as module:attribute"),
    source: Path = typer.Option(..., "--source", "-s", help="Recording .db or .mcap to read"),
    output: Path | None = typer.Option(None, "--output", help="Dataset output directory"),
    quality_mode: Literal["strict", "fill"] | None = typer.Option(
        None, "--quality-mode", help="Override episode validation: strict | fill"
    ),
) -> None:
    """Build a dataset from a recording (lerobot/hdf5 + dimos_meta.json)."""
    build(profile, source, output, quality_mode)


@dataprep_app.command("inspect")
def dataprep_inspect(
    dataset: Path = typer.Argument(
        ..., help="Recording .db/.mcap, built .hdf5 file, or lerobot directory"
    ),
    profile: str | None = typer.Option(
        None, "--profile", help="Validate a recording with this module:attribute profile"
    ),
    quality_mode: Literal["strict", "fill"] | None = typer.Option(
        None, "--quality-mode", help="Override episode validation: strict | fill"
    ),
) -> None:
    """Summarize a recording or built dataset, including incomplete episodes."""
    inspect(dataset, profile, quality_mode)

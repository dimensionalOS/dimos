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
from typing import Literal, cast

import typer

from dimos.imitation.dataprep.cli import build, inspect

dataprep_app = typer.Typer(help="Build and inspect learning datasets from recordings")


@dataprep_app.command("build")
def dataprep_build(
    source: Path | None = typer.Option(None, "--source", "-s", help="Recording .db to read"),
    output: Path | None = typer.Option(None, "--output", help="Dataset output directory"),
    output_format: str = typer.Option(None, "--format", "-f", help="Output format: lerobot | hdf5"),
    config_path: Path | None = typer.Option(
        None, "--config", "-c", help="JSON DataPrepConfig (needed for obs/action stream maps)"
    ),
) -> None:
    """Build a dataset from a recording (lerobot/hdf5 + dimos_meta.json)."""
    build(config_path, source, output, cast("Literal['lerobot', 'hdf5'] | None", output_format))


@dataprep_app.command("inspect")
def dataprep_inspect(
    dataset: Path | None = typer.Argument(
        None, help="Recording .db, built .hdf5 file, or lerobot directory"
    ),
    output_format: str = typer.Option(
        None, "--format", "-f", help="lerobot | hdf5 (auto-detected from the path if omitted)"
    ),
) -> None:
    """Summarize a recording or built dataset, including incomplete episodes."""
    inspect(dataset, cast("Literal['lerobot', 'hdf5'] | None", output_format))

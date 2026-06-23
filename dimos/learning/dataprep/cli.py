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

"""Implementation of the `dimos dataprep` subcommand (build + inspect).

DataPrep is a one-shot batch transform, not a long-lived module, so it runs
as a plain command over the pure helpers in `dimos.learning.dataprep.core`
and exits with a 0/1 status — no coordinator, no blocking loop.

The obs/action stream maps are nested, so they come from a JSON
`DataPrepConfig` via `--config`; simple flags override `source`/`output`/
`format` on top. See `dimos/learning/dataprep/example_config.json`.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import typer

if TYPE_CHECKING:
    from dimos.learning.dataprep.core import DataPrepConfig


def _load_config(
    config_path: Path | None,
    source: Path | None,
    output: Path | None,
    output_format: Literal["lerobot", "hdf5"] | None,
) -> DataPrepConfig:
    """Build a DataPrepConfig from an optional JSON file + flag overrides."""
    from dimos.learning.dataprep.core import DataPrepConfig, OutputConfig

    if config_path is not None:
        cfg = DataPrepConfig.model_validate_json(Path(config_path).read_text())
    else:
        cfg = DataPrepConfig()

    updates: dict[str, object] = {}
    if source is not None:
        updates["source"] = str(source)
    if output is not None or output_format is not None:
        updates["output"] = OutputConfig(
            format=output_format or cfg.output.format,
            path=output or cfg.output.path,
            metadata=cfg.output.metadata,
        )
    return cfg.model_copy(update=updates) if updates else cfg


def build(
    config_path: Path | None,
    source: Path | None,
    output: Path | None,
    output_format: Literal["lerobot", "hdf5"] | None,
) -> None:
    from dimos.learning.dataprep.build import run_dataprep

    cfg = _load_config(config_path, source, output, output_format)
    if not cfg.source:
        typer.echo("error: no source given (use --source or set it in --config)", err=True)
        raise typer.Exit(2)
    if not cfg.observation and not cfg.action:
        typer.echo(
            "error: no observation/action streams configured; pass --config with the "
            "stream maps (see dimos/learning/dataprep/example_config.json)",
            err=True,
        )
        raise typer.Exit(2)

    try:
        path = run_dataprep(cfg)
    except Exception as e:
        # CLI boundary: any failure becomes a clean message + non-zero exit
        # instead of a traceback. run_dataprep raises specific errors internally.
        typer.echo(f"dataprep build failed: {e}", err=True)
        raise typer.Exit(1)
    typer.echo(f"✓ wrote dataset to {path}")


def inspect(dataset: Path | None, output_format: Literal["lerobot", "hdf5"] | None) -> None:
    from dimos.learning.dataprep.build import inspect_dataset

    if dataset is None:
        typer.echo("error: no dataset given (pass a .hdf5 file or a lerobot directory)", err=True)
        raise typer.Exit(2)

    try:
        info = inspect_dataset(dataset, output_format)
    except Exception as e:
        # CLI boundary: surface failures as a message + non-zero exit, not a traceback.
        typer.echo(f"dataprep inspect failed: {e}", err=True)
        raise typer.Exit(1)
    typer.echo(json.dumps(info, indent=2, default=str))

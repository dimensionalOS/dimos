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
as a plain command over the pure helpers in `dimos.imitation.dataprep.core`
and exits with a 0/1 status — no coordinator, no blocking loop.

The obs/action stream maps are nested, so they come from a JSON
`DataPrepConfig` via `--config`; simple flags override `source`/`output`/
`format` on top. See `dimos/imitation/dataprep/example_config.json`.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import typer

if TYPE_CHECKING:
    from dimos.imitation.dataprep.core import DataPrepConfig


def _load_config(
    config_path: Path | None,
    profile: Literal["openyam"] | None,
    source: Path | None,
    output: Path | None,
    output_format: Literal["lerobot", "hdf5"] | None,
) -> DataPrepConfig:
    """Build a DataPrepConfig from an optional JSON file + flag overrides."""
    from dimos.imitation.dataprep.core import DataPrepConfig, OutputConfig

    if config_path is not None and profile is not None:
        raise ValueError("--config and --profile are mutually exclusive")
    if profile == "openyam":
        from dimos.robot.manipulators.openyam.learning import OPENYAM_LEARNING_PROFILE

        cfg = OPENYAM_LEARNING_PROFILE.dataprep_config(
            source=source or "",
            output=output or DataPrepConfig().output.path,
        )
    elif config_path is not None:
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
    profile: Literal["openyam"] | None,
    source: Path | None,
    output: Path | None,
    output_format: Literal["lerobot", "hdf5"] | None,
    quality_mode: Literal["strict", "fill"] | None = None,
) -> None:
    try:
        cfg = _load_config(config_path, profile, source, output, output_format)
    except ValueError as error:
        typer.echo(f"error: {error}", err=True)
        raise typer.Exit(2) from error
    if quality_mode is not None:
        cfg = cfg.model_copy(
            update={"quality": cfg.quality.model_copy(update={"mode": quality_mode})}
        )
    if not cfg.source:
        typer.echo("error: no source given (use --source or set it in --config)", err=True)
        raise typer.Exit(2)
    if not cfg.observation and not cfg.action:
        typer.echo(
            "error: no observation/action streams configured; pass --config or --profile with "
            "stream maps (see dimos/imitation/dataprep/example_config.json)",
            err=True,
        )
        raise typer.Exit(2)

    try:
        if cfg.output.format == "lerobot":
            from dimos.imitation.dataprep.lerobot import run_lerobot_dataprep

            path = run_lerobot_dataprep(cfg)
        else:
            from dimos.imitation.dataprep.build import run_dataprep

            path = run_dataprep(cfg)
    except Exception as e:
        # CLI boundary: any failure becomes a clean message + non-zero exit
        # instead of a traceback. run_dataprep raises specific errors internally.
        typer.echo(f"dataprep build failed: {e}", err=True)
        raise typer.Exit(1)
    typer.echo(f"✓ wrote dataset to {path}")


def inspect(
    dataset: Path | None,
    output_format: Literal["lerobot", "hdf5"] | None,
    config_path: Path | None = None,
    profile: Literal["openyam"] | None = None,
    quality_mode: Literal["strict", "fill"] | None = None,
) -> None:
    from dimos.imitation.dataprep.build import inspect_dataset, inspect_recording

    if dataset is None:
        typer.echo(
            "error: no path given (pass a recording .db/.mcap, .hdf5 file, or lerobot directory)",
            err=True,
        )
        raise typer.Exit(2)

    try:
        if config_path is not None or profile is not None:
            cfg = _load_config(config_path, profile, dataset, None, None)
            if quality_mode is not None:
                cfg = cfg.model_copy(
                    update={"quality": cfg.quality.model_copy(update={"mode": quality_mode})}
                )
            info = inspect_recording(dataset, config=cfg)
        else:
            info = inspect_dataset(dataset, output_format)
    except Exception as e:
        # CLI boundary: surface failures as a message + non-zero exit, not a traceback.
        typer.echo(f"dataprep inspect failed: {e}", err=True)
        raise typer.Exit(1)
    typer.echo(json.dumps(info, indent=2, default=str))

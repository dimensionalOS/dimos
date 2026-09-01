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

DataPrep is a one-shot batch transform, not a long-lived module. A profile
supplies the schema template while the CLI supplies run-specific paths.
"""

from __future__ import annotations

import importlib
import json
from pathlib import Path
from typing import Literal

import typer

from dimos.imitation.dataprep.core import DataPrepConfig


def _load_profile(reference: str) -> DataPrepConfig:
    """Load and validate a ``module:attribute`` profile reference."""
    from dimos.imitation.dataprep.core import DataPrepProfile

    module_name, separator, attribute = reference.partition(":")
    if not separator or not module_name or not attribute:
        raise ValueError("--profile must use module:attribute syntax")
    module = importlib.import_module(module_name)
    try:
        profile = getattr(module, attribute)
    except AttributeError as error:
        raise ValueError(f"profile attribute {attribute!r} not found in {module_name!r}") from error
    if not isinstance(profile, DataPrepProfile):
        raise TypeError(f"{reference!r} does not implement DataPrepProfile")
    config = profile.dataprep_config()
    if not isinstance(config, DataPrepConfig):
        raise TypeError(f"{reference!r}.dataprep_config() did not return DataPrepConfig")
    return config


def _load_config(
    profile: str,
    source: Path,
    output: Path | None,
    quality_mode: Literal["strict", "fill"] | None,
) -> DataPrepConfig:
    """Apply one invocation's paths and quality mode to a profile template."""
    cfg = _load_profile(profile)
    updates: dict[str, object] = {"source": str(source)}
    if output is not None:
        updates["output"] = cfg.output.model_copy(update={"path": output})
    if quality_mode is not None:
        updates["quality"] = cfg.quality.model_copy(update={"mode": quality_mode})
    return cfg.model_copy(update=updates)


def build(
    profile: str,
    source: Path,
    output: Path | None,
    quality_mode: Literal["strict", "fill"] | None = None,
) -> None:
    try:
        cfg = _load_config(profile, source, output, quality_mode)
    except (ImportError, TypeError, ValueError) as error:
        typer.echo(f"error: {error}", err=True)
        raise typer.Exit(2) from error
    if not cfg.observation and not cfg.action:
        typer.echo("error: profile has no observation/action streams", err=True)
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
    dataset: Path,
    profile: str | None = None,
    quality_mode: Literal["strict", "fill"] | None = None,
) -> None:
    from dimos.imitation.dataprep.build import inspect_dataset, inspect_recording

    try:
        if profile is not None:
            cfg = _load_config(profile, dataset, None, quality_mode)
            info = inspect_recording(dataset, config=cfg)
        else:
            info = inspect_dataset(dataset)
    except Exception as e:
        # CLI boundary: surface failures as a message + non-zero exit, not a traceback.
        typer.echo(f"dataprep inspect failed: {e}", err=True)
        raise typer.Exit(1)
    typer.echo(json.dumps(info, indent=2, default=str))

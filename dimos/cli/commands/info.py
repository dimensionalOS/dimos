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

"""Introspection commands: current config values and available blueprints."""

from __future__ import annotations

import typer

from dimos.core.global_config import global_config


def show_config() -> None:
    """Show current config settings and their values."""
    for field_name, value in global_config.model_dump().items():
        typer.echo(f"{field_name}: {value}")


def list_blueprints() -> None:
    """List all available blueprints."""
    from dimos.robot.all_blueprints import all_blueprints
    from dimos.robot.external_blueprints import (
        ExternalBlueprintError,
        list_external_blueprint_names,
    )

    blueprints = [name for name in all_blueprints.keys() if not name.startswith("demo-")]
    typer.echo("Built-in blueprints:")
    for blueprint_name in sorted(blueprints):
        typer.echo(f"  {blueprint_name}")

    try:
        external_blueprints = list_external_blueprint_names()
    except ExternalBlueprintError as exc:
        typer.echo(typer.style(str(exc), fg=typer.colors.RED), err=True)
        raise typer.Exit(1) from exc

    if external_blueprints:
        typer.echo("")
        typer.echo("External blueprints:")
        for blueprint_name in external_blueprints:
            typer.echo(f"  {blueprint_name}")

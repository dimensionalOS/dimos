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

"""Root-level `dimos` options generated dynamically from GlobalConfig fields."""

from __future__ import annotations

from functools import wraps
import inspect
import types
from typing import Any, get_args, get_origin

import click
from pydantic import ValidationError
import typer
from typer.core import TyperGroup

from dimos.core.global_config import GlobalConfig, get_global_config


class GlobalOptionsGroup(TyperGroup):
    """Apply settings when a command executes, after Click has handled help."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        for name, command in self.commands.items():
            # These two commands merge root flags with blueprint-specific options.
            if name not in {"run", "rerun-bridge"}:
                _configure_callback(command)


def _configure_callback(command: click.Command) -> None:
    if isinstance(command, click.Group):
        for child in command.commands.values():
            _configure_callback(child)
    callback = command.callback
    if callback is None:
        return

    @wraps(callback)
    def configured(*args: Any, **kwargs: Any) -> Any:
        ctx = click.get_current_context()
        # Click consumes the help flag it handles. One left among the forwarded
        # arguments belongs to the tool that receives them (e.g. lerobot-train).
        delegated_help = any(arg in {"--help", "-h"} for arg in ctx.args)
        if ctx.invoked_subcommand is None and not delegated_help:
            try:
                get_global_config(**ctx.find_root().obj)
            except ValidationError as error:
                detail = error.errors()[0]
                field = ".".join(map(str, detail["loc"]))
                raise typer.BadParameter(detail["msg"], param_hint=field) from error
        return callback(*args, **kwargs)

    command.callback = configured


def create_dynamic_callback():  # type: ignore[no-untyped-def]
    fields = GlobalConfig.model_fields

    # Build the function signature dynamically
    params = [
        inspect.Parameter("ctx", inspect.Parameter.POSITIONAL_OR_KEYWORD, annotation=typer.Context),
    ]

    # Create parameters for each field in GlobalConfig
    for field_name, field_info in fields.items():
        field_type = field_info.annotation

        # Container generics (e.g. `tuple[...]` fields) have no single-flag CLI
        # representation; they're configured via environment variables or JSON.
        if isinstance(field_type, types.GenericAlias):
            continue

        # Handle Optional types
        # Check for Optional/Union with None
        if get_origin(field_type) is type(str | None):
            inner_types = get_args(field_type)
            if len(inner_types) == 2 and type(None) in inner_types:
                # It's Optional[T], get the actual type T
                actual_type = next(t for t in inner_types if t != type(None))
            else:
                actual_type = field_type
        else:
            actual_type = field_type

        # Convert field name from snake_case to kebab-case for CLI
        cli_option_name = field_name.replace("_", "-")
        help_text = field_info.description or f"Override {field_name} in GlobalConfig"

        # Special handling for boolean fields
        if actual_type is bool:
            # For boolean fields, create --flag/--no-flag pattern
            param = inspect.Parameter(
                field_name,
                inspect.Parameter.KEYWORD_ONLY,
                default=typer.Option(
                    None,  # None means use the model's default if not provided
                    f"--{cli_option_name}/--no-{cli_option_name}",
                    help=help_text,
                ),
                annotation=bool | None,
            )
        else:
            # For non-boolean fields, use regular option
            param = inspect.Parameter(
                field_name,
                inspect.Parameter.KEYWORD_ONLY,
                default=typer.Option(
                    None,  # None means use the model's default if not provided
                    f"--{cli_option_name}",
                    help=help_text,
                ),
                annotation=actual_type | None,
            )
        params.append(param)

    def callback(**kwargs) -> None:  # type: ignore[no-untyped-def]
        ctx = kwargs.pop("ctx")
        overrides = {k: v for k, v in kwargs.items() if v is not None}
        ctx.obj = overrides

    callback.__signature__ = inspect.Signature(params)  # type: ignore[attr-defined]

    return callback

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

import inspect
import types
from typing import get_args, get_origin

import typer

from dimos.core.global_config import GlobalConfig, global_config


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

        # Special handling for boolean fields
        if actual_type is bool:
            # For boolean fields, create --flag/--no-flag pattern
            param = inspect.Parameter(
                field_name,
                inspect.Parameter.KEYWORD_ONLY,
                default=typer.Option(
                    None,  # None means use the model's default if not provided
                    f"--{cli_option_name}/--no-{cli_option_name}",
                    help=f"Override {field_name} in GlobalConfig",
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
                    help=f"Override {field_name} in GlobalConfig",
                ),
                annotation=actual_type | None,
            )
        params.append(param)

    def callback(**kwargs) -> None:  # type: ignore[no-untyped-def]
        ctx = kwargs.pop("ctx")
        overrides = {k: v for k, v in kwargs.items() if v is not None}
        ctx.obj = overrides
        # Apply overrides (e.g. --transport, --viewer) to the process-global config
        # up front so every subcommand honors flags given before the subcommand name.
        global_config.update(**overrides)

    callback.__signature__ = inspect.Signature(params)  # type: ignore[attr-defined]

    return callback

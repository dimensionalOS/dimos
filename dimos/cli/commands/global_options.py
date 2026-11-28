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

from collections.abc import Callable, Sequence
import inspect
import sys
import types
from typing import Any, get_args, get_origin

from pydantic import ValidationError
import typer
from typer.core import TyperGroup

from dimos.core.global_config import DEFAULT_SIMULATOR, GlobalConfig, get_global_config

BLUEPRINT_COMMANDS = {"run", "rerun-bridge"}
OPTIONAL_VALUE_FLAGS = {"--simulation": DEFAULT_SIMULATOR, "--record": "sqlite"}


class GlobalOptionsGroup(TyperGroup):
    """Apply settings when a command executes, after Click has handled help."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # These two commands merge root flags with blueprint-specific options.
        commands = [cmd for name, cmd in self.commands.items() if name not in BLUEPRINT_COMMANDS]
        while commands:
            command = commands.pop()
            command.context_class = _GlobalOptionsContext
            if isinstance(command, TyperGroup):
                commands.extend(command.commands.values())

    def normalize_argv(self, argv: list[str]) -> list[str]:
        """Expand bare settings only in the root or a blueprint's option list."""
        if not any(arg in OPTIONAL_VALUE_FLAGS for arg in argv[1:]):
            return argv.copy()
        value_options = {
            opt
            for param in self.params
            if not getattr(param, "is_flag", False)
            for opt in param.opts
        }
        out = argv[:1]
        at_root = True
        index = 1
        while index < len(argv):
            arg = argv[index]
            if arg == "--":
                # A root terminator ends global options, then Click enters the
                # next command. A terminator within that command protects its tail.
                if (
                    not at_root
                    or index + 1 == len(argv)
                    or argv[index + 1] not in BLUEPRINT_COMMANDS
                ):
                    break
                out.append(arg)
                index += 1
                arg = argv[index]
            if at_root and not arg.startswith("-"):
                if arg not in BLUEPRINT_COMMANDS:
                    break
                at_root = False
            out.append(arg)
            index += 1
            following = argv[index] if index < len(argv) else None
            if arg in OPTIONAL_VALUE_FLAGS and (
                following is None
                or following.startswith("-")
                or (at_root and following in self.commands)
            ):
                out.append(OPTIONAL_VALUE_FLAGS[arg])
            elif at_root and arg in value_options and following is not None:
                # A root option's value may itself spell a command name.
                out.append(following)
                index += 1
        return [*out, *argv[index:]]

    def main(self, args: Sequence[str] | None = None, *rest: Any, **kwargs: Any) -> Any:
        if args is None:
            sys.argv = self.normalize_argv(sys.argv)
        else:
            args = self.normalize_argv(["dimos", *args])[1:]
        return super().main(args, *rest, **kwargs)


class _GlobalOptionsContext(typer.Context):
    def invoke(self, callback: Callable[..., Any], /, *args: Any, **kwargs: Any) -> Any:
        # Click consumes the help flag it handles. One left among the forwarded
        # arguments belongs to the tool that receives them (e.g. lerobot-train).
        delegated_help = any(arg in {"--help", "-h"} for arg in self.args)
        if (
            callback is self.command.callback
            and self.invoked_subcommand is None
            and not delegated_help
        ):
            try:
                get_global_config(**self.find_root().obj)
            except ValidationError as error:
                detail = error.errors()[0]
                field = ".".join(map(str, detail["loc"]))
                raise typer.BadParameter(detail["msg"], ctx=self, param_hint=field) from error
        return super().invoke(callback, *args, **kwargs)


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

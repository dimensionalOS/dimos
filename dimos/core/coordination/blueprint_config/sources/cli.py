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

"""The command-line source: `--root.field value`, `--field=value`, `--no-flag`."""

from __future__ import annotations

from collections.abc import Callable, Sequence
import difflib
from typing import Any, NoReturn

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.schema import (
    OptionTarget,
    ParserSchema,
    cli_path,
    coerce_cli_value,
    display_normalized_option,
    normalize_option_name,
)
from dimos.core.coordination.blueprint_config.values import deep_set

Resolver = Callable[[str], OptionTarget | None]


def split_run_arguments(tokens: Sequence[str]) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Split Typer's variadic run arguments into blueprint names and config tokens.

    Blueprint names must form the leading positional segment.  Once any
    dash-prefixed token is seen, all remaining tokens belong to option parsing.
    """
    split_at = next((i for i, token in enumerate(tokens) if token.startswith("-")), len(tokens))
    blueprint_names = tuple(tokens[:split_at])
    if not blueprint_names:
        raise BlueprintConfigError(
            "At least one blueprint name must precede configuration options. "
            "Usage: dimos run <blueprint> [--config-field value]."
        )
    return blueprint_names, tuple(tokens[split_at:])


def read_cli(
    tokens: tuple[str, ...],
    resolve: Resolver,
    *,
    on_unknown: Callable[[str], NoReturn] | None = None,
) -> dict[str, Any]:
    """Walk the tokens into a nested source mapping keyed by section root.

    With `on_unknown`, stray tokens and unknown options are errors (the blueprint
    parse). Without it they are skipped: the GlobalConfig pre-pass runs before the
    blueprint, and its options, are known.
    """
    strict = on_unknown is not None
    values: dict[str, Any] = {}
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if strict and (token in ("-o", "--option") or token.startswith(("-o=", "--option="))):
            raise BlueprintConfigError(
                "The legacy -o/--option syntax was removed. "
                "Use a blueprint option directly, for example "
                "`--map-file recording_go2`."
            )
        option, separator, attached_value = token[2:].partition("=")
        if not token.startswith("--") or not option:
            if strict:
                raise BlueprintConfigError(
                    "Empty configuration option '--'."
                    if token.startswith("--")
                    else f"Unexpected configuration argument {token!r}; options must start with '--'."
                )
            index += 1
            continue

        negated = False
        normalized = normalize_option_name(option)
        target = resolve(normalized)
        if target is None and normalized.startswith("no_"):
            candidate = resolve(normalized.removeprefix("no_"))
            if candidate is not None and candidate.section == "global" and candidate.is_bool:
                target, negated = candidate, True
        if target is None:
            if on_unknown is not None:
                on_unknown(option)
            index += 1
            continue

        if separator:
            raw_value: Any = attached_value
        elif negated:
            raw_value = False
        elif target.section == "global" and target.is_bool:
            if index + 1 < len(tokens) and not tokens[index + 1].startswith("--"):
                index += 1
                raw_value = tokens[index]
            else:
                raw_value = True
        else:
            if index + 1 >= len(tokens) or tokens[index + 1].startswith("--"):
                raise BlueprintConfigError(
                    f"Option --{option} requires a value. "
                    f"Use --{option}=VALUE when the value starts with '--'."
                )
            index += 1
            raw_value = tokens[index]

        if negated and separator:
            raise BlueprintConfigError(f"Negated option --{option} does not accept a value.")
        deep_set(values, target.source_path, coerce_cli_value(raw_value, target, option))
        index += 1
    return values


def resolve_target(normalized: str, schema: ParserSchema) -> OptionTarget | None:
    candidates = schema.aliases.get(normalized)
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]
    choices = ", ".join(f"--{candidate.qualified_name}" for candidate in candidates)
    paths = {c.path for c in candidates}
    if len(paths) == 1 and all(c.section == "module" for c in candidates):
        choices += f", or --{cli_path(('shared', *paths.pop()))} to set every one"
    raise BlueprintConfigError(
        f"Option --{display_normalized_option(normalized)} is ambiguous. Use one of: {choices}."
    )


def raise_unknown_option(option: str, schema: ParserSchema) -> NoReturn:
    normalized = normalize_option_name(option)
    suggestions = difflib.get_close_matches(normalized, schema.aliases.keys(), n=3)
    hint = ""
    if suggestions:
        rendered = ", ".join(f"--{display_normalized_option(name)}" for name in suggestions)
        hint = f" Did you mean {rendered}?"
    raise BlueprintConfigError(f"Unknown blueprint configuration option --{option}.{hint}")

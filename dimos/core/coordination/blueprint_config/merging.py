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

"""Merging raw source values into per-section configuration dicts."""

from __future__ import annotations

from collections.abc import Mapping
import difflib
from typing import Any

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.schema import (
    OptionTarget,
    ParserSchema,
    coerce_cli_value,
    coerce_environment_value,
    display_normalized_option,
    normalize_option_name,
)
from dimos.core.coordination.blueprint_config.sources import global_environment_names
from dimos.core.coordination.blueprint_config.values import (
    deep_merge,
    deep_set,
    normalize_mapping_keys,
)
from dimos.core.coordination.blueprints import config_key


def merge_root_source(
    module_values: dict[str, dict[str, Any]],
    global_values: dict[str, Any],
    transport_values: dict[str, Any],
    values: Mapping[str, Any],
    *,
    source: str,
    schema: ParserSchema,
) -> None:
    module_roots: dict[str, str] = {}
    for module in schema.modules:
        module_roots[module.atom.name.lower()] = module.atom.name
        module_roots[config_key(module.atom.name).lower()] = module.atom.name

    for raw_root, raw_value in values.items():
        if not isinstance(raw_root, str):
            raise BlueprintConfigError(f"{source} contains a non-string root key.")
        root = raw_root.lower().replace("-", "_")
        if raw_value is None:
            continue
        if not isinstance(raw_value, Mapping):
            raise BlueprintConfigError(
                f"{source} section {raw_root!r} must be an object, not {type(raw_value).__name__}."
            )
        normalized = normalize_mapping_keys(raw_value)
        if root == "g":
            deep_merge(global_values, normalized)
        elif root == "transports":
            deep_merge(transport_values, normalized)
        elif root in module_roots:
            deep_merge(module_values[module_roots[root]], normalized)
        else:
            choices = [*module_roots, "g", "transports"]
            suggestion = difflib.get_close_matches(root, choices, n=1)
            hint = f" Did you mean {suggestion[0]!r}?" if suggestion else ""
            raise BlueprintConfigError(
                f"Unknown configuration section {raw_root!r} in {source}.{hint}"
            )


def merge_environment(
    module_values: dict[str, dict[str, Any]],
    global_values: dict[str, Any],
    transport_values: dict[str, Any],
    environ: Mapping[str, str],
    schema: ParserSchema,
) -> None:
    roots: dict[str, str] = {"g": "g", "transports": "transports"}
    for module in schema.modules:
        roots[config_key(module.atom.name).lower()] = module.atom.name
        roots[module.atom.name.lower()] = module.atom.name

    known_transports = {transport.name for transport in schema.transports}
    global_env_names = global_environment_names()
    targets = {target.identity: target for target in schema.targets}
    env_source: dict[str, Any] = {}

    def set_coerced(path: tuple[str, ...], raw_name: str, value: str) -> None:
        identity = _environment_identity(path)
        target = targets.get(identity) if identity is not None else None
        coerced = value if target is None else coerce_environment_value(value, target, raw_name)
        deep_set(env_source, path, coerced)

    for raw_name, value in environ.items():
        lowered = raw_name.lower()
        global_field = global_env_names.get(lowered)
        if global_field is not None:
            set_coerced(("g", global_field), raw_name, value)
            continue

        parts = tuple(part.lower().replace("-", "_") for part in raw_name.split("__"))
        if len(parts) < 2 or parts[0] not in roots:
            continue
        if parts[0] == "transports" and (len(parts) < 3 or parts[1] not in known_transports):
            continue
        set_coerced((roots[parts[0]], *parts[1:]), raw_name, value)

    if env_source:
        merge_root_source(
            module_values,
            global_values,
            transport_values,
            env_source,
            source="environment",
            schema=schema,
        )


def merge_cli(
    module_values: dict[str, dict[str, Any]],
    global_values: dict[str, Any],
    transport_values: dict[str, Any],
    tokens: tuple[str, ...],
    schema: ParserSchema,
) -> None:
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if (
            token == "-o"
            or token.startswith("-o=")
            or token == "--option"
            or token.startswith("--option=")
        ):
            raise BlueprintConfigError(
                "The legacy -o/--option syntax was removed. "
                "Use a blueprint option directly, for example "
                "`--map-file recording_go2`."
            )
        if not token.startswith("--"):
            raise BlueprintConfigError(
                f"Unexpected configuration argument {token!r}; options must start with '--'."
            )

        option, separator, attached_value = token[2:].partition("=")
        if not option:
            raise BlueprintConfigError("Empty configuration option '--'.")

        negated_global = False
        normalized = normalize_option_name(option)
        target = _resolve_target(normalized, schema)
        if target is None and normalized.startswith("no_"):
            positive = normalized.removeprefix("no_")
            candidate = _resolve_target(positive, schema)
            if candidate is not None and candidate.section == "global" and candidate.is_bool:
                target = candidate
                negated_global = True
        if target is None:
            _raise_unknown_option(option, schema)

        assert target is not None
        if separator:
            raw_value: Any = attached_value
        elif negated_global:
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

        if negated_global and separator:
            raise BlueprintConfigError(f"Negated option --{option} does not accept a value.")
        value = coerce_cli_value(raw_value, target, option)
        if target.section == "module":
            deep_set(module_values[target.root], target.path, value)
        elif target.section == "global":
            deep_set(global_values, target.path, value)
        else:
            section = transport_values.setdefault(target.root, {})
            if not isinstance(section, dict):
                raise BlueprintConfigError(f"Transport section {target.root!r} is not an object.")
            deep_set(section, target.path, value)
        index += 1


def _environment_identity(path: tuple[str, ...]) -> tuple[str, str, tuple[str, ...]] | None:
    """Map an environment source path to an OptionTarget identity."""
    root, *rest = path
    if root == "g":
        return ("global", "g", tuple(rest))
    if root == "transports":
        if len(rest) < 2:
            return None
        return ("transport", rest[0], tuple(rest[1:]))
    return ("module", root, tuple(rest))


def _resolve_target(normalized: str, schema: ParserSchema) -> OptionTarget | None:
    candidates = schema.aliases.get(normalized)
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]
    choices = ", ".join(f"--{candidate.qualified_name}" for candidate in candidates)
    raise BlueprintConfigError(
        f"Option --{display_normalized_option(normalized)} is ambiguous. Use one of: {choices}."
    )


def _raise_unknown_option(option: str, schema: ParserSchema) -> None:
    normalized = normalize_option_name(option)
    suggestions = difflib.get_close_matches(normalized, schema.aliases.keys(), n=3)
    hint = ""
    if suggestions:
        rendered = ", ".join(f"--{display_normalized_option(name)}" for name in suggestions)
        hint = f" Did you mean {rendered}?"
    raise BlueprintConfigError(f"Unknown blueprint configuration option --{option}.{hint}")

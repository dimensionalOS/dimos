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

"""Merging one source's nested mapping into the per-section configuration dicts."""

from __future__ import annotations

from collections.abc import Iterator, Mapping
import difflib
from typing import Any

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.schema import (
    ParserSchema,
    cli_path,
    section_roots,
    shared_recipients,
)
from dimos.core.coordination.blueprint_config.values import (
    deep_merge,
    deep_set,
    normalize_mapping_keys,
)


def _leaves(
    values: Mapping[str, Any], prefix: tuple[str, ...] = ()
) -> Iterator[tuple[tuple[str, ...], Any]]:
    """Every non-mapping value in a nested mapping, with its path."""
    for key, value in values.items():
        path = (*prefix, key)
        if isinstance(value, Mapping) and value:
            yield from _leaves(value, path)
        else:
            yield path, value


def merge_root_source(
    module_values: dict[str, dict[str, Any]],
    global_values: dict[str, Any],
    transport_values: dict[str, Any],
    values: Mapping[str, Any],
    *,
    source: str,
    schema: ParserSchema,
) -> None:
    roots = section_roots(schema)

    # shared merges first so a module's own section wins within one source
    for raw_root, raw_value in sorted(
        values.items(), key=lambda kv: str(kv[0]).lower() != "shared"
    ):
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
        elif root == "shared":
            for path, value in _leaves(normalized):
                names = shared_recipients(schema, path)
                if not names:
                    raise BlueprintConfigError(
                        f"Unknown shared option {cli_path(path)!r} in {source}: "
                        "no module has that field."
                    )
                for name in names:
                    deep_set(module_values[name], path, value)
        elif root in roots:
            deep_merge(module_values[roots[root]], normalized)
        else:
            choices = list(roots)
            suggestion = difflib.get_close_matches(root, choices, n=1)
            hint = f" Did you mean {suggestion[0]!r}?" if suggestion else ""
            raise BlueprintConfigError(
                f"Unknown configuration section {raw_root!r} in {source}.{hint}"
            )

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

from collections.abc import Mapping
import difflib
from typing import Any

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.schema import (
    ParserSchema,
    modules_with_field,
    section_roots,
)
from dimos.core.coordination.blueprint_config.values import deep_merge, normalize_mapping_keys


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
            for key, value in normalized.items():
                names = modules_with_field(schema, key)
                if not names:
                    raise BlueprintConfigError(
                        f"Unknown shared option {key!r} in {source}: no module has that field."
                    )
                for name in names:
                    deep_merge(module_values[name], {key: value})
        elif root in roots:
            deep_merge(module_values[roots[root]], normalized)
        else:
            choices = list(roots)
            suggestion = difflib.get_close_matches(root, choices, n=1)
            hint = f" Did you mean {suggestion[0]!r}?" if suggestion else ""
            raise BlueprintConfigError(
                f"Unknown configuration section {raw_root!r} in {source}.{hint}"
            )

# Copyright 2025-2026 Dimensional Inc.
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

"""Global setting aliases needed before CLI imports resolve state paths."""

from collections.abc import Mapping
from typing import TypeVar

_T = TypeVar("_T")

# Pydantic checks aliases within one settings source in their declared order.
GLOBAL_ALIASES = {"transport": ("DIMOS_TRANSPORT", "transport")}


def resolve_global_values(
    values: Mapping[str, _T],
    aliases_by_field: Mapping[str, tuple[str, ...]],
    *,
    allow_qualified: bool = False,
) -> dict[str, _T]:
    """Within one source, check bare names first and preserve declared alias priority."""
    lower = {name.lower(): value for name, value in values.items()}
    prefixes = ("", "g__") if allow_qualified else ("",)
    resolved = {}
    for field, aliases in aliases_by_field.items():
        candidates = (prefix + alias.lower() for prefix in prefixes for alias in aliases)
        for name in candidates:
            if name in lower:
                resolved[field] = lower[name]
                break
    return resolved

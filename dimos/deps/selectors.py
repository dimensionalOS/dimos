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

"""Configuration inputs that select registry implementations, read without a schema.

The launcher plans before it imports a blueprint, so it cannot run the full
configuration parser (that needs the blueprint's pydantic schema). What it
can do is recognize the few fields whose values pick an implementation from
a registry manifest: ``hardware`` and ``tasks`` on the control coordinator.
This module reads those fields syntactically from the same three sources the
parser uses, the CLI tokens, the config file sections and the environment,
and leaves everything else alone. An input it cannot attribute or parse makes
the plan incomplete instead of being guessed.
"""

from __future__ import annotations

from collections.abc import Collection, Mapping
from dataclasses import dataclass
import json
from typing import Any

FAMILIES: Mapping[str, tuple[str, str | None] | None] = {
    "adapter": ("adapter_type", "mock"),
    "task": ("type", None),
    "connection": None,
    "simulation": None,
}
"""Registry family -> (key, default) inside each item of a list-valued field, or
``None`` for a family keyed on a scalar global configuration value."""
ROOT_SECTIONS = ("g", "transports")


@dataclass(frozen=True)
class SelectorInput:
    source: str
    """Where the value came from, for messages: ``--controlcoordinator.hardware``."""
    instance: str | None
    """Module instance the field belongs to; ``None`` for a relative ``--hardware``."""
    field: str
    value: object
    """The raw string from the CLI or environment, or the parsed config file value."""


def normalize(name: str) -> str:
    return name.replace("-", "_").lower()


def collect_selector_inputs(
    tokens: Collection[str],
    config_sections: Mapping[str, Any],
    environ: Mapping[str, str],
    fields: Collection[str],
) -> tuple[SelectorInput, ...]:
    """Inputs naming one of ``fields``, from the CLI, the config file and the environment."""
    found: list[SelectorInput] = []
    for root, section in config_sections.items():
        instance = normalize(str(root))
        if instance in ROOT_SECTIONS or not isinstance(section, Mapping):
            continue
        for key, value in section.items():
            field = normalize(str(key))
            if field in fields:
                found.append(SelectorInput(f"config file section {root!r}", instance, field, value))
    for raw, value in environ.items():
        parts = normalize(raw).split("__")
        if len(parts) == 2 and parts[0] not in ROOT_SECTIONS and parts[1] in fields:
            found.append(SelectorInput(raw, parts[0], parts[1], value))
    found.extend(_cli_inputs(list(tokens), fields))
    return tuple(found)


def _cli_inputs(tokens: list[str], fields: Collection[str]) -> list[SelectorInput]:
    found: list[SelectorInput] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        index += 1
        if not token.startswith("--"):
            continue
        option, separator, attached = token[2:].partition("=")
        parts = normalize(option).split(".")
        if parts[0] == "modules":
            parts = parts[1:]
        if len(parts) not in (1, 2) or parts[-1] not in fields:
            continue
        value: str | None = attached if separator else None
        if value is None and index < len(tokens) and not tokens[index].startswith("--"):
            value = tokens[index]
            index += 1
        instance = parts[0] if len(parts) == 2 else None
        found.append(SelectorInput(f"--{option}", instance, parts[-1], value))
    return found


def identifiers(family: str, value: object) -> list[str]:
    """Registry names a field value selects; raises ``ValueError`` when it cannot tell."""
    spec = FAMILIES[family]
    if spec is None:
        if not isinstance(value, str) or not value:
            raise ValueError(f"expected a {family} name, got {value!r}")
        return [value.lower()]
    key, default = spec
    if isinstance(value, str):
        if not value.lstrip().startswith("["):
            raise ValueError("expected a JSON list")
        try:
            value = json.loads(value)
        except json.JSONDecodeError as error:
            raise ValueError(f"is not valid JSON ({error.msg})") from None
    if not isinstance(value, list) or not all(isinstance(item, Mapping) for item in value):
        raise ValueError("expected a list of objects")
    names: list[str] = []
    for item in value:
        selected = item.get(key, default)
        if not isinstance(selected, str) or not selected:
            raise ValueError(f"an item has no {key!r}")
        names.append(selected.lower())
    return names

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

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from dimos.core.coordination.blueprints import Blueprint, autoconnect


class ChoiceResolutionError(ValueError):
    """A blueprint choice is missing, invalid, or ambiguous."""


@dataclass(frozen=True)
class ChoiceAtom:
    """Select one blueprint fragment when a MetaBlueprint is solidified."""

    name: str
    choices: Mapping[str, Blueprint | MetaBlueprint]
    default: str | None = None

    def __post_init__(self) -> None:
        if not self.name or "=" in self.name:
            raise ChoiceResolutionError("Choice names must be nonempty and cannot contain '='")
        if not self.choices or any(not option for option in self.choices):
            raise ChoiceResolutionError(f"Choice {self.name!r} must have nonempty option names")
        if self.default is not None and self.default not in self.choices:
            raise ChoiceResolutionError(
                f"Invalid default {self.default!r} for choice {self.name!r}. "
                f"Available options: {', '.join(self.choices)}"
            )
        object.__setattr__(self, "choices", MappingProxyType(dict(self.choices)))


@dataclass(frozen=True)
class MetaBlueprint:
    """A composition whose only operation is resolving choices into a Blueprint."""

    blueprints: tuple[Blueprint | MetaBlueprint | ChoiceAtom, ...]

    def solidify(self, *, choices: Mapping[str, str] | None = None) -> Blueprint:
        """Resolve selected alternatives and compose a concrete Blueprint.

        Only selected alternatives are traversed. Distinct active ChoiceAtoms
        must have unique names; a shared ChoiceAtom may be reused. Overrides
        must name active choices, so misspellings and unused selections fail.

        Raises:
            ChoiceResolutionError: A required choice is missing, an option is
                invalid, a name is ambiguous, or an override is unused.
        """
        selections = dict(choices or {})
        active_choices: dict[str, ChoiceAtom] = {}

        def resolve(part: Blueprint | MetaBlueprint | ChoiceAtom) -> Blueprint:
            if isinstance(part, Blueprint):
                return part
            if isinstance(part, MetaBlueprint):
                return autoconnect(*(resolve(child) for child in part.blueprints))

            previous = active_choices.get(part.name)
            if previous is not None and previous is not part:
                raise ChoiceResolutionError(f"Duplicate choice name: {part.name!r}")
            active_choices[part.name] = part
            option = selections.get(part.name, part.default)
            available = f"Available options: {', '.join(part.choices)}"
            if option is None:
                raise ChoiceResolutionError(
                    f"Choice {part.name!r} is required. {available}. "
                    f"Select it with --choice {part.name}=<option>"
                )
            if option not in part.choices:
                raise ChoiceResolutionError(
                    f"Invalid option {option!r} for choice {part.name!r}. {available}"
                )
            return resolve(part.choices[option])

        blueprint = resolve(self)
        unused = selections.keys() - active_choices.keys()
        if unused:
            raise ChoiceResolutionError(f"Unknown or inactive choices: {', '.join(sorted(unused))}")
        return blueprint


def autoconnect_with_choices(*blueprints: Blueprint | MetaBlueprint | ChoiceAtom) -> MetaBlueprint:
    """Compose fragments, deferring choice resolution until solidify()."""
    return MetaBlueprint(blueprints=blueprints)

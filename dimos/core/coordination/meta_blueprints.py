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

from collections.abc import Iterator, Mapping
from dataclasses import dataclass
import difflib
from types import MappingProxyType

from dimos.core.coordination.blueprints import Blueprint, autoconnect

# The (choice, option) selections that make a nested choice active.
_Conditions = tuple[tuple[str, str], ...]


class ChoiceResolutionError(ValueError):
    """A choice selection is invalid, unknown, inactive, or ambiguous."""


def _check_type(value: object, expected: tuple[type, ...], what: str) -> None:
    if isinstance(value, expected):
        return
    names = " or ".join(cls.__name__ for cls in expected)
    # A class here is most often a Module that is missing its .blueprint() call.
    got = f"the class {value.__name__}" if isinstance(value, type) else type(value).__name__
    raise TypeError(f"{what} must be a {names}, got {got}")


@dataclass(frozen=True)
class ChoiceAtom:
    """Select one blueprint fragment when a MetaBlueprint is solidified."""

    name: str
    choices: Mapping[str, Blueprint | MetaBlueprint]
    default: str

    def __post_init__(self) -> None:
        if not self.name or "=" in self.name:
            raise ValueError("Choice names must be nonempty and cannot contain '='")
        if not self.choices:
            raise ValueError(f"Choice {self.name!r} must have at least one option")
        for option, alternative in self.choices.items():
            if not isinstance(option, str) or not option:
                raise ValueError(
                    f"Option names of choice {self.name!r} must be nonempty strings, got {option!r}"
                )
            _check_type(
                alternative,
                (Blueprint, MetaBlueprint),
                f"Option {option!r} of choice {self.name!r}",
            )
        if self.default not in self.choices:
            raise ValueError(
                f"Invalid default {self.default!r} for choice {self.name!r}. "
                f"Available options: {', '.join(self.choices)}"
            )
        object.__setattr__(self, "choices", MappingProxyType(dict(self.choices)))


@dataclass(frozen=True)
class MetaBlueprint:
    """A composition whose choices are resolved into a Blueprint by solidify()."""

    parts: tuple[Blueprint | MetaBlueprint | ChoiceAtom, ...]

    def __post_init__(self) -> None:
        for part in self.parts:
            _check_type(
                part, (Blueprint, MetaBlueprint, ChoiceAtom), "Each autoconnect_with_choices() part"
            )

    def solidify(self, *, choices: Mapping[str, str] | None = None) -> Blueprint:
        """Resolve selected alternatives and compose a concrete Blueprint.

        Only selected alternatives are traversed. Active ChoiceAtoms with the
        same name must be equal. Overrides must name active choices, so
        misspellings and unused selections fail.

        Raises:
            ChoiceResolutionError: An option is invalid, a name is ambiguous,
                or an override names an unknown or inactive choice.
        """
        return self._resolve(choices or {})[0]

    def active_choices(self, *, choices: Mapping[str, str] | None = None) -> tuple[ChoiceAtom, ...]:
        """The choices that solidify() resolves for these selections, in composition order.

        Raises the same errors as solidify().
        """
        return self._resolve(choices or {})[1]

    def _resolve(self, selections: Mapping[str, str]) -> tuple[Blueprint, tuple[ChoiceAtom, ...]]:
        active: dict[str, ChoiceAtom] = {}

        def resolve(part: Blueprint | MetaBlueprint | ChoiceAtom) -> Blueprint:
            if isinstance(part, Blueprint):
                return part
            if isinstance(part, MetaBlueprint):
                return autoconnect(*(resolve(child) for child in part.parts))

            if active.setdefault(part.name, part) != part:
                raise ChoiceResolutionError(f"Duplicate choice name: {part.name!r}")
            option = selections.get(part.name, part.default)
            if option not in part.choices:
                raise ChoiceResolutionError(
                    f"Invalid option {option!r} for choice {part.name!r}. "
                    f"Available options: {', '.join(part.choices)}"
                )
            return resolve(part.choices[option])

        blueprint = resolve(self)
        for name in selections:
            if name not in active:
                selected = {
                    atom.name: selections.get(atom.name, atom.default) for atom in active.values()
                }
                raise ChoiceResolutionError(self._explain_unused(name, selected))
        return blueprint, tuple(active.values())

    def _explain_unused(self, name: str, selected: Mapping[str, str]) -> str:
        """Tell a misspelled choice name from one under an unselected alternative.

        `selected` maps each active choice to its resolved option.
        """
        conditions: dict[str, _Conditions] = {}
        for known, required in _all_choices(self):
            conditions.setdefault(known, required)

        if not conditions:
            return f"Unknown choice {name!r}. This blueprint has no choices"
        if name not in conditions:
            message = f"Unknown choice {name!r}. Available choices: {', '.join(conditions)}"
            suggestions = difflib.get_close_matches(name, conditions)
            if suggestions:
                message += f". Did you mean: {', '.join(suggestions)}?"
            return message

        # The first condition names a top-level choice, and each later one is
        # active when those before it hold. So the first one that fails is active.
        parent, option = next(
            (parent, option) for parent, option in conditions[name] if selected[parent] != option
        )
        return (
            f"Choice {name!r} is inactive because {parent}={selected[parent]}. "
            f"It needs {parent}={option}"
        )


def _all_choices(
    part: Blueprint | MetaBlueprint | ChoiceAtom, conditions: _Conditions = ()
) -> Iterator[tuple[str, _Conditions]]:
    """Every choice name below part, selected or not, with what activates it."""
    if isinstance(part, MetaBlueprint):
        for child in part.parts:
            yield from _all_choices(child, conditions)
    elif isinstance(part, ChoiceAtom):
        yield part.name, conditions
        for option, alternative in part.choices.items():
            yield from _all_choices(alternative, (*conditions, (part.name, option)))


def autoconnect_with_choices(*parts: Blueprint | MetaBlueprint | ChoiceAtom) -> MetaBlueprint:
    """Compose fragments, deferring choice resolution until solidify()."""
    return MetaBlueprint(parts=parts)

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

"""Config leaf traversal and short field flags for ``dimos run``."""

from __future__ import annotations

from collections.abc import Collection, Iterator, Sequence
from dataclasses import dataclass
import inspect
import types
from typing import TYPE_CHECKING, Any, Union, get_args, get_origin

from pydantic import BaseModel
from pydantic.fields import FieldInfo
from pydantic_core import PydanticUndefined

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint


_MODULE_INTERNAL_FIELDS = ("g", "instance_name")


class FlagResolutionError(Exception):
    """A CLI field flag could not be turned into one config address."""


@dataclass(frozen=True)
class ConfigLeaf:
    """One field in a blueprint config tree.

    Leaves inside a multi-variant model union are help-only: rendered by
    ``arg_help`` for the display variant, never matched to a field flag.
    """

    path: tuple[str, ...]
    annotation: Any
    default: Any
    required: bool
    in_union_variant: bool = False

    @property
    def name(self) -> str:
        return self.path[-1]

    @property
    def dotted(self) -> str:
        return ".".join(self.path)

    @property
    def flag(self) -> str:
        """The path-form flag spelling, e.g. ``--go2connection.frame-id``."""
        return "--" + ".".join(_kebab(part) for part in self.path)


def iter_config_leaves(config: type[BaseModel], blueprint: Blueprint) -> Iterator[ConfigLeaf]:
    """Yield nested ``BaseModel`` leaves in a blueprint config tree.

    A multi-variant model union is traversed through its display variant (the
    one matching the ``backend`` default, else the first) so ``--help`` can
    render it, but its leaves are marked ``in_union_variant`` and never match
    a short field flag: a field in a backend variant needs an explicit address
    (an address flag or ``-o``).
    """
    yield from _walk(
        config, blueprint, path=(), atom=None, defaults=None, is_root=True, in_union_variant=False
    )


def _walk(
    model: type[BaseModel],
    blueprint: Blueprint,
    path: tuple[str, ...],
    atom: Any,
    defaults: Any,
    is_root: bool,
    in_union_variant: bool,
) -> Iterator[ConfigLeaf]:
    from dimos.core.coordination.blueprints import config_key

    for name, info in model.model_fields.items():
        if not is_root and name in _MODULE_INTERNAL_FIELDS:
            continue

        annotation = info.annotation
        if isinstance(annotation, types.GenericAlias):
            continue

        field_defaults = _get_default_value(defaults, name, _field_default(info))
        child_path = (*path, name)
        child_in_variant = in_union_variant
        nested_model = _base_model(annotation)
        if nested_model is None:
            variants = _union_base_models(annotation)
            if variants:
                nested_model = _select_base_model_candidate(variants, field_defaults)
                child_in_variant = True
        if nested_model is not None:
            if is_root:
                atom = next(
                    (item for item in blueprint.blueprints if config_key(item.name) == name), None
                )
                child_defaults = atom.kwargs if atom is not None else field_defaults
            else:
                child_defaults = field_defaults
            yield from _walk(
                nested_model,
                blueprint,
                child_path,
                atom,
                child_defaults,
                is_root=False,
                in_union_variant=child_in_variant,
            )
            continue

        yield ConfigLeaf(
            path=child_path,
            annotation=annotation,
            default=field_defaults,
            required=info.is_required() and not _has_default_value(defaults, name),
            in_union_variant=in_union_variant,
        )


def _base_model(annotation: Any) -> type[BaseModel] | None:
    if inspect.isclass(annotation) and issubclass(annotation, BaseModel):
        return annotation
    if get_origin(annotation) not in {Union, types.UnionType}:
        return None
    candidates = [arg for arg in get_args(annotation) if arg is not type(None)]
    if len(candidates) == 1 and inspect.isclass(candidates[0]):
        candidate = candidates[0]
        if issubclass(candidate, BaseModel):
            return candidate
    return None


def _union_base_models(annotation: Any) -> tuple[type[BaseModel], ...]:
    if get_origin(annotation) not in {Union, types.UnionType}:
        return ()
    return tuple(
        arg for arg in get_args(annotation) if inspect.isclass(arg) and issubclass(arg, BaseModel)
    )


def _select_base_model_candidate(
    candidates: Sequence[type[BaseModel]], defaults: Any
) -> type[BaseModel]:
    backend = _backend_default(defaults)
    if backend is not PydanticUndefined:
        for candidate in candidates:
            backend_info = candidate.model_fields.get("backend")
            if backend_info is not None and _field_default(backend_info) == backend:
                return candidate
    return candidates[0]


def _backend_default(defaults: Any) -> Any:
    if isinstance(defaults, BaseModel):
        return getattr(defaults, "backend", PydanticUndefined)
    if isinstance(defaults, dict):
        return defaults.get("backend", PydanticUndefined)
    return PydanticUndefined


def _field_default(info: FieldInfo) -> Any:
    if info.default is not PydanticUndefined:
        return info.default
    if info.default_factory is not None and not info.default_factory_takes_validated_data:
        return info.get_default(call_default_factory=True)
    return PydanticUndefined


def _has_default_value(defaults: Any, key: str) -> bool:
    if isinstance(defaults, BaseModel):
        return key in defaults.model_fields_set
    if isinstance(defaults, dict):
        return key in defaults
    return False


def _get_default_value(defaults: Any, key: str, fallback: Any) -> Any:
    if isinstance(defaults, BaseModel) and key in defaults.model_fields_set:
        return getattr(defaults, key)
    if isinstance(defaults, dict):
        return defaults.get(key, fallback)
    return fallback


def _kebab(name: str) -> str:
    return name.replace("_", "-")


def split_run_tokens(tokens: Sequence[str]) -> tuple[list[str], list[str]]:
    """Split pass-through tokens into initial blueprint names and field flags."""
    names: list[str] = []
    flags: list[str] = []
    for index, token in enumerate(tokens):
        if token.startswith("-"):
            flags = list(tokens[index:])
            break
        names.append(token)

    if not names:
        raise FlagResolutionError(
            "no blueprint name given; blueprint names must come before options, "
            "for example `dimos run unitree-go2 --map-file recording_go2`."
        )
    return names, flags


def expand_field_flags(tokens: Sequence[str], leaves: Sequence[ConfigLeaf]) -> list[str]:
    """Convert ``--field value`` and ``--module.field value`` into ``-o`` overrides."""
    by_name: dict[str, list[ConfigLeaf]] = {}
    for leaf in leaves:
        if leaf.in_union_variant:
            continue
        by_name.setdefault(leaf.name, []).append(leaf)

    expanded: list[str] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        index += 1
        if not token.startswith("--"):
            if token.startswith("-"):
                raise FlagResolutionError(
                    f"unknown field flag {token!r}; field flags use --field value."
                )
            raise FlagResolutionError(
                f"unexpected argument {token!r}; blueprint names must come before options."
            )

        raw = token[2:]
        if "=" in raw:
            raise FlagResolutionError(f"--{raw} is not supported; field flags use --field value.")

        name = raw.replace("-", "_")
        if raw != _kebab(name):
            raise FlagResolutionError(
                f"unknown configuration field flag: --{raw}. "
                "Run `dimos run <blueprint> --help` to see available field flags."
            )

        if "." in name:
            # Explicit address flag, e.g. --go2connection.frame-id. Pure -o
            # sugar: an unknown address is reported by config validation.
            path = name
        else:
            matches = by_name.get(name, [])
            if not matches:
                raise FlagResolutionError(
                    f"unknown configuration field flag: --{raw}. "
                    "Run `dimos run <blueprint> --help` to see available field flags."
                )
            if len(matches) > 1:
                raise FlagResolutionError(_ambiguous_message(name, matches))
            path = matches[0].dotted

        if index >= len(tokens) or tokens[index].startswith("-"):
            raise FlagResolutionError(f"--{raw} expects one following value.")

        expanded.append(f"{path}={tokens[index]}")
        index += 1

    return expanded


def _ambiguous_message(name: str, matches: Sequence[ConfigLeaf]) -> str:
    candidates = sorted(leaf.flag for leaf in matches)
    return "\n".join(
        [
            f"--{_kebab(name)} matches multiple config fields; use a full address flag:",
            *(f"  {candidate}" for candidate in candidates),
        ]
    )


def display_type(annotation: Any) -> str:
    """Render an annotation for ``dimos run --help``."""
    return annotation.__name__ if isinstance(annotation, type) else str(annotation)


def leaf_flag_annotations(
    leaves: Sequence[ConfigLeaf], reserved: Collection[str] = ()
) -> dict[tuple[str, ...], str]:
    """Return the short flag or the full address flag for each leaf."""
    by_name: dict[str, list[ConfigLeaf]] = {}
    for leaf in leaves:
        if leaf.in_union_variant:
            continue
        by_name.setdefault(leaf.name, []).append(leaf)

    annotations: dict[tuple[str, ...], str] = {}
    for leaf in leaves:
        short = f"--{_kebab(leaf.name)}"
        unique = len(by_name.get(leaf.name, ())) == 1
        if leaf.in_union_variant or short in reserved or not unique:
            annotations[leaf.path] = f" [{leaf.flag}]"
        else:
            annotations[leaf.path] = f" [{short}]"
    return annotations

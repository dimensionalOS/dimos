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

"""Introspection of pydantic models and their field annotations."""

from __future__ import annotations

from collections.abc import Mapping
import inspect
from types import UnionType
from typing import Annotated, Any, Union, get_args, get_origin, get_type_hints

from pydantic import BaseModel, TypeAdapter
from pydantic.errors import PydanticInvalidForJsonSchema, PydanticSchemaGenerationError
from pydantic.fields import FieldInfo
from pydantic_core import PydanticUndefined

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.values import plain, plain_mapping
from dimos.core.coordination.blueprints import BlueprintAtom


def module_config_cls(atom: BlueprintAtom) -> type[BaseModel]:
    try:
        config_cls = get_type_hints(atom.module)["config"]
    except (KeyError, NameError, TypeError) as error:
        raise BlueprintConfigError(
            f"Could not resolve configuration type for {atom.module.__name__}: {error}"
        ) from error
    if not inspect.isclass(config_cls) or not issubclass(config_cls, BaseModel):
        raise BlueprintConfigError(
            f"{atom.module.__name__}.config must be a pydantic BaseModel type."
        )
    return config_cls


def leaf_fields(
    model: type[BaseModel],
    prefix: tuple[str, ...] = (),
    *,
    excluded: set[str] | None = None,
    ancestors: frozenset[type[BaseModel]] = frozenset(),
) -> list[tuple[tuple[str, ...], Any]]:
    leaves: dict[tuple[str, ...], Any] = {}
    excluded = excluded or set()
    if model in ancestors:
        return []
    next_ancestors = ancestors | {model}

    for name, info in model.model_fields.items():
        if not prefix and name in excluded:
            continue
        path = (*prefix, name)
        nested_models = _base_model_types(info.annotation)
        if nested_models:
            nested_leaves: list[tuple[tuple[str, ...], Any]] = []
            for nested in nested_models:
                nested_leaves.extend(
                    leaf_fields(nested, path, excluded=set(), ancestors=next_ancestors)
                )
            if nested_leaves:
                for nested_path, annotation in nested_leaves:
                    existing = leaves.get(nested_path)
                    if existing is None:
                        leaves[nested_path] = annotation
                    elif existing != annotation:
                        leaves[nested_path] = existing | annotation
                continue
        if _contains_runtime_type(info.annotation):
            continue
        if not _is_cli_settable(info.annotation):
            continue
        leaves[path] = info.annotation
    return list(leaves.items())


def _base_model_types(annotation: Any) -> tuple[type[BaseModel], ...]:
    annotation = _unwrap_annotated(annotation)
    candidates = (
        get_args(annotation) if get_origin(annotation) in (Union, UnionType) else (annotation,)
    )
    return tuple(
        candidate
        for candidate in candidates
        if inspect.isclass(candidate) and issubclass(candidate, BaseModel)
    )


def _unwrap_annotated(annotation: Any) -> Any:
    while get_origin(annotation) is Annotated:
        annotation = get_args(annotation)[0]
    return annotation


def _contains_runtime_type(annotation: Any) -> bool:
    """Whether an annotation requires a Python class object, not CLI data."""
    annotation = _unwrap_annotated(annotation)
    if annotation is type or get_origin(annotation) is type:
        return True
    if get_origin(annotation) in (Union, UnionType):
        return any(_contains_runtime_type(candidate) for candidate in get_args(annotation))
    return False


def all_annotation_types(annotation: Any) -> set[Any]:
    annotation = _unwrap_annotated(annotation)
    if get_origin(annotation) in (Union, UnionType):
        result: set[Any] = set()
        for candidate in get_args(annotation):
            result.update(all_annotation_types(candidate))
        return result
    return {annotation}


def scalar_annotation_types(annotation: Any) -> set[Any]:
    return {
        candidate for candidate in all_annotation_types(annotation) if candidate is not type(None)
    }


def _is_cli_settable(annotation: Any) -> bool:
    """Whether pydantic can build some union member from CLI-provided data.

    Arbitrary classes (permitted via ``arbitrary_types_allowed``) validate by
    isinstance only, and callables cannot be produced from a string either; a
    bare ``TypeAdapter`` refuses to generate a JSON schema for exactly those
    types, even nested inside containers.
    """
    return any(_member_is_cli_settable(member) for member in scalar_annotation_types(annotation))


def _member_is_cli_settable(member: Any) -> bool:
    try:
        TypeAdapter(member).json_schema()
    except (PydanticInvalidForJsonSchema, PydanticSchemaGenerationError):
        return False
    return True


def prepare_model_input(
    model: type[BaseModel],
    values: Mapping[str, Any],
) -> dict[str, Any]:
    """Add only schema defaults needed to validate sparse nested overrides.

    A discriminated union cannot validate ``{"solver": "foo"}`` even when its
    field has a default backend model. Copy that model's discriminator into the
    sparse input, while leaving ordinary defaults for construction in workers.
    """
    prepared = plain_mapping(values)
    for name, info in model.model_fields.items():
        raw_value = prepared.get(name)
        if not isinstance(raw_value, Mapping):
            continue
        nested_models = _base_model_types(info.annotation)
        if not nested_models:
            continue

        selected: type[BaseModel] | None = None
        discriminator = info.discriminator if isinstance(info.discriminator, str) else None
        nested_values = plain_mapping(raw_value)
        if discriminator is not None:
            discriminator_value = nested_values.get(discriminator, PydanticUndefined)
            if discriminator_value is PydanticUndefined:
                default = _parse_field_default(info)
                if isinstance(default, BaseModel):
                    discriminator_value = getattr(
                        default,
                        discriminator,
                        PydanticUndefined,
                    )
                elif isinstance(default, Mapping):
                    discriminator_value = default.get(discriminator, PydanticUndefined)
                if discriminator_value is not PydanticUndefined:
                    nested_values[discriminator] = plain(discriminator_value)
            if discriminator_value is not PydanticUndefined:
                selected = _select_discriminated_model(
                    nested_models,
                    discriminator,
                    discriminator_value,
                )
        elif len(nested_models) == 1:
            selected = nested_models[0]

        if selected is not None:
            nested_values = prepare_model_input(selected, nested_values)
        prepared[name] = nested_values
    return prepared


def _parse_field_default(info: FieldInfo) -> Any:
    if info.default is not PydanticUndefined:
        return info.default
    if info.default_factory is not None:
        return info.get_default(call_default_factory=True)
    return PydanticUndefined


def _select_discriminated_model(
    candidates: tuple[type[BaseModel], ...],
    discriminator: str,
    value: Any,
) -> type[BaseModel] | None:
    for candidate in candidates:
        info = candidate.model_fields.get(discriminator)
        if info is None:
            continue
        if _parse_field_default(info) == value:
            return candidate
    return None


def display_annotation(annotations: tuple[Any, ...]) -> str:
    names = []
    for annotation in annotations:
        annotation = _unwrap_annotated(annotation)
        if isinstance(annotation, type):
            name = annotation.__name__
        else:
            name = str(annotation).replace("typing.", "")
        if name not in names:
            names.append(name)
    return " | ".join(names)


def nested_get(values: Any, path: tuple[str, ...]) -> Any:
    current = values
    for part in path:
        if isinstance(current, BaseModel):
            if part not in current.model_fields_set:
                return PydanticUndefined
            current = getattr(current, part)
        elif isinstance(current, Mapping):
            if part not in current:
                return PydanticUndefined
            current = current[part]
        else:
            return PydanticUndefined
    return current


def safe_field_default(model: type[BaseModel], path: tuple[str, ...]) -> Any:
    if not path:
        return PydanticUndefined
    info = model.model_fields.get(path[0])
    if info is None:
        return PydanticUndefined
    if len(path) == 1:
        return info.default

    if info.default is not PydanticUndefined:
        configured = nested_get(info.default, path[1:])
        if configured is not PydanticUndefined:
            return configured

    for nested_model in _base_model_types(info.annotation):
        nested_default = safe_field_default(nested_model, path[1:])
        if nested_default is not PydanticUndefined:
            return nested_default
    return PydanticUndefined


def field_is_required(model: type[BaseModel], path: tuple[str, ...]) -> bool:
    current_models: tuple[type[BaseModel], ...] = (model,)
    for index, part in enumerate(path):
        infos = [
            info
            for candidate in current_models
            if (info := candidate.model_fields.get(part)) is not None
        ]
        if not infos:
            return False
        if not all(info.is_required() for info in infos):
            return False
        if index == len(path) - 1:
            return True
        nested: list[type[BaseModel]] = []
        for info in infos:
            nested.extend(_base_model_types(info.annotation))
        current_models = tuple(dict.fromkeys(nested))
    return False


def field_has_required_parent(model: type[BaseModel], path: tuple[str, ...]) -> bool:
    current_models: tuple[type[BaseModel], ...] = (model,)
    for index, part in enumerate(path[:-1]):
        infos = [
            info
            for candidate in current_models
            if (info := candidate.model_fields.get(part)) is not None
        ]
        if not infos:
            return False
        if all(info.is_required() for info in infos):
            return True
        nested: list[type[BaseModel]] = []
        for info in infos:
            nested.extend(_base_model_types(info.annotation))
        current_models = tuple(dict.fromkeys(nested))
        if not current_models and index < len(path) - 2:
            return False
    return False

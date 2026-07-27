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

"""Config machinery: collect flat class fields, synthesize the frozen pydantic model.

Spec: dimos/pure/tasks/t2-config.md. Flat annotated fields on a PureModule
subclass are the API; the synthesized per-subclass frozen pydantic model
(``extra="forbid"``, ``frozen=True``) is the substance, and its
``model_dump()`` is THE canonical config serialization (memo keys,
checkpoints, sweeps — module identity = class + config).
"""

from __future__ import annotations

from collections.abc import Mapping
import dataclasses
import typing
from typing import Any, ClassVar, Final, get_origin, get_type_hints

from pydantic import BaseModel, ConfigDict, Field, create_model
import typing_extensions

from dimos.protocol.service.spec import BaseConfig
from dimos.pure.stepspec import PureModuleDefinitionError

# Names owned by PureModule machinery (present and future tasks); a config
# field with one of these names raises ConfigFieldError at class definition.
RESERVED_CONFIG_FIELDS: Final[frozenset[str]] = frozenset(
    {
        "config",
        "step",
        "fold",
        "In",
        "Out",
        "State",
        "warmup",
        "start",
        "stop",
        "over",
        "checkpoint",
        "restore",
        "i",
        "o",
        "health",
        "health_hz",
    }
)


class ConfigFieldError(PureModuleDefinitionError):
    """A class body declares an invalid config field (raised at class definition).

    T3 seam (spec §10.1), ACTIVE: the base is stepspec.PureModuleDefinitionError
    (itself a TypeError subclass) so all definition-time errors share one
    user-facing type. config imports stepspec, never the reverse (acyclic).
    """


class FrozenModuleError(AttributeError):
    """Attribute assignment/deletion on a PureModule instance (config is frozen)."""


class PureModuleConfig(BaseConfig):
    """Root of every synthesized per-module config model."""

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        extra="forbid",
        frozen=True,
        validate_default=True,
        protected_namespaces=(),
    )

    health_hz: float = Field(default=1.0, ge=0.0)
    """Live health row cadence, Hz (T9); 0 disables rows (ring drain continues)."""


# Sentinel: a field with no assigned value is REQUIRED (pydantic marks it with
# ``...``). Distinct from a user-written ``= ...`` default, which §3.3.6 rejects.
_REQUIRED: Final = object()


def _is_final(hint: Any) -> bool:
    """True for both ``Final`` and ``Final[T]`` (typing or typing_extensions)."""
    return (
        get_origin(hint) is typing.Final
        or get_origin(hint) is typing_extensions.Final
        or hint is typing.Final
        or hint is typing_extensions.Final
    )


def _is_initvar(hint: Any) -> bool:
    """True for bare ``InitVar`` and subscripted ``InitVar[T]``."""
    return hint is dataclasses.InitVar or isinstance(hint, dataclasses.InitVar)


def _is_field_spec(default: Any) -> bool:
    """True if *default* is a T1 row specifier (lazy, tolerant of rows.py absent)."""
    try:
        from dimos.pure import rows
    except ImportError:
        return False
    spec = getattr(rows, "FieldSpec", None)
    return isinstance(spec, type) and isinstance(default, spec)


def _collect_config_fields(cls: type) -> dict[str, tuple[Any, Any]]:
    """Collect ``{name: (resolved type, default | ...)}`` from one class's own body.

    Annotated, non-ClassVar names become fields; nested classes, descriptors,
    methods, and dunders never do. Raises ConfigFieldError per spec §3/§11.
    """
    own_ann = cls.__dict__.get("__annotations__", {})
    ns = dict(cls.__dict__)

    # §3.3.9 — __post_init__ machinery is never run.
    if "__post_init__" in ns:
        raise ConfigFieldError(
            f"{cls.__qualname__}: __post_init__ is unsupported — derive values in "
            f"@resource factories or in step"
        )

    # §3.3.8 — an unannotated plain-data assignment is field-or-constant ambiguity.
    for name, value in ns.items():
        if name in own_ann or name.startswith("_"):
            continue
        if callable(value) or isinstance(value, type) or hasattr(value, "__get__"):
            continue
        raise ConfigFieldError(
            f"{cls.__qualname__}.{name} = {value!r}: assignment without a type "
            f"annotation is ambiguous — write {name}: {type(value).__name__} = ... to "
            f"make it a config field, or annotate with ClassVar to keep a class constant"
        )

    # §3.3.10 — field types must resolve at runtime (localns mirrors sketch §3 scoping).
    try:
        hints = get_type_hints(cls, include_extras=True, localns=dict(vars(cls)))
    except NameError as err:
        missing = getattr(err, "name", None)
        raw: Any = missing
        for ann in own_ann.values():  # best-effort field attribution
            if missing and missing in str(ann):
                raw = ann
                break
        raise ConfigFieldError(
            f"{cls.__qualname__}: config field annotation {raw!r} is not resolvable at "
            f"runtime ({err}); config field types must be runtime-importable"
        ) from err

    fields: dict[str, tuple[Any, Any]] = {}
    for name in own_ann:  # body declaration order
        hint = hints[name]
        if get_origin(hint) is ClassVar:
            continue  # constant, exempt (§3.2)

        # gauntlet §3.3.1–7, in order
        if name.startswith("_"):  # 1 leading underscore
            raise ConfigFieldError(
                f"{cls.__qualname__}.{name}: config field names must not start with '_' "
                f"(pydantic treats them as private while mypy treats them as fields); "
                f"use ClassVar for internal constants"
            )
        if name in RESERVED_CONFIG_FIELDS:  # 2 reserved machinery name
            raise ConfigFieldError(
                f"{cls.__qualname__}.{name}: {name!r} is reserved by PureModule "
                f"machinery and cannot be a config field"
            )
        if hasattr(BaseModel, name):  # 3 pydantic namespace collision
            raise ConfigFieldError(
                f"{cls.__qualname__}.{name}: {name!r} collides with a pydantic "
                f"BaseModel attribute on the synthesized config model; rename the field"
            )
        if _is_final(hint):  # 4 Final
            raise ConfigFieldError(
                f"{cls.__qualname__}.{name}: Final is not allowed on config fields — "
                f"they are already frozen, and pydantic would silently drop the field; "
                f"use ClassVar for a class constant"
            )
        if _is_initvar(hint):  # 5 InitVar
            raise ConfigFieldError(
                f"{cls.__qualname__}.{name}: InitVar is not supported — PureModule has "
                f"no __post_init__"
            )
        default = ns.get(name, _REQUIRED)
        if default is ...:  # 6 Ellipsis default
            raise ConfigFieldError(
                f"{cls.__qualname__}.{name}: Ellipsis is not a valid config default"
            )
        if default is not _REQUIRED and (  # 7 descriptor / row-specifier default
            hasattr(default, "__get__") or _is_field_spec(default)
        ):
            raise ConfigFieldError(
                f"{cls.__qualname__}.{name}: descriptors cannot be config field "
                f"defaults — row specifiers belong inside In/Out bundles, resources "
                f"use @resource"
            )

        fields[name] = (hint, ... if default is _REQUIRED else default)
    return fields


def _synthesize_config_model(
    cls: type,
    fields: Mapping[str, tuple[Any, Any]],
    bases: tuple[type[PureModuleConfig], ...],
) -> type[PureModuleConfig]:
    """Build the per-subclass frozen model ``<ClassName>Config`` extending *bases*.

    ``create_model`` collects inherited fields from the mirrored *bases* then
    appends this class's own fields — the §4.4 canonical order by construction.
    """
    field_definitions: dict[str, Any] = dict(fields)
    return create_model(
        f"{cls.__name__}Config",
        __base__=bases,
        __module__=cls.__module__,
        **field_definitions,
    )

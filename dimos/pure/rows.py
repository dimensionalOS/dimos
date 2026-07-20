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

"""Row bundles: In/Out bases and field specifiers.

Spec: dimos/pure/tasks/t1-rows.md. Data layer only — no engine, no streams;
imports are stdlib + typing_extensions.
"""

from __future__ import annotations

from collections.abc import Mapping
import dataclasses
from dataclasses import MISSING
import string
from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    Final,
    Literal,
    TypeVar,
    cast,
    get_type_hints,
    overload,
)

from typing_extensions import dataclass_transform

if TYPE_CHECKING:
    from _typeshed import DataclassInstance

__all__ = [
    "UNSTAMPED",
    "BundleDefinitionError",
    "ContractSpec",
    "FieldSpec",
    "In",
    "InterpolateSpec",
    "LatestSpec",
    "Out",
    "PlainSpec",
    "TfOutSpec",
    "TfSpec",
    "TickSpec",
    "contract",
    "format_frame",
    "interpolate",
    "latest",
    "normalize_frame",
    "tf",
    "tf_out",
    "tick",
]

_T = TypeVar("_T")

UNSTAMPED: Final[float] = float("-inf")
"""Default ts of a freshly constructed Out row, before the engine stamps it."""


class BundleDefinitionError(TypeError):
    """Raised at class-definition time for a malformed In/Out bundle."""


# ── introspection model ──────────────────────────────────────────────────────


@dataclasses.dataclass(frozen=True)
class FieldSpec:
    """Introspection record for one declared bundle field."""

    kind: ClassVar[str]
    side: ClassVar[Literal["in", "out"]]

    name: str = ""
    annotation: Any = None
    default: Any = dataclasses.field(default_factory=lambda: MISSING)

    @property
    def required(self) -> bool:
        """True when the field has no default (constructor-required)."""
        return self.default is MISSING


@dataclasses.dataclass(frozen=True)
class TickSpec(FieldSpec):
    """The In trigger field; expect_hz is a rate hint for health."""

    kind = "tick"
    side = "in"
    expect_hz: float | None = None


@dataclasses.dataclass(frozen=True)
class LatestSpec(FieldSpec):
    """In field resolved to the newest message at tick time."""

    kind = "latest"
    side = "in"


@dataclasses.dataclass(frozen=True)
class InterpolateSpec(FieldSpec):
    """In field interpolated to the tick timestamp."""

    kind = "interpolate"
    side = "in"


@dataclasses.dataclass(frozen=True)
class TfSpec(FieldSpec):
    """In field resolved to a chain-composed, interpolated transform at tick ts."""

    kind = "tf"
    side = "in"
    parent: str = ""
    child: str = ""


@dataclasses.dataclass(frozen=True)
class ContractSpec(FieldSpec):
    """Out field carrying a minimum-cadence contract."""

    kind = "contract"
    side = "out"
    min_hz: float = 0.0


@dataclasses.dataclass(frozen=True)
class PlainSpec(FieldSpec):
    """Out field with no engine policy; default=None fields are sparse ports."""

    kind = "plain"
    side = "out"


@dataclasses.dataclass(frozen=True)
class TfOutSpec(FieldSpec):
    """Out field asserting one tf edge; payload frames validate on emission."""

    kind = "tf_out"
    side = "out"
    parent: str = ""
    child: str = ""
    min_hz: float | None = None


# ── field specifiers ─────────────────────────────────────────────────────────


def tick(*, expect_hz: float | None = None) -> Any:
    """Declare the In trigger field: a new message here fires the tick."""
    if expect_hz is not None and expect_hz <= 0:
        raise ValueError(f"tick(): expect_hz must be > 0, got {expect_hz}")
    return TickSpec(expect_hz=expect_hz)


@overload
def latest() -> Any: ...
@overload
def latest(*, default: _T) -> _T: ...
def latest(*, default: Any = MISSING) -> Any:
    """Declare an In field resolved to the newest message at tick time."""
    return LatestSpec(default=default)


@overload
def interpolate() -> Any: ...
@overload
def interpolate(*, default: _T) -> _T: ...
def interpolate(*, default: Any = MISSING) -> Any:
    """Declare an In field interpolated to the tick timestamp."""
    return InterpolateSpec(default=default)


@overload
def contract(*, min_hz: float) -> Any: ...
@overload
def contract(*, min_hz: float, default: _T) -> _T: ...
def contract(*, min_hz: float, default: Any = MISSING) -> Any:
    """Declare an Out field carrying a minimum-cadence contract."""
    if min_hz <= 0:
        raise ValueError(f"contract(): min_hz must be > 0, got {min_hz}")
    return ContractSpec(min_hz=min_hz, default=default)


@overload
def tf(parent: str, child: str) -> Any: ...
@overload
def tf(parent: str, child: str, *, default: _T) -> _T: ...
def tf(parent: str, child: str, *, default: Any = MISSING) -> Any:
    """Declare an In field sampling the parent<-child transform chain at tick ts."""
    _check_edge_templates("tf", parent, child)
    return TfSpec(default=default, parent=parent, child=child)


@overload
def tf_out(parent: str, child: str, *, min_hz: float | None = None) -> Any: ...
@overload
def tf_out(parent: str, child: str, *, min_hz: float | None = None, default: _T) -> _T: ...
def tf_out(parent: str, child: str, *, min_hz: float | None = None, default: Any = MISSING) -> Any:
    """Declare an Out field asserting the parent->child tf edge."""
    if min_hz is not None and min_hz <= 0:
        raise ValueError(f"tf_out(): min_hz must be > 0, got {min_hz}")
    _check_edge_templates("tf_out", parent, child)
    return TfOutSpec(default=default, parent=parent, child=child, min_hz=min_hz)


# ── frame templates (t11 spec §3) ────────────────────────────────────────────

_TEMPLATE_FORMATTER: Final = string.Formatter()


def normalize_frame(frame: str) -> str:
    """Join the non-empty '/'-segments of a frame name (empty segments drop)."""
    return "/".join(seg for seg in frame.split("/") if seg)


def format_frame(template: str, values: Mapping[str, Any]) -> str:
    """Resolve a frame template against config values, then normalize.

    Raises KeyError for an unknown placeholder, ValueError for an unusable
    value; callers (module build) wrap with module + field + template context.
    None resolves empty — its segment drops with the separator.
    """
    out: list[str] = []
    for literal, field_name, _spec, _conv in _TEMPLATE_FORMATTER.parse(template):
        out.append(literal)
        if field_name is None:
            continue
        if field_name not in values:
            raise KeyError(field_name)
        value = values[field_name]
        if value is None:
            continue
        if not isinstance(value, (str, int, float, bool)):
            raise ValueError(
                f"config field {field_name!r} value {value!r} is not usable in a frame "
                f"name — use str, int, float, bool, or None (None resolves empty and "
                f"its segment drops)"
            )
        out.append(str(value))
    return normalize_frame("".join(out))


def _check_frame_template(fn: str, template: object) -> None:
    """Specifier-call validation of one frame template (t11 spec §3.1)."""
    if not isinstance(template, str) or not template:
        raise ValueError(f"{fn}(): frame template must be a non-empty string, got {template!r}")
    try:
        parts = list(_TEMPLATE_FORMATTER.parse(template))
    except ValueError as exc:
        raise ValueError(f"{fn}(): frame template {template!r} is malformed: {exc}") from None
    placeholders = 0
    for _literal, field_name, format_spec, conversion in parts:
        if field_name is None:
            continue
        placeholders += 1
        if not field_name.isidentifier() or format_spec or conversion:
            raise ValueError(
                f"{fn}(): frame template {template!r}: placeholder must be a plain "
                f"config field name ('{{prefix}}'-style; no positions, attributes, "
                f"indexing, conversions, or format specs)"
            )
    if placeholders == 0 and not normalize_frame(template):
        raise ValueError(f"{fn}(): frame template {template!r} is empty after empty segments drop")


def _check_edge_templates(fn: str, parent: object, child: object) -> None:
    """Specifier-call validation of a declared edge (t11 spec §3.1)."""
    _check_frame_template(fn, parent)
    _check_frame_template(fn, child)
    if parent == child:
        raise ValueError(
            f"{fn}(): parent and child templates are identical ({parent!r}) — a tf edge "
            f"relates two distinct frames"
        )


# ── processing pipeline (spec §4.2) ──────────────────────────────────────────

_RESERVED: Final = frozenset({"ts", "fields"})
_NO_VALUE: Final = object()


def _err(cls: type, msg: str) -> BundleDefinitionError:
    """Build a BundleDefinitionError prefixed with the bundle's qualified path."""
    return BundleDefinitionError(f"{cls.__module__}.{cls.__qualname__}: {msg}")


def _names(names: list[str]) -> str:
    return ", ".join(names)


def _is_initvar(ann: object) -> bool:
    """Textual InitVar detection (annotations are strings under future-annotations)."""
    if isinstance(ann, str):
        return "InitVar" in ann
    return ann is dataclasses.InitVar or type(ann) is dataclasses.InitVar


def _dataclassify(cls: type[_Bundle]) -> None:
    """Apply the frozen kw-only dataclass in place (spec §4.1)."""
    dataclasses.dataclass(frozen=True, kw_only=True, eq=True, repr=True)(cls)


def _merged_specs(cls: type) -> dict[str, FieldSpec]:
    """Merge per-class own-spec tables base-first over the MRO (nearest wins)."""
    merged: dict[str, FieldSpec] = {}
    for klass in reversed(cls.__mro__):
        own = klass.__dict__.get("__pure_own_specs__")
        if own:
            merged.update(own)
    return merged


def _process(cls: type[_Bundle]) -> None:
    """Validate the bundle and turn it into a real frozen kw-only dataclass."""
    # 1. Root path: In/Out assign __bundle_side__ in their own body.
    if "__bundle_side__" in cls.__dict__:
        _dataclassify(cls)
        cls.__pure_own_specs__ = {}
        return

    # 2. Side (inherited from In/Out).
    side: str | None = getattr(cls, "__bundle_side__", None)
    if side is None:
        raise _err(cls, "bundles must subclass pm.In or pm.Out")

    # 3. Both roots.
    if issubclass(cls, In) and issubclass(cls, Out):
        raise _err(cls, "a bundle cannot inherit from both In and Out")

    # 4. Foreign dataclass bases.
    for base in cls.__mro__:
        if dataclasses.is_dataclass(base) and not issubclass(base, _Bundle):
            raise _err(
                cls,
                f"base {base!r} is a dataclass but not a bundle; bundles may only "
                "inherit fields from other In/Out bundles",
            )

    # 5. Generic.
    if getattr(cls, "__parameters__", ()):
        raise _err(
            cls,
            "generic bundles are not supported; rows are concrete data "
            "(share fields via bundle inheritance instead)",
        )

    own_ann: dict[str, Any] = cls.__dict__.get("__annotations__", {})

    # 6. Per-annotation checks (own annotations only; not evaluated).
    for name, ann in own_ann.items():
        if name in _RESERVED:
            raise _err(
                cls, f"field name {name!r} is reserved (row infrastructure: ['fields', 'ts'])"
            )
        if side == "in" and name == "tf":
            raise _err(
                cls,
                "In field name 'tf' is reserved — it is the tf stream key "
                "(over(tf=...), m.i.tf) that feeds tf() samplers; rename the field",
            )
        if _is_initvar(ann):
            raise _err(
                cls,
                f"field {name!r} is an InitVar; bundles are plain rows and take no "
                "init-only parameters",
            )
        val = cls.__dict__.get(name, _NO_VALUE)
        if isinstance(val, property):
            raise _err(
                cls,
                f"{name!r} is annotated as a field but its value is a property; "
                "bundles are plain data",
            )
        if isinstance(val, dataclasses.Field):
            raise _err(
                cls,
                f"{name!r} uses dataclasses.field(); use a field specifier "
                "(tick()/latest()/interpolate()/contract()) or a plain default instead",
            )

    # 7. Specifier/annotation pairing.
    raw_specs: dict[str, FieldSpec] = {}
    for name, val in cls.__dict__.items():
        if isinstance(val, FieldSpec):
            if name not in own_ann:
                raise _err(
                    cls,
                    f"{name!r} is assigned a field specifier but has no field annotation "
                    f"(write `{name}: <Type> = ...`)",
                )
            raw_specs[name] = val

    # 8. Normalization — specifier objects die here (Field marker or plain default).
    for name, spec in raw_specs.items():
        if spec.default is MISSING:
            setattr(cls, name, dataclasses.field())
        else:
            setattr(cls, name, spec.default)

    # 9. Dataclassify (stdlib semantics from here).
    _dataclassify(cls)
    fields = dataclasses.fields(cast("type[DataclassInstance]", cls))

    # 10. Leftover: specifiers that did not become fields (ClassVar etc.).
    field_names = {f.name for f in fields}
    leftover = [name for name in raw_specs if name not in field_names]
    if leftover:
        raise _err(
            cls,
            f"{_names(leftover)} carry field specifiers but are not fields "
            "(ClassVar-annotated?); specifiers are only valid on data fields",
        )

    # 11. Own spec table (own fields only, ts excluded).
    own_specs: dict[str, FieldSpec] = {}
    for f in fields:
        fname = f.name
        if fname == "ts" or fname not in own_ann:
            continue
        if fname in raw_specs:
            spec = raw_specs[fname]
            if spec.side != side:
                raise _err(
                    cls,
                    f"{spec.kind}() is not valid on an {side.capitalize()} bundle field "
                    f"({fname!r})",
                )
            own_specs[fname] = spec
        elif side == "in":
            raise _err(
                cls,
                f"In field {fname!r} needs a sampler specifier "
                "(tick()/latest()/interpolate()); plain defaults are only valid on Out bundles",
            )
        else:
            own_specs[fname] = PlainSpec(default=f.default)
    cls.__pure_own_specs__ = own_specs

    # 12. Merged validation.
    merged = _merged_specs(cls)
    ticks = [n for n, s in merged.items() if isinstance(s, TickSpec)]
    if len(ticks) > 1:
        raise _err(
            cls,
            f"multiple tick() fields ({_names(ticks)}); an In bundle declares at most one trigger",
        )

    # 13. Leak sweep (internal invariant backstop).
    for name, val in cls.__dict__.items():
        if isinstance(val, FieldSpec):
            raise _err(cls, f"specifier object survived as class attribute {name!r}")


# ── bundle bases ─────────────────────────────────────────────────────────────


@dataclass_transform(
    kw_only_default=True,
    frozen_default=True,
    field_specifiers=(tick, latest, interpolate, contract, tf, tf_out),
)
class _Bundle:
    """Shared machinery for In/Out row bundles."""

    __bundle_side__: ClassVar[Literal["in", "out"]]
    __pure_own_specs__: ClassVar[dict[str, FieldSpec]]  # per-class raw spec table
    __pure_fields_cache__: ClassVar[dict[str, FieldSpec]]  # per-class resolved fields()

    def __init_subclass__(cls, **kwargs: object) -> None:
        """Validate the bundle and apply frozen kw-only dataclass machinery."""
        super().__init_subclass__(**kwargs)
        _process(cls)

    @classmethod
    def fields(cls) -> dict[str, FieldSpec]:
        """Declared data fields (ports) by name; ts excluded, declaration-ordered."""
        cached: dict[str, FieldSpec] | None = cls.__dict__.get("__pure_fields_cache__")
        if cached is not None:
            return dict(cached)
        merged = _merged_specs(cls)
        try:
            hints = get_type_hints(cls, include_extras=True)
        except Exception as exc:
            raise _err(
                cls,
                f"cannot resolve field types ({exc}); field types must be importable "
                "at module scope — reference nested classes by qualified path "
                "(e.g. 'Tagger.Out')",
            ) from exc
        resolved: dict[str, FieldSpec] = {
            name: dataclasses.replace(spec, name=name, annotation=hints[name])
            for name, spec in merged.items()
        }
        cls.__pure_fields_cache__ = resolved
        return dict(resolved)


class In(_Bundle):
    """Base for input row bundles; ts is the tick time, required at construction."""

    __bundle_side__: ClassVar[Literal["in", "out"]] = "in"
    ts: float


class Out(_Bundle):
    """Base for output row bundles; ts stays UNSTAMPED until the engine stamps it."""

    __bundle_side__: ClassVar[Literal["in", "out"]] = "out"
    ts: float = UNSTAMPED

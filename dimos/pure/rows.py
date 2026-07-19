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

"""Row bundles: In/Out bases and field specifiers (T1 skeleton).

Spec: dimos/pure/tasks/t1-rows.md. Data layer only — no engine, no streams;
imports are stdlib + typing_extensions. Bodies raise NotImplementedError.
"""

from __future__ import annotations

import dataclasses
from dataclasses import MISSING
from typing import Any, ClassVar, Final, Literal, TypeVar, overload

from typing_extensions import dataclass_transform

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
    "TickSpec",
    "contract",
    "interpolate",
    "latest",
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
        raise NotImplementedError


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


# ── field specifiers ─────────────────────────────────────────────────────────


def tick(*, expect_hz: float | None = None) -> Any:
    """Declare the In trigger field: a new message here fires the tick."""
    raise NotImplementedError


@overload
def latest() -> Any: ...
@overload
def latest(*, default: _T) -> _T: ...
def latest(*, default: Any = MISSING) -> Any:
    """Declare an In field resolved to the newest message at tick time."""
    raise NotImplementedError


@overload
def interpolate() -> Any: ...
@overload
def interpolate(*, default: _T) -> _T: ...
def interpolate(*, default: Any = MISSING) -> Any:
    """Declare an In field interpolated to the tick timestamp."""
    raise NotImplementedError


@overload
def contract(*, min_hz: float) -> Any: ...
@overload
def contract(*, min_hz: float, default: _T) -> _T: ...
def contract(*, min_hz: float, default: Any = MISSING) -> Any:
    """Declare an Out field carrying a minimum-cadence contract."""
    raise NotImplementedError


# ── bundle bases ─────────────────────────────────────────────────────────────


@dataclass_transform(
    kw_only_default=True,
    frozen_default=True,
    field_specifiers=(tick, latest, interpolate, contract),
)
class _Bundle:
    """Shared machinery for In/Out row bundles."""

    __bundle_side__: ClassVar[Literal["in", "out"]]
    __pure_own_specs__: ClassVar[dict[str, FieldSpec]]

    def __init_subclass__(cls, **kwargs: object) -> None:
        """Validate the bundle and apply frozen kw-only dataclass machinery."""
        super().__init_subclass__(**kwargs)
        if cls.__module__ != __name__:  # keep the In/Out roots below importable
            raise NotImplementedError

    @classmethod
    def fields(cls) -> dict[str, FieldSpec]:
        """Declared data fields (ports) by name; ts excluded, declaration-ordered."""
        raise NotImplementedError


class In(_Bundle):
    """Base for input row bundles; ts is the tick time, required at construction."""

    __bundle_side__: ClassVar[Literal["in", "out"]] = "in"
    ts: float


class Out(_Bundle):
    """Base for output row bundles; ts stays UNSTAMPED until the engine stamps it."""

    __bundle_side__: ClassVar[Literal["in", "out"]] = "out"
    ts: float = UNSTAMPED

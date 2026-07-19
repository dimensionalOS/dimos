#!/usr/bin/env python3
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

"""Step-shape classification + definition-time validation (T3).

Skeleton per ``tasks/t3-validation.md`` — signatures are the contract, bodies
land with the T3 implementation. ``classify(cls)`` is pure and structural:
it inspects a module class's ``step``/``fold`` signature (never bodies, never
instances) and returns a ``StepSpec`` or raises ``PureModuleDefinitionError``.
T2's ``PureModule.__init_subclass__`` stores the result at ``__pure_step__``.

Implementation note (spec §1): the row-base checks import T1's bases via
``from dimos.pure.rows import In, Out`` at module level — added when T1
lands; never import the ``dimos.pure`` package surface from here (cycle).
"""

from __future__ import annotations

from dataclasses import dataclass
import enum
import types
from typing import Any, Final, NoReturn

PURE_STEP_ATTR: Final = "__pure_step__"
"""Class-dict key where T2 stores the ``StepSpec`` (own dict per subclass)."""


class Rule(enum.Enum):
    """Every definition-time rule, by its message slug (spec §9.3)."""

    STEP_MISSING = "step-missing"
    STEP_AND_FOLD = "step-and-fold"
    STEP_NOT_FUNCTION = "step-not-function"
    STEP_NO_SELF = "step-no-self"
    STEP_PARAMS = "step-params"
    STEP_PARAM_DEFAULT = "step-param-default"
    STEP_ARITY = "step-arity"
    ASYNC_MEALY = "async-mealy"
    FOLD_ARITY = "fold-arity"
    FOLD_ASYNC = "fold-async"
    STEP_UNANNOTATED = "step-unannotated"
    STEP_UNRESOLVABLE = "step-unresolvable"
    IN_NOT_ROW = "in-not-row"
    OUT_NOT_ROW = "out-not-row"
    OUT_UNION = "out-union"
    STEP_RETURNS_NOTHING = "step-returns-nothing"
    STEP_RETURNS_AWAITABLE = "step-returns-awaitable"
    MEALY_NO_STATE = "mealy-no-state"
    STATE_MISMATCH = "state-mismatch"
    STATE_IS_ROW = "state-is-row"
    STATE_NOT_DEFAULT_CONSTRUCTIBLE = "state-not-default-constructible"
    MEALY_RETURN = "mealy-return"
    FOLD_ROWS_PARAM = "fold-rows-param"
    FOLD_RETURN = "fold-return"
    STATE_UNUSED = "state-unused"
    FOLD_STATE = "fold-state"
    BUNDLE_SHADOWED = "bundle-shadowed"
    NOT_CLASSIFIED = "not-classified"


class PureModuleDefinitionError(TypeError):
    """A PureModule subclass violated a definition-time shape rule."""

    rule: Rule

    def __init__(self, message: str, rule: Rule) -> None:
        """Message is release copy per spec §9; ``rule`` is machine-readable."""
        raise NotImplementedError


class StepKind(enum.Enum):
    """The four step shapes a module class can classify as."""

    STATELESS = "stateless"
    ASYNC_STATELESS = "async-stateless"
    MEALY = "mealy"
    FOLD = "fold"


@dataclass(frozen=True)
class StepSpec:
    """Definition-time classification of one module class (spec §7)."""

    kind: StepKind
    in_type: type[Any]
    out_type: type[Any]
    state_type: type[Any] | None
    skips: bool
    owner: type[Any]

    @property
    def impl_name(self) -> str:
        """``"fold"`` if kind is FOLD else ``"step"``."""
        raise NotImplementedError

    @property
    def arity(self) -> int:
        """Data-parameter count: 2 if MEALY else 1."""
        raise NotImplementedError

    @property
    def is_async(self) -> bool:
        """Whether kind is ASYNC_STATELESS."""
        raise NotImplementedError


def classify(cls: type[Any]) -> StepSpec:
    """Classify + validate a module class; pure, raises on any violation."""
    raise NotImplementedError


def step_spec(cls: type[Any]) -> StepSpec:
    """Return the stored StepSpec from cls's own dict, else ``[not-classified]``."""
    raise NotImplementedError


def _find_owner(cls: type[Any], name: str) -> type[Any] | None:
    """First class in cls.__mro__ whose own dict defines ``name`` (spec G0)."""
    raise NotImplementedError


def _resolve_step_hints(fn: types.FunctionType, owner: type[Any], cls: type[Any]) -> dict[str, Any]:
    """get_type_hints with owner-body localns + owner-name injection (spec §4.1)."""
    raise NotImplementedError


def _split_optional(tp: Any) -> tuple[Any, bool]:
    """Strip ``| None`` in any spelling; return (inner, had_none) (spec §5)."""
    raise NotImplementedError


def _require_row(cls: type[Any], tp: Any, base: type[Any], where: str) -> type[Any]:
    """Assert ``tp`` is a class subclassing ``base`` (pm.In/pm.Out), else fail."""
    raise NotImplementedError


def _fail(cls: type[Any], rule: Rule, message: str) -> NoReturn:
    """Raise PureModuleDefinitionError with the formatted, slug-suffixed message."""
    raise NotImplementedError

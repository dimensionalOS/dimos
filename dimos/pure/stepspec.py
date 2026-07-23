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

``classify(cls)`` is pure and structural:
it inspects a module class's ``step``/``fold`` signature (never bodies, never
instances) and returns a ``StepSpec`` or raises ``PureModuleDefinitionError``.
T2's ``PureModule.__init_subclass__`` stores the result at ``__pure_step__``.

Gate order (= deterministic error precedence): G0 candidate discovery, G1
function kind, G2 parameter shape, G3 arity x async -> kind, G4 annotation
resolution, G5 per-kind validation plus the State/bundle coherence rules.
Purity guarantees (spec §6): no instantiation, no descriptor execution (raw
``vars`` access of the five reserved names only), no mutation of ``cls``.

Layering (spec §1): the only intra-package import is ``dimos.pure.rows``
(T1's bases, for the row checks) — never the ``dimos.pure`` package surface
(``module.py`` imports this file and ``__init__.py`` imports ``module.py``;
importing the surface from here would cycle).
"""

from __future__ import annotations

import collections.abc
from dataclasses import dataclass
import enum
import inspect
import types
import typing
from typing import Any, Final, NoReturn, TypeVar, cast, get_args, get_origin

from dimos.pure.rows import In, Out

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
    IN_FIELD_UNSAMPLED = "in-field-unsampled"
    FINISH_NOT_MEALY = "finish-not-mealy"
    FINISH_NOT_FUNCTION = "finish-not-function"
    FINISH_PARAMS = "finish-params"
    FINISH_UNANNOTATED = "finish-unannotated"
    FINISH_STATE = "finish-state"
    FINISH_RETURN = "finish-return"
    NOT_CLASSIFIED = "not-classified"


class PureModuleDefinitionError(TypeError):
    """A PureModule subclass violated a definition-time shape rule."""

    rule: Rule | None

    def __init__(self, message: str, rule: Rule | None = None) -> None:
        """Message is release copy per spec §9; ``rule`` is machine-readable.

        ``rule`` defaults to None so subsystems reusing this type (T2's
        ``ConfigFieldError`` re-bases onto it) can raise
        with a message alone; every T3-raised instance carries its Rule.
        """
        super().__init__(message)
        self.rule = rule


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
    has_finish: bool = False  # a Mealy class defines the optional finish() tail-flush hook

    @property
    def impl_name(self) -> str:
        """``"fold"`` if kind is FOLD else ``"step"``."""
        return "fold" if self.kind is StepKind.FOLD else "step"

    @property
    def arity(self) -> int:
        """Data-parameter count: 2 if MEALY else 1."""
        return 2 if self.kind is StepKind.MEALY else 1

    @property
    def is_async(self) -> bool:
        """Whether kind is ASYNC_STATELESS."""
        return self.kind is StepKind.ASYNC_STATELESS


_NONE_TYPE: Final = type(None)
_UNION_ORIGINS: Final = (typing.Union, types.UnionType)
_AWAITABLE_ORIGINS: Final = (collections.abc.Awaitable, collections.abc.Coroutine)
_PARAM_KIND_LABELS: Final[dict[Any, str]] = {
    inspect.Parameter.VAR_POSITIONAL: "*args",
    inspect.Parameter.VAR_KEYWORD: "**kwargs",
    inspect.Parameter.KEYWORD_ONLY: "keyword-only",
}
_RETURNS_NOTHING = (
    "step returns None — a module that never emits has no outputs to wire. Return an "
    "Out row; use -> Out | None if some ticks skip; effects are outputs too."
)


def classify(cls: type[Any]) -> StepSpec:
    """Classify + validate a module class; pure, raises on any violation."""
    # ── G0 — candidate discovery ─────────────────────────────────────────────
    step_owner = _find_owner(cls, "step")
    fold_owner = _find_owner(cls, "fold")
    if step_owner is None and fold_owner is None:
        _fail(
            cls,
            Rule.STEP_MISSING,
            "defines neither step nor fold. A pure module declares exactly one: "
            "def step(self, i: In) -> Out for per-tick, or "
            "def fold(self, rows: Iterator[In]) -> Iterator[Out] for whole-stream. "
            "An abstract shape declares its In/Out and a step whose body raises "
            "NotImplementedError.",
        )
    if step_owner is not None and fold_owner is not None:
        _fail(
            cls,
            Rule.STEP_AND_FOLD,
            f"defines both step (from {step_owner.__qualname__}) and fold (from "
            f"{fold_owner.__qualname__}) — a module is per-tick or whole-stream, never "
            f"both. Remove one (rename it if it is a helper: step, fold, In, Out and "
            f"State are reserved).",
        )
    if step_owner is not None:
        name, owner = "step", step_owner
    else:
        assert fold_owner is not None
        name, owner = "fold", fold_owner

    # ── G1 — function kind ───────────────────────────────────────────────────
    fn = vars(owner)[name]
    if isinstance(fn, staticmethod):
        _fail(
            cls,
            Rule.STEP_NOT_FUNCTION,
            f"{name} must be a plain instance method — def {name}(self, i: In) -> Out — "
            f"not a staticmethod. It needs self for config and resources.",
        )
    if isinstance(fn, classmethod):
        _fail(
            cls,
            Rule.STEP_NOT_FUNCTION,
            f"{name} must be a plain instance method — def {name}(self, i: In) -> Out — "
            f"not a classmethod.",
        )
    if not isinstance(fn, types.FunctionType):
        _fail(
            cls,
            Rule.STEP_NOT_FUNCTION,
            f"{name} must be a function defined in the class body, got "
            f"{type(fn).__name__}. (step, fold, In, Out and State are reserved member "
            f"names.)",
        )

    # ── G2 — parameter shape ─────────────────────────────────────────────────
    params = list(inspect.signature(fn).parameters.values())
    if not params:
        _fail(
            cls,
            Rule.STEP_NO_SELF,
            f"{name} takes no parameters — expected def {name}(self, i: In) -> Out.",
        )
    for p in params:
        label = _PARAM_KIND_LABELS.get(p.kind)
        if label is not None:
            _fail(
                cls,
                Rule.STEP_PARAMS,
                f"{name} parameter '{p.name}' is {label}; {name} parameters must be plain "
                f"positional: (self, i) or (self, state, i). The engine calls {name} "
                f"positionally.",
            )
    for p in params:
        if p.default is not inspect.Parameter.empty:
            _fail(
                cls,
                Rule.STEP_PARAM_DEFAULT,
                f"{name} parameter '{p.name}' has a default. The engine always passes "
                f"every argument, and defaults change how annotations resolve across "
                f"Python versions — remove it.",
            )
    data_params = params[1:]

    # ── G3 — arity x async -> kind ───────────────────────────────────────────
    is_coro = inspect.iscoroutinefunction(fn)
    is_agen = inspect.isasyncgenfunction(fn)
    n = len(data_params)
    kind: StepKind
    if name == "step":
        if is_agen:
            _fail(
                cls,
                Rule.STEP_NOT_FUNCTION,
                "step is an async generator — a step maps one tick to one result. For "
                "whole-stream processing write fold (sync); async concurrency stays "
                "per-tick: async def step(self, i: In) -> Out.",
            )
        if n == 0:
            _fail(
                cls,
                Rule.STEP_ARITY,
                f"step takes no input row — expected def step(self, i: In) -> "
                f"Out.{_maybe_forgot_self(fn, owner, cls, params[0])}",
            )
        if n >= 3:
            _fail(
                cls,
                Rule.STEP_ARITY,
                f"step takes {n} inputs — a step takes exactly one In row (Mealy: state "
                f"first, then the row). Multiple inputs are fields of one In bundle: "
                f"class In(pm.In) with one field per input.",
            )
        if n == 2 and is_coro:
            _fail(
                cls,
                Rule.ASYNC_MEALY,
                "async def step cannot take State — in-flight ticks would race the state "
                "thread. Make step sync, or split: an async stateless module for the I/O "
                "feeding a sync Mealy module for the state.",
            )
        if n == 2:
            kind = StepKind.MEALY
        elif is_coro:
            kind = StepKind.ASYNC_STATELESS
        else:
            kind = StepKind.STATELESS
    else:
        if n != 1:
            _fail(
                cls,
                Rule.FOLD_ARITY,
                f"fold takes {n} data parameters — expected exactly def fold(self, rows: "
                f"Iterator[In]) -> Iterator[Out]. Fold state lives in generator locals, "
                f"not parameters.",
            )
        if is_coro or is_agen:
            _fail(
                cls,
                Rule.FOLD_ASYNC,
                "fold cannot be async — fold owns its loop synchronously. Async "
                "concurrency is a per-tick policy: async def step(self, i: In) -> Out.",
            )
        kind = StepKind.FOLD

    # ── G4 — annotation resolution ───────────────────────────────────────────
    hints = _resolve_step_hints(fn, owner, cls, name=name)
    for p in data_params:
        if p.name not in hints:
            _fail(
                cls,
                Rule.STEP_UNANNOTATED,
                f"{name} parameter '{p.name}' has no annotation. The {name} signature is "
                f"the module's whole contract — annotate it with the row type.",
            )
    if "return" not in hints:
        _fail(
            cls,
            Rule.STEP_UNANNOTATED,
            f"{name} has no return annotation. The {name} signature is the module's "
            f"whole contract — annotate the return: -> Out, -> Out | None, or "
            f"tuple[State, Out] for Mealy.",
        )

    # ── G5 — per-kind annotation validation ──────────────────────────────────
    state_type: type[Any] | None = None
    skips = False
    if kind is StepKind.MEALY:
        in_type, out_type, state_type, skips = _check_mealy(cls, owner, hints, data_params)
    elif kind is StepKind.FOLD:
        in_type, out_type = _check_fold(cls, hints, data_params)
    else:
        in_type = _require_row(cls, hints[data_params[0].name], In, data_params[0].name)
        out_type, skips = _check_step_return(cls, hints["return"], kind)

    # §5.5 — State coherence: own-body State on a non-Mealy kind is dead weight.
    if kind is not StepKind.MEALY and "State" in vars(cls):
        if kind is StepKind.FOLD:
            _fail(
                cls,
                Rule.FOLD_STATE,
                "declares State but fold keeps its state in generator locals — "
                "engine-threaded State is a step affair. Delete State; if you need "
                "checkpointable state, respell as a Mealy step.",
            )
        word = "async stateless" if kind is StepKind.ASYNC_STATELESS else "stateless"
        _fail(
            cls,
            Rule.STATE_UNUSED,
            f"declares State but step is {word} — State is threaded only through a Mealy "
            f"step: def step(self, state: State, i: In) -> tuple[State, Out]. Take it or "
            f"delete it.",
        )

    # §2.4 (T13) — a module step's In bundle must sample every field; a BareSpec
    # field (specifier-less, legal only on graph rims) fails at the module class
    # statement, the same loudness the old bundle-time check gave.
    for fname, fspec in in_type.fields().items():
        if fspec.kind == "bare":
            _fail(
                cls,
                Rule.IN_FIELD_UNSAMPLED,
                f"In field {fname!r} needs a sampler specifier "
                f"(tick()/latest()/interpolate()); plain defaults are only valid on Out bundles",
            )

    # §5.6 — bundle coherence: a nested declaration the signature doesn't use.
    for attr, used in (("In", in_type), ("Out", out_type)):
        if attr in vars(cls) and vars(cls)[attr] is not used:
            if owner is cls:
                _fail(
                    cls,
                    Rule.BUNDLE_SHADOWED,
                    f"declares a nested {attr} but {name} reads {_type_repr(used)} — the "
                    f"{name} signature is the single typing authority, so the nested "
                    f"declaration is dead. Use it in {name} or delete it.",
                )
            _fail(
                cls,
                Rule.BUNDLE_SHADOWED,
                f"declares {attr} but inherits {name} from {owner.__qualname__}, which is "
                f"typed against {owner.__qualname__}.{attr}. Override {name} or remove "
                f"the declaration.",
            )

    # finish() — the optional Mealy tail-flush hook (ACCEPTED design, api_improvements.md).
    has_finish = _check_finish(cls, kind, out_type, state_type)

    return StepSpec(
        kind=kind,
        in_type=in_type,
        out_type=out_type,
        state_type=state_type,
        skips=skips,
        owner=owner,
        has_finish=has_finish,
    )


def step_spec(cls: type[Any]) -> StepSpec:
    """Return the stored StepSpec from cls's own dict, else ``[not-classified]``."""
    ns = vars(cls)
    if PURE_STEP_ATTR not in ns:
        _fail(
            cls,
            Rule.NOT_CLASSIFIED,
            "has no step spec — it was not validated as a PureModule subclass. Only "
            "classes that ran PureModule.__init_subclass__ carry one.",
        )
    return cast("StepSpec", ns[PURE_STEP_ATTR])


def _find_owner(cls: type[Any], name: str) -> type[Any] | None:
    """First class in cls.__mro__ whose own dict defines ``name`` (spec G0)."""
    for klass in cls.__mro__:
        if name in vars(klass):
            return klass
    return None


def _resolve_step_hints(
    fn: types.FunctionType, owner: type[Any], cls: type[Any], *, name: str = "step"
) -> dict[str, Any]:
    """get_type_hints with owner-body localns + owner-name injection (spec §4.1)."""
    localns: dict[str, Any] = {**vars(owner), owner.__name__: owner}
    try:
        return typing.get_type_hints(fn, localns=localns)
    except Exception as e:
        failing = getattr(e, "name", None)
        if failing in ("In", "Out", "State"):
            lesson = (
                f"Bare {failing} resolves only inside the class body that defines it — "
                f"class attributes are not lexically scoped. In a subclass write "
                f"{owner.__name__}.{failing} (or redeclare the bundle in this class body)."
            )
        else:
            who = f"'{failing}'" if failing is not None else "a name in the annotation"
            lesson = (
                f"If {who} is imported under TYPE_CHECKING or inside a function, move it "
                f"to a module-level runtime import — deferred annotations cannot see "
                f"function locals or checking-only imports."
            )
        raise PureModuleDefinitionError(
            f"{_clspath(cls)}: cannot resolve {name}'s annotations: {e!r}.\n"
            f"{lesson}\n"
            f"Annotations resolve at import time against the defining module's globals "
            f"plus the defining class body. [{Rule.STEP_UNRESOLVABLE.value}]",
            Rule.STEP_UNRESOLVABLE,
        ) from e


def _maybe_forgot_self(
    fn: types.FunctionType, owner: type[Any], cls: type[Any], receiver: inspect.Parameter
) -> str:
    """Best-effort forgot-self hint for the zero-row arity error (spec G3)."""
    try:
        first = _resolve_step_hints(fn, owner, cls).get(receiver.name)
        if isinstance(first, type) and issubclass(first, In):
            return f" (Its first parameter '{receiver.name}' is a pm.In row — did you forget self?)"
    except Exception:  # resolution failure here is swallowed (spec G3)
        pass
    return ""


def _check_step_return(cls: type[Any], r: Any, kind: StepKind) -> tuple[type[Any], bool]:
    """Validate a (async) stateless return annotation -> (out_type, skips) (spec §5.1/§5.2)."""
    if r is _NONE_TYPE:
        _fail(cls, Rule.STEP_RETURNS_NOTHING, _RETURNS_NOTHING)
    out_ann, skips = _split_optional(r)
    if get_origin(out_ann) in _UNION_ORIGINS:
        _fail(
            cls,
            Rule.OUT_UNION,
            f"step returns {_type_repr(r)} — a step returns exactly one Out row type, "
            f"plus | None if some ticks skip. Alternative results are sparse fields of "
            f"one Out bundle.",
        )
    if get_origin(out_ann) in _AWAITABLE_ORIGINS:
        if kind is StepKind.ASYNC_STATELESS:
            _fail(
                cls,
                Rule.STEP_RETURNS_AWAITABLE,
                f"async step's return annotation is the awaited result — write -> Out, "
                f"not -> {_type_repr(out_ann)}.",
            )
        _fail(
            cls,
            Rule.STEP_RETURNS_AWAITABLE,
            f"step returns {_type_repr(out_ann)} but is not async. One spelling per "
            f"capability: declare it async def step(self, i: In) -> Out and return the "
            f"row.",
        )
    return _require_row(cls, out_ann, Out, "return"), skips


def _check_mealy(
    cls: type[Any],
    owner: type[Any],
    hints: dict[str, Any],
    data_params: list[inspect.Parameter],
) -> tuple[type[Any], type[Any], type[Any], bool]:
    """Validate the Mealy shape -> (in_type, out_type, state_type, skips) (spec §5.3)."""
    state_owner = _find_owner(cls, "State")
    if state_owner is None:
        _fail(
            cls,
            Rule.MEALY_NO_STATE,
            f"step takes (state, i) but {_clspath(cls)} declares no State. Declare class "
            f"State(NamedTuple) with a default for every field — a run starts from "
            f"State().",
        )
    state_cls = vars(state_owner)["State"]
    if isinstance(state_cls, type) and issubclass(state_cls, (In, Out)):
        base_name = "pm.In" if issubclass(state_cls, In) else "pm.Out"
        _fail(
            cls,
            Rule.STATE_IS_ROW,
            f"State is a row bundle (subclass of {base_name}) — State is engine-threaded "
            f"plain data (e.g. NamedTuple); rows are dataflow. Give State its own class.",
        )
    state_ann = hints[data_params[0].name]
    if state_ann is not state_cls:
        first_is_row = isinstance(state_ann, type) and issubclass(state_ann, In)
        if hints[data_params[1].name] is state_cls and first_is_row:
            _fail(
                cls,
                Rule.STATE_MISMATCH,
                "step's parameters are swapped — Mealy order is "
                "def step(self, state: State, i: In).",
            )
        if owner is not cls and "State" in vars(cls):
            _fail(
                cls,
                Rule.STATE_MISMATCH,
                f"declares State but inherits step from {owner.__qualname__}, which is "
                f"typed against {owner.__qualname__}.State. Override step or remove the "
                f"State declaration.",
            )
        _fail(
            cls,
            Rule.STATE_MISMATCH,
            f"step's state parameter is {_type_repr(state_ann)} but {_clspath(cls)}.State "
            f"is {_type_repr(state_cls)} — the state parameter must be the class's own "
            f"State.",
        )
    in_type = _require_row(cls, hints[data_params[1].name], In, data_params[1].name)

    r = hints["return"]
    args = get_args(r)
    if get_origin(r) is not tuple or len(args) != 2 or Ellipsis in args:
        _fail(
            cls,
            Rule.MEALY_RETURN,
            f"a Mealy step returns tuple[State, Out] (or tuple[State, Out | None] to "
            f"skip), got {_type_repr(r)}.",
        )
    if args[0] is not state_cls:
        _fail(
            cls,
            Rule.MEALY_RETURN,
            f"Mealy step's return tuple carries {_type_repr(args[0])} first, but the "
            f"state slot must be {_clspath(cls)}.State ({_type_repr(state_cls)}).",
        )
    slot = args[1]
    # builtin tuple[State, None] keeps the literal None in get_args (no
    # NoneType normalization, unlike typing.Tuple) — treat both spellings.
    if slot is _NONE_TYPE or slot is None:
        _fail(cls, Rule.STEP_RETURNS_NOTHING, _RETURNS_NOTHING)
    out_ann, skips = _split_optional(slot)
    if get_origin(out_ann) in _UNION_ORIGINS:
        _fail(
            cls,
            Rule.OUT_UNION,
            f"step returns {_type_repr(slot)} — a step returns exactly one Out row type, "
            f"plus | None if some ticks skip. Alternative results are sparse fields of "
            f"one Out bundle.",
        )
    out_type = _require_row(cls, out_ann, Out, "return")

    # §5.3.6 — default-constructibility by inspection, never by calling; skipped
    # when the signature itself is undeterminable (permissive on undeterminable).
    try:
        state_sig = inspect.signature(state_cls)
    except Exception:
        state_sig = None
    if state_sig is not None:
        for p in state_sig.parameters.values():
            if p.kind in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD):
                continue
            if p.default is inspect.Parameter.empty:
                _fail(
                    cls,
                    Rule.STATE_NOT_DEFAULT_CONSTRUCTIBLE,
                    f"State() must construct with no arguments — a run starts from "
                    f"State(), and field '{p.name}' has no default. Give every State "
                    f"field a default.",
                )
    return in_type, out_type, cast("type[Any]", state_cls), skips


_FINISH_KIND_WORD: Final[dict[StepKind, str]] = {
    StepKind.STATELESS: "stateless",
    StepKind.ASYNC_STATELESS: "async stateless",
    StepKind.FOLD: "fold",
}


def _check_finish(
    cls: type[Any], kind: StepKind, out_type: type[Any], state_type: type[Any] | None
) -> bool:
    """Validate the optional Mealy ``finish(self, s: State) -> Out | None`` hook (§finish)."""
    owner = _find_owner(cls, "finish")
    if owner is None:
        return False
    if kind is not StepKind.MEALY:
        _fail(
            cls,
            Rule.FINISH_NOT_MEALY,
            f"defines finish but step is {_FINISH_KIND_WORD[kind]} — finish is the Mealy "
            f"tail-flush hook (def finish(self, s: State) -> Out | None), called once when "
            f"the tick stream ends. A fold already flushes its tail from generator locals; "
            f"a stateless step accumulates nothing to flush.",
        )
    assert state_type is not None  # a MEALY spec always carries State (§5.3)
    fn = vars(owner)["finish"]
    if isinstance(fn, (staticmethod, classmethod)) or not isinstance(fn, types.FunctionType):
        _fail(
            cls,
            Rule.FINISH_NOT_FUNCTION,
            "finish must be a plain instance method — def finish(self, s: State) -> "
            "Out | None — it reads config and resources through self.",
        )
    if (
        inspect.iscoroutinefunction(fn)
        or inspect.isasyncgenfunction(fn)
        or inspect.isgeneratorfunction(fn)
    ):
        _fail(
            cls,
            Rule.FINISH_NOT_FUNCTION,
            "finish cannot be async or a generator — it is called once at stream end and "
            "returns one optional Out row: def finish(self, s: State) -> Out | None.",
        )
    params = list(inspect.signature(fn).parameters.values())
    for p in params:
        label = _PARAM_KIND_LABELS.get(p.kind)
        if label is not None:
            _fail(
                cls,
                Rule.FINISH_PARAMS,
                f"finish parameter '{p.name}' is {label}; finish takes exactly "
                f"(self, s: State), plain positional. The engine calls it positionally.",
            )
        if p.default is not inspect.Parameter.empty:
            _fail(
                cls,
                Rule.FINISH_PARAMS,
                f"finish parameter '{p.name}' has a default — the engine always passes the "
                f"final State; remove it.",
            )
    data_params = params[1:]
    if len(data_params) != 1:
        _fail(
            cls,
            Rule.FINISH_PARAMS,
            f"finish takes {len(data_params)} data parameter(s) — expected exactly "
            f"def finish(self, s: State) -> Out | None (the final State only, no input row).",
        )
    hints = _resolve_step_hints(fn, owner, cls, name="finish")
    sp = data_params[0]
    if sp.name not in hints:
        _fail(
            cls,
            Rule.FINISH_UNANNOTATED,
            f"finish parameter '{sp.name}' has no annotation — annotate it with the "
            f"module's State: def finish(self, s: State) -> Out | None.",
        )
    if "return" not in hints:
        _fail(
            cls,
            Rule.FINISH_UNANNOTATED,
            "finish has no return annotation — annotate it -> Out | None (None when there "
            "is no tail to flush).",
        )
    if hints[sp.name] is not state_type:
        _fail(
            cls,
            Rule.FINISH_STATE,
            f"finish's state parameter is {_type_repr(hints[sp.name])} but {_clspath(cls)}"
            f".State is {_type_repr(state_type)} — finish receives the class's own State "
            f"(bare State inside the defining body; {_clspath(cls)}.State in a subclass).",
        )
    inner, _ = _split_optional(hints["return"])
    if inner is not out_type:
        _fail(
            cls,
            Rule.FINISH_RETURN,
            f"finish returns {_type_repr(hints['return'])} — a finish returns "
            f"{out_type.__qualname__} | None (the step's Out, or None to flush nothing).",
        )
    return True


def _check_fold(
    cls: type[Any], hints: dict[str, Any], data_params: list[inspect.Parameter]
) -> tuple[type[Any], type[Any]]:
    """Validate the fold shape -> (in_type, out_type) (spec §5.4)."""
    rows_ann = hints[data_params[0].name]
    if (
        get_origin(rows_ann) not in (collections.abc.Iterator, collections.abc.Iterable)
        or len(get_args(rows_ann)) != 1
    ):
        _fail(
            cls,
            Rule.FOLD_ROWS_PARAM,
            f"fold's rows parameter is {_type_repr(rows_ann)} — expected Iterator[In] "
            f"(Iterable[In] also accepted). The driver hands fold one lazy pass over the "
            f"aligned rows.",
        )
    in_type = _require_row(cls, get_args(rows_ann)[0], In, data_params[0].name, impl="fold")

    r = hints["return"]
    r_origin = get_origin(r)
    r_args = get_args(r)
    if r_origin is collections.abc.Iterable and len(r_args) == 1:
        _fail(
            cls,
            Rule.FOLD_RETURN,
            f"fold declares -> Iterable[{_type_repr(r_args[0])}] — declaring Iterable "
            f"does not promise an Iterator, and the driver consumes a single pass. "
            f"Declare -> Iterator[{_type_repr(r_args[0])}].",
        )
    if r_origin is collections.abc.Iterator and len(r_args) == 1:
        element = r_args[0]
    elif r_origin is collections.abc.Generator and len(r_args) == 3:
        element = r_args[0]
    else:
        _fail(
            cls,
            Rule.FOLD_RETURN,
            f"fold must return Iterator[Out] (a generator function qualifies), got "
            f"{_type_repr(r)}.",
        )
    out_type = _require_row(cls, element, Out, "return", impl="fold")
    return in_type, out_type


def _split_optional(tp: Any) -> tuple[Any, bool]:
    """Strip ``| None`` in any spelling; return (inner, had_none) (spec §5).

    A union with two or more non-None members is not an optional and is
    returned unchanged (the caller's union error rows handle it).
    """
    if get_origin(tp) in _UNION_ORIGINS:
        non_none = tuple(a for a in get_args(tp) if a is not _NONE_TYPE)
        if len(non_none) == 1:
            return non_none[0], True
    return tp, False


def _require_row(
    cls: type[Any], tp: Any, base: type[Any], where: str, *, impl: str = "step"
) -> type[Any]:
    """Assert ``tp`` is a class subclassing ``base`` (pm.In/pm.Out), else fail."""
    side_in = base is In
    rule = Rule.IN_NOT_ROW if side_in else Rule.OUT_NOT_ROW
    row = "pm.In" if side_in else "pm.Out"
    head = f"{impl} input '{where}' is" if side_in else f"{impl} returns"
    if isinstance(tp, TypeVar):
        _fail(
            cls,
            rule,
            f"{head} a TypeVar ({_type_repr(tp)}) — module classes are not generic by "
            f"design; annotate with a concrete {row} row.",
        )
    if get_origin(tp) in _UNION_ORIGINS:
        if side_in:
            _fail(
                cls,
                rule,
                f"{head} a union ({_type_repr(tp)}) — a {impl} takes exactly one In row "
                f"type. Optional inputs are latest(default=None) fields inside the "
                f"bundle, not unions of bundles.",
            )
        _fail(
            cls,
            rule,
            f"{head} a union ({_type_repr(tp)}) — a {impl} returns exactly one Out row "
            f"type. Alternative results are sparse fields of one Out bundle.",
        )
    if get_origin(tp) is not None:
        _fail(
            cls,
            rule,
            f"{head} a parameterized alias ({_type_repr(tp)}) — rows are plain classes; "
            f"annotate with the row class itself.",
        )
    if not (isinstance(tp, type) and issubclass(tp, base)):
        if side_in:
            _fail(
                cls,
                rule,
                f"{head} {_type_repr(tp)}, not a pm.In row. Declare class In(pm.In) with "
                f"one field per input and annotate {impl} with it (bare In inside the "
                f"defining body; {_clspath(cls)}.In elsewhere).",
            )
        _fail(
            cls,
            rule,
            f"{head} {_type_repr(tp)}, not a pm.Out row. Declare class Out(pm.Out) and "
            f"construct it in {impl} — effects are outputs too.",
        )
    return tp


def _clspath(cls: type[Any]) -> str:
    """``{module}.{qualname}`` — the ``{X}`` of every catalog message (spec §9.2)."""
    return f"{cls.__module__}.{cls.__qualname__}"


def _type_repr(tp: Any) -> str:
    """Render an annotation for a message: class qualname, else repr."""
    if isinstance(tp, type):
        return tp.__qualname__
    return repr(tp)


def _fail(cls: type[Any], rule: Rule, message: str) -> NoReturn:
    """Raise PureModuleDefinitionError with the formatted, slug-suffixed message."""
    raise PureModuleDefinitionError(f"{_clspath(cls)}: {message} [{rule.value}]", rule)

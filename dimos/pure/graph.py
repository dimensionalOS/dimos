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

"""PureGraph: compose pure modules into validated DAGs (T13).

Spec: ``dimos/pure/tasks/t13-graph.md``. ``wire(self, i: In) -> Out`` runs once,
symbolically, at build: ``i.x`` is a ``Port`` riding under the payload's static
type, application returns the member's Out as ports, and the build validates the
wiring (§5 catalog) before anything runs. One ``wire()``, three targets:
``over()`` (local), ``blueprint()`` (coordinator lowering), ``partition()``
(future native). Engine layer — module scope imports the data layer only;
drivers/legacy/memory2 load lazily inside the entry points.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping, Sequence
import dataclasses
import enum
import inspect
from itertools import tee
import re
import sys
import threading
import types
from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    Final,
    Generic,
    NoReturn,
    Protocol,
    TypeVar,
    Union,
    cast,
    get_args,
    get_origin,
)
import warnings

from typing_extensions import dataclass_transform

from dimos.pure.config import (
    FrozenModuleError,
    PureModuleConfig,
    _collect_config_fields,
    _synthesize_config_model,
)
from dimos.pure.drivers import PureModuleRunError, RunHooks, run_over
from dimos.pure.rows import UNSTAMPED, In as PmIn, Out as PmOut, TfSpec, TickSpec
from dimos.pure.stepspec import (
    PureModuleDefinitionError,
    _find_owner,
    _resolve_step_hints,
)
from dimos.pure.typing import Streamable

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

__all__ = [
    "Binding",
    "EdgeStream",
    "GraphPlan",
    "GraphRule",
    "GraphRun",
    "GraphRunRule",
    "GraphSpec",
    "Port",
    "PortRef",
    "PureGraph",
    "PureGraphDefinitionError",
    "PureGraphRunError",
    "Wires",
    "apply_symbolic",
    "feedback",
    "named",
]

_IMPL_PENDING: Final = "T13 impl pending (see dimos/pure/tasks/t13-graph.md)"

# ── type variables (T4 doctrine: variant declaration pair + invariant twins) ──

TIn = TypeVar("TIn", contravariant=True)
TOut = TypeVar("TOut", covariant=True)
_TIn = TypeVar("_TIn")
_TOut = TypeVar("_TOut")
_TMsg = TypeVar("_TMsg")  # a port's payload type
_TModule = TypeVar("_TModule")


class Wires(Protocol[TIn, TOut]):
    """A graph's shape: wire applied once to a symbolic In row, returning Out."""

    def wire(self, i: TIn, /) -> TOut: ...


# ── errors (spec §5) ─────────────────────────────────────────────────────────


class GraphRule(enum.Enum):
    """Every definition/build-time graph rule, by its message slug (spec §5)."""

    WIRE_MISSING = "graph-wire-missing"
    STEP = "graph-step"
    WIRE_SHAPE = "graph-wire-shape"
    WIRE_UNANNOTATED = "graph-wire-unannotated"
    WIRE_UNRESOLVABLE = "graph-wire-unresolvable"
    IN_NOT_ROW = "graph-in-not-row"
    OUT_NOT_ROW = "graph-out-not-row"
    PORT_SAMPLER = "graph-port-sampler"
    PORT_DEFAULT = "graph-port-default"
    APPLY_CONTEXT = "graph-apply-context"
    UNKNOWN_KWARG = "graph-unknown-kwarg"
    NOT_A_PORT = "graph-not-a-port"
    UNBOUND_INPUT = "graph-unbound-input"
    TYPE_MISMATCH = "graph-type-mismatch"
    TICK_CYCLE = "graph-tick-cycle"
    TF_UNEXPECTED = "graph-tf-unexpected"
    TF_MISSING = "graph-tf-missing"
    UNCLOSED_FEEDBACK = "graph-unclosed-feedback"
    FEEDBACK_RECLOSED = "graph-feedback-reclosed"
    CLOSE_NOT_FEEDBACK = "graph-close-not-feedback"
    FOREIGN_PORT = "graph-foreign-port"
    DUPLICATE_NAME = "graph-duplicate-name"
    WIRE_RETURN = "graph-wire-return"
    UNSET_OUTPUT = "graph-unset-output"
    EXPORT_INPUT = "graph-export-input"
    EXPORT_ALIAS = "graph-export-alias"
    UNUSED_INPUT = "graph-unused-input"


class GraphRunRule(enum.Enum):
    """Every run-surface graph rule, by its message slug (spec §5)."""

    UNKNOWN_STREAM = "graph-unknown-stream"
    UNBOUND_RIM = "graph-unbound-rim"
    STORE_MISSING = "graph-store-missing"
    UNKNOWN_PATH = "graph-unknown-path"
    IMPL_PENDING = "graph-impl-pending"


class PureGraphDefinitionError(PureModuleDefinitionError):
    """A PureGraph subclass or its wire() violated a definition/build rule."""

    graph_rule: GraphRule | None

    def __init__(self, message: str, graph_rule: GraphRule | None = None) -> None:
        """Message is release copy per spec §5; ``graph_rule`` is machine-readable."""
        super().__init__(message, None)
        self.graph_rule = graph_rule


class PureGraphRunError(PureModuleRunError):
    """A graph run surface (over()/run handle) violated a runtime contract."""

    graph_rule: GraphRunRule | None

    def __init__(self, message: str, graph_rule: GraphRunRule | None = None) -> None:
        """Message is release copy per spec §5; ``graph_rule`` is machine-readable."""
        super().__init__(message, None)
        self.graph_rule = graph_rule


# ── message templates (release copy: spec §5) ────────────────────────────────


def _clspath(cls: type[Any]) -> str:
    """``{module}.{qualname}`` — the ``{g}``/``{cls}`` of every catalog message."""
    return f"{cls.__module__}.{cls.__qualname__}"


def _names(names: Sequence[str]) -> str:
    return ", ".join(repr(n) for n in names)


def _type_name(value: Any) -> str:
    """Render a bound value's type for a message."""
    return type(value).__qualname__


def _ann_name(ann: Any) -> str:
    """Render an annotation (payload type) for a message: class qualname, else repr."""
    if isinstance(ann, type):
        return ann.__qualname__
    return repr(ann)


def _strip_opt(ann: Any) -> Any:
    """Drop a single ``| None`` in any spelling; leave everything else as-is."""
    if get_origin(ann) in (types.UnionType, Union):
        non_none = tuple(a for a in get_args(ann) if a is not type(None))
        if len(non_none) == 1:
            return non_none[0]
    return ann


def _fail_def(cls: type[Any], rule: GraphRule, message: str) -> NoReturn:
    """Raise PureGraphDefinitionError with the formatted, slug-suffixed message."""
    raise PureGraphDefinitionError(f"{_clspath(cls)}: {message} [{rule.value}]", rule)


def _e_apply_context(cls: type[Any]) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(cls)}: applied to ports outside wire() — symbolic application only "
        f"means something during a graph build. To run on real streams use over(). "
        f"[graph-apply-context]",
        GraphRule.APPLY_CONTEXT,
    )


def _e_unknown_kwarg(
    g: type[Any], cls: type[Any], unknown: Sequence[str], declared: Sequence[str]
) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: wire applies {_clspath(cls)} with unknown port(s) "
        f"{_names(unknown)}; In fields are: {_names(declared)}. Application kwargs must "
        f"match the member's In field names (tf= is the reserved tf side channel). "
        f"[graph-unknown-kwarg]",
        GraphRule.UNKNOWN_KWARG,
    )


def _e_not_a_port(g: type[Any], cls: type[Any], field: str, value: Any) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: wire binds {cls.__qualname__}.{field} to {_type_name(value)} — "
        f"application binds ports, not values; pass a graph input (i.{field}), a member "
        f"output, or a feedback(). Constants ride module config. [graph-not-a-port]",
        GraphRule.NOT_A_PORT,
    )


def _e_close_not_a_port(value: Any) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"close() got {_type_name(value)} — close ties a feedback to a producer port; "
        f"pass a member application's output. [graph-not-a-port]",
        GraphRule.NOT_A_PORT,
    )


def _e_unbound_input(
    g: type[Any], cls: type[Any], missing: Sequence[str]
) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: wire applies {_clspath(cls)} without required port(s) "
        f"{_names(missing)} — bind each, or give the member field a default to make it "
        f"optional. [graph-unbound-input]",
        GraphRule.UNBOUND_INPUT,
    )


def _e_type_mismatch(
    g: type[Any], cls: type[Any], field: str, want: str, src: str, got: str
) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: wire binds {cls.__qualname__}.{field} (a {want} port) from "
        f"{src}, which carries {got} — edge and field payload types must agree. "
        f"[graph-type-mismatch]",
        GraphRule.TYPE_MISMATCH,
    )


def _e_tick_cycle(g: type[Any], cls: type[Any], field: str) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: feedback closes into {cls.__qualname__}.{field}, a tick() input "
        f"— a back edge through the trigger would define time by itself. Cycles are "
        f"legal only through a sampled input (latest/interpolate/tf). [graph-tick-cycle]",
        GraphRule.TICK_CYCLE,
    )


def _e_tf_unexpected(g: type[Any], cls: type[Any]) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: wire applies {_clspath(cls)} with tf= but its In bundle "
        f"declares no tf() fields — the tf stream feeds tf() samplers only. "
        f"[graph-tf-unexpected]",
        GraphRule.TF_UNEXPECTED,
    )


def _e_tf_missing(g: type[Any], cls: type[Any], fields: Sequence[str]) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: wire applies {_clspath(cls)}, whose In declares required tf() "
        f"field(s) {_names(fields)}, without tf= — bind a transform-carrying port, or "
        f"give the fields default= to make them optional. [graph-tf-missing]",
        GraphRule.TF_MISSING,
    )


def _e_unclosed_feedback(g: type[Any], n: int) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: {n} feedback port(s) never closed — every feedback() names a "
        f"back edge that close(...) must tie to a producer port exactly once. "
        f"[graph-unclosed-feedback]",
        GraphRule.UNCLOSED_FEEDBACK,
    )


def _e_feedback_reclosed() -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        "feedback port closed twice — every feedback closes exactly once; rebuild the "
        "graph for a different wiring. [graph-feedback-reclosed]",
        GraphRule.FEEDBACK_RECLOSED,
    )


def _e_close_not_feedback() -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        "close() on a non-feedback port — only feedback() ports close; ordinary edges "
        "are tied by application kwargs. [graph-close-not-feedback]",
        GraphRule.CLOSE_NOT_FEEDBACK,
    )


def _e_foreign_port(g: type[Any]) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: wire received a port minted by a different build — ports do "
        f"not survive across wire() runs; use only this build's i.* and application "
        f"results. [graph-foreign-port]",
        GraphRule.FOREIGN_PORT,
    )


def _e_duplicate_name(g: type[Any], name: str) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: member name {name!r} used twice — named() names must be unique "
        f"within a build. [graph-duplicate-name]",
        GraphRule.DUPLICATE_NAME,
    )


def _e_wire_return(g: type[Any], got: Any, out_type: type[Any]) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: wire returned {_type_name(got)} — construct and return "
        f"{out_type.__qualname__} (the declared Out bundle). [graph-wire-return]",
        GraphRule.WIRE_RETURN,
    )


def _e_unset_output(g: type[Any], field: str, value: Any) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: Out field {field!r} is not a wired port (got {_type_name(value)}) "
        f"— set every graph output from a member application's Out. [graph-unset-output]",
        GraphRule.UNSET_OUTPUT,
    )


def _e_export_input(g: type[Any], field: str, inp: str) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: Out field {field!r} exports rim input {inp!r} unchanged — a "
        f"graph output is a member's output; passthrough re-export is not wiring. "
        f"[graph-export-input]",
        GraphRule.EXPORT_INPUT,
    )


def _e_export_alias(g: type[Any], a: str, b: str) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: Out fields {a!r} and {b!r} both export the same producer port "
        f"— one export name per port (an edge has one topic when lowered). "
        f"[graph-export-alias]",
        GraphRule.EXPORT_ALIAS,
    )


def _e_unused_input(g: type[Any], names: Sequence[str]) -> PureGraphDefinitionError:
    return PureGraphDefinitionError(
        f"{_clspath(g)}: rim input(s) {_names(names)} are consumed by no member — delete "
        f"them, or wire them (an unused port is a wiring bug, the graph twin of an "
        f"unread variable). [graph-unused-input]",
        GraphRule.UNUSED_INPUT,
    )


def _e_unknown_stream(
    g: type[Any], unknown: Sequence[str], declared: Sequence[str]
) -> PureGraphRunError:
    return PureGraphRunError(
        f"{_clspath(g)}.over(): got unknown stream(s) {_names(unknown)}; rim In ports "
        f"are: {_names(declared)}. Stream kwargs must match the graph's In field names. "
        f"[graph-unknown-stream]",
        GraphRunRule.UNKNOWN_STREAM,
    )


def _e_unbound_rim(g: type[Any], names: Sequence[str]) -> PureGraphRunError:
    return PureGraphRunError(
        f"{_clspath(g)}.over(): rim input(s) {_names(names)} have no source — pass a "
        f"store carrying them by name, a named stream kwarg, or remap= a store channel "
        f"onto them. [graph-unbound-rim]",
        GraphRunRule.UNBOUND_RIM,
    )


def _e_store_missing(g: type[Any], chan: str, field: str) -> PureGraphRunError:
    return PureGraphRunError(
        f"{_clspath(g)}.over(): the store has no stream named {chan!r} for rim input "
        f"{field!r} — remap= it to the store's channel name, or pass {field}= "
        f"explicitly. [graph-store-missing]",
        GraphRunRule.STORE_MISSING,
    )


def _e_unknown_path(g: type[Any], path: str, available: Sequence[str]) -> PureGraphRunError:
    return PureGraphRunError(
        f"{_clspath(g)}: no edge {path!r} in this run — edges are addressed "
        f"'<member_path>.<field>'; available: {_names(available)}. [graph-unknown-path]",
        GraphRunRule.UNKNOWN_PATH,
    )


def _e_impl_pending(what: str) -> PureGraphRunError:
    return PureGraphRunError(
        f"{what} is not implemented yet (T13 Phase C) — see tasks/t13-graph.md §8. "
        f"[graph-impl-pending]",
        GraphRunRule.IMPL_PENDING,
    )


# ── ports (spec §3.2) ────────────────────────────────────────────────────────


@dataclasses.dataclass(frozen=True)
class PortRef:
    """Address of one producer port: member path (None = graph rim) + field name."""

    member: str | None
    field: str

    @property
    def path(self) -> str:
        """Dotted edge address (``voxel_mapper.global_map``; rim ports are bare)."""
        return self.field if self.member is None else f"{self.member}.{self.field}"


_FEEDBACK_REF: Final = PortRef(member=None, field="<feedback>")
"""Placeholder ref a feedback port carries until close() resolves its producer."""


class Port(Generic[_TMsg]):
    """Symbolic reference to one producer port; the build-time stand-in for its payload."""

    __slots__ = ("_closed_with", "_feedback", "_token", "annotation", "ref")

    def __init__(
        self,
        ref: PortRef,
        annotation: Any,
        *,
        feedback: bool = False,
        token: object | None = None,
    ) -> None:
        """Minted by the build (or by feedback()); user code never constructs ports."""
        self.ref = ref
        self.annotation = annotation
        self._feedback = feedback
        self._token = token
        self._closed_with: Port[Any] | None = None

    def close(self, producer: _TMsg) -> None:
        """Tie this feedback port to its producer port — exactly once (spec §4.3)."""
        if not self._feedback:
            raise _e_close_not_feedback()
        if self._closed_with is not None:
            raise _e_feedback_reclosed()
        if not isinstance(producer, Port):
            raise _e_close_not_a_port(producer)
        self._closed_with = producer

    def __repr__(self) -> str:
        """``Port(<path>: <payload>)``; feedback ports render their state."""
        ann = getattr(self.annotation, "__name__", None) or repr(self.annotation)
        if self._feedback:
            state = "closed" if self._closed_with is not None else "open"
            return f"Port(<feedback {state}>: {ann})"
        return f"Port({self.ref.path}: {ann})"


def feedback() -> Port[Any]:
    """Declare a sampled back edge before its producer exists; close() ties it once."""
    port: Port[Any] = Port(_FEEDBACK_REF, None, feedback=True)
    ctx = _current_build()
    if ctx is not None:  # inside a build: stamp the token and register for close-out
        port._token = ctx.token
        ctx.feedbacks_open.append(port)
    return port


def named(name: str, module: _TModule) -> _TModule:
    """Name ``module``'s next application in the current build (default: snake_case)."""
    ctx = _current_build()
    if ctx is not None:
        ctx.pending_names[id(module)] = name
    return module


def snake_case(name: str) -> str:
    """CamelCase class name to snake_case member name (``VoxelMapper`` → ``voxel_mapper``)."""
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])", "_", name).lower()


# ── the build product (spec §4.5) ────────────────────────────────────────────


@dataclasses.dataclass(frozen=True)
class Binding:
    """One wired edge end: producer port → consumer member In field."""

    src: PortRef
    dst: PortRef


@dataclasses.dataclass(frozen=True)
class GraphSpec:
    """Definition-time classification of one PureGraph subclass (spec §2.1)."""

    in_type: type[Any]
    out_type: type[Any]
    owner: type[Any]


@dataclasses.dataclass(frozen=True)
class GraphPlan:
    """A validated build: members, edges, rim — a value; equality is graph equality."""

    graph: type[Any]
    members: tuple[tuple[str, Any], ...]
    bindings: tuple[Binding, ...]
    tf_bindings: tuple[Binding, ...]
    exports: tuple[tuple[str, PortRef], ...]
    inputs: tuple[tuple[str, Any], ...]
    feedbacks: tuple[Binding, ...]


# ── class-statement gate copy (spec §5; helpers below are the single source) ──


def _param_kind_label(p: inspect.Parameter) -> str | None:
    """A disallowed parameter-kind label, or None if the parameter is plain positional."""
    if p.kind is inspect.Parameter.VAR_POSITIONAL:
        return "*args"
    if p.kind is inspect.Parameter.VAR_KEYWORD:
        return "**kwargs"
    if p.kind is inspect.Parameter.KEYWORD_ONLY:
        return "keyword-only"
    return None


def classify_graph(cls: type[Any]) -> GraphSpec:
    """Classify + validate a graph class (gates G0..G5); pure, raises on violation."""
    # ── G0 — discovery: wire over the MRO's own dicts; step/fold forbidden ──
    wire_owner = _find_owner(cls, "wire")
    if wire_owner is None:
        _fail_def(
            cls,
            GraphRule.WIRE_MISSING,
            "defines no wire. A pure graph declares exactly def wire(self, i: In) -> Out "
            "— members applied to typed ports, run once at build. Per-tick computation "
            "belongs in a member PureModule.",
        )
    for bad in ("step", "fold"):
        if _find_owner(cls, bad) is not None:
            _fail_def(
                cls,
                GraphRule.STEP,
                f"defines {bad} — a graph is wiring, not computation; it has no tick of its "
                f"own. Move per-tick code into a member PureModule and apply it in wire().",
            )
    fn = vars(wire_owner)["wire"]

    # ── G1 — function kind ──
    if isinstance(fn, staticmethod):
        _fail_def(cls, GraphRule.WIRE_SHAPE, _wire_not_a("a staticmethod"))
    if isinstance(fn, classmethod):
        _fail_def(cls, GraphRule.WIRE_SHAPE, _wire_not_a("a classmethod"))
    if not isinstance(fn, types.FunctionType):
        _fail_def(cls, GraphRule.WIRE_SHAPE, _wire_not_a(f"a {type(fn).__name__}"))
    if inspect.iscoroutinefunction(fn) or inspect.isasyncgenfunction(fn):
        _fail_def(
            cls,
            GraphRule.WIRE_SHAPE,
            "wire cannot be async — it runs once, symbolically, at build; runtime "
            "concurrency is a member affair.",
        )
    if inspect.isgeneratorfunction(fn):
        _fail_def(
            cls,
            GraphRule.WIRE_SHAPE,
            "wire cannot be a generator — it runs once, symbolically, at build.",
        )

    # ── G2 — parameter shape: exactly (self, i), plain positional, no defaults ──
    params = list(inspect.signature(fn).parameters.values())
    data_params = params[1:]
    if len(data_params) != 1:
        _fail_def(
            cls,
            GraphRule.WIRE_SHAPE,
            f"wire takes {len(data_params)} data parameters — expected exactly "
            f"def wire(self, i: In) -> Out.",
        )
    p = data_params[0]
    label = _param_kind_label(p)
    if label is not None:
        _fail_def(
            cls,
            GraphRule.WIRE_SHAPE,
            f"wire parameter {p.name!r} is {label}; wire's parameters must be plain "
            f"positional: (self, i).",
        )
    if p.default is not inspect.Parameter.empty:
        _fail_def(
            cls,
            GraphRule.WIRE_SHAPE,
            f"wire parameter {p.name!r} has a default. The build always passes the symbolic "
            f"In row — remove it.",
        )

    # ── G3 — annotation resolution (reuse T3's resolver; wire substituted) ──
    hints = _resolve_step_hints(fn, wire_owner, cls, name="wire")
    if p.name not in hints:
        _fail_def(
            cls,
            GraphRule.WIRE_UNANNOTATED,
            f"wire parameter {p.name!r} has no annotation. The wire signature is the "
            f"graph's whole contract — annotate it with the In row.",
        )
    if "return" not in hints:
        _fail_def(
            cls,
            GraphRule.WIRE_UNANNOTATED,
            "wire has no return annotation. The wire signature is the graph's whole "
            "contract — annotate the return: -> Out.",
        )

    # ── G4 — row types: i is a pm.In row; return is exactly a pm.Out row ──
    in_ann = hints[p.name]
    if not (isinstance(in_ann, type) and issubclass(in_ann, PmIn)):
        _fail_def(
            cls,
            GraphRule.IN_NOT_ROW,
            f"wire input {p.name!r} is {_ann_name(in_ann)}, not a pm.In row. Declare "
            f"class In(pm.In) with one bare-typed field per rim input.",
        )
    ret = hints["return"]
    if get_origin(ret) is not None or not (isinstance(ret, type) and issubclass(ret, PmOut)):
        _fail_def(
            cls,
            GraphRule.OUT_NOT_ROW,
            f"wire returns {_ann_name(ret)}, not a pm.Out row — a graph always wires every "
            f"declared output (no unions, no | None). Declare class Out(pm.Out) and "
            f"construct it in wire().",
        )

    # ── G5 — bundle policy: In fields bare + required; Out fields plain ──
    for fname, fs in in_ann.fields().items():
        if fs.kind != "bare":
            _fail_def(
                cls,
                GraphRule.PORT_SAMPLER,
                f"In field {fname!r} carries {fs.kind}() — graph ports are bare types; "
                f"samplers live on member In fields, the graph only wires.",
            )
        if not fs.required:
            _fail_def(
                cls,
                GraphRule.PORT_DEFAULT,
                f"In field {fname!r} has a default — graph rim inputs are bare, required "
                f"ports; optionality lives on the consuming member's sampler "
                f"(latest(default=...)).",
            )
    for fname, fs in ret.fields().items():
        if fs.kind != "plain":
            _fail_def(
                cls,
                GraphRule.PORT_SAMPLER,
                f"Out field {fname!r} carries {fs.kind}() — graph ports are bare types; an "
                f"exported output inherits its producing member's contract.",
            )

    return GraphSpec(in_type=in_ann, out_type=ret, owner=wire_owner)


def _wire_not_a(what: str) -> str:
    """The G1 `wire is not a plain instance method` variant body."""
    return f"wire must be a plain instance method — def wire(self, i: In) -> Out — not {what}."


# ── the build context (spec §4.1) — a thread-local stack, nesting recurses ────


@dataclasses.dataclass
class _BuildCtx:
    """Mutable accumulator for one active graph build; frozen into a GraphPlan."""

    graph_cls: type[Any]
    token: object
    members: list[tuple[str, Any]] = dataclasses.field(default_factory=list)
    bindings: list[Binding] = dataclasses.field(default_factory=list)
    tf_bindings: list[Binding] = dataclasses.field(default_factory=list)
    feedbacks_open: list[Port[Any]] = dataclasses.field(default_factory=list)
    feedback_consumers: list[tuple[Port[Any], PortRef]] = dataclasses.field(default_factory=list)
    resolved_feedbacks: list[Binding] = dataclasses.field(default_factory=list)
    used_inputs: set[str] = dataclasses.field(default_factory=set)
    name_counts: dict[str, int] = dataclasses.field(default_factory=dict)
    used_names: set[str] = dataclasses.field(default_factory=set)
    pending_names: dict[int, str] = dataclasses.field(default_factory=dict)
    exports: list[tuple[str, PortRef]] = dataclasses.field(default_factory=list)


_builds = threading.local()


def _build_stack() -> list[_BuildCtx]:
    """The current thread's build stack (created on first use)."""
    stack: list[_BuildCtx] | None = getattr(_builds, "stack", None)
    if stack is None:
        stack = []
        _builds.stack = stack
    return stack


def _current_build() -> _BuildCtx | None:
    """The innermost active build, or None outside any build."""
    stack = _build_stack()
    return stack[-1] if stack else None


def _member_name(ctx: _BuildCtx, module: Any) -> str:
    """Name a member: explicit named() tag, else snake_case with _2/_3 dedup (spec §4.1)."""
    explicit = ctx.pending_names.pop(id(module), None)
    if explicit is not None:
        if explicit in ctx.used_names:
            raise _e_duplicate_name(ctx.graph_cls, explicit)
        ctx.used_names.add(explicit)
        return explicit
    base = snake_case(type(module).__name__)
    n = ctx.name_counts.get(base, 0) + 1
    name = base if n == 1 else f"{base}_{n}"
    while name in ctx.used_names:  # skip past a colliding explicit name
        n += 1
        name = f"{base}_{n}"
    ctx.name_counts[base] = n
    ctx.used_names.add(name)
    return name


def _check_port(g: type[Any], cls: type[Any], field: str, value: Any, ctx: _BuildCtx) -> Port[Any]:
    """A bound value must be a Port minted by this build (spec §4.2 steps 2)."""
    if not isinstance(value, Port):
        raise _e_not_a_port(g, cls, field, value)
    if value._token is not ctx.token:
        raise _e_foreign_port(g)
    return value


def _src_repr(port: Port[Any]) -> str:
    """Render a source port for the type-mismatch message (``i.x`` for rim, else path)."""
    if port._feedback:
        return "a feedback() port"
    if port.ref.member is None:
        return f"i.{port.ref.field}"
    return port.ref.path


def _check_type(g: type[Any], cls: type[Any], field: str, want_ann: Any, port: Port[Any]) -> None:
    """Payload agreement after Optional-stripping; permissive on undeterminable (spec §4.2/4)."""
    want = _strip_opt(want_ann)
    got = _strip_opt(port.annotation)
    if isinstance(want, type) and isinstance(got, type) and not issubclass(got, want):
        raise _e_type_mismatch(g, cls, field, _ann_name(want), _src_repr(port), _ann_name(got))


def _out_ports(out_type: type[Any], member: str, token: object) -> Any:
    """Mint the member/graph Out row carrying one Port per field (spec §3.5)."""
    kwargs: dict[str, Port[Any]] = {
        fname: Port(PortRef(member, fname), fs.annotation, token=token)
        for fname, fs in out_type.fields().items()
    }
    return cast("Any", out_type)(**kwargs)


def apply_symbolic(module: Any, ports: Mapping[str, Any]) -> Any:
    """One symbolic application inside the active build; returns an Out row of ports."""
    ctx = _current_build()
    if ctx is None:
        raise _e_apply_context(type(module))
    if isinstance(module, PureGraph):
        return _apply_subgraph(ctx, module, dict(ports))
    return _apply_module(ctx, module, dict(ports))


def _apply_module(ctx: _BuildCtx, module: Any, ports: dict[str, Any]) -> Any:
    """Validate + record one member PureModule application (spec §4.2)."""
    g = ctx.graph_cls
    cls = type(module)
    spec = cls.__pure_step__
    in_fields = spec.in_type.fields()
    tf_in = {n: s for n, s in in_fields.items() if isinstance(s, TfSpec)}
    tf_port = ports.pop("tf", None)

    unknown = [k for k in ports if k not in in_fields]
    if unknown:
        raise _e_unknown_kwarg(g, cls, unknown, list(in_fields))
    if tf_port is not None:
        if not tf_in:
            raise _e_tf_unexpected(g, cls)
        _check_port(g, cls, "tf", tf_port, ctx)
    else:
        required_tf = [n for n, s in tf_in.items() if s.required]
        if required_tf:
            raise _e_tf_missing(g, cls, required_tf)
    for k, v in ports.items():
        _check_port(g, cls, k, v, ctx)
    missing = [
        n
        for n, s in in_fields.items()
        if s.required and not isinstance(s, TfSpec) and n not in ports
    ]
    if missing:
        raise _e_unbound_input(g, cls, missing)

    name = _member_name(ctx, module)
    ctx.members.append((name, module))
    for k, v in ports.items():
        fs = in_fields[k]
        if isinstance(fs, TickSpec) and v._feedback:
            raise _e_tick_cycle(g, cls, k)
        _check_type(g, cls, k, fs.annotation, v)
        dst = PortRef(name, k)
        if v._feedback:
            ctx.feedback_consumers.append((v, dst))
        else:
            ctx.bindings.append(Binding(v.ref, dst))
            if v.ref.member is None:
                ctx.used_inputs.add(v.ref.field)
    if tf_port is not None:
        dst = PortRef(name, "tf")
        if tf_port._feedback:
            ctx.feedback_consumers.append((tf_port, dst))
        else:
            ctx.tf_bindings.append(Binding(tf_port.ref, dst))
            if tf_port.ref.member is None:
                ctx.used_inputs.add(tf_port.ref.field)
    return _out_ports(spec.out_type, name, ctx.token)


def _apply_subgraph(ctx: _BuildCtx, subgraph: Any, ports: dict[str, Any]) -> Any:
    """Recursively build + splice a nested graph, namespacing members by path (spec §4.2)."""
    g = ctx.graph_cls
    subcls = type(subgraph)
    gspec: GraphSpec = subcls.__pure_graph__
    sub_in_fields = gspec.in_type.fields()
    if ports.pop("tf", None) is not None:
        raise _e_tf_unexpected(g, subcls)  # a graph rim declares no tf side channel
    unknown = [k for k in ports if k not in sub_in_fields]
    if unknown:
        raise _e_unknown_kwarg(g, subcls, unknown, list(sub_in_fields))
    for k, v in ports.items():
        _check_port(g, subcls, k, v, ctx)
    missing = [n for n, s in sub_in_fields.items() if s.required and n not in ports]
    if missing:
        raise _e_unbound_input(g, subcls, missing)
    for k, v in ports.items():
        _check_type(g, subcls, k, sub_in_fields[k].annotation, v)

    sub_plan = _build_with(subgraph, ctx.token)
    prefix = _member_name(ctx, subgraph)

    def remap_src(ref: PortRef) -> PortRef:
        if ref.member is None:  # sub rim input → caller's bound port
            caller: Port[Any] = ports[ref.field]
            if caller.ref.member is None:
                ctx.used_inputs.add(caller.ref.field)
            return caller.ref
        return PortRef(f"{prefix}.{ref.member}", ref.field)

    for sub_path, m in sub_plan.members:
        ctx.members.append((f"{prefix}.{sub_path}", m))
    for b in sub_plan.bindings:
        ctx.bindings.append(
            Binding(remap_src(b.src), PortRef(f"{prefix}.{b.dst.member}", b.dst.field))
        )
    for b in sub_plan.tf_bindings:
        ctx.tf_bindings.append(
            Binding(remap_src(b.src), PortRef(f"{prefix}.{b.dst.member}", b.dst.field))
        )
    for b in sub_plan.feedbacks:
        ctx.resolved_feedbacks.append(
            Binding(
                PortRef(f"{prefix}.{b.src.member}", b.src.field),
                PortRef(f"{prefix}.{b.dst.member}", b.dst.field),
            )
        )
    out_fields = gspec.out_type.fields()
    kwargs: dict[str, Port[Any]] = {
        out_name: Port(
            PortRef(f"{prefix}.{ref.member}", ref.field),
            out_fields[out_name].annotation,
            token=ctx.token,
        )
        for out_name, ref in sub_plan.exports
    }
    return cast("Any", gspec.out_type)(**kwargs)


def _build_with(graph: Any, token: object) -> GraphPlan:
    """Run wire() under a build with the given token, validate, freeze (spec §4)."""
    cls = type(graph)
    gspec: GraphSpec = cls.__pure_graph__
    in_type = gspec.in_type
    out_type = gspec.out_type
    ctx = _BuildCtx(graph_cls=cls, token=token)
    stack = _build_stack()
    stack.append(ctx)
    try:
        in_fields = in_type.fields()
        rim: dict[str, Port[Any]] = {
            fname: Port(PortRef(None, fname), fs.annotation, token=token)
            for fname, fs in in_fields.items()
        }
        out = graph.wire(cast("Any", in_type)(ts=UNSTAMPED, **rim))
        _close_out(ctx, cls, out, out_type, in_fields)
        feedbacks = tuple(ctx.resolved_feedbacks) + tuple(
            Binding(cast("Port[Any]", fb._closed_with).ref, dst)
            for fb, dst in ctx.feedback_consumers
        )
        return GraphPlan(
            graph=cls,
            members=tuple(ctx.members),
            bindings=tuple(ctx.bindings),
            tf_bindings=tuple(ctx.tf_bindings),
            exports=tuple(ctx.exports),
            inputs=tuple((n, fs.annotation) for n, fs in in_fields.items()),
            feedbacks=feedbacks,
        )
    finally:
        stack.pop()


def _close_out(
    ctx: _BuildCtx, cls: type[Any], out: Any, out_type: type[Any], in_fields: Mapping[str, Any]
) -> None:
    """Validate the rim close-out (spec §4.5) and record the exports on the context."""
    if not isinstance(out, out_type):
        raise _e_wire_return(cls, out, out_type)
    seen: dict[PortRef, str] = {}
    for fname in out_type.fields():
        value = getattr(out, fname)
        if not isinstance(value, Port) or value._token is not ctx.token or value._feedback:
            raise _e_unset_output(cls, fname, value)
        ref = value.ref
        if ref.member is None:
            raise _e_export_input(cls, fname, ref.field)
        if ref in seen:
            raise _e_export_alias(cls, seen[ref], fname)
        seen[ref] = fname
        ctx.exports.append((fname, ref))
    unused = [n for n in in_fields if n not in ctx.used_inputs]
    if unused:
        # Q2 (Ivan): warn, don't fail — an unused rim input is a soft wiring smell,
        # not a hard error (it may be wired in a later revision of the graph).
        warnings.warn(str(_e_unused_input(cls, unused)), stacklevel=2)
    open_fb = sum(1 for fb in ctx.feedbacks_open if fb._closed_with is None)
    if open_fb:
        raise _e_unclosed_feedback(cls, open_fb)


def build_graph(graph: Any) -> GraphPlan:
    """Run wire() on a symbolic In, validate the wiring, freeze the plan (spec §4)."""
    return _build_with(graph, object())


# ── the local runtime: bounded streaming, synchronous topological pull ──────
# §0.2/§0.3: NO edge logs. Each exported output is driven by a fresh, bounded
# pull over its ancestor cone; fan-out within a cone tees the shared producer
# (branch-lag bounded); a second output comes from a fresh run (over() is
# deterministic and re-invokable), never from buffering one output while
# draining another.


def _payload_iter(value: Any) -> Iterator[Any]:
    """Iterate a rim source as ``.data`` payloads; a memory2 Stream unwraps (T6 boundary)."""
    mod = sys.modules.get("dimos.memory2.stream")
    if mod is not None:
        stream_cls = getattr(mod, "Stream", None)
        if isinstance(stream_cls, type) and isinstance(value, stream_cls):
            return (obs.data for obs in cast("Iterable[Any]", value))
    return iter(value)


def _project(rows: Iterator[Any], field: str) -> Iterator[Any]:
    """One consumer's view of a member's Out edge: its field's payloads, engine-ts stamped, None dropped."""
    for row in rows:
        payload = getattr(row, field, None)
        if payload is not None:
            _carry_row_ts(row, payload)
            yield payload


def _carry_row_ts(row: Any, payload: Any) -> None:
    """Stamp the edge currency with the producer's engine ts — the firing tick's ts (T13 §0.6).

    The engine stamps the Out ROW from the tick that produced it, but a nested field
    payload keeps whatever ts its own producer set — which may be wall-clock (a fresh
    ``Path`` defaults ts to ``time.time()``) or stale/cached (a voxel grid snapshot
    whose ts lags, and on a PGO loop-closure rebuild runs backwards). Downstream aligns
    on the payload ts, so a non-monotonic or off-scale payload ts silently drops the
    value (``nonmonotonic``) or, on a ``latest`` sampler, pins a far-future pending head
    that starves the producer. The edge currency IS the tick's value: carry the row's
    engine ts onto it so offline distribution stays lossless (§0.2) regardless of how a
    member stamps its nested payloads.
    """
    row_ts = getattr(row, "ts", None)
    if row_ts is not None and getattr(payload, "ts", None) != row_ts:
        try:
            payload.ts = row_ts
        except (AttributeError, TypeError):
            pass  # a payload with a read-only ts keeps its own (best effort, not fatal)


def _cone(plan: GraphPlan, target: PortRef) -> tuple[set[str], set[str]]:
    """The ancestor members + rim inputs feeding one export (drive only what it needs)."""
    members: set[str] = set()
    rims: set[str] = set()
    stack = [target.member] if target.member is not None else []
    while stack:
        path = stack.pop()
        if path in members:
            continue
        members.add(path)
        for b in (*plan.bindings, *plan.tf_bindings):
            if b.dst.member == path:
                if b.src.member is None:
                    rims.add(b.src.field)
                else:
                    stack.append(b.src.member)
    return members, rims


class _RunInstance:
    """One bounded pull over an export's cone; owns the member drivers for teardown."""

    def __init__(
        self,
        plan: GraphPlan,
        sources: Mapping[str, Any],
        target_name: str,
        debug_config: Any = None,
    ) -> None:
        """Wire the cone eagerly (drivers created, resources not yet acquired)."""
        self._members: list[tuple[str, Iterator[Any]]] = []
        self._closed = False
        self._debug_config = debug_config
        self._export = self._build(plan, sources, target_name)

    def _build(
        self, plan: GraphPlan, sources: Mapping[str, Any], target_name: str
    ) -> Iterator[Any]:
        target = next(ref for n, ref in plan.exports if n == target_name)
        members, rims = _cone(plan, target)
        sinks: dict[PortRef, list[tuple[str, Any]]] = {}
        for b in plan.bindings:
            if b.dst.member in members:
                sinks.setdefault(b.src, []).append(("edge", b.dst))
        for b in plan.tf_bindings:
            if b.dst.member in members:
                sinks.setdefault(b.src, []).append(("tf", b.dst))
        sinks.setdefault(target, []).append(("export", target_name))

        edges: dict[PortRef, Iterator[Any]] = {}
        tfs: dict[str, Iterator[Any]] = {}
        exports: dict[str, Iterator[Any]] = {}

        def assign(sink: tuple[str, Any], it: Iterator[Any]) -> None:
            kind, ref = sink
            if kind == "edge":
                edges[ref] = it
            elif kind == "tf":
                tfs[ref.member] = it
            else:
                exports[ref] = it

        for name in rims:
            branches = _fanout(_payload_iter(sources[name]), sinks.get(PortRef(None, name), []))
            for branch, sink in branches:
                assign(sink, branch)

        for path, module in plan.members:
            if path not in members:
                continue
            spec = module.__pure_step__
            streams = {b.dst.field: edges[b.dst] for b in plan.bindings if b.dst.member == path}
            gen = run_over(
                module,
                spec,
                streams,
                tf=tfs.get(path),
                hooks=RunHooks(),
                debug=_member_debug_session(self._debug_config, path, module),
            )
            self._members.append((path, gen))
            member_sinks = [
                (ref.field, sink)
                for ref, slist in sinks.items()
                if ref.member == path
                for sink in slist
            ]
            for (field, sink), branch in _tee_members(gen, member_sinks):
                assign(sink, _project(branch, field))
        return exports[target_name]

    def export(self) -> Iterator[Any]:
        """The (single) requested export's payload stream; iteration drives the cone."""
        return self._export

    def close(self) -> None:
        """Close member drivers in reverse application order — idempotent (spec §6.4)."""
        if self._closed:
            return
        self._closed = True
        for _, gen in reversed(self._members):
            closer = getattr(gen, "close", None)
            if closer is not None:
                closer()


class _TerminalRun:
    """One compute-once pass over the union cone of several recorded producers (§0.5).

    Every member in the union of the recorded producers' cones is driven by a
    single ``run_over`` — shared upstream runs EXACTLY ONCE — and each producer's
    Out row is teed to its downstream consumers and to one record branch per
    recorded output. The record branches are the terminal: a store drains them,
    a sink module consumes them. Bounded (~one msg per edge in flight), no edge
    logs, no per-output re-run.
    """

    def __init__(
        self,
        plan: GraphPlan,
        sources: Mapping[str, Any],
        records: Sequence[tuple[str, PortRef]],
        debug_config: Any = None,
    ) -> None:
        """Wire the union cone eagerly (drivers created, resources not yet acquired)."""
        self._members: list[tuple[str, Iterator[Any]]] = []
        self._closed = False
        self._debug_config = debug_config
        self.record_iters: dict[str, Iterator[Any]] = self._build(plan, sources, records)

    def _build(
        self, plan: GraphPlan, sources: Mapping[str, Any], records: Sequence[tuple[str, PortRef]]
    ) -> dict[str, Iterator[Any]]:
        members: set[str] = set()
        rims: set[str] = set()
        for _, ref in records:
            cone_members, cone_rims = _cone(plan, ref)
            members |= cone_members
            rims |= cone_rims

        sinks: dict[PortRef, list[tuple[str, Any]]] = {}
        for b in plan.bindings:
            if b.dst.member in members:
                sinks.setdefault(b.src, []).append(("edge", b.dst))
        for b in plan.tf_bindings:
            if b.dst.member in members:
                sinks.setdefault(b.src, []).append(("tf", b.dst))
        for name, ref in records:
            sinks.setdefault(ref, []).append(("record", name))

        edges: dict[PortRef, Iterator[Any]] = {}
        tfs: dict[str, Iterator[Any]] = {}
        recorded: dict[str, Iterator[Any]] = {}

        def assign(sink: tuple[str, Any], it: Iterator[Any]) -> None:
            kind, ref = sink
            if kind == "edge":
                edges[ref] = it
            elif kind == "tf":
                tfs[ref.member] = it
            else:
                recorded[ref] = it

        for name in rims:
            branches = _fanout(_payload_iter(sources[name]), sinks.get(PortRef(None, name), []))
            for branch, sink in branches:
                assign(sink, branch)

        for path, module in plan.members:
            if path not in members:
                continue
            spec = module.__pure_step__
            streams = {b.dst.field: edges[b.dst] for b in plan.bindings if b.dst.member == path}
            gen = run_over(
                module,
                spec,
                streams,
                tf=tfs.get(path),
                hooks=RunHooks(),
                debug=_member_debug_session(self._debug_config, path, module),
            )
            self._members.append((path, gen))
            member_sinks = [
                (ref.field, sink)
                for ref, slist in sinks.items()
                if ref.member == path
                for sink in slist
            ]
            for (field, sink), branch in _tee_members(gen, member_sinks):
                assign(sink, _project(branch, field))
        return recorded

    def close(self) -> None:
        """Close every member driver in reverse creation order — idempotent (spec §6.4)."""
        if self._closed:
            return
        self._closed = True
        for _, gen in reversed(self._members):
            closer = getattr(gen, "close", None)
            if closer is not None:
                closer()


def _fanout(
    base: Iterator[Any], sinks: Sequence[tuple[str, Any]]
) -> list[tuple[Iterator[Any], Any]]:
    """Tee a rim source across its sinks (payloads pass through unprojected)."""
    if not sinks:
        return []
    if len(sinks) == 1:
        return [(base, sinks[0])]
    return list(zip(tee(base, len(sinks)), sinks, strict=True))


def _tee_members(
    gen: Iterator[Any], sinks: Sequence[tuple[str, Any]]
) -> list[tuple[tuple[str, Any], Iterator[Any]]]:
    """Tee a member's Out-row stream across its (field, sink) uses within the cone."""
    if not sinks:
        return []
    if len(sinks) == 1:
        return [(sinks[0], gen)]
    return list(zip(sinks, tee(gen, len(sinks)), strict=True))


def _member_debug_session(debug_config: Any, path: str, module: Any) -> Any:
    """T15 seam: a per-member DebugSession (or None) for the graph's run_over calls.

    Explicit ``.debug(config)`` scopes capture to its matched members; without
    it, ``session_for`` consults ``DIMOS_PURE_DEBUG``. Member paths are the T13
    dot-separated member paths the graph mints (spec §C7/§C9).
    """
    from dimos.pure import debugrec  # lazy: sanctioned recorder edge (engine-level)

    if debug_config is not None:
        rule = debugrec.resolve_debug(debug_config, path)
        if rule is None:
            return None  # explicit config: an unmatched member is off, no env fallback
        return debugrec.session_for(module, path=path, debug=rule)
    return debugrec.session_for(module, path=path, debug=None)  # env-driven default


class EdgeStream:
    """A re-iterable exported output; each iteration is a fresh bounded run (spec §0.2)."""

    def __init__(self, run: GraphRun[Any], name: str) -> None:
        """Bind one export name to its run handle."""
        self._run = run
        self._name = name

    def __iter__(self) -> Iterator[Any]:
        """A fresh run over this export's cone; deterministic and re-invokable."""
        return self._run._drive(self._name)

    def to_list(self) -> list[Any]:
        """Drive this export fully; return its raw payloads (Q4)."""
        return list(self)


class GraphRun(Generic[_TOut]):
    """One local graph run: exported outputs as attributes (raw payload streams)."""

    def __init__(self, graph: Any, plan: GraphPlan, sources: Mapping[str, Any]) -> None:
        """Hold the built plan + resolved rim sources; iteration spawns bounded runs."""
        self._graph = graph
        self._plan = plan
        self._sources = sources
        self._live: list[_RunInstance] = []
        self._terminals: list[_TerminalRun] = []
        self._closed = False
        self._debug: Any = None  # T15: coerced DebugConfig from .debug(), else env-driven

    def debug(self, config: bool | str | Any = True) -> GraphRun[_TOut]:
        """Enable T15 capture for this run's members (spec §Toggle & API: graph.over(...).debug(...)).

        ``config`` is any ``over(debug=)`` value (bool / grammar string / ``pm.Debug`` /
        rule tuples); default ``True`` = decisions everywhere. Returns self.
        """
        from dimos.pure import debugrec  # lazy: sanctioned recorder edge

        self._debug = debugrec.coerce_debug(config)
        return self

    def __getattr__(self, name: str) -> EdgeStream:
        """Exported rim output as a stream; a non-export is a plain AttributeError."""
        if name.startswith("_"):
            raise AttributeError(name)
        plan: GraphPlan | None = self.__dict__.get("_plan")
        if plan is not None and any(n == name for n, _ in plan.exports):
            return EdgeStream(self, name)
        raise AttributeError(name)

    def _drive(self, export_name: str) -> Iterator[Any]:
        """Spawn a bounded run for one export, tearing it down when the caller stops."""
        inst = _RunInstance(self._plan, self._sources, export_name, self._debug)
        self._live.append(inst)
        try:
            yield from inst.export()
        finally:
            inst.close()
            if inst in self._live:
                self._live.remove(inst)

    def save(
        self,
        sink: Any,
        *,
        remap: Mapping[str, str] | None = None,
        record: Mapping[str, str] | None = None,
    ) -> Any:
        """Terminal drive (§0.5): compute the whole DAG ONCE, fan every output to ``sink``.

        ``sink`` is a mem2 store (each output → a named ``Observation`` stream) or a
        pure module whose In ports match the exported output names. ``remap`` maps a
        module-sink In field to a differently-named output; ``record`` maps an interior
        edge path (``'member.field'``) to an extra recorded output name. Shared upstream
        runs exactly once. Returns the store (store sink) or the frame count (module sink).
        """
        remap = remap or {}
        record = record or {}
        if isinstance(sink, PureGraph):
            raise _e_impl_pending(f"{_clspath(type(self._graph))}.save(<graph sink>)")
        if hasattr(type(sink), "__pure_step__"):
            return self._save_to_module(sink, remap, record)
        if callable(getattr(sink, "stream", None)):
            return self._save_to_store(sink, record)
        raise TypeError(
            f"{_clspath(type(self._graph))}.save() sink {type(sink).__qualname__} is neither a "
            f"mem2 store (has .stream(name, type)) nor a pure module (has __pure_step__)."
        )

    def _producer_ref(self, path: str) -> PortRef:
        """Resolve an output name or interior edge path to its producing port (spec §0.5)."""
        for name, ref in self._plan.exports:
            if name == path:
                return ref
        if "." in path:
            member, _, field = path.rpartition(".")
            for p, m in self._plan.members:
                if p == member and field in m.__pure_step__.out_type.fields():
                    return PortRef(member, field)
        available = [n for n, _ in self._plan.exports] + [
            f"{p}.{f}" for p, m in self._plan.members for f in m.__pure_step__.out_type.fields()
        ]
        raise _e_unknown_path(self._plan.graph, path, available)

    def _out_type(self, name: str) -> type[Any] | None:
        """The exported output's payload type, for the store codec (None if not a concrete type)."""
        fields = self._plan.graph.__pure_graph__.out_type.fields()
        ann = fields[name].annotation if name in fields else None
        return ann if isinstance(ann, type) else None

    def _save_to_store(self, store: Any, record: Mapping[str, str]) -> Any:
        """Record every exported output (+ any interior ``record`` edges) as named streams."""
        records: list[tuple[str, PortRef]] = list(self._plan.exports)
        for path, name in record.items():
            records.append((name, self._producer_ref(path)))
        run = _TerminalRun(self._plan, self._sources, records, self._debug)
        self._terminals.append(run)
        try:
            streams = {name: store.stream(name, self._out_type(name)) for name, _ in records}
            iters = run.record_iters
            active = [name for name, _ in records]
            while active:
                still: list[str] = []
                for name in active:  # round-robin: one pull per branch keeps tees bounded
                    try:
                        payload = next(iters[name])
                    except StopIteration:
                        continue
                    streams[name].append(payload, ts=getattr(payload, "ts", None))
                    still.append(name)
                active = still
        finally:
            run.close()
        return store

    def _save_to_module(
        self, sink: Any, remap: Mapping[str, str], record: Mapping[str, str]
    ) -> int:
        """Drive the DAG into a pure-module sink whose In ports name the outputs (spec §0.5)."""
        from dimos.pure.rows import TfSpec  # data layer; local to keep the surface honest

        spec = type(sink).__pure_step__
        produced: dict[str, PortRef] = {n: ref for n, ref in self._plan.exports}
        for path, name in record.items():
            produced[name] = self._producer_ref(path)
        records: dict[str, PortRef] = {}
        mapping: dict[str, str] = {}  # sink In field → recorded output name
        for fld, fs in spec.in_type.fields().items():
            if isinstance(fs, TfSpec):
                continue  # tf side channel is not fed from named outputs here
            src = remap.get(fld, fld)
            if src in produced:
                records[src] = produced[src]
                mapping[fld] = src
        run = _TerminalRun(self._plan, self._sources, list(records.items()), self._debug)
        self._terminals.append(run)
        try:
            streams = {fld: run.record_iters[src] for fld, src in mapping.items()}
            gen = run_over(sink, spec, streams, hooks=RunHooks())
            run._members.append(("<sink>", gen))  # closed with the run (sink last-created)
            return sum(1 for _ in gen)
        finally:
            run.close()

    def close(self) -> None:
        """Tear down every still-live run in reverse order — idempotent (spec §6.4)."""
        self._closed = True
        for inst in reversed(self._live):
            inst.close()
        self._live.clear()
        for term in reversed(self._terminals):
            term.close()
        self._terminals.clear()

    def __enter__(self) -> GraphRun[_TOut]:
        """Context-managed run; exit closes."""
        return self

    def __exit__(self, *exc: object) -> None:
        """Close on scope exit."""
        self.close()


def _store_stream(source: Any, chan: str) -> Any | None:
    """Structurally resolve a named stream from a store (no memory2 import; spec §6.1)."""
    getter = getattr(source, "stream", None)
    if callable(getter):
        try:
            return getter(chan)
        except Exception:
            return None
    accessor = getattr(source, "streams", None)
    if accessor is not None:
        return getattr(accessor, chan, None)
    return None


def _resolve_sources(
    plan: GraphPlan,
    source: Any | None,
    remap: Mapping[str, str] | None,
    streams: Mapping[str, Streamable],
) -> dict[str, Any]:
    """Bind every rim input to a source: explicit kwargs, then the store (spec §6.1)."""
    g = plan.graph
    rim_names = [n for n, _ in plan.inputs]
    unknown = [k for k in streams if k not in rim_names]
    if unknown:
        raise _e_unknown_stream(g, unknown, rim_names)
    remap = remap or {}
    resolved: dict[str, Any] = {}
    for name in rim_names:
        if name in streams:
            resolved[name] = streams[name]
        elif source is not None:
            chan = remap.get(name, name)
            found = _store_stream(source, chan)
            if found is None:
                raise _e_store_missing(g, chan, name)
            resolved[name] = found
    unbound = [n for n in rim_names if n not in resolved]
    if unbound:
        raise _e_unbound_rim(g, unbound)
    return resolved


def _rebuild_graph(cls: type[PureGraph], dump: dict[str, Any]) -> PureGraph:
    """Pickle helper: reconstruct a graph as cls(**dump)."""
    return cls(**dump)


# ── blueprint lowering (spec §7.2) — the graph's one dimos.core edge ─────────


def _lower_blueprint(cls: type[Any], namespace: str | None, config: Mapping[str, Any]) -> Blueprint:
    """Lower a built plan to a coordinator Blueprint namespace (spec §7.2).

    Each member deploys through the T8 bridge (one process); the graph's explicit
    edges GENERATE the wiring — an interior edge path-qualifies its producer topic
    and every consumer joins it; a name-crossing edge becomes a ``.remappings``
    entry; rim In/Out ports stay exposed (bare) so the graph links by the
    autoconnect (name, type) convention. tf edges ride the shared tf rail, not a
    stream topic. Lazy imports: this is the one ``dimos.core`` edge (spec §1).
    """
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.pure.legacy import TF_IN_STREAM, legacy_blueprint

    ns = namespace or snake_case(cls.__name__)
    plan = build_graph(cast("Any", cls)(**config))

    atoms = [
        legacy_blueprint(type(m), instance_name=path.replace(".", "/"), **m.config.model_dump())
        for path, m in plan.members
    ]

    export_by_ref: dict[PortRef, str] = {ref: name for name, ref in plan.exports}
    remaps: dict[tuple[str, str], str] = {}
    expose: set[str] = set()

    def _key(member: str) -> str:
        return member.replace(".", "/")  # blueprint-land uses '/', pure-land '.'

    def _producer_topic(ref: PortRef) -> str:
        """The topic a member producer publishes on: its export name, else path-qualified."""
        name = export_by_ref.get(ref)
        if name is not None:
            return name  # exported → the exposed rim name (one topic, one name)
        return f"{_key(cast('str', ref.member))}/{ref.field}"

    # Rim exports: each producer publishes under its export name (exposed, bare).
    for name, ref in plan.exports:
        remaps[_key(cast("str", ref.member)), ref.field] = name
        expose.add(name)

    # Every edge: the consumer joins the producer's topic. Feedback edges lower
    # like any interior edge (the sampler rides the member; pubsub is cyclic).
    for b in (*plan.bindings, *plan.feedbacks):
        dst_member = cast("str", b.dst.member)  # a binding dst is always a member
        if b.src.member is None:  # rim input → consumer subscribes the rim name (exposed)
            remaps[_key(dst_member), b.dst.field] = b.src.field
            expose.add(b.src.field)
        else:
            topic = _producer_topic(b.src)
            remaps[_key(dst_member), b.dst.field] = topic
            if b.src not in export_by_ref:  # unexported producer path-qualifies its topic
                remaps[_key(b.src.member), b.src.field] = topic

    # T14: tf is one global rail (like the legacy TF service's topic), so members'
    # tf_in stays bare — namespacing it would point the pin at a topic nobody publishes.
    if any(s.name == TF_IN_STREAM for bp in atoms for atom in bp.blueprints for s in atom.streams):
        expose.add(TF_IN_STREAM)

    entries = [(inst, old, new) for (inst, old), new in remaps.items()]
    return autoconnect(*atoms).remappings(entries).namespace(ns, expose=sorted(expose))


# ── the graph base (spec §2, §3.4) ───────────────────────────────────────────
# INVARIANT: PureGraph must never declare `wire` — a base-level wire would make
# every subclass match Wires with the base's types (the EngineSurface rule).


@dataclass_transform(kw_only_default=True, frozen_default=True)
class PureGraph:
    """Base for pure graphs: flat config fields + wire(i: In) -> Out, built symbolically."""

    __pure_config_model__: ClassVar[type[PureModuleConfig]]
    __pure_config__: PureModuleConfig
    __pure_graph__: ClassVar[GraphSpec]  # stamped LAST — presence certifies all gates

    def __init_subclass__(cls, **kwargs: object) -> None:
        """Synthesize the frozen config model (T2 twin), then classify wire (spec §2.1)."""
        super().__init_subclass__(**kwargs)
        fields = _collect_config_fields(cls)
        bases: tuple[type[PureModuleConfig], ...] = tuple(
            b.__pure_config_model__
            for b in cls.__bases__
            if b is not PureGraph and issubclass(b, PureGraph)
        ) or (PureModuleConfig,)
        cls.__pure_config_model__ = _synthesize_config_model(cls, fields, bases)
        cls.__pure_graph__ = classify_graph(cls)  # LAST — presence certifies all gates

    def __init__(self, **kwargs: object) -> None:
        """Validate kwargs through the synthesized model; flatten values onto self."""
        model = getattr(type(self), "__pure_config_model__", None)
        if model is None:  # only PureGraph itself lacks a synthesized model
            raise TypeError(
                "PureGraph cannot be instantiated directly — subclass it "
                "(config fields + In/Out + wire)"
            )
        cfg = model(**kwargs)
        object.__setattr__(self, "__pure_config__", cfg)
        for name in model.model_fields:
            object.__setattr__(self, name, getattr(cfg, name))

    @property
    def config(self) -> PureModuleConfig:
        """The frozen synthesized config model; ``model_dump()`` is canonical."""
        return self.__pure_config__

    def __setattr__(self, name: str, value: object) -> None:
        """Always raises FrozenModuleError — rebuild the graph, never mutate."""
        model = getattr(type(self), "__pure_config_model__", None)
        if model is not None and name in model.model_fields:
            raise FrozenModuleError(
                f"{type(self).__name__}.{name} is frozen config — rebuild: "
                f"{type(self).__name__}(**{{**g.config.model_dump(), {name!r}: ...}})"
            )
        raise FrozenModuleError(
            f"{type(self).__name__} is immutable — configuration is frozen; rebuild it"
        )

    def __delattr__(self, name: str) -> None:
        """Always raises FrozenModuleError."""
        raise FrozenModuleError(
            f"{type(self).__name__}.{name} cannot be deleted — graphs are immutable"
        )

    def __eq__(self, other: object) -> bool:
        """Identity = class + config: same exact class and equal config."""
        if other is self:
            return True
        if not isinstance(other, PureGraph):
            return NotImplemented
        return type(other) is type(self) and other.config == self.config

    def __hash__(self) -> int:
        """hash((type(self), config)) — frozen models hash by value."""
        return hash((type(self), self.config))

    def __repr__(self) -> str:
        """``ClassName(field=value, ...)`` in canonical field order."""
        inner = ", ".join(f"{k}={v!r}" for k, v in self.config.model_dump().items())
        return f"{type(self).__name__}({inner})"

    def __reduce__(self) -> tuple[Any, ...]:
        """Pickle/copy as rebuild-from-config: (_rebuild_graph, (cls, dump))."""
        return (_rebuild_graph, (type(self), self.config.model_dump()))

    def __call__(self: Wires[_TIn, _TOut], **ports: Any) -> _TOut:
        """Symbolic application: bind member/graph ports, return Out as port refs."""
        return cast("_TOut", apply_symbolic(self, ports))

    def build(self: Wires[_TIn, _TOut]) -> GraphPlan:
        """Build + validate the DAG; pure and rerunnable — inspect, diff, throw away."""
        return build_graph(self)

    def over(
        self: Wires[_TIn, _TOut],
        source: Any | None = None,
        *,
        remap: Mapping[str, str] | None = None,
        at: Mapping[str, Iterable[Any]] | None = None,
        **streams: Streamable,
    ) -> GraphRun[_TOut]:
        """Run the whole DAG in-process over real streams (store and/or kwargs; spec §6)."""
        if at is not None:
            raise _e_impl_pending(f"{_clspath(type(self))}.over(at=...)")
        plan = build_graph(self)
        sources = _resolve_sources(plan, source, remap, streams)
        return cast("GraphRun[_TOut]", GraphRun(self, plan, sources))

    @classmethod
    def blueprint(cls, *, namespace: str | None = None, **config: Any) -> Blueprint:
        """Lower the graph to a coordinator Blueprint namespace (spec §7.2, Phase B)."""
        return _lower_blueprint(cls, namespace, config)

    def bind(self, transport: Any) -> Any:
        """Bind rim ports to a live transport (native live graph; spec §8, Phase C)."""
        raise _e_impl_pending("PureGraph.bind()")

    def partition(self, **groups: Sequence[str]) -> Any:
        """Cut the graph into pubsub-linked subgraphs by member path (spec §8, Phase C)."""
        raise _e_impl_pending("PureGraph.partition()")

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
import re
from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    Final,
    Generic,
    NoReturn,
    Protocol,
    TypeVar,
)

from typing_extensions import dataclass_transform

from dimos.pure.config import PureModuleConfig
from dimos.pure.drivers import PureModuleRunError
from dimos.pure.stepspec import PureModuleDefinitionError
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
    return Port(_FEEDBACK_REF, None, feedback=True)


def named(name: str, module: _TModule) -> _TModule:
    """Name ``module``'s next application in the current build (default: snake_case)."""
    raise NotImplementedError(_IMPL_PENDING)


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


# ── classification + symbolic application (spec §2.1, §4.2) ─────────────────


def classify_graph(cls: type[Any]) -> GraphSpec:
    """Classify + validate a graph class (gates G0..G5); pure, raises on violation."""
    raise NotImplementedError(_IMPL_PENDING)


def apply_symbolic(module: Any, ports: Mapping[str, Any]) -> Any:
    """One symbolic application inside the active build; returns an Out row of ports."""
    raise NotImplementedError(_IMPL_PENDING)


def build_graph(graph: Any) -> GraphPlan:
    """Run wire() on a symbolic In, validate the wiring, freeze the plan (spec §4)."""
    raise NotImplementedError(_IMPL_PENDING)


# ── the run handle (spec §6.3) ───────────────────────────────────────────────


class EdgeStream:
    """Re-iterable view of one edge's log; iteration pulls the DAG lazily."""

    def __iter__(self) -> Iterator[Any]:
        """Fresh cursor over the edge; pulls the producer as needed."""
        raise NotImplementedError(_IMPL_PENDING)

    def to_list(self) -> list[Any]:
        """Drain the edge fully; return its payloads."""
        raise NotImplementedError(_IMPL_PENDING)


class GraphRun(Generic[_TOut]):
    """One local graph run: exported outputs as attributes, interior edges by path."""

    def stream(self, path: str) -> EdgeStream:
        """Edge by dotted address (``voxel_mapper.global_map``); rim exports by name."""
        raise NotImplementedError(_IMPL_PENDING)

    def __getattr__(self, name: str) -> EdgeStream:
        """Exported rim output as a stream; unknown names raise ``[graph-unknown-path]``."""
        raise NotImplementedError(_IMPL_PENDING)

    def close(self) -> None:
        """Close member drivers in reverse order, then rim sources — idempotent."""
        raise NotImplementedError(_IMPL_PENDING)

    def __enter__(self) -> GraphRun[_TOut]:
        """Context-managed run; exit closes."""
        return self

    def __exit__(self, *exc: object) -> None:
        """Close on scope exit."""
        self.close()


# ── the graph base (spec §2, §3.4) ───────────────────────────────────────────
# INVARIANT: PureGraph must never declare `wire` — a base-level wire would make
# every subclass match Wires with the base's types (the EngineSurface rule).


@dataclass_transform(kw_only_default=True, frozen_default=True)
class PureGraph:
    """Base for pure graphs: flat config fields + wire(i: In) -> Out, built symbolically."""

    __pure_config_model__: ClassVar[type[PureModuleConfig]]
    __pure_graph__: ClassVar[GraphSpec]  # stamped LAST — presence certifies all gates

    def __init_subclass__(cls, **kwargs: object) -> None:
        """Synthesize the frozen config model (T2 twin), then classify wire (spec §2.1)."""
        super().__init_subclass__(**kwargs)
        raise NotImplementedError(_IMPL_PENDING)

    def __init__(self, **kwargs: object) -> None:
        """Validate kwargs through the synthesized model; flatten values onto self."""
        raise NotImplementedError(_IMPL_PENDING)

    def __call__(self: Wires[_TIn, _TOut], **ports: Any) -> _TOut:
        """Symbolic application: bind member/graph ports, return Out as port refs."""
        raise NotImplementedError(_IMPL_PENDING)

    def build(self: Wires[_TIn, _TOut]) -> GraphPlan:
        """Build + validate the DAG; pure and rerunnable — inspect, diff, throw away."""
        raise NotImplementedError(_IMPL_PENDING)

    def over(
        self: Wires[_TIn, _TOut],
        source: Any | None = None,
        *,
        remap: Mapping[str, str] | None = None,
        at: Mapping[str, Iterable[Any]] | None = None,
        **streams: Streamable,
    ) -> GraphRun[_TOut]:
        """Run the whole DAG in-process over real streams (store and/or kwargs; spec §6)."""
        raise NotImplementedError(_IMPL_PENDING)

    @classmethod
    def blueprint(cls, *, namespace: str | None = None, **config: Any) -> Blueprint:
        """Lower the graph to a coordinator Blueprint namespace (spec §7.2, Phase B)."""
        raise NotImplementedError(_IMPL_PENDING)

    def bind(self, transport: Any) -> Any:
        """Bind rim ports to a live transport (native live graph; spec §8, Phase C)."""
        raise _e_impl_pending("PureGraph.bind()")

    def partition(self, **groups: Sequence[str]) -> Any:
        """Cut the graph into pubsub-linked subgraphs by member path (spec §8, Phase C)."""
        raise _e_impl_pending("PureGraph.partition()")

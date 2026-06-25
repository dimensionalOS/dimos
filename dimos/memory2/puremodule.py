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

"""PureModule — a module whose core is a pure function of aligned inputs.

Instead of subscribing to ports and mutating ``self``, a PureModule
declares *when* it runs and *how* each input is sampled at that moment,
then implements one pure ``step``. The same class runs live on pubsub
ports or offline over stored memory2 streams — ``step`` cannot tell the
difference, which is what buys replay, time-travel debugging, migration,
and parallelism.

::

    class Follower(PureModule):
        image: In[Image] = tick()                # fires the ticks
        pose: In[PoseStamped] = interpolate()    # slerped to frame time
        imu: In[Imu] = latest(max_age=0.1)       # newest, or None if stale

        cmd_vel: Out[Twist]

        def step(self, image: Image, pose: PoseStamped, imu: Imu | None) -> Twist:
            return chase(image, pose)

``step`` parameters are bound by name to the declared inputs:

- the parameter's annotation picks the payload — ``Image`` gets
  ``obs.data``, ``Observation[Image]`` gets the full observation,
  ``window()`` inputs take ``list[X]`` or ``list[Observation[X]]``;
- ``X | None`` means a missing value is passed as ``None``; a missing
  value for a non-optional parameter *drops the tick*;
- reserved names: ``ts: float`` receives the tick time; ``out`` receives
  a per-tick output writer; declaring ``state`` (first parameter) makes
  the module a Mealy machine with the initial value from the
  ``initial_state`` attribute.

Outputs: with one ``Out`` port, return the value (or ``None`` to emit
nothing). With several, declare ``out`` and assign — set = emit, skip =
quiet, unknown ports raise at the assignment::

    def step(self, pose: Pose, out: Outputs) -> None:
        out.cmd_vel = drive(pose)
        if blocked:
            out.alerts = "obstacle"

With ``out`` declared, stateless steps return ``None`` and stateful steps
return just ``new_state`` (no tuple). Returning ``{port_name: value}``
instead of using the writer is also accepted.

**Live** (``module.start()``): each ``In`` port feeds a memory2 stream in
the module's store — a :class:`~dimos.memory2.store.null.NullStore` by
default, so inputs behave as live-only streams (no history, queries come
back empty). Override :meth:`make_store` to return a storage-backed store
and the module records every input *and* output as a side effect of
running. Outputs publish to the ``Out`` ports.

**Offline** (:meth:`over`): run the same module over stored streams —
no ports, no transports, no threads, just lazy iteration::

    store = SqliteStore(path="walk.db")
    out = Follower.over(image=store.streams.image, pose=store.streams.pose,
                        imu=store.streams.imu)
    out.to_list()                                # or .save(...), .map(...)

Called on the class, ``over`` builds a machinery-free instance via
:meth:`offline` (pass config there when needed:
``Follower.offline(gain=2.0).over(...)``).

Alignment semantics live in :mod:`dimos.memory2.tick`.
"""

from __future__ import annotations

from dataclasses import dataclass
import functools
import inspect
import queue
import sys
import threading
import time
import types
import typing
from typing import TYPE_CHECKING, Any, Union, get_args, get_origin, get_type_hints

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.resource import CompositeResource
from dimos.core.stream import In, Out
from dimos.memory2.buffer import BackpressureBuffer, ClosedError, KeepLast
from dimos.memory2.health import Health, HealthConfig, HealthMonitor, ModuleContracts
from dimos.memory2.store.null import NullStore
from dimos.memory2.tick import (
    MISSING,
    Latest,
    Sampler,
    Tick,
    TickMachine,
    Window,
    interpolate,
    latest,
    merge_events,
    tick,
    window,
)
from dimos.memory2.type.observation import Observation
from dimos.protocol.service.spec import Configurable
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.stream import Stream

logger = setup_logger()

__all__ = [
    "OutContract",
    "Outputs",
    "PureModule",
    "contract",
    "interpolate",
    "latest",
    "tick",
    "window",
]

_STOP = object()


@dataclass(frozen=True)
class _Param:
    name: str
    kind: str  # "ts" | "trigger" | "field"
    optional: bool
    wants_obs: bool


@dataclass(frozen=True)
class _Plan:
    trigger: str
    samplers: dict[str, Sampler]  # secondaries only (no trigger)
    ins: dict[str, type]
    outs: dict[str, type]
    params: tuple[_Param, ...]
    stateful: bool
    uses_out: bool  # step declares the reserved `out` writer parameter
    expect_hz: dict[str, float]  # class-declared per-input rates
    missing_ratio: dict[str, float]  # class-declared per-input missing thresholds
    out_min_hz: dict[str, float]  # class-declared per-output rate contracts


class OutContract:
    """Class-level health contract on an ``Out`` port (see :func:`contract`)."""

    def __init__(self, min_hz: float | None = None) -> None:
        if min_hz is not None and min_hz <= 0:
            raise ValueError(f"contract(min_hz) requires min_hz > 0, got {min_hz}")
        self.min_hz = min_hz

    def __repr__(self) -> str:
        return f"OutContract(min_hz={self.min_hz})"


def contract(min_hz: float | None = None) -> Any:
    """Declare a per-output health contract as the port's default value::

        cmd: Out[Twist] = contract(min_hz=10)   # this port must emit >= 10 Hz
        alerts: Out[str]                        # sparse by design — no contract

    Class-level declaration — per-port contracts live here, not in
    deployment config. Typed ``Any`` so it can sit on an ``Out[X]``
    annotation.
    """
    return OutContract(min_hz=min_hz)


class Outputs:
    """Per-tick output writer — assignment emits; skipping a port stays quiet.

    Handed to ``step`` as the reserved ``out`` parameter. Fresh per tick
    and collected right after the call, so the step stays referentially
    pure — this is the return value, passed inside-out. Unknown ports
    raise at the assignment line; reassignment last-wins.
    """

    __slots__ = ("_allowed", "_values")

    def __init__(self, allowed: frozenset[str]) -> None:
        object.__setattr__(self, "_allowed", allowed)
        object.__setattr__(self, "_values", {})

    def __setattr__(self, name: str, value: Any) -> None:
        if name not in self._allowed:
            raise AttributeError(
                f"unknown output {name!r} — declared Out ports: {sorted(self._allowed)}"
            )
        self._values[name] = value

    def __getattr__(self, name: str) -> Any:
        try:
            return self._values[name]
        except KeyError:
            raise AttributeError(name) from None


def _unwrap_optional(ann: Any) -> tuple[Any, bool]:
    if get_origin(ann) in (Union, types.UnionType):
        args = [a for a in get_args(ann) if a is not type(None)]
        if len(args) == 1:
            return args[0], True
    return ann, False


def _wants_obs(ann: Any) -> bool:
    return ann is Observation or get_origin(ann) is Observation


class _class_or_instance:
    """Method usable on the class (auto-creating an offline instance) or an instance.

    ``Walker.over(...)`` in a notebook needs no module machinery;
    ``walker.over(...)`` reuses a configured instance.
    """

    def __init__(self, fn: Any) -> None:
        self.fn = fn

    def __get__(self, obj: Any, objtype: type[PureModule] | None = None) -> Any:
        if obj is None:

            def bound(*args: Any, **kwargs: Any) -> Any:
                assert objtype is not None
                return self.fn(objtype.offline(), *args, **kwargs)

            return bound
        return types.MethodType(self.fn, obj)


class PureModuleConfig(ModuleConfig):
    """Deployment-side module-wide contracts and health mechanics.

    Per-port contracts live in the module's class declaration only —
    samplers (``tick(expect_hz=30)``, ``latest(max_missing_ratio=...)``)
    and ``contract(min_hz=...)`` on ``Out`` ports. Per-port *deployment*
    overrides were tried and removed (no consumer; three ways to say one
    thing) — re-add when a real deployment needs them. This config
    carries what genuinely varies per deployment::

        Follower(
            contracts={"max_drop_ratio": 0.8},
            health={"warmup_s": 1.0},
        )
    """

    contracts: ModuleContracts = Field(default_factory=ModuleContracts)
    """Module-wide promises: ``min_output_hz``, ``max_drop_ratio``,
    ``max_tick_latency_s``, global ``max_missing_ratio``."""

    health: HealthConfig = Field(default_factory=HealthConfig)
    """Reporting mechanics: ``interval_s``, ``warmup_s``,
    ``unhealthy_log_every_s``, ``stall_after_s``, ``ratio_min_samples``,
    ``stream``."""

    max_pending_ticks: int = 64
    """Cap on live ticks awaiting interpolation brackets — bounds memory
    when an ``interpolate()`` input dies; evictions count as
    ``drops_blocked``. Offline ``over()`` is uncapped (exact)."""


class HealthView:
    """Read-only health surface for a running module — ``module.health``.

    Wraps the internal :class:`HealthMonitor` and the ``_health`` stream so
    callers never touch either directly. ``state``/``latest`` are in-process
    (a supervisor, readiness probe, or test assertion); ``stream`` and
    ``subscribe`` ride the module's store — discarded on a ``NullStore``,
    recorded next to the data on a ``SqliteStore``.
    """

    __slots__ = ("_monitor", "_stream")

    def __init__(self, monitor: HealthMonitor, stream: Any | None) -> None:
        self._monitor = monitor
        self._stream = stream

    @property
    def state(self) -> str:
        """Current state: ``'OK'`` | ``'DEGRADED'`` | ``'STALLED'``."""
        return self._monitor.state

    @property
    def latest(self) -> Health | None:
        """The most recent snapshot, or ``None`` before the first report."""
        return self._monitor.latest

    @property
    def stream(self) -> Any | None:
        """The ``_health`` memory2 stream (``None`` if ``health.stream`` is off).

        An ordinary stream: ``.live()`` to tail, ``.before(t).to_list()`` to
        query a recording. ``None`` when health snapshots are disabled.
        """
        return self._stream

    def subscribe(self, on_health: Any) -> Any:
        """Subscribe to live snapshots. Raises if ``health.stream`` is off."""
        if self._stream is None:
            raise RuntimeError(
                "health snapshots are disabled (config.health.stream=False) — "
                "nothing to subscribe to"
            )
        return self._stream.live().subscribe(on_health)


class PureModule(Module):
    """Base class for modules implementing a pure ``step`` over aligned inputs.

    See the module docstring for the full declaration language.
    """

    config: PureModuleConfig

    initial_state: Any = None

    backpressure: BackpressureBuffer[Any] = KeepLast()
    """Policy for resolved ticks awaiting ``step`` when deployed live.

    ``KeepLast()`` (default) always steps the freshest tick — controller
    semantics; skipped ticks are counted as ``drops_backpressure``. Use
    ``Unbounded()`` for must-process-everything consumers (recorders,
    indexers) or ``Bounded(n)``/``DropNew(n)`` in between. The instance is
    a template — each ``start()`` gets a fresh ``clone()``."""

    input_sources: dict[str, Any] | None = None
    """Per-input source overrides for the live runner — the live↔stored switch.

    Set before ``start()``: ``{input_name: source}`` where a source is
    anything with ``.observable()`` (a :class:`ReplayStream` for
    wall-clock-paced recordings, a stored memory2 stream for
    fast-as-possible feeding) or a raw RxPY observable. Inputs not listed
    keep their pub/sub port. Sources emitting :class:`Observation` keep
    their recorded timestamps; raw payloads are stamped like port arrivals
    (``msg.ts`` if present, else now). A module with *every* input sourced
    needs no transports at all::

        m = Navigator()
        m.input_sources = {"pose": db.replay(speed=2.0).streams.pose}
        m.start()   # same module, fed from a recording, paced 2x
    """

    def step(self, *args: Any, **kwargs: Any) -> Any:  # pragma: no cover - overridden
        raise NotImplementedError(f"{type(self).__name__} must define step()")

    @property
    def health(self) -> HealthView:
        """Read-only health surface (see :class:`HealthView`).

        Available only after ``start()`` — a pure ``over()`` run has no
        runtime to monitor.
        """
        view: HealthView | None = getattr(self, "_health_view", None)
        if view is None:
            raise RuntimeError(
                f"{type(self).__name__}.health is available only after start() "
                f"(no health monitor on an offline over() run)"
            )
        return view

    # -- plan -----------------------------------------------------------------

    @classmethod
    def _plan(cls) -> _Plan:
        plan = cls.__dict__.get("_cached_plan")
        if plan is None:
            plan = cls._build_plan()
            cls._cached_plan = plan  # type: ignore[attr-defined]
        return plan

    @classmethod
    def _build_plan(cls) -> _Plan:
        if cls.step is PureModule.step:
            raise TypeError(f"{cls.__name__} must define step()")

        hints = get_type_hints(cls, include_extras=True)
        ins: dict[str, type] = {}
        outs: dict[str, type] = {}
        samplers: dict[str, Sampler] = {}
        trigger: str | None = None
        expect_hz: dict[str, float] = {}
        missing_ratio: dict[str, float] = {}
        out_min_hz: dict[str, float] = {}

        for name, ann in hints.items():
            if get_origin(ann) is typing.Annotated:
                inner = get_args(ann)[0]
                if get_origin(inner) in (In, Out):
                    raise TypeError(
                        f"{cls.__name__}.{name}: Annotated ports are not supported "
                        f"(Module won't create the port) — declare the sampler as a "
                        f"default instead: `{name}: In[...] = latest()`"
                    )
                continue
            origin = get_origin(ann)
            if origin is In:
                if name in ("ts", "state", "out", "health"):
                    raise TypeError(
                        f"{cls.__name__}.{name}: 'ts', 'state', 'out' and 'health' "
                        f"are reserved names"
                    )
                ins[name] = (get_args(ann) or (object,))[0]
                sampler = inspect.getattr_static(cls, name, None)
                if isinstance(sampler, Tick):
                    if trigger is not None:
                        raise TypeError(
                            f"{cls.__name__}: multiple tick() inputs ({trigger!r}, {name!r}) — "
                            f"exactly one input must fire the ticks"
                        )
                    trigger = name
                elif isinstance(sampler, Sampler):
                    samplers[name] = sampler
                else:
                    samplers[name] = latest()
                declared = sampler if isinstance(sampler, Sampler) else None
                if declared is not None:
                    if declared.expect_hz is not None:
                        expect_hz[name] = declared.expect_hz
                    if declared.max_missing_ratio is not None:
                        missing_ratio[name] = declared.max_missing_ratio
            elif origin is Out:
                if name == "health":
                    raise TypeError(
                        f"{cls.__name__}.{name}: 'health' is a reserved name "
                        f"(shadows module.health)"
                    )
                outs[name] = (get_args(ann) or (object,))[0]
                marker = inspect.getattr_static(cls, name, None)
                if isinstance(marker, OutContract) and marker.min_hz is not None:
                    out_min_hz[name] = marker.min_hz

        if trigger is None:
            raise TypeError(
                f"{cls.__name__}: no tick() input — mark exactly one input with "
                f"`name: In[...] = tick()` to define when the module steps"
            )

        sig = inspect.signature(cls.step)
        try:
            step_hints = get_type_hints(cls.step, include_extras=True)
        except (NameError, AttributeError, TypeError):
            step_hints = {}

        params: list[_Param] = []
        stateful = False
        uses_out = False
        names = [n for n in sig.parameters if n != "self"]
        for name in names:
            if name == "state":
                if names[0] != "state":
                    raise TypeError(f"{cls.__name__}.step: 'state' must be the first parameter")
                stateful = True
                continue
            if name == "ts":
                params.append(_Param("ts", "ts", optional=False, wants_obs=False))
                continue
            if name == "out":
                uses_out = True
                continue
            if name not in ins:
                raise TypeError(
                    f"{cls.__name__}.step parameter {name!r} doesn't match an input — "
                    f"declared inputs: {sorted(ins)} (reserved: ts, state, out)"
                )
            ann, optional = _unwrap_optional(step_hints.get(name, Any))
            if isinstance(samplers.get(name), Window):
                args = get_args(ann)
                wants = bool(args) and _wants_obs(args[0])
            else:
                wants = _wants_obs(ann)
            kind = "trigger" if name == trigger else "field"
            params.append(_Param(name, kind, optional=optional, wants_obs=wants))

        return _Plan(
            trigger=trigger,
            samplers=samplers,
            ins=ins,
            outs=outs,
            params=tuple(params),
            stateful=stateful,
            uses_out=uses_out,
            expect_hz=expect_hz,
            missing_ratio=missing_ratio,
            out_min_hz=out_min_hz,
        )

    # -- binding & dispatch -----------------------------------------------------

    def _bind(
        self, plan: _Plan, tobs: Observation[Any], row: dict[str, Any]
    ) -> tuple[dict[str, Any] | None, dict[str, float], list[str]]:
        """Build step kwargs for one tick.

        Returns ``(kwargs, ages, missing)`` — kwargs is None when the tick
        must drop, ``missing`` names the required fields that were absent,
        ``ages`` is staleness of consumed ``latest()`` values (tick time
        minus observation time), for health reporting.
        """
        kwargs: dict[str, Any] = {}
        ages: dict[str, float] = {}
        missing: list[str] = []
        for p in plan.params:
            if p.kind == "ts":
                kwargs[p.name] = tobs.ts
            elif p.kind == "trigger":
                kwargs[p.name] = tobs if p.wants_obs else tobs.data
            else:
                val = row[p.name]
                if val is MISSING:
                    if not p.optional:
                        missing.append(p.name)
                    kwargs[p.name] = None
                elif isinstance(val, list):
                    kwargs[p.name] = val if p.wants_obs else [o.data for o in val]
                else:
                    if isinstance(plan.samplers.get(p.name), Latest):
                        ages[p.name] = tobs.ts - val.ts
                    kwargs[p.name] = val if p.wants_obs else val.data
        if missing:
            return None, ages, missing
        return kwargs, ages, missing

    def _invoke(
        self, plan: _Plan, state: Any, kwargs: dict[str, Any]
    ) -> tuple[Any, dict[str, Any]]:
        """Run step; returns (new_state, {out_name: value})."""
        if plan.uses_out:
            writer = Outputs(frozenset(plan.outs))
            if plan.stateful:
                # With the writer, the return value is just the new state.
                state = self.step(state, out=writer, **kwargs)
            else:
                ret = self.step(out=writer, **kwargs)
                if ret is not None:
                    raise TypeError(
                        f"{type(self).__name__}.step declares 'out' so outputs are "
                        f"set on it — return None, got {type(ret).__name__}"
                    )
            return state, dict(writer._values)

        if plan.stateful:
            result = self.step(state, **kwargs)
            if not (isinstance(result, tuple) and len(result) == 2):
                raise TypeError(
                    f"{type(self).__name__}.step declares 'state' so it must "
                    f"return (new_state, output), got {type(result).__name__}"
                )
            state, out = result
        else:
            out = self.step(**kwargs)

        if out is None or not plan.outs:
            return state, {}
        if len(plan.outs) == 1:
            return state, {next(iter(plan.outs)): out}
        if not isinstance(out, dict):
            raise TypeError(
                f"{type(self).__name__}.step must return a dict over its Out ports "
                f"{sorted(plan.outs)}, got {type(out).__name__}"
            )
        unknown = set(out) - set(plan.outs)
        if unknown:
            raise TypeError(
                f"{type(self).__name__}.step returned unknown outputs {sorted(unknown)}"
            )
        return state, out

    # -- offline ------------------------------------------------------------------

    @classmethod
    def offline(cls, **config: Any) -> Self:
        """Construct without module machinery (no event loop, RPC, or ports).

        Enough of an instance to run :meth:`over` — ``self.config`` works,
        live deployment doesn't. This is what notebooks want.
        """
        self = cls.__new__(cls)
        Configurable.__init__(self, **config)
        CompositeResource.__init__(self)
        return self

    @_class_or_instance
    def over(self, _strict: bool = False, **streams: Any) -> Stream[Any]:
        """Run this module over stored/finite streams; returns a lazy output stream.

        Pass one memory2 stream per declared input, by name. The result
        is a regular lazy :class:`~dimos.memory2.stream.Stream` — chain
        ``.to_list()``, ``.save(...)``, ``.map_data(...)`` etc. With one
        ``Out`` port the observations carry the output values; with
        several they carry ``{port_name: value}`` dicts. Output
        observations derive from the trigger observation (its ``ts``,
        ``pose``, ``tags``).

        Each stream must iterate in ascending ``ts`` (stored streams in
        insertion order do; otherwise prepend ``.order_by("ts")``). Don't
        pass ``.live()`` streams — deploy the module for that.

        ``_strict=True`` raises on the first tick dropped for missing
        required inputs instead of counting it — offline a drop usually
        means a data or declaration bug, and replay-determinism tests
        should fail loudly.
        """
        plan = self._plan()
        missing = set(plan.ins) - set(streams)
        extra = set(streams) - set(plan.ins)
        if missing or extra:
            raise TypeError(
                f"{type(self).__name__}.over() inputs mismatch — "
                f"missing: {sorted(missing) or '—'}, unknown: {sorted(extra) or '—'}"
            )

        secondaries = {name: streams[name] for name in plan.samplers}

        def _run(upstream: Any) -> Any:
            machine = TickMachine(plan.trigger, plan.samplers)
            state = self.initial_state
            dropped: dict[str, int] = {}

            def emit(resolved: list[tuple[Observation[Any], dict[str, Any]]]) -> Any:
                nonlocal state
                for tobs, row in resolved:
                    kwargs, _ages, missing_fields = self._bind(plan, tobs, row)
                    if kwargs is None:
                        if _strict:
                            raise ValueError(
                                f"{type(self).__name__}: tick at ts={tobs.ts} missing "
                                f"required inputs {missing_fields} (strict mode)"
                            )
                        for f in missing_fields:
                            dropped[f] = dropped.get(f, 0) + 1
                        continue
                    state, outs = self._invoke(plan, state, kwargs)
                    if not outs:
                        continue
                    if len(plan.outs) <= 1:
                        yield tobs.derive(data=next(iter(outs.values())))
                    else:
                        yield tobs.derive(data=outs)

            for name, obs in merge_events(plan.trigger, upstream, secondaries):
                yield from emit(machine.process(name, obs))
            yield from emit(machine.flush())
            if dropped:
                logger.info(
                    "%s: dropped %d ticks with missing required inputs (%s)",
                    type(self).__name__,
                    sum(dropped.values()),
                    ", ".join(f"{f}: {n}" for f, n in sorted(dropped.items())),
                )

        trigger_stream = streams[plan.trigger]
        return trigger_stream.transform(_run)  # type: ignore[no-any-return]

    # -- live -----------------------------------------------------------------------

    def make_store(self) -> Store:
        """Store bridging the ports to memory2 streams when deployed.

        Default is a :class:`NullStore` — inputs/outputs behave as
        live-only streams with no history. Return a storage-backed store
        (e.g. ``SqliteStore``) to record every input and output of the
        running module.
        """
        return NullStore()

    @rpc
    def start(self) -> None:
        super().start()
        plan = self._plan()
        cfg = self.config

        store = self._store = self.register_disposable(self.make_store())
        store.start()
        self._streams = {name: store.stream(name, port.type) for name, port in self.inputs.items()}
        self._out_streams = {
            name: store.stream(name, port.type) for name, port in self.outputs.items()
        }

        health_stream = self._health_stream = (
            store.stream("_health", dict) if cfg.health.stream else None
        )

        def _sink(h: Health) -> None:
            assert health_stream is not None
            health_stream.append(
                h.metrics, ts=h.ts, tags={"state": h.state, "violations": list(h.violations)}
            )

        monitor = HealthMonitor(
            str(self),
            contracts=cfg.contracts,
            health=cfg.health,
            # per-port contracts come from the class declaration (the plan)
            expected_hz=plan.expect_hz,
            out_min_hz=plan.out_min_hz,
            missing_ratio_by_input=plan.missing_ratio,
            sink=_sink if health_stream is not None else None,
        )
        self.health_monitor = monitor
        self._health_view = HealthView(monitor, health_stream)

        q: queue.SimpleQueue[Any] = queue.SimpleQueue()
        self._queue = q
        ticks = self._tick_buffer = self.backpressure.clone()
        machine = TickMachine(plan.trigger, plan.samplers, max_pending=cfg.max_pending_ticks)
        monitor.attach_gauges(
            buffer_len=lambda: len(ticks), pending_len=lambda: len(machine.pending)
        )

        sources = dict(self.input_sources or {})
        unknown_sources = set(sources) - set(plan.ins)
        if unknown_sources:
            raise TypeError(
                f"{type(self).__name__}.input_sources has unknown inputs "
                f"{sorted(unknown_sources)} — declared: {sorted(plan.ins)}"
            )

        def _ingest(name: str, item: Any) -> None:
            """Feed one arrival — a raw payload (port/replay) or an Observation."""
            if isinstance(item, Observation):
                obs = item  # sourced from a store: keep the recorded ts
                self._streams[name].append(obs.data, ts=obs.ts)
            else:
                ts = getattr(item, "ts", None) or time.time()
                self._streams[name].append(item, ts=ts)
                obs = Observation(ts=ts, data_type=type(item), _data=item)
            monitor.on_input(name)
            q.put((name, obs))

        def _source_error(name: str, e: Exception) -> None:
            logger.exception("%s: input source %r failed: %s", self, name, e)

        for name, port in self.inputs.items():
            if name not in plan.ins:
                continue
            source = sources.get(name)
            if source is None:
                self.register_disposable(
                    Disposable(port.subscribe(functools.partial(_ingest, name)))
                )
            else:
                observable = source.observable() if hasattr(source, "observable") else source
                self.register_disposable(
                    observable.subscribe(
                        on_next=functools.partial(_ingest, name),
                        on_error=functools.partial(_source_error, name),
                    )
                )

        def _align_loop() -> None:
            """Drain raw events fast; resolve + bind ticks; never blocks on step."""
            blocked_seen = 0
            while True:
                try:
                    item = q.get(timeout=cfg.health.interval_s)
                except queue.Empty:
                    monitor.maybe_report()
                    continue
                try:
                    resolved = machine.flush() if item is _STOP else machine.process(*item)
                    if machine.blocked_dropped > blocked_seen:
                        monitor.on_blocked(machine.blocked_dropped - blocked_seen)
                        blocked_seen = machine.blocked_dropped
                    for tobs, row in resolved:
                        monitor.on_resolved()
                        kwargs, ages, missing = self._bind(plan, tobs, row)
                        if kwargs is None:
                            monitor.on_missing(missing)
                            continue
                        monitor.on_queued()
                        ticks.put((tobs, kwargs, ages))
                except Exception:
                    logger.exception("%s: alignment failed for an event", self)
                monitor.maybe_report()
                if item is _STOP:
                    ticks.close()
                    return

        def _step_loop() -> None:
            """Pull ticks at step pace — the backpressure policy decides what it sees."""
            state = self.initial_state
            while True:
                try:
                    tobs, kwargs, ages = ticks.take(timeout=cfg.health.interval_s)
                except TimeoutError:
                    monitor.maybe_report()
                    continue
                except ClosedError:
                    return
                t0 = time.perf_counter()
                try:
                    state, outs = self._invoke(plan, state, kwargs)
                except Exception:
                    logger.exception("%s.step failed for tick ts=%s", self, tobs.ts)
                    continue
                duration = time.perf_counter() - t0
                for out_name, value in outs.items():
                    monitor.on_output(out_name)
                    try:
                        self.outputs[out_name].publish(value)
                        self._out_streams[out_name].append(value, ts=tobs.ts)
                    except Exception:
                        logger.exception("%s: publishing %s failed", self, out_name)
                # Live observation ts is arrival wall-clock, so this spans the
                # whole path: queue wait + alignment + buffer wait + step + publish.
                monitor.on_step(
                    duration,
                    ages,
                    emitted=bool(outs),
                    latency_s=max(0.0, time.time() - tobs.ts),
                )
                monitor.maybe_report()

        self._threads = [
            threading.Thread(target=_align_loop, name=f"{self}.align", daemon=True),
            threading.Thread(target=_step_loop, name=f"{self}.step", daemon=True),
        ]
        for t in self._threads:
            t.start()

        def _shutdown() -> None:
            q.put(_STOP)
            for t in self._threads:
                t.join(2.0)

        self.register_disposable(Disposable(_shutdown))

    @rpc
    def stop(self) -> None:
        super().stop()

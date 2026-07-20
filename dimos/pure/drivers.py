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

"""Step drivers + ``run_over``: drive classified steps over aligned rows (T6).

Spec: ``dimos/pure/tasks/t6-drivers.md``. Engine layer: imports the data
layer (``rows``/``stepspec``/``typing``) and, lazily at run time, the T5
aligner — NEVER ``module.py``/``config.py`` (drivers drive anything
step-shaped; the class layer's only engine edge is ``over()``'s lazy import,
spec §1). ``run_over`` composes: align streams -> drive rows -> yield
stamped Out rows, with teardown per spec §8.
"""

from __future__ import annotations

import asyncio
from collections import deque
from collections.abc import AsyncGenerator, Awaitable, Callable, Iterator, Mapping
import dataclasses
import enum
import re
import sys
from typing import TYPE_CHECKING, Any, Final, TypeVar

from dimos.pure.rows import UNSTAMPED, TfOutSpec, TfSpec
from dimos.pure.stepspec import StepKind, StepSpec
from dimos.pure.typing import AsyncStateless, Fold, Mealy, Stamped, Stateless, Streamable
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from _typeshed import DataclassInstance

__all__ = [
    "DEFAULT_MAX_INFLIGHT",
    "PureModuleRunError",
    "RunHooks",
    "RunRule",
    "StepError",
    "drive_async",
    "drive_fold",
    "drive_mealy",
    "drive_stateless",
    "run_over",
]

DEFAULT_MAX_INFLIGHT: Final[int] = 1
"""Async in-flight window for modules declaring no ``max_inflight`` field (spec §6.1)."""

_LOG: Final = setup_logger()  # reachable in forkserver workers (bare getLogger is swallowed)


class RunRule(enum.Enum):
    """Every runtime driver rule, by its message slug (spec §10)."""

    UNKNOWN_STREAM = "run-unknown-stream"
    INSIDE_LOOP = "run-inside-loop"
    BAD_MAX_INFLIGHT = "run-bad-max-inflight"
    STEP_ERROR = "step-error"
    STEP_NONE_NO_SKIP = "step-none-no-skip"
    FOLD_YIELDED_NONE = "fold-yielded-none"
    FOLD_UNSTAMPED = "fold-unstamped"
    FOLD_FUTURE_TS = "fold-future-ts"
    FOLD_NONMONOTONIC = "fold-nonmonotonic"


class PureModuleRunError(RuntimeError):
    """A pure-module run violated a runtime contract (spec §10.1)."""

    rule: RunRule | None

    def __init__(self, message: str, rule: RunRule | None = None) -> None:
        """Message is release copy per spec §10.3; ``rule`` is machine-readable."""
        super().__init__(message)
        self.rule = rule


class StepError(PureModuleRunError):
    """User step/fold code raised; always chained ``from`` the original (spec §10.2)."""


# T7 RESOURCE SEAM — _noop/_anoop are the default warmup+teardown seam callables
def _noop() -> None:
    """Default ``RunHooks`` seam callable: nothing attached (T7 fills the seam)."""
    return None


async def _anoop() -> None:
    """Default ``RunHooks`` seam callable: nothing attached (T7 fills the seam)."""
    return None


@dataclasses.dataclass
class RunHooks:
    """Per-run tick accounting plus the T7 disposal seam (spec §9, §8.4).

    Counters are plain monotonic ints mutated only by the run's own thread;
    readers (T9) take lock-free, possibly momentarily-stale snapshots.
    """

    ticks: int = 0
    emits: int = 0
    skips: int = 0
    drops: int = 0
    errors: int = 0
    last_error: BaseException | None = None
    warmup: Callable[[], None] = _noop  # T7 RESOURCE SEAM — sync-run creation (t7 §6)
    awarmup: Callable[[], Awaitable[None]] = _anoop  # T7 RESOURCE SEAM — async creation on loop
    teardown: Callable[[], None] = _noop
    ateardown: Callable[[], Awaitable[None]] = _anoop


_T = TypeVar("_T")
_TInRow = TypeVar("_TInRow", bound=Stamped)
_TOutRow = TypeVar("_TOutRow", bound="DataclassInstance")  # replace()-stamped (spec §2)
_TFoldOut = TypeVar("_TFoldOut", bound=Stamped)  # fold ts is read, never replaced
_TState = TypeVar("_TState")


# ── message helpers (templates: spec §10.3) ──────────────────────────────────

_IDENTIFIER_RE: Final = re.compile(r"[A-Za-z_][A-Za-z0-9_.]*\Z")


def _cls(module: Any) -> str:
    """``{module}.{qualname}`` of the module's class — the ``{cls}`` of §10.3."""
    t = type(module)
    return f"{t.__module__}.{t.__qualname__}"


def _out_annotation(module: Any) -> str:
    """Best-effort Out name for the none-no-skip message; ``"Out"`` when unrecoverable."""
    fn = getattr(type(module), "step", None)
    ann = getattr(fn, "__annotations__", {}).get("return")
    if isinstance(ann, type):
        return ann.__name__
    if isinstance(ann, str) and _IDENTIFIER_RE.match(ann):
        return ann
    return "Out"


def _none_no_skip_msg(module: Any, ts: float) -> str:
    out = _out_annotation(module)
    return (
        f"{_cls(module)}.step returned None at tick ts={ts} but is annotated '-> {out}' — "
        f"a never-skip step must emit every tick. Annotate '-> {out} | None' if some "
        f"ticks legitimately skip. [step-none-no-skip]"
    )


def _step_error_msg(module: Any, ts: float, n: int, exc: Exception) -> str:
    return f"{_cls(module)}.step raised at tick ts={ts} (tick #{n}): {exc!r} [step-error]"


def _fold_error_msg(module: Any, cursor: _FoldCursor, exc: Exception) -> str:
    return (
        f"{_cls(module)}.fold raised after consuming {cursor.pulled} rows, newest input "
        f"ts={cursor.newest_in}: {exc!r} [step-error]"
    )


# ── run_over: eager validation + composition (spec §4) ───────────────────────


def run_over(
    module: Any,  # Any by design: drivers never import the class layer (spec §1)
    spec: StepSpec,
    streams: Mapping[str, Streamable],
    *,
    tf: Any | None = None,  # runtime-untyped like module; static type lives on over() (spec §8.3)
    hooks: RunHooks | None = None,
) -> Iterator[Any]:
    """Validate eagerly, align via T5, dispatch on ``spec.kind`` (spec §4)."""
    hooks = RunHooks() if hooks is None else hooks
    # ── eager (at the over() call): pure validation, zero acquisition (D1) ──
    _check_unknown_streams(module, spec, streams)
    max_inflight = DEFAULT_MAX_INFLIGHT
    if spec.kind is StepKind.ASYNC_STATELESS:
        raw = getattr(module, "max_inflight", DEFAULT_MAX_INFLIGHT)
        max_inflight = _checked_max_inflight(module, raw)
        _reject_running_loop(module)
    initial: Any = None
    if spec.kind is StepKind.MEALY:
        state_type = spec.state_type
        assert state_type is not None  # T3: a MEALY spec always carries State
        initial = state_type()  # D16: a buggy State.__init__ errors at the call site
    from dimos.pure.resources import attach_resources  # T7 RESOURCE SEAM — lazy, permanently

    attach_resources(module, hooks, async_run=spec.kind is StepKind.ASYNC_STATELESS)
    # ── tf side channel (T11): lazy, only when tf= given or the bundles declare edges ──
    in_tf, out_tf = _has_tf_fields(spec)
    ctx: Any | None = None
    if tf is not None or in_tf or out_tf:
        from dimos.pure.tfbuffer import TfContext  # lazy — leaf engine (spec §8.3)

        ctx = TfContext.for_over(module, spec, tf)  # claims/wiring raise here, caller thread
    from dimos.pure.align import align  # lazy, permanently (spec §4)

    rows = align(spec.in_type, {name: _coerce_stream(s) for name, s in streams.items()}, tf=ctx)
    # ── dispatch: drivers self-finalize (§8.2); the tf tap wraps the out iterator ──
    if spec.kind is StepKind.MEALY:
        driver = drive_mealy(module, rows, initial, skips=spec.skips, hooks=hooks)
    elif spec.kind is StepKind.ASYNC_STATELESS:
        driver = drive_async(module, rows, max_inflight=max_inflight, skips=spec.skips, hooks=hooks)
    elif spec.kind is StepKind.FOLD:
        driver = drive_fold(module, rows, hooks=hooks)
    else:
        driver = drive_stateless(module, rows, skips=spec.skips, hooks=hooks)
    if ctx is None:
        return driver
    if out_tf:
        from dimos.pure.tfbuffer import tf_out_tap  # lazy (spec §7.1)

        return tf_out_tap(driver, module, ctx)
    return _tf_released(driver, ctx)  # no tf_out fields: still release claims on teardown


def _has_tf_fields(spec: StepSpec) -> tuple[bool, bool]:
    """(In declares tf(), Out declares tf_out()) — the tf-context trigger (spec §8.3)."""
    in_tf = any(isinstance(s, TfSpec) for s in spec.in_type.fields().values())
    out_tf = any(isinstance(s, TfOutSpec) for s in spec.out_type.fields().values())
    return in_tf, out_tf


def _tf_released(driver: Iterator[_T], ctx: Any) -> Iterator[_T]:
    """Wrap a tf-consumer driver (no tf_out) so claims release once on teardown (spec §8.3)."""
    try:
        yield from driver
    finally:
        ctx.release()


def _check_unknown_streams(module: Any, spec: StepSpec, streams: Mapping[str, Streamable]) -> None:
    """Fast unknown-name check (A3); T5's ``[align-unknown-port]`` stays authoritative."""
    fields = spec.in_type.fields()
    unknown = [name for name in streams if name not in fields]
    if not unknown:
        return
    cls = _cls(module)
    names = ", ".join(repr(n) for n in unknown)
    declared = ", ".join(repr(n) for n in fields)
    raise PureModuleRunError(
        f"{cls}.over() got unknown stream(s) {names}; In fields are: {declared}. "
        f"Stream kwargs must match {cls}.In field names. [run-unknown-stream]",
        RunRule.UNKNOWN_STREAM,
    )


def _checked_max_inflight(module: Any, value: Any) -> int:
    """Validate the D6 config-field convention: an int >= 1, bools excluded."""
    if isinstance(value, int) and not isinstance(value, bool) and value >= 1:
        return value
    raise PureModuleRunError(
        f"{_cls(module)}.max_inflight must be an int >= 1, got {value!r} — it bounds the "
        f"async in-flight window (1 = serial). [run-bad-max-inflight]",
        RunRule.BAD_MAX_INFLIGHT,
    )


def _reject_running_loop(module: Any) -> None:
    """Sync ``over()`` cannot be driven from inside a running event loop (spec §6.2)."""
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return
    raise PureModuleRunError(
        f"{_cls(module)}.over() cannot run inside a running event loop — over() blocks, "
        f"driving a private loop for async steps. Iterate it from sync code, or move the "
        f"call off the loop (e.g. a worker thread). [run-inside-loop]",
        RunRule.INSIDE_LOOP,
    )


# ── the memory2 boundary (index reconciliation; t5-align.md §11.1) ───────────


def _coerce_stream(value: Streamable) -> Streamable:
    """Adapt the memory2 currency at the ``over()`` boundary; all else verbatim.

    A ``dimos.memory2.stream.Stream`` yields ``Observation`` envelopes; passed
    verbatim they would land in row fields (T5 D2 puts stream items in fields
    unchanged). Unwrap to the ``obs.data`` payloads — the ONE ts authority is
    the payload's own ``ts`` (T5 §3); an unstamped payload then fails loudly at
    ingestion (``[align-bad-item]``, naming the port) rather than silently
    riding the discarded envelope ts. Recognition via ``sys.modules``: a Stream
    instance cannot exist unless its module is already imported, so the engine
    never imports memory2 itself.
    """
    mod = sys.modules.get("dimos.memory2.stream")
    if mod is None:
        return value
    stream_cls = getattr(mod, "Stream", None)
    if isinstance(stream_cls, type) and isinstance(value, stream_cls):
        return _ObservationPayloads(value)
    return value


class _ObservationPayloads:
    """Re-iterable payload view of a memory2 ``Stream`` (``obs.data`` per item)."""

    def __init__(self, stream: Any) -> None:
        self._stream = stream

    def __iter__(self) -> Iterator[Any]:
        # The outermost iter() runs eagerly at genexpr creation, so a Stream
        # that cannot iterate (e.g. unbound) still fails at align's iter() time.
        return (obs.data for obs in iter(self._stream))


# ── teardown composition (spec §8.2, D8) ─────────────────────────────────────


def _finalized(inner: Iterator[_T], rows: Iterator[Any], hooks: RunHooks) -> Iterator[_T]:
    """Compose a drive loop with the §8.2 teardown: inner-first, seam last."""
    try:
        hooks.warmup()  # T7 RESOURCE SEAM — create before the first row pull (t7 §7)
        yield from inner
    finally:
        try:
            close = getattr(rows, "close", None)
            if close is not None:
                close()  # T5 aligner cleanup (A2); drivers own rows (spec §2)
        finally:
            hooks.teardown()  # T7 seam — LAST, always, exactly once (§8.4)


# ── sync step drivers (spec §5) ──────────────────────────────────────────────


def drive_stateless(
    module: Stateless[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    *,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]:
    """Drive a sync stateless step over rows, stamping emissions with tick ts (spec §5.1)."""
    return _finalized(_stateless_rows(module, rows, skips, hooks), rows, hooks)


def _stateless_rows(
    module: Stateless[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]:
    n = 0
    for row in rows:
        hooks.ticks += 1
        n += 1
        try:
            out = module.step(row)
        except Exception as exc:
            hooks.errors += 1
            hooks.last_error = exc  # the raw original; the wrapper adds coordinates
            raise StepError(_step_error_msg(module, row.ts, n, exc), RunRule.STEP_ERROR) from exc
        if out is None:
            if skips:
                hooks.skips += 1
                continue
            hooks.errors += 1
            raise PureModuleRunError(_none_no_skip_msg(module, row.ts), RunRule.STEP_NONE_NO_SKIP)
        hooks.emits += 1
        yield dataclasses.replace(out, ts=row.ts)  # D11: unconditional engine stamp


def drive_mealy(
    module: Mealy[_TState, _TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    initial: _TState,
    *,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]:
    """Thread State from ``initial`` through a Mealy step, stamping emissions (spec §5.2)."""
    return _finalized(_mealy_rows(module, rows, initial, skips, hooks), rows, hooks)


def _mealy_rows(
    module: Mealy[_TState, _TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    initial: _TState,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]:
    state = initial  # ONE reference, rebound per tick — never copied or aliased
    n = 0
    for row in rows:
        hooks.ticks += 1
        n += 1
        try:
            state, out = module.step(state, row)  # non-2-tuple unpack fails here (D14)
        except Exception as exc:
            hooks.errors += 1
            hooks.last_error = exc
            raise StepError(_step_error_msg(module, row.ts, n, exc), RunRule.STEP_ERROR) from exc
        if out is None:
            if skips:
                hooks.skips += 1
                continue
            hooks.errors += 1
            raise PureModuleRunError(_none_no_skip_msg(module, row.ts), RunRule.STEP_NONE_NO_SKIP)
        hooks.emits += 1
        yield dataclasses.replace(out, ts=row.ts)


# ── the async driver (spec §6) ───────────────────────────────────────────────


def drive_async(
    module: AsyncStateless[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    *,
    max_inflight: int,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]:
    """Windowed async driver behind a sync facade owning a private loop (spec §6)."""
    return _finalized(_async_facade(module, rows, max_inflight, skips, hooks), rows, hooks)


def _async_facade(
    module: AsyncStateless[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    max_inflight: int,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]:
    # Generator start (first next()): re-validate for direct callers (§6.1) and
    # re-probe for iterators smuggled into a running loop (§6.2).
    max_inflight = _checked_max_inflight(module, max_inflight)
    _reject_running_loop(module)
    loop = asyncio.new_event_loop()  # private; never set_event_loop, no policy touch
    agen = _drive_async_rows(module, rows, max_inflight, skips, hooks)
    try:
        # Stepped, not threaded: the loop runs only inside these calls (§6.2).
        while True:
            try:
                out = loop.run_until_complete(agen.__anext__())
            except StopAsyncIteration:
                return
            yield out
    finally:
        try:
            try:
                # Triggers CLOSING (§6.3): cancel window, reap, await ateardown.
                loop.run_until_complete(agen.aclose())
            finally:
                loop.run_until_complete(loop.shutdown_asyncgens())
        finally:
            loop.close()


async def _drive_async_rows(
    module: AsyncStateless[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    max_inflight: int,
    skips: bool,
    hooks: RunHooks,
) -> AsyncGenerator[_TOutRow, None]:
    # The window deque IS the reorder buffer: completed-but-unemitted results
    # wait inside their Tasks, in tick order, popped head-first (§6.3).
    window: deque[tuple[float, asyncio.Task[_TOutRow | None]]] = deque()
    exhausted = False
    try:
        await hooks.awarmup()  # T7 RESOURCE SEAM — create on the run loop, before FILL (t7 §7.3)
        while True:
            # FILL — admit tasks up to max_inflight, in tick order.
            while not exhausted and len(window) < max_inflight:
                try:
                    row = next(rows)
                except StopIteration:
                    exhausted = True
                    break
                hooks.ticks += 1
                task = asyncio.ensure_future(module.step(row))  # Awaitable, not Coroutine
                window.append((row.ts, task))
            if not window:
                return  # DONE — exhausted and drained (FILL admits otherwise)
            # AWAIT_HEAD — head-of-line; younger tasks progress under the same
            # loop. DRAIN is emergent: once exhausted, FILL admits nothing and
            # this loop empties the window in tick order.
            ts, task = window.popleft()
            try:
                out = await task
            except asyncio.CancelledError:
                raise  # cancellation machinery — never counted (§6.4)
            except Exception as exc:
                # D5: counted dropped tick; loudness by accounting, not dying.
                hooks.drops += 1
                hooks.last_error = exc
                _LOG.warning(
                    "%s.step raised at tick ts=%s; tick dropped, run continues",
                    _cls(module),
                    ts,
                    exc_info=exc,
                )
                continue
            if out is None:
                if skips:
                    hooks.skips += 1
                    continue
                hooks.errors += 1  # engine violations propagate, never drop (§6.4)
                raise PureModuleRunError(_none_no_skip_msg(module, ts), RunRule.STEP_NONE_NO_SKIP)
            hooks.emits += 1
            yield dataclasses.replace(out, ts=ts)  # D11 stamp at the tick ts
    finally:
        # CLOSING — discard only where emission is impossible by definition:
        # cancel the window, bounded reap, then the async seam (§6.3, §8.4).
        try:
            if window:
                tasks = [task for _, task in window]
                for pending in tasks:
                    pending.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
        finally:
            await hooks.ateardown()  # on the run loop, after quiescence (§8.4)


# ── the fold driver (spec §7) ────────────────────────────────────────────────


@dataclasses.dataclass
class _FoldCursor:
    """Driver-local fold cursors (§7.3): running-max input ts + emission cursor."""

    newest_in: float = float("-inf")
    last_emitted: float = float("-inf")
    pulled: int = 0


class _CountingRows(Iterator[_TInRow]):
    """Lazy single-pass counting proxy handed to fold (§7.1); fold's pulls only."""

    def __init__(self, rows: Iterator[_TInRow], hooks: RunHooks, cursor: _FoldCursor) -> None:
        self._rows = rows
        self._hooks = hooks
        self._cursor = cursor

    def __next__(self) -> _TInRow:
        row = next(self._rows)  # StopIteration propagates to the fold
        self._hooks.ticks += 1
        self._cursor.pulled += 1
        ts = row.ts
        if ts > self._cursor.newest_in:  # running max — a T5 equal-ts tie can't trip it
            self._cursor.newest_in = ts
        return row


def drive_fold(
    module: Fold[_TInRow, _TFoldOut],
    rows: Iterator[_TInRow],
    *,
    hooks: RunHooks,
) -> Iterator[_TFoldOut]:
    """Hand fold one lazy pass over rows; validate self-stamped Out ts (spec §7)."""
    return _finalized(_fold_rows(module, rows, hooks), rows, hooks)


def _fold_rows(
    module: Fold[_TInRow, _TFoldOut],
    rows: Iterator[_TInRow],
    hooks: RunHooks,
) -> Iterator[_TFoldOut]:
    cursor = _FoldCursor()
    try:
        gen = module.fold(_CountingRows(rows, hooks, cursor))
    except Exception as exc:  # a non-generator fold can raise at the call itself
        hooks.errors += 1
        hooks.last_error = exc
        raise StepError(_fold_error_msg(module, cursor, exc), RunRule.STEP_ERROR) from exc
    try:
        while True:
            try:
                out = next(gen)
            except StopIteration:
                return
            except Exception as exc:  # incl. PEP 479's RuntimeError for escaped StopIteration
                hooks.errors += 1
                hooks.last_error = exc
                raise StepError(_fold_error_msg(module, cursor, exc), RunRule.STEP_ERROR) from exc
            _check_fold_row(module, out, cursor, hooks)
            hooks.emits += 1
            yield out  # self-stamped: NEVER replaced (T1 D11 split)
    finally:
        close = getattr(gen, "close", None)
        if close is not None:
            close()  # exactly once; GeneratorExit into the fold's frame (§7.1)


def _check_fold_row(module: Any, out: Stamped | None, cursor: _FoldCursor, hooks: RunHooks) -> None:
    """The §7.3 validation chain: None -> UNSTAMPED -> causality cap -> monotonic."""
    cls = _cls(module)
    if out is None:
        hooks.errors += 1
        raise PureModuleRunError(
            f"{cls}.fold yielded None — a fold emits rows only; to skip a tick, don't "
            f"yield. [fold-yielded-none]",
            RunRule.FOLD_YIELDED_NONE,
        )
    ts = out.ts
    if ts == UNSTAMPED:
        hooks.errors += 1
        raise PureModuleRunError(
            f"{cls}.fold yielded an unstamped row (ts is UNSTAMPED) — fold stamps its own "
            f"rows: construct {type(out).__qualname__}(ts=..., ...) from an input row's ts "
            f"(newest consumed: ts={cursor.newest_in}). [fold-unstamped]",
            RunRule.FOLD_UNSTAMPED,
        )
    if ts > cursor.newest_in:
        hooks.errors += 1
        if cursor.pulled == 0:
            raise PureModuleRunError(
                f"{cls}.fold yielded ts={ts} but it has not consumed any input yet — a "
                f"fold's time authority is its input rows. [fold-future-ts]",
                RunRule.FOLD_FUTURE_TS,
            )
        raise PureModuleRunError(
            f"{cls}.fold yielded ts={ts} but the newest input it has consumed is "
            f"{cursor.newest_in} — a fold cannot stamp ahead of its input. [fold-future-ts]",
            RunRule.FOLD_FUTURE_TS,
        )
    if ts <= cursor.last_emitted:
        hooks.errors += 1
        raise PureModuleRunError(
            f"{cls}.fold yielded ts={ts} after already emitting ts={cursor.last_emitted} — "
            f"fold output ts must be strictly increasing (replay identity requires a total "
            f"order per edge). [fold-nonmonotonic]",
            RunRule.FOLD_NONMONOTONIC,
        )
    cursor.last_emitted = ts

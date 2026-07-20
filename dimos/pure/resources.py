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

"""``@resource``: lazy cached property outside runs, per-run lifecycle inside (T7).

Spec: ``dimos/pure/tasks/t7-resources.md``. Outside a run the descriptor is a
thread-safe lazy cache per module instance (tests just touch ``m.grid``);
inside a run, resources are created at warmup in declaration order and
disposed in reverse at stop — ``dispose()``/``close()``/``aclose()`` sniffed
in that order, ``over()`` teardown on the same path via the T6 ``RunHooks``
seam. Imports drivers top-level (``RunHooks``/``PureModuleRunError``);
drivers imports us lazily inside ``run_over`` — acyclic (spec §1).
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable, Coroutine
from contextvars import ContextVar
import enum
import inspect
import threading
from typing import Any, Final, Generic, Protocol, TypeVar, overload

from dimos.pure.drivers import PureModuleRunError, RunHooks
from dimos.pure.stepspec import PureModuleDefinitionError
from dimos.utils.logging_config import setup_logger

__all__ = [
    "IN_HANDLER",
    "Resource",
    "ResourceDefinitionError",
    "ResourceError",
    "ResourceRule",
    "RunResources",
    "attach_resources",
    "resource",
]

_T = TypeVar("_T")

_LOG: Final = setup_logger()  # reachable in forkserver workers (bare getLogger is swallowed)

_RUN_ATTR: Final = "__pure_run_resources__"
"""Instance slot for the per-run ``RunResources`` context (spec §5.2); ``None`` between runs."""

_LAZY_ATTR: Final = "__pure_lazy_resources__"
"""Instance slot for the lazy test-mode cache dict (spec §5.1)."""

_LAZY_LOCK: Final = threading.RLock()
"""One coarse reentrant lock for all lazy creation — deadlock-free, test-mode only (spec §5.1)."""

_CREATING: Final = object()
"""In-progress sentinel: a slot holding it is under construction (cycle guard, spec §5.1)."""

_MISSING: Final = object()
"""Absent sentinel — presence is by key, never truthiness (``None`` is a valid value)."""

IN_HANDLER: Final[ContextVar[bool]] = ContextVar("dimos_pure_in_handler", default=False)
"""RPC handler-context flag (spec §10): while True, any resource access raises."""


class ResourceDefinitionError(PureModuleDefinitionError):
    """A ``@resource`` declaration is malformed (raised at class definition, spec §3.1)."""


class ResourceRule(enum.Enum):
    """Every runtime resource rule, by its message slug (spec §11)."""

    WARMUP_ERROR = "resource-warmup-error"
    ORDER = "resource-order"
    CYCLE = "resource-cycle"
    CONCURRENT_RUN = "resource-concurrent-run"
    ASYNC_SYNC_RUN = "resource-async-sync-run"
    ASYNC_LAZY_IN_LOOP = "resource-async-lazy-in-loop"
    DISPOSE_ERROR = "resource-dispose-error"
    IN_HANDLER = "resource-in-handler"


class ResourceError(PureModuleRunError):
    """Resource machinery violated a runtime contract (spec §11.1)."""

    resource_rule: ResourceRule | None

    def __init__(self, message: str, resource_rule: ResourceRule | None = None) -> None:
        """Message is release copy per spec §11.2; ``resource_rule`` is machine-readable."""
        super().__init__(message, None)
        self.resource_rule = resource_rule


# ── message templates (release copy: spec §11.2) ─────────────────────────────


def _cls(module: object) -> str:
    """``{module}.{qualname}`` of the module's class — the ``{cls}`` of §11.2."""
    t = type(module)
    return f"{t.__module__}.{t.__qualname__}"


def _warmup_error_msg(module: object, name: str, i: int, n: int, exc: BaseException) -> str:
    return (
        f"{_cls(module)}.{name} factory raised during warmup (resource {i} of {n}): "
        f"{exc!r} — already-created resources were disposed in reverse order. "
        f"[resource-warmup-error]"
    )


def _order_msg(module: object, name: str) -> str:
    return (
        f"{_cls(module)}.{name} was touched during a run before it was created — factories "
        f"may only use resources declared ABOVE them; move {name} earlier in the class body "
        f"or drop the reference. [resource-order]"
    )


def _cycle_msg(module: object, name: str) -> str:
    return (
        f"{_cls(module)}.{name} is being created and was touched again — resource factories "
        f"form a cycle; break it by removing the circular reference. [resource-cycle]"
    )


def _concurrent_msg(module: object) -> str:
    return (
        f"{_cls(module)} already has a run in flight — resources are per-run, so one "
        f"instance drives one run at a time. Build a second instance for a second run: "
        f"identity is class + config, and construction is free. [resource-concurrent-run]"
    )


def _async_sync_msg(module: object, name: str) -> str:
    return (
        f"{_cls(module)}.{name} is an async factory but the run is sync — a sync run has no "
        f"event loop for the resource to live on. Make step async, or make the factory sync. "
        f"[resource-async-sync-run]"
    )


def _async_lazy_msg(module: object, name: str) -> str:
    return (
        f"{_cls(module)}.{name} is an async factory touched from inside a running event "
        f"loop — attribute access cannot await. Touch it from sync code (it runs via "
        f"asyncio.run), or use it in a run. [resource-async-lazy-in-loop]"
    )


def _dispose_error_msg(module: object, k: int, n: int, names: str, exc: BaseException) -> str:
    return (
        f"{_cls(module)} teardown: {k} of {n} resources failed to dispose ({names}); "
        f"first failure: {exc!r} — every remaining resource was still disposed. "
        f"[resource-dispose-error]"
    )


def _in_handler_msg(module: object, name: str) -> str:
    return (
        f"{_cls(module)}.{name} was touched inside an RPC handler — handlers are pure "
        f"functions of (config, state, request); all I/O belongs behind step's resources. "
        f"[resource-in-handler]"
    )


class Resource(Generic[_T]):
    """Descriptor behind ``@resource``: run-context instance in runs, lazy cache outside."""

    factory: Callable[[Any], Any]  # undecorated (self) -> _T | Coroutine[Any, Any, _T]
    disposes: bool  # False under @resource(dispose=False) — never sniffed (spec §9)
    is_async: bool  # iscoroutinefunction(factory) (spec §7.3)
    name: str  # attribute name, from __set_name__
    owner: type | None  # defining class, from __set_name__

    def __init__(self, factory: Callable[[Any], Any], *, dispose: bool = True) -> None:
        """Validate the factory at decoration time (plain function, exactly ``(self)``)."""
        if isinstance(factory, Resource):
            raise ResourceDefinitionError(
                f"{factory.factory.__qualname__} is already a resource — apply @resource once."
            )
        if not inspect.isfunction(factory):
            raise ResourceDefinitionError(
                f"@resource must wrap a plain method taking only self — got "
                f"{type(factory).__name__}. It needs self for config and sibling resources."
            )
        sig = inspect.signature(factory)
        params = list(sig.parameters.values())
        positional = (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.POSITIONAL_ONLY)
        if not (
            len(params) == 1
            and params[0].kind in positional
            and params[0].default is inspect.Parameter.empty
        ):
            raise ResourceDefinitionError(
                f"@resource factory {factory.__qualname__} must take exactly (self) — "
                f"resources are built from config on self, not from arguments; got "
                f"signature {sig}."
            )
        self.factory = factory
        self.disposes = dispose
        self.is_async = inspect.iscoroutinefunction(factory)
        self.name = ""
        self.owner = None

    def __set_name__(self, owner: type, name: str) -> None:
        """Capture owner + name; reject binding one descriptor under two names (spec §3.2)."""
        if self.owner is not None:
            raise ResourceDefinitionError(
                f"resource {self.owner.__qualname__}.{self.name} is also bound as "
                f"{owner.__qualname__}.{name} — each @resource declaration is one descriptor; "
                f"declare a second factory instead."
            )
        self.owner = owner
        self.name = name

    @overload
    def __get__(self, obj: None, owner: type) -> Resource[_T]: ...
    @overload
    def __get__(self, obj: Any, owner: type | None = None) -> _T: ...
    def __get__(self, obj: Any, owner: type | None = None) -> Any:
        """Class access → self; in a run → per-run instance; else lazy cache (spec §3.3)."""
        if obj is None:
            return self  # class access: the descriptor, side-effect free
        if IN_HANDLER.get():
            raise ResourceError(_in_handler_msg(obj, self.name), ResourceRule.IN_HANDLER)
        ctx = getattr(obj, _RUN_ATTR, None)
        if ctx is not None:
            return ctx.get(self.name)  # run world shadows the lazy world (§5.2)
        return self._lazy_get(obj)

    def _lazy_get(self, obj: Any) -> Any:
        """Double-checked lazy cache (spec §5.1): fast lock-free hit, else create under lock."""
        cache = getattr(obj, _LAZY_ATTR, None)
        if cache is not None:
            val = cache.get(self.name, _MISSING)
            if val is not _MISSING and val is not _CREATING:
                return val
        with _LAZY_LOCK:
            cache = getattr(obj, _LAZY_ATTR, None)
            if cache is None:
                cache = {}
                object.__setattr__(obj, _LAZY_ATTR, cache)
            val = cache.get(self.name, _MISSING)
            if val is _CREATING:
                raise ResourceError(_cycle_msg(obj, self.name), ResourceRule.CYCLE)
            if val is not _MISSING:
                return val
            cache[self.name] = _CREATING
            try:
                value = self._invoke_lazy(obj)
            except BaseException:
                cache.pop(self.name, None)  # failure = no cache; next touch retries (§5.1)
                raise
            cache[self.name] = value
            return value

    def _invoke_lazy(self, obj: Any) -> Any:
        """Call the factory in test mode; async factories run via ``asyncio.run`` (spec §5.1)."""
        if self.is_async:
            try:
                asyncio.get_running_loop()
            except RuntimeError:
                return asyncio.run(self.factory(obj))
            raise ResourceError(_async_lazy_msg(obj, self.name), ResourceRule.ASYNC_LAZY_IN_LOOP)
        return self.factory(obj)


def _resource_specs(cls: type) -> tuple[Resource[Any], ...]:
    """Declaration order = base-first MRO own-``__dict__`` walk, dict-update override (spec §4)."""
    by_name: dict[str, Resource[Any]] = {}
    for klass in reversed(cls.__mro__):
        if klass is object:
            continue
        for key, val in vars(klass).items():  # raw mapping: no descriptor execution (§4)
            if isinstance(val, Resource):
                by_name[key] = val  # override replaces value, keeps first-declared position
    return tuple(by_name.values())


class _ResourceMarker(Protocol):
    """Return shape of ``resource(dispose=...)`` — applies to sync or async factories."""

    @overload
    def __call__(self, factory: Callable[[Any], Coroutine[Any, Any, _T]], /) -> Resource[_T]: ...
    @overload
    def __call__(self, factory: Callable[[Any], _T], /) -> Resource[_T]: ...


@overload
def resource(factory: Callable[[Any], Coroutine[Any, Any, _T]], /) -> Resource[_T]: ...
@overload
def resource(factory: Callable[[Any], _T], /) -> Resource[_T]: ...
@overload
def resource(*, dispose: bool = ...) -> _ResourceMarker: ...
def resource(factory: Callable[[Any], Any] | None = None, /, *, dispose: bool = True) -> Any:
    """Declare a per-run/lazily-cached resource from a ``(self) -> T`` factory (spec §2.2)."""
    if factory is None:

        def marker(fn: Callable[[Any], Any], /) -> Resource[Any]:
            return Resource(fn, dispose=dispose)

        return marker
    return Resource(factory, dispose=dispose)


def _dispose_one_sync(value: Any) -> None:
    """Sniff dispose → close → aclose (asyncio.run); first hit wins, miss is silent (spec §8.1)."""
    disposer = getattr(value, "dispose", None)
    if callable(disposer):
        disposer()
        return
    closer = getattr(value, "close", None)
    if callable(closer):
        closer()
        return
    acloser = getattr(value, "aclose", None)
    if callable(acloser):
        asyncio.run(acloser())  # sync ctx: best-effort, correct for off-loop objects (§8.4)


async def _dispose_one_async(value: Any) -> None:
    """Sniff dispose → close → aclose (awaited on the run loop); miss is silent (spec §8.1)."""
    disposer = getattr(value, "dispose", None)
    if callable(disposer):
        disposer()
        return
    closer = getattr(value, "close", None)
    if callable(closer):
        closer()
        return
    acloser = getattr(value, "aclose", None)
    if callable(acloser):
        await acloser()  # on the still-live run loop (§8.4)


class RunResources(Generic[_T]):
    """Per-RUN resource context: ordered creation, reverse disposal, shared unwind (spec §7, §8)."""

    module: object
    specs: tuple[Resource[Any], ...]
    async_run: bool
    created: list[tuple[Resource[Any], object]]  # creation order; reversed() at disposal
    by_name: dict[str, object]  # __get__'s run path; missing name → [resource-order]
    closed: bool  # exactly-once disposal latch (spec §8.2)

    def __init__(
        self, module: object, specs: tuple[Resource[Any], ...], *, async_run: bool
    ) -> None:
        """Inert container — no instance write, no creation until ``create()``."""
        self.module = module
        self.specs = specs
        self.async_run = async_run
        self.created = []
        self.by_name = {}
        self.closed = False

    def create(self) -> None:
        """Sync-run warmup: guard concurrent run, install ctx, create in declaration order."""
        if getattr(self.module, _RUN_ATTR, None) is not None:
            raise ResourceError(_concurrent_msg(self.module), ResourceRule.CONCURRENT_RUN)
        object.__setattr__(self.module, _RUN_ATTR, self)  # BEFORE any factory (§7.1)
        n = len(self.specs)
        for i, spec in enumerate(self.specs):
            if spec.is_async:
                raise ResourceError(
                    _async_sync_msg(self.module, spec.name), ResourceRule.ASYNC_SYNC_RUN
                )
            try:
                value = spec.factory(self.module)
            except ResourceError:
                raise  # machinery error (e.g. sibling [resource-order]) — never re-wrapped (§7.2)
            except Exception as exc:
                raise ResourceError(
                    _warmup_error_msg(self.module, spec.name, i + 1, n, exc),
                    ResourceRule.WARMUP_ERROR,
                ) from exc
            self.created.append((spec, value))
            self.by_name[spec.name] = value

    async def acreate(self) -> None:
        """Async-run warmup on the run loop: sync factories inline, async awaited (spec §7.3)."""
        if getattr(self.module, _RUN_ATTR, None) is not None:
            raise ResourceError(_concurrent_msg(self.module), ResourceRule.CONCURRENT_RUN)
        object.__setattr__(self.module, _RUN_ATTR, self)  # BEFORE any factory (§7.1)
        n = len(self.specs)
        for i, spec in enumerate(self.specs):
            try:
                value = (
                    await spec.factory(self.module) if spec.is_async else spec.factory(self.module)
                )
            except ResourceError:
                raise
            except Exception as exc:
                raise ResourceError(
                    _warmup_error_msg(self.module, spec.name, i + 1, n, exc),
                    ResourceRule.WARMUP_ERROR,
                ) from exc
            self.created.append((spec, value))
            self.by_name[spec.name] = value

    def dispose(self) -> None:
        """Sniff + dispose ``reversed(created)``; ExitStack error policy; idempotent (spec §8)."""
        if self.closed:
            return
        self.closed = True
        self._relinquish()
        failed: list[str] = []
        first: BaseException | None = None
        for spec, value in reversed(self.created):
            if value is None or not spec.disposes:
                continue
            try:
                _dispose_one_sync(value)
            except Exception as exc:  # BaseException aborts the sweep — Ctrl-C wins (§8.3)
                _LOG.warning(
                    "%s.%s failed to dispose; continuing teardown",
                    _cls(self.module),
                    spec.name,
                    exc_info=exc,
                )
                failed.append(spec.name)
                if first is None:
                    first = exc
        if first is not None:
            raise ResourceError(
                _dispose_error_msg(
                    self.module, len(failed), len(self.created), ", ".join(failed), first
                ),
                ResourceRule.DISPOSE_ERROR,
            ) from first

    async def adispose(self) -> None:
        """Disposal on the run loop: ``aclose`` awaited, sync disposers inline (spec §8.4)."""
        if self.closed:
            return
        self.closed = True
        self._relinquish()
        failed: list[str] = []
        first: BaseException | None = None
        for spec, value in reversed(self.created):
            if value is None or not spec.disposes:
                continue
            try:
                await _dispose_one_async(value)
            except Exception as exc:
                _LOG.warning(
                    "%s.%s failed to dispose; continuing teardown",
                    _cls(self.module),
                    spec.name,
                    exc_info=exc,
                )
                failed.append(spec.name)
                if first is None:
                    first = exc
        if first is not None:
            raise ResourceError(
                _dispose_error_msg(
                    self.module, len(failed), len(self.created), ", ".join(failed), first
                ),
                ResourceRule.DISPOSE_ERROR,
            ) from first

    def get(self, name: str) -> Any:
        """Run-path lookup for ``__get__``; not-yet-created → ``[resource-order]`` (spec §7.4)."""
        if name in self.by_name:  # presence by key: None is a valid value (§5.1)
            return self.by_name[name]
        raise ResourceError(_order_msg(self.module, name), ResourceRule.ORDER)

    def _relinquish(self) -> None:
        """Clear the run slot only if this ctx still owns it (a stillborn guard leaves it be)."""
        if getattr(self.module, _RUN_ATTR, None) is self:
            object.__setattr__(self.module, _RUN_ATTR, None)


def _chain(first: Callable[[], None], second: Callable[[], None]) -> Callable[[], None]:
    """Sequential sync composition (spec §6.1 LIFO): ``first`` then ``second``."""

    def chained() -> None:
        first()
        second()

    return chained


def _achain(
    first: Callable[[], Awaitable[None]], second: Callable[[], Awaitable[None]]
) -> Callable[[], Awaitable[None]]:
    """Sequential async composition (spec §6.1 LIFO): ``await first`` then ``await second``."""

    async def chained() -> None:
        await first()
        await second()

    return chained


def attach_resources(module: object, hooks: RunHooks, *, async_run: bool) -> None:
    """Bind per-run creation/disposal onto the T6 hook seam by chaining (spec §6.1).

    No-op for resource-free classes; pure closure binding otherwise — creation
    happens when the driver invokes ``hooks.warmup``/``awarmup`` at run start.
    """
    specs = _resource_specs(type(module))
    if not specs:  # the Tagger floor: a resource-free module pays nothing (§6.1)
        return
    ctx: RunResources[Any] = RunResources(module, specs, async_run=async_run)
    if async_run:
        # Creation belongs on the loop; teardown gets a sync backstop (§6.1, §8.5).
        hooks.awarmup = _achain(hooks.awarmup, ctx.acreate)
        hooks.ateardown = _achain(ctx.adispose, hooks.ateardown)
        hooks.teardown = _chain(ctx.dispose, hooks.teardown)
    else:
        hooks.warmup = _chain(hooks.warmup, ctx.create)
        hooks.teardown = _chain(ctx.dispose, hooks.teardown)

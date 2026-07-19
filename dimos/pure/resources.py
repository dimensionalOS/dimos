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

from collections.abc import Callable, Coroutine
from contextvars import ContextVar
import enum
from typing import Any, Final, Generic, Protocol, TypeVar, overload

from dimos.pure.drivers import PureModuleRunError, RunHooks
from dimos.pure.stepspec import PureModuleDefinitionError

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

_RUN_ATTR: Final = "__pure_run_resources__"
"""Instance slot for the per-run ``RunResources`` context (spec §5.2); ``None`` between runs."""

_LAZY_ATTR: Final = "__pure_lazy_resources__"
"""Instance slot for the lazy test-mode cache dict (spec §5.1)."""

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


class Resource(Generic[_T]):
    """Descriptor behind ``@resource``: run-context instance in runs, lazy cache outside."""

    factory: Callable[[Any], Any]  # undecorated (self) -> _T | Coroutine[Any, Any, _T]
    disposes: bool  # False under @resource(dispose=False) — never sniffed (spec §9)
    is_async: bool  # iscoroutinefunction(factory) (spec §7.3)
    name: str  # attribute name, from __set_name__
    owner: type | None  # defining class, from __set_name__

    def __init__(self, factory: Callable[[Any], Any], *, dispose: bool = True) -> None:
        """Validate the factory at decoration time (plain function, exactly ``(self)``)."""
        raise NotImplementedError

    def __set_name__(self, owner: type, name: str) -> None:
        """Capture owner + name; reject binding one descriptor under two names (spec §3.2)."""
        raise NotImplementedError

    @overload
    def __get__(self, obj: None, owner: type) -> Resource[_T]: ...
    @overload
    def __get__(self, obj: Any, owner: type | None = None) -> _T: ...
    def __get__(self, obj: Any, owner: type | None = None) -> Any:
        """Class access → self; in a run → per-run instance; else lazy cache (spec §3.3)."""
        raise NotImplementedError


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
    raise NotImplementedError


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
        raise NotImplementedError

    def create(self) -> None:
        """Sync-run warmup: guard concurrent run, install ctx, create in declaration order."""
        raise NotImplementedError

    async def acreate(self) -> None:
        """Async-run warmup on the run loop: sync factories inline, async awaited (spec §7.3)."""
        raise NotImplementedError

    def dispose(self) -> None:
        """Sniff + dispose ``reversed(created)``; ExitStack error policy; idempotent (spec §8)."""
        raise NotImplementedError

    async def adispose(self) -> None:
        """Disposal on the run loop: ``aclose`` awaited, sync disposers inline (spec §8.4)."""
        raise NotImplementedError

    def get(self, name: str) -> Any:
        """Run-path lookup for ``__get__``; not-yet-created → ``[resource-order]`` (spec §7.4)."""
        raise NotImplementedError


def attach_resources(module: object, hooks: RunHooks, *, async_run: bool) -> None:
    """Bind per-run creation/disposal onto the T6 hook seam by chaining (spec §6.1).

    No-op for resource-free classes; pure closure binding otherwise — creation
    happens when the driver invokes ``hooks.warmup``/``awarmup`` at run start.
    """
    raise NotImplementedError

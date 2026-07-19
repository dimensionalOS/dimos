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

"""T7 ``@resource`` unit tests (spec: dimos/pure/tasks/t7-resources.md §14).

Real bodies against the spec'd behavior; the module-level skip comes off with
the implementation. Every resource-bearing module class is defined INSIDE its
test (decoration must not run at collection against the skeleton).
"""

from __future__ import annotations

import asyncio
import dataclasses
import threading
from typing import NamedTuple

import pytest

from dimos import pure as pm
from dimos.pure.resources import (
    IN_HANDLER,
    Resource,
    ResourceDefinitionError,
    ResourceError,
    ResourceRule,
    resource,
)

Event = tuple[str, str]  # ("create" | "dispose" | "close" | "aclose" | "step" | ..., name)


@dataclasses.dataclass(frozen=True)
class Sample:
    ts: float
    v: float


def _stream(*pairs: tuple[float, float]) -> list[Sample]:
    return [Sample(ts=t, v=v) for t, v in pairs]


# ── recording fakes (no skeleton calls at module scope) ──────────────────────


class Disposable:
    """Sniff target with ``dispose`` only."""

    def __init__(self, log: list[Event], name: str) -> None:
        self.log, self.name = log, name

    def dispose(self) -> None:
        self.log.append(("dispose", self.name))


class AllThree:
    """dispose + close + aclose — precedence must pick dispose."""

    def __init__(self, log: list[Event], name: str) -> None:
        self.log, self.name = log, name

    def dispose(self) -> None:
        self.log.append(("dispose", self.name))

    def close(self) -> None:
        self.log.append(("close", self.name))

    async def aclose(self) -> None:
        self.log.append(("aclose", self.name))


class CloseAclose:
    """close + aclose — precedence must pick close."""

    def __init__(self, log: list[Event], name: str) -> None:
        self.log, self.name = log, name

    def close(self) -> None:
        self.log.append(("close", self.name))

    async def aclose(self) -> None:
        self.log.append(("aclose", self.name))


class AcloseOnly:
    """aclose only — awaited on the loop (async runs) or asyncio.run (sync)."""

    def __init__(self, log: list[Event], name: str) -> None:
        self.log, self.name = log, name

    async def aclose(self) -> None:
        self.log.append(("aclose", self.name))


class Inert:
    """No disposal methods — sniff-miss is a silent no-op."""


class FailingDispose:
    def __init__(self, log: list[Event], name: str) -> None:
        self.log, self.name = log, name

    def dispose(self) -> None:
        self.log.append(("dispose-attempt", self.name))
        raise RuntimeError(f"dispose boom: {self.name}")


class CoreResourceFake:
    """Shape-fake of dimos/core/resource.py Resource: dispose() delegates to stop()."""

    def __init__(self, log: list[Event]) -> None:
        self.log = log

    def start(self) -> None:
        self.log.append(("start", "core"))

    def stop(self) -> None:
        self.log.append(("stop", "core"))

    def dispose(self) -> None:
        self.log.append(("dispose", "core"))
        self.stop()

    def __enter__(self) -> CoreResourceFake:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()


# ── §5.1 lazy test mode ──────────────────────────────────────────────────────


def test_lazy_cached_identity() -> None:
    calls: list[str] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def res(self) -> object:
            calls.append("create")
            return object()

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    m = M()
    first = m.res
    assert m.res is first and calls == ["create"]
    other = M()
    assert other.res is not first  # per-instance isolation
    assert calls == ["create", "create"]


def test_lazy_thread_safety() -> None:
    calls: list[str] = []
    barrier = threading.Barrier(8)
    seen: list[object] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def res(self) -> object:
            calls.append("create")
            return object()

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    m = M()

    def touch() -> None:
        barrier.wait()
        seen.append(m.res)

    threads = [threading.Thread(target=touch) for _ in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert calls == ["create"]
    assert len({id(o) for o in seen}) == 1


def test_lazy_factory_error_no_cache_retry() -> None:
    attempts: list[int] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def res(self) -> object:
            attempts.append(1)
            if len(attempts) == 1:
                raise ValueError("factory boom")
            return object()

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    m = M()
    with pytest.raises(ValueError, match="factory boom"):  # RAW, not wrapped (§5.1)
        _ = m.res
    value = m.res  # retry succeeds and caches
    assert m.res is value
    assert len(attempts) == 2


def test_lazy_cycle_detection() -> None:
    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def a(self) -> object:
            return self.b

        @resource
        def b(self) -> object:
            return self.a

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    with pytest.raises(ResourceError) as ei:
        _ = M().a
    assert ei.value.resource_rule is ResourceRule.CYCLE
    assert "[resource-cycle]" in str(ei.value)


def test_lazy_none_value_cached() -> None:
    calls: list[str] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def premap(self) -> object | None:
            calls.append("create")
            return None  # a disabled resource is plain data (sketch §5)

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    m = M()
    assert m.premap is None
    assert m.premap is None
    assert calls == ["create"]  # None cached by presence, not truthiness


# ── §7/§8 per-run lifecycle ──────────────────────────────────────────────────


def _three_resource_module(log: list[Event]) -> type[pm.PureModule]:
    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def a(self) -> Disposable:
            log.append(("create", "a"))
            return Disposable(log, "a")

        @resource
        def b(self) -> Disposable:
            log.append(("create", "b"))
            return Disposable(log, "b")

        @resource
        def c(self) -> Disposable:
            log.append(("create", "c"))
            return Disposable(log, "c")

        def step(self, i: In) -> Out:
            log.append(("step", ""))
            return M.Out(y=i.x.v)

    return M


def test_run_create_dispose_order() -> None:
    log: list[Event] = []
    rows = list(_three_resource_module(log)().over(x=_stream((1.0, 1.0), (2.0, 2.0))))
    assert [r.y for r in rows] == [1.0, 2.0]
    assert log == [
        ("create", "a"),
        ("create", "b"),
        ("create", "c"),
        ("step", ""),
        ("step", ""),
        ("dispose", "c"),
        ("dispose", "b"),
        ("dispose", "a"),
    ]


def test_run_created_before_first_tick() -> None:
    log: list[Event] = []
    cls = _three_resource_module(log)
    list(cls().over(x=_stream((1.0, 1.0))))
    first_step = log.index(("step", ""))
    creates = [i for i, e in enumerate(log) if e[0] == "create"]
    assert creates and all(i < first_step for i in creates)


def test_partial_warmup_unwind() -> None:
    log: list[Event] = []
    boom_once: list[int] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def a(self) -> Disposable:
            log.append(("create", "a"))
            return Disposable(log, "a")

        @resource
        def b(self) -> Disposable:
            if not boom_once:
                boom_once.append(1)
                raise RuntimeError("warmup boom")
            log.append(("create", "b"))
            return Disposable(log, "b")

        @resource
        def c(self) -> Disposable:
            log.append(("create", "c"))
            return Disposable(log, "c")

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    m = M()
    with pytest.raises(ResourceError) as ei:
        next(iter(m.over(x=_stream((1.0, 1.0)))))
    assert ei.value.resource_rule is ResourceRule.WARMUP_ERROR
    assert isinstance(ei.value.__cause__, RuntimeError)
    assert log == [("create", "a"), ("dispose", "a")]  # reverse unwind of the prefix; c untouched

    log.clear()
    assert [r.y for r in m.over(x=_stream((1.0, 3.0)))] == [3.0]  # same instance recovers
    assert log[-3:] == [("dispose", "c"), ("dispose", "b"), ("dispose", "a")]


def test_sniff_precedence() -> None:
    log: list[Event] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def all3(self) -> AllThree:
            return AllThree(log, "all3")

        @resource
        def ca(self) -> CloseAclose:
            return CloseAclose(log, "ca")

        @resource
        def ao(self) -> AcloseOnly:
            return AcloseOnly(log, "ao")

        @resource
        def inert(self) -> Inert:
            return Inert()

        @resource
        def absent(self) -> object | None:
            return None

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    list(M().over(x=_stream((1.0, 1.0))))
    assert ("dispose", "all3") in log and ("close", "all3") not in log
    assert ("aclose", "all3") not in log
    assert ("close", "ca") in log and ("aclose", "ca") not in log
    assert ("aclose", "ao") in log  # sync run: asyncio.run best-effort (§8.4)


def test_core_resource_shape_interop() -> None:
    log: list[Event] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def transport(self) -> CoreResourceFake:
            t = CoreResourceFake(log)
            t.start()  # D8: start() is the FACTORY's job, never the engine's
            return t

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    list(M().over(x=_stream((1.0, 1.0))))
    assert log.count(("start", "core")) == 1  # only the factory's start
    assert log.count(("dispose", "core")) == 1  # engine sniffed dispose first
    assert log.count(("stop", "core")) == 1  # dispose() delegated to stop()


def test_ownership_dispose_false() -> None:
    log: list[Event] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource(dispose=False)
        def shared(self) -> Disposable:
            log.append(("create", "shared"))
            return Disposable(log, "shared")

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    list(M().over(x=_stream((1.0, 1.0))))
    assert ("create", "shared") in log
    assert ("dispose", "shared") not in log  # never disposed: not engine-owned


def test_fresh_per_run() -> None:
    seen: list[object] = []
    log: list[Event] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def res(self) -> Disposable:
            d = Disposable(log, "res")
            log.append(("create", "res"))
            return d

        def step(self, i: In) -> Out:
            seen.append(self.res)
            return M.Out(y=i.x.v)

    m = M()
    list(m.over(x=_stream((1.0, 1.0))))
    list(m.over(x=_stream((2.0, 2.0))))
    assert len(seen) == 2 and seen[0] is not seen[1]  # fresh instance per run
    assert log.count(("create", "res")) == 2 and log.count(("dispose", "res")) == 2


def test_run_shadows_lazy() -> None:
    log: list[Event] = []
    seen: list[object] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def res(self) -> Disposable:
            return Disposable(log, "res")

        def step(self, i: In) -> Out:
            seen.append(self.res)
            return M.Out(y=i.x.v)

    m = M()
    lazy = m.res  # test-mode instance
    list(m.over(x=_stream((1.0, 1.0))))
    assert seen[0] is not lazy  # run world shadows the lazy world
    assert m.res is lazy  # after the run: lazy cache intact, same object
    assert log == [("dispose", "res")]  # only the RUN instance was disposed


# ── §7.3/§8.4 async runs ─────────────────────────────────────────────────────


def test_async_factory_on_run_loop() -> None:
    loops: dict[str, object] = {}

    class Client:
        async def aclose(self) -> None:
            loops["aclose"] = asyncio.get_running_loop()

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        async def client(self) -> Client:
            loops["factory"] = asyncio.get_running_loop()
            return Client()

        async def step(self, i: In) -> Out:
            loops["step"] = asyncio.get_running_loop()
            assert isinstance(self.client, Client)
            return M.Out(y=i.x.v)

    rows = list(M().over(x=_stream((1.0, 1.0), (2.0, 2.0))))
    assert [r.y for r in rows] == [1.0, 2.0]
    assert "aclose" in loops  # awaited before the run ended
    assert loops["factory"] is loops["step"] is loops["aclose"]  # ONE run loop


def test_async_partial_warmup_unwind() -> None:
    loops: dict[str, object] = {}
    log: list[Event] = []

    class Client:
        async def aclose(self) -> None:
            loops["aclose"] = asyncio.get_running_loop()
            log.append(("aclose", "first"))

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        async def first(self) -> Client:
            loops["factory"] = asyncio.get_running_loop()
            return Client()

        @resource
        async def second(self) -> Client:
            raise RuntimeError("async warmup boom")

        async def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    with pytest.raises(ResourceError) as ei:
        list(M().over(x=_stream((1.0, 1.0))))
    assert ei.value.resource_rule is ResourceRule.WARMUP_ERROR
    assert log == [("aclose", "first")]  # created prefix unwound
    assert loops["factory"] is loops["aclose"]  # on the same (run) loop


def test_async_sync_run_error() -> None:
    log: list[Event] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def a(self) -> Disposable:
            log.append(("create", "a"))
            return Disposable(log, "a")

        @resource
        async def b(self) -> object:
            return object()

        def step(self, i: In) -> Out:  # SYNC step — no loop anywhere in this run
            return M.Out(y=i.x.v)

    with pytest.raises(ResourceError) as ei:
        next(iter(M().over(x=_stream((1.0, 1.0)))))
    assert ei.value.resource_rule is ResourceRule.ASYNC_SYNC_RUN
    assert "[resource-async-sync-run]" in str(ei.value)
    assert log == [("create", "a"), ("dispose", "a")]  # prefix unwound


def test_lazy_async_factory() -> None:
    calls: list[str] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        async def client(self) -> object:
            calls.append("create")
            return object()

        async def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    m = M()
    value = m.client  # sync touch: runs via asyncio.run (§5.1)
    assert m.client is value and calls == ["create"]

    async def touch_in_loop() -> None:
        _ = M().client

    with pytest.raises(ResourceError) as ei:
        asyncio.run(touch_in_loop())
    assert ei.value.resource_rule is ResourceRule.ASYNC_LAZY_IN_LOOP


# ── teardown paths and policy ────────────────────────────────────────────────


def test_early_break_disposes_once() -> None:
    log: list[Event] = []
    cls = _three_resource_module(log)
    it = iter(cls().over(x=_stream((1.0, 1.0), (2.0, 2.0), (3.0, 3.0))))
    next(it)
    it.close()  # consumer breaks out — GeneratorExit path
    it.close()  # second close: teardown already ran
    assert log.count(("dispose", "a")) == 1
    assert log.count(("dispose", "b")) == 1
    assert log.count(("dispose", "c")) == 1


def test_dispose_error_policy() -> None:
    log: list[Event] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def a(self) -> Disposable:
            return Disposable(log, "a")

        @resource
        def bad(self) -> FailingDispose:  # last created → first swept
            return FailingDispose(log, "bad")

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    with pytest.raises(ResourceError) as ei:
        list(M().over(x=_stream((1.0, 1.0))))
    assert ei.value.resource_rule is ResourceRule.DISPOSE_ERROR
    assert isinstance(ei.value.__cause__, RuntimeError)  # first failure chained
    assert ("dispose-attempt", "bad") in log
    assert ("dispose", "a") in log  # sweep continued past the failure


# ── ordering, inheritance, references, guards ────────────────────────────────


def test_declaration_order_inheritance() -> None:
    log: list[Event] = []

    class Base(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def a(self) -> Disposable:
            log.append(("create", "a-base"))
            return Disposable(log, "a")

        def step(self, i: In) -> Out:
            return Base.Out(y=i.x.v)

    class Sub(Base):
        @resource
        def a(self) -> Disposable:  # override: new factory, FIRST-declared position
            log.append(("create", "a-sub"))
            return Disposable(log, "a")

        @resource
        def b(self) -> Disposable:
            log.append(("create", "b"))
            return Disposable(log, "b")

    list(Sub().over(x=_stream((1.0, 1.0))))
    assert log == [
        ("create", "a-sub"),  # override factory ran, in a's inherited position
        ("create", "b"),
        ("dispose", "b"),  # reverse order
        ("dispose", "a"),
    ]


def test_run_sibling_reference() -> None:
    seen: list[object] = []

    class Down(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def a(self) -> object:
            return object()

        @resource
        def b(self) -> object:  # reads a sibling declared ABOVE — legal (§7.4)
            seen.append(self.a)
            return object()

        def step(self, i: In) -> Out:
            seen.append(self.a)
            return Down.Out(y=i.x.v)

    list(Down().over(x=_stream((1.0, 1.0))))
    assert seen[0] is seen[1]  # factory saw the RUN instance, same one step sees

    class Up(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def c(self) -> object:  # reads a sibling declared BELOW — order error
            return self.d

        @resource
        def d(self) -> object:
            return object()

        def step(self, i: In) -> Out:
            return Up.Out(y=i.x.v)

    with pytest.raises(ResourceError) as ei:
        next(iter(Up().over(x=_stream((1.0, 1.0)))))
    assert ei.value.resource_rule is ResourceRule.ORDER  # unwrapped machinery error (§7.2)


def test_concurrent_run_guard() -> None:
    log: list[Event] = []
    cls = _three_resource_module(log)
    m = cls()
    xs = _stream((1.0, 1.0), (2.0, 2.0))
    it1 = iter(m.over(x=xs))
    next(it1)
    it2 = iter(m.over(x=xs))
    with pytest.raises(ResourceError) as ei:
        next(it2)  # guard fires at WARMUP of the second run, not at over()
    assert ei.value.resource_rule is ResourceRule.CONCURRENT_RUN
    assert next(it1).y == 2.0  # first run unaffected
    it1.close()
    assert [r.y for r in m.over(x=xs)] == [1.0, 2.0]  # sequential rerun fine

    class Free(pm.PureModule):  # resource-free: concurrency stays unrestricted
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        def step(self, i: In) -> Out:
            return Free.Out(y=i.x.v)

    f = Free()
    a, b = iter(f.over(x=xs)), iter(f.over(x=xs))
    assert next(a).y == next(b).y == 1.0


# ── flags, definition errors, statics ────────────────────────────────────────


def test_handler_flag() -> None:
    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def res(self) -> object:
            return object()

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    m = M()
    token = IN_HANDLER.set(True)
    try:
        with pytest.raises(ResourceError) as ei:
            _ = m.res
        assert ei.value.resource_rule is ResourceRule.IN_HANDLER
        assert isinstance(M.res, Resource)  # class access stays flag-exempt
    finally:
        IN_HANDLER.reset(token)
    assert m.res is m.res  # flag off: normal lazy behavior


def test_definition_errors() -> None:
    with pytest.raises(ResourceDefinitionError):

        class NoSelf(pm.PureModule):
            class In(pm.In):
                x: Sample = pm.tick()

            class Out(pm.Out):
                y: float = 0.0

            @resource  # type: ignore[arg-type]
            def r() -> object:  # zero params — needs self
                return object()

            def step(self, i: In) -> Out:
                return NoSelf.Out(y=i.x.v)

    with pytest.raises(ResourceDefinitionError):

        class ExtraArg(pm.PureModule):
            class In(pm.In):
                x: Sample = pm.tick()

            class Out(pm.Out):
                y: float = 0.0

            @resource  # type: ignore[arg-type]
            def r(self, size: int) -> object:  # arguments come from config, not calls
                return object()

            def step(self, i: In) -> Out:
                return ExtraArg.Out(y=i.x.v)

    with pytest.raises(ResourceDefinitionError):

        class OnClassmethod(pm.PureModule):
            class In(pm.In):
                x: Sample = pm.tick()

            class Out(pm.Out):
                y: float = 0.0

            @resource  # type: ignore[arg-type]
            @classmethod
            def r(cls) -> object:
                return object()

            def step(self, i: In) -> Out:
                return OnClassmethod.Out(y=i.x.v)

    def plain(self: object) -> object:
        return object()

    once = resource(plain)
    with pytest.raises(ResourceDefinitionError):
        resource(once)  # type: ignore[arg-type]  # double decoration

    with pytest.raises(ResourceDefinitionError):

        class Aliased:  # one descriptor object bound under two names
            x = once
            y = once


def test_class_access_returns_descriptor() -> None:
    calls: list[str] = []

    class M(pm.PureModule):
        class In(pm.In):
            x: Sample = pm.tick()

        class Out(pm.Out):
            y: float = 0.0

        @resource
        def res(self) -> object:
            calls.append("create")
            return object()

        def step(self, i: In) -> Out:
            return M.Out(y=i.x.v)

    assert isinstance(M.res, Resource)
    assert M.res.name == "res"
    assert calls == []  # class access never runs the factory


# ── §14 #22: e2e over() with a resource-bearing Mealy module ─────────────────


def test_e2e_over_with_resource() -> None:
    log: list[Event] = []

    class Grid:
        def __init__(self) -> None:
            self.values: list[float] = []

        def add(self, v: float) -> None:
            self.values.append(v)

        def total(self) -> float:
            return sum(self.values)

        def dispose(self) -> None:
            log.append(("dispose", "grid"))

    class Mapper(pm.PureModule):
        emit_every: int = 2

        class In(pm.In):
            lidar: Sample = pm.tick()

        class Out(pm.Out):
            total: float = 0.0

        class State(NamedTuple):
            n: int = 0

        @resource
        def grid(self) -> Grid:
            log.append(("create", "grid"))
            return Grid()

        def step(self, state: State, i: In) -> tuple[State, Out | None]:
            self.grid.add(i.lidar.v)
            state = state._replace(n=state.n + 1)
            if state.n % self.emit_every:
                return state, None
            return state, Mapper.Out(total=self.grid.total())

    m = Mapper()
    rows = list(m.over(lidar=_stream((1.0, 1.0), (2.0, 2.0), (3.0, 3.0), (4.0, 4.0))))
    assert [(r.ts, r.total) for r in rows] == [(2.0, 3.0), (4.0, 10.0)]  # tick-ts stamped
    assert log == [("create", "grid"), ("dispose", "grid")]

    rows2 = list(m.over(lidar=_stream((1.0, 5.0), (2.0, 5.0))))
    assert [r.total for r in rows2] == [10.0]  # fresh grid: no carry-over from run 1

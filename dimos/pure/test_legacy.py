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

"""T8b legacy-bridge parity tests (spec: tasks/t8-rim.md §13.3-§13.4, TL- cases).

Each test pins a parity-matrix row (§2.2, P-numbers referenced inline). Real
bodies behind a module-level skip; the T8b wave deletes the ``pytestmark``
line. Classes are module-level so they pickle by reference through the deploy
pipe (the ``_test_module.py`` precedent).
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import dataclasses
import pickle
import sys
import threading
import time
from typing import Any, get_type_hints

import pytest
from reactivex.disposable import Disposable

from dimos import pure as pm
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom, autoconnect
from dimos.core.coordination.module_coordinator import (
    ModuleCoordinator,
    _all_name_types,
    _verify_no_name_conflicts,
)
from dimos.core.coordination.python_worker import _handle_request, _WorkerState
from dimos.core.coordination.worker_messages import (
    DeployModuleRequest,
    UndeployModuleRequest,
)
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import In, Out, RemoteOut, Transport
from dimos.pure import rim
from dimos.pure.legacy import legacy_actor, legacy_blueprint
from dimos.pure.stepspec import PureModuleDefinitionError

WAIT_S = 10.0


# ── payload + modules (module-level: pickle-by-reference over the pipe) ──────


@dataclasses.dataclass(frozen=True)
class Ping:
    ts: float
    v: float


class OtherPing:
    """Different type under the same stream name → autoconnect conflict."""

    ts: float = 0.0


class PureEcho(pm.PureModule):
    gain: float = 2.0

    class In(pm.In):
        ping: Ping = pm.tick()

    class Out(pm.Out):
        echo: Ping = pm.contract(min_hz=1)
        maybe: Ping | None = None  # sparse: annotation strips to Out[Ping] (P5)

    def step(self, i: In) -> Out:
        return PureEcho.Out(echo=Ping(ts=i.ts, v=i.ping.v * self.gain))


PROBE_LOG: list[str] = []


class PureWithResource(pm.PureModule):
    class In(pm.In):
        ping: Ping = pm.tick()

    class Out(pm.Out):
        echo: Ping = pm.contract(min_hz=1)

    @pm.resource
    def probe(self) -> Any:
        PROBE_LOG.append("create")
        return None

    def step(self, i: In) -> Out:
        return PureWithResource.Out(echo=i.ping)


# — invalid-for-bridging pure modules (legal pure; the FACTORY rejects, D13) —


class OverlapMod(pm.PureModule):
    class In(pm.In):
        x: Ping = pm.tick()

    class Out(pm.Out):
        x: Ping = pm.contract(min_hz=1)  # In∩Out name overlap

    def step(self, i: In) -> Out:
        return OverlapMod.Out(x=i.x)


class SurfaceClash(pm.PureModule):
    class In(pm.In):
        ref: Ping = pm.tick()  # shadows legacy Module surface

    class Out(pm.Out):
        y: Ping = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return SurfaceClash.Out(y=i.ref)


class ConfigClash(pm.PureModule):
    instance_name: str = ""  # collides with ModuleConfig

    class In(pm.In):
        x: Ping = pm.tick()

    class Out(pm.Out):
        y: Ping = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return ConfigClash.Out(y=i.x)


class LegacyPinger(Module):
    """Test-local legacy module: emits pings, collects echoes (spec §13.4)."""

    ping: Out[Ping]
    echo: In[Ping]

    @rpc
    def start(self) -> None:
        super().start()
        stop = threading.Event()

        def run() -> None:
            n = 0
            while not stop.is_set():
                n += 1
                self.ping.publish(Ping(ts=time.time(), v=float(n)))
                stop.wait(0.05)

        t = threading.Thread(target=run, daemon=True)
        t.start()
        self.register_disposable(Disposable(stop.set))
        self.register_disposable(Disposable(lambda: t.join(timeout=2.0)))


class ConflictingPinger(Module):
    """Same stream name, different type — must fail the blueprint (P10)."""

    ping: Out[OtherPing]


class FakeLegacyTransport(Transport[Any]):
    """In-memory legacy Transport: broadcast→subscribers, no wire (spec §13.3)."""

    def __init__(self) -> None:
        self.sent: list[Any] = []
        self._subs: list[Callable[[Any], Any]] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Any, value: Any) -> None:
        self.sent.append(value)
        for cb in list(self._subs):
            cb(value)

    def subscribe(self, callback: Callable[[Any], Any], selfstream: Any = None) -> Any:
        self._subs.append(callback)
        return lambda: self._subs.remove(callback)


def wait_for(cond: Callable[[], bool], timeout: float = WAIT_S) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return
        time.sleep(0.01)
    raise AssertionError("condition not reached before timeout")


@pytest.fixture(autouse=True)
def _clear_probe() -> Iterator[None]:
    PROBE_LOG.clear()
    yield


@pytest.fixture
def actor_instance() -> Iterator[Any]:
    """A constructed (in-process) actor; always stopped — thread hygiene."""
    actor_cls = legacy_actor(PureEcho)
    inst = actor_cls(g=GlobalConfig(viewer="none"))
    try:
        yield inst
    finally:
        inst.stop()


# ── the factory (P1, P7, P8, P9, P20, D13) ───────────────────────────────────


class TestFactory:
    def test_cached_and_module_subclass(self) -> None:
        a = legacy_actor(PureEcho)
        assert a is legacy_actor(PureEcho)  # one actor class per pure class
        assert issubclass(a, Module)

    def test_pickle_roundtrip(self) -> None:  # P1 — the deploy-pipe contract (G1)
        a = legacy_actor(PureEcho)
        # copyreg reducer → _rebuild_actor → cached class; identity survives the pipe:
        assert pickle.loads(pickle.dumps(a)) is a
        # DeployModuleRequest-shaped payload round-trips (class + kwargs):
        payload = pickle.dumps((1, a, {"gain": 2.0, "g": GlobalConfig(viewer="none")}))
        _, a2, kwargs = pickle.loads(payload)
        assert a2 is a and kwargs["gain"] == 2.0

    def test_name_override(self) -> None:  # G8 / D16 — the migration collision escape
        aliased = legacy_actor(PureEcho, name="PureEchoLive")
        assert aliased.__name__ == "PureEchoLive"
        assert aliased.name == "pureecholive"  # distinct instance key
        assert aliased is not legacy_actor(PureEcho)  # cache keyed (cls, name)
        assert aliased is legacy_actor(PureEcho, name="PureEchoLive")  # cached
        assert pickle.loads(pickle.dumps(aliased)) is aliased  # name rides the recipe

    def test_identity_attrs(self) -> None:  # P7, P8, P9, P20
        a = legacy_actor(PureEcho)
        assert a.__name__ == "PureEcho"
        assert a.name == "pureecho"  # instance key default
        assert a.deployment == "python"
        assert a.dedicated_worker is True  # one module per process (amendment)
        assert not hasattr(a, "on_system_modules")

    def test_rejects_in_out_overlap(self) -> None:  # D13
        with pytest.raises(PureModuleDefinitionError, match="x"):
            legacy_actor(OverlapMod)

    def test_rejects_surface_collision(self) -> None:
        with pytest.raises(PureModuleDefinitionError, match="ref"):
            legacy_actor(SurfaceClash)

    def test_rejects_config_collision(self) -> None:
        with pytest.raises(PureModuleDefinitionError, match="instance_name"):
            legacy_actor(ConfigClash)


# ── blueprint surface (P5, P6, P21) ──────────────────────────────────────────


class TestBlueprintSurface:
    def test_atom_streams(self) -> None:  # P5 + P21
        atom = BlueprintAtom.create(legacy_actor(PureEcho), {})
        refs = {(s.name, s.type, s.direction) for s in atom.streams}
        assert ("ping", Ping, "in") in refs
        assert ("echo", Ping, "out") in refs
        assert ("maybe", Ping, "out") in refs  # Optional stripped for matching
        assert any(name == "health" and d == "out" for name, _, d in refs)  # D12

    def test_config_model(self) -> None:  # P6 + P2/P3/P4 statically
        a = legacy_actor(PureEcho)
        cfg_cls = get_type_hints(a)["config"]
        fields = cfg_cls.model_fields
        assert fields["gain"].default == 2.0  # pure field + default replicated
        for inherited in ("g", "instance_name", "frame_id_prefix"):
            assert inherited in fields  # ModuleConfig absorption
        with pytest.raises(Exception, match="typo_field"):
            cfg_cls(typo_field=1)  # extra="forbid" parity

    def test_blueprint_config_flow(self) -> None:
        bp = legacy_blueprint(PureEcho, gain=3.0)
        model = bp.config()  # the CLI/env override surface builds
        assert "pureecho" in model.model_fields

    def test_autoconnect_grouping(self) -> None:  # P10, blueprint-level
        bp = autoconnect(legacy_blueprint(PureEcho), Blueprint.create(LegacyPinger))
        pairs = _all_name_types(bp)
        assert ("ping", Ping) in pairs and ("echo", Ping) in pairs
        _verify_no_name_conflicts(bp)  # no conflict: shared (name, type) keys

    def test_autoconnect_type_conflict(self) -> None:  # P10, conflict arm
        bp = autoconnect(legacy_blueprint(PureEcho), Blueprint.create(ConflictingPinger))
        with pytest.raises(ValueError, match="ping"):
            _verify_no_name_conflicts(bp)

    def test_remapping_renames_streams(self) -> None:  # P32 (G5): name-based, adapter-agnostic
        bp = legacy_blueprint(PureEcho).remappings([(legacy_actor(PureEcho), "echo", "echo_out")])
        pairs = _all_name_types(bp)
        assert ("echo_out", Ping) in pairs and ("echo", Ping) not in pairs  # renamed
        assert ("ping", Ping) in pairs  # untouched — exactly a legacy module's behavior

    def test_namespace_prefixes_streams(self) -> None:  # P32 (G5)
        bp = legacy_blueprint(PureEcho).namespace("robot0", expose={"ping"})
        pairs = _all_name_types(bp)
        assert ("robot0/echo", Ping) in pairs  # prefixed like any legacy stream
        assert ("ping", Ping) in pairs  # exposed → crosses the namespace boundary


# ── constructed actor, in-process (P2-P4, P11-P15, P17, P18, P28) ────────────


class TestActorInstance:
    def test_ctor_absorbs_coordinator_kwargs(self) -> None:  # P2, P3, P4
        a = legacy_actor(PureEcho)
        inst = a(g=GlobalConfig(viewer="none"), instance_name="pe1", frame_id_prefix="r0")
        try:
            assert inst.config.instance_name == "pe1"
            assert inst.config.frame_id_prefix == "r0"
            assert inst.config.gain == 2.0
        finally:
            inst.stop()

    def test_set_transport(self, actor_instance: Any) -> None:  # P11
        t = FakeLegacyTransport()
        assert actor_instance.set_transport("ping", t) is True
        assert actor_instance.ping._transport is t
        with pytest.raises(ValueError, match="nope"):
            actor_instance.set_transport("nope", t)

    def test_rpcs_surface(self) -> None:  # P12, P27
        a = legacy_actor(PureEcho)
        assert {"build", "start", "stop", "set_transport"} <= set(a.rpcs)

    def test_ref_and_module_ref(self, actor_instance: Any) -> None:  # P17, P28
        actor_instance.ref = "stub-actor"  # SetRefRequest's plain setattr
        assert actor_instance.ref == "stub-actor"
        actor_instance.set_module_ref("planner", "stub-proxy")
        assert actor_instance.planner == "stub-proxy"  # lands on the ACTOR only

    def test_stream_pickles_remote(self, actor_instance: Any) -> None:  # P18
        actor_instance.ref = "stub-actor"
        # R1 (T8b relitigation): read the Out stream `echo` — `ping` is PureEcho's
        # tick INPUT (wired from LegacyPinger.ping[Out] in the e2e), so it pickles to
        # RemoteIn, never the asserted+imported RemoteOut. Out→RemoteOut is the
        # verified P18 path; see appendix R1.
        remote = pickle.loads(pickle.dumps(actor_instance.echo))
        assert isinstance(remote, RemoteOut)
        assert remote.name == "echo" and remote.type is Ping

    def test_lifecycle_flow(self) -> None:  # P13, P14 — no processes, no wire
        a = legacy_actor(PureWithResource)
        inst = a(g=GlobalConfig(viewer="none"))
        tin, tout = FakeLegacyTransport(), FakeLegacyTransport()
        try:
            inst.set_transport("ping", tin)
            inst.set_transport("echo", tout)
            assert PROBE_LOG == []
            inst.build()  # P13: build == rim warmup == resource creation
            assert PROBE_LOG == ["create"]
            inst.start()
            tin.broadcast(None, Ping(ts=1.0, v=7.0))
            wait_for(lambda: len(tout.sent) == 1)
            assert tout.sent[0].v == 7.0
        finally:
            inst.stop()

    def test_rim_session_reachable_through_shell(self) -> None:  # G7
        a = legacy_actor(PureWithResource)
        inst = a(g=GlobalConfig(viewer="none"))
        tin, tout = FakeLegacyTransport(), FakeLegacyTransport()
        try:
            inst.set_transport("ping", tin)
            inst.set_transport("echo", tout)
            inst.build()
            inst.start()
            # the live rim session is reachable through the shell's pure instance:
            assert rim.stats(inst._pure).state == "running"
        finally:
            inst.stop()
        assert rim.stats(inst._pure).state == "stopped"

    def test_stop_twice(self) -> None:  # P15 — the normal shutdown path
        a = legacy_actor(PureEcho)
        inst = a(g=GlobalConfig(viewer="none"))
        inst.stop()
        inst.stop()  # coordinator stop + worker-shutdown stop

    def test_restart_shape(self) -> None:  # P25 — redeploy-in-place lifecycle
        a = legacy_actor(PureWithResource)
        inst = a(g=GlobalConfig(viewer="none"))
        tin, tout = FakeLegacyTransport(), FakeLegacyTransport()
        try:
            inst.set_transport("ping", tin)
            inst.set_transport("echo", tout)
            inst.build()
            inst.start()
            tin.broadcast(None, Ping(ts=1.0, v=1.0))
            wait_for(lambda: len(tout.sent) == 1)
        finally:
            inst.stop()
        assert PROBE_LOG == ["create"]  # exactly one session's resources


# ── the deploy path without processes (P1, P22) ──────────────────────────────


class TestDeployPath:
    def test_handle_deploy_roundtrip(self) -> None:
        state = _WorkerState(instances={}, worker_id=0)
        req = DeployModuleRequest(
            module_id=1,
            module_class=legacy_actor(PureEcho),
            kwargs={"g": GlobalConfig(viewer="none")},  # the forced kwarg (P2)
        )
        resp = _handle_request(req, state)
        assert resp.error is None and resp.result == 1
        assert type(state.instances[1]).__name__ == "PureEcho"
        resp = _handle_request(UndeployModuleRequest(module_id=1), state)
        assert resp.error is None  # undeploy → instance.stop() (P16)

    def test_deploy_bad_config_raises_named(self) -> None:  # P22
        state = _WorkerState(instances={}, worker_id=0)
        req = DeployModuleRequest(
            module_id=2,
            module_class=legacy_actor(PureEcho),
            kwargs={"g": GlobalConfig(viewer="none"), "not_a_field": 5},
        )
        with pytest.raises(Exception, match="not_a_field"):
            # _worker_loop catches this and ships it as WorkerResponse.error —
            # the same channel a legacy module's config typo rides.
            _handle_request(req, state)

    def test_restart_reload_limitation_pinned(self) -> None:  # P26 (G9)
        a = legacy_actor(PureEcho)
        # G9: __module__ is honest — the actor lives in no importable namespace, so
        # its synthesized-ness is visible in logs/tracebacks (not the pure module):
        assert a.__module__ == "dimos.pure.legacy"
        # restart(reload_source=True) does getattr(reload(sys.modules[__module__]),
        # __name__); legacy.py defines no 'PureEcho' symbol, so it fails LOUDLY:
        with pytest.raises(AttributeError):
            getattr(sys.modules[a.__module__], a.__name__)


# ── the acceptance bar (spec §13.4): blueprint e2e over real workers ─────────


@pytest.mark.skipif_macos_bug
class TestBlueprintE2E:
    def test_pure_next_to_legacy(self) -> None:
        bp = autoconnect(Blueprint.create(LegacyPinger), legacy_blueprint(PureEcho))
        mc = ModuleCoordinator.build(bp, {"g": {"viewer": "none"}})
        try:
            # deployed side by side, each on its own worker (P9):
            assert set(mc._deployed_modules) == {"legacypinger", "pureecho"}
            workers = mc._managers["python"].workers  # type: ignore[attr-defined]
            owners = {w.worker_id for w in workers for m in w.module_names}
            assert len(owners) >= 2

            # autoconnect wired shared transports (P10) + the health topic (P21):
            registry_names = {name for name, _ in mc._transport_registry}
            assert {"ping", "echo", "health"} <= registry_names

            # data BOTH directions: pinger → pure step → pinger's In (P19, P31):
            pinger = mc.get_instance("legacypinger")
            echoed = pinger.peek_stream("echo", 10.0)
            assert isinstance(echoed, Ping) and echoed.v > 0

            # the health topic is wired and subscribable (T9 speaks later):
            health_key = next(k for k in mc._transport_registry if k[0] == "health")
            unsub = mc._transport_registry[health_key].subscribe(lambda _m: None)
            if callable(unsub):
                unsub()
        finally:
            mc.stop()
            mc.stop()  # P15 at the coordinator level too
            # R2 (T8b teardown hygiene): mc.stop() does not stop the coordinator-side
            # _transport_registry transports (only modules + managers + coordinator RPC).
            # The health topic we subscribed to above left a live LCM dispatcher thread;
            # reverse-stop the registry so the global thread-leak monitor stays green
            # (the "build_coordinator-style (reverse stop)" teardown spec §13.4 calls for).
            for _t in list(mc._transport_registry.values()):
                try:
                    _t.stop()
                except Exception:
                    pass

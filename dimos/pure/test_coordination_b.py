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

"""T8b adapter tests (spec: tasks/t8-rim-b.md §15.2, §15.4 e2e). Real
bodies, skipped until the T8b implementation lands — the implementing wave
deletes the module-level skip line and nothing else."""

from __future__ import annotations

import dataclasses
import pickle
import threading
import time

import pytest

from dimos import pure as pm
from dimos.core.coordination.blueprints import BlueprintAtom, autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.pure import rim_b as rim
from dimos.pure.coordination_b import LiveSynthesisError, live

pytestmark = pytest.mark.skip(reason="T8 skeleton — bodies land with T8b")


# ── currency + probe modules (module-level: workers import by name) ──────────


@dataclasses.dataclass(frozen=True)
class Sample:
    ts: float
    v: float


class PureEcho(pm.PureModule):
    gain: float = 2.0

    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return PureEcho.Out(y=Sample(ts=i.x.ts, v=i.x.v * self.gain))


class SparsePure(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()
        aux: Sample | None = pm.latest(default=None)

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)
        alert: Sample | None = None

    def step(self, i: In) -> Out:
        return SparsePure.Out(y=i.x, alert=i.aux)


class CollidingPorts(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        x: Sample = pm.contract(min_hz=1)  # same name both directions (T1-legal)

    def step(self, i: In) -> Out:
        return CollidingPorts.Out(x=i.x)


class ReservedPort(pm.PureModule):
    class In(pm.In):
        config: Sample = pm.tick()  # collides with a legacy Module member

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return ReservedPort.Out(y=i.config)


class ReservedConfig(pm.PureModule):
    instance_name: str = "nope"  # collides with ModuleConfig plumbing

    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return ReservedConfig.Out(y=i.x)


class Feeder(Module):
    """Legacy producer for the e2e."""

    x: Out[Sample]

    from dimos.core.core import rpc as _rpc  # local alias keeps the tag import obvious

    @_rpc
    def feed(self, n: int) -> None:
        for t in range(1, n + 1):
            self.x.publish(Sample(ts=float(t), v=float(t)))


class Sink(Module):
    """Legacy consumer for the e2e; collects into an attr readable via Actor."""

    y: In[Sample]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.received = []

    async def handle_y(self, msg) -> None:
        self.received = [*self.received, msg]


class FakeTransport:
    def __init__(self):
        self.published = []
        self._subs = []

    def publish(self, msg):
        self.published.append(msg)

    def subscribe(self, callback):
        self._subs.append(callback)

        def _unsub():
            self._subs.remove(callback)

        return _unsub

    def deliver(self, *msgs):
        def _run():
            for m in msgs:
                for cb in list(self._subs):
                    cb(m)

        t = threading.Thread(target=_run)
        t.start()
        t.join()


def wait_until(pred, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return False


# ── 1: synthesis (parity row P9 without processes) ───────────────────────────


class TestSynthesis:
    def test_stream_annotations(self):
        atom = BlueprintAtom.create(live(PureEcho), {})
        refs = {(s.name, s.direction): s.type for s in atom.streams}
        assert refs[("x", "in")] is Sample
        assert refs[("y", "out")] is Sample
        assert not atom.module_refs  # P22: pure modules declare no refs

    def test_optional_annotation_stripped(self):
        atom = BlueprintAtom.create(live(SparsePure), {})
        refs = {(s.name, s.direction): s.type for s in atom.streams}
        assert refs[("aux", "in")] is Sample  # | None stripped for In[T]
        assert refs[("alert", "out")] is Sample

    def test_class_shape(self):
        cls = live(PureEcho)
        assert cls.__name__ == "PureEcho"
        assert issubclass(cls, Module)
        assert cls.dedicated_worker is True  # P18: one module per process
        assert not hasattr(cls, "on_system_modules")  # P19
        for method in ("build", "start", "stop", "set_transport"):
            assert method in cls.rpcs  # P7: lifecycle rides the RPC rail

    def test_name_override(self):
        cls = live(PureEcho, name="PureEchoLive")
        assert cls.__name__ == "PureEchoLive"
        assert cls is not live(PureEcho)

    def test_not_pure_rejected(self):
        with pytest.raises(LiveSynthesisError, match="live-not-pure"):
            live(Feeder)  # a legacy Module is not a PureModule

    def test_port_collision_rejected(self):
        with pytest.raises(LiveSynthesisError, match="live-port-collision"):
            live(CollidingPorts)

    def test_reserved_port_rejected(self):
        with pytest.raises(LiveSynthesisError, match="live-reserved-port"):
            live(ReservedPort)

    def test_config_collision_rejected(self):
        with pytest.raises(LiveSynthesisError, match="live-config-collision"):
            live(ReservedConfig)


# ── 2: config (P2–P4, P14) ───────────────────────────────────────────────────


class TestConfig:
    def test_g_instance_name_frame_id_prefix_absorbed(self):
        a = live(PureEcho)(
            gain=3.0,
            g=GlobalConfig(),
            instance_name="echo1",
            frame_id_prefix="robo",
        )
        assert a.config.instance_name == "echo1"
        assert a.config.frame_id_prefix == "robo"
        assert a.config.gain == 3.0
        assert a._pure.gain == 3.0  # pure instance built from the pure subset
        a.stop()

    def test_typo_kwarg_rejected(self):
        with pytest.raises(Exception, match="gian"):
            live(PureEcho)(gian=3.0)

    def test_blueprint_config_surface(self):
        bp = live(PureEcho).blueprint()  # P13
        config_model = bp.config()
        sub = config_model.model_fields["pureecho"].annotation
        # the per-module override model carries the pure field
        assert "gain" in str(sub)


# ── 3: identity + pickle (P1, P5, P6) ────────────────────────────────────────


class TestPickleAndIdentity:
    def test_memoized(self):
        assert live(PureEcho) is live(PureEcho)

    def test_class_pickles_by_recipe(self):
        cls = live(PureEcho)
        clone = pickle.loads(pickle.dumps(cls))
        assert clone is cls  # memoization makes the recipe converge in-process
        payload = pickle.dumps((1, cls, {"gain": 2.0, "g": GlobalConfig()}))
        _, cls2, kwargs = pickle.loads(payload)  # DeployModuleRequest-shaped
        assert cls2 is cls and kwargs["gain"] == 2.0

    def test_ref_mutable_pure_frozen(self):
        a = live(PureEcho)()
        a.ref = object()  # P5: the shell accepts set_ref-style mutation
        with pytest.raises(Exception):
            a._pure.gain = 9.0  # the pure instance stays frozen
        a.stop()

    def test_instance_pickles_pre_start(self):
        a = live(PureEcho)(gain=4.0)
        clone = pickle.loads(pickle.dumps(a))
        assert clone.config.gain == 4.0
        a.stop()


# ── 4: lifecycle without a coordinator (P10, P15, P16, P20, P26) ─────────────


class TestLifecycleLocal:
    def test_data_flows_and_stops_twice(self):
        a = live(PureEcho)(gain=2.0)
        tin, tout = FakeTransport(), FakeTransport()
        assert a.set_transport("x", tin)  # P10: inherited legacy wiring call
        assert a.set_transport("y", tout)
        a.build()  # P15: warmup slot
        a.start()  # P26: inert legacy start machinery + rim start
        tin.deliver(Sample(ts=1.0, v=1.0), Sample(ts=2.0, v=2.0))
        assert wait_until(lambda: len(tout.published) == 2)
        assert [s.v for s in tout.published] == [2.0, 4.0]
        a.stop()
        a.stop()  # P16: second rail — idempotent

    def test_legacy_local_subscribers_fed(self):
        # P20: rim fans out through legacy Out.publish, so peek-style local
        # subscribers on the legacy stream see live data.
        a = live(PureEcho)()
        tin, tout = FakeTransport(), FakeTransport()
        a.set_transport("x", tin)
        a.set_transport("y", tout)
        seen = []
        a.outputs["y"].subscribe(seen.append)
        a.build()
        a.start()
        tin.deliver(Sample(ts=1.0, v=1.0))
        assert wait_until(lambda: len(seen) == 1)
        a.stop()

    def test_rim_session_reachable_through_shell(self):
        a = live(PureEcho)()
        tin, tout = FakeTransport(), FakeTransport()
        a.set_transport("x", tin)
        a.set_transport("y", tout)
        a.build()
        a.start()
        s = rim.session(a._pure)
        assert s is not None and s.state is rim.SessionState.RUNNING
        a.stop()
        assert s.state is rim.SessionState.STOPPED


# ── 5: the acceptance-bar e2e (§15.4) ────────────────────────────────────────


@pytest.mark.skipif_macos_bug
class TestBlueprintE2E:
    def test_pure_next_to_legacy_autoconnected(self):
        from dimos.core.coordination.module_coordinator import ModuleCoordinator

        blueprint = autoconnect(
            Feeder.blueprint(),
            live(PureEcho).blueprint(gain=2.0),
            Sink.blueprint(),
        )
        coordinator = ModuleCoordinator.build(blueprint)
        try:
            # one-module-per-process (P18): the pure module owns a dedicated worker
            from dimos.core.coordination.worker_manager_python import WorkerManagerPython

            python_wm = coordinator._managers["python"]
            assert isinstance(python_wm, WorkerManagerPython)
            dedicated = [w for w in python_wm.workers if w.dedicated]
            assert any(w.module_names == ["PureEcho"] for w in dedicated)

            # data both directions through the pure module: legacy Feeder ->
            # /x -> PureEcho -> /y -> legacy Sink (topic naming per P11).
            feeder = coordinator.get_instance(Feeder)
            feeder.feed(3)
            sink = coordinator.get_instance(Sink)
            assert wait_until(lambda: len(sink.received) == 3, timeout=15.0)
            assert sorted(s.v for s in sink.received) == [2.0, 4.0, 6.0]
        finally:
            coordinator.stop()  # P16/P17: clean reverse-order teardown

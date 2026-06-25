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

"""PureModule — offline alignment, binding rules, and live wiring."""

from __future__ import annotations

import math
import threading
import time
from typing import TYPE_CHECKING, Annotated, Any

import pytest

from dimos.core.stream import In, Out
from dimos.memory2.puremodule import (
    Outputs,
    PureModule,
    contract,
    interpolate,
    latest,
    tick,
    window,
)
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.tick import Interpolate, Latest, TickMachine, Window
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Pose import Pose

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos.memory2.stream import Stream

# -- helpers -----------------------------------------------------------------


@pytest.fixture
def store() -> Iterator[MemoryStore]:
    s = MemoryStore()
    s.start()
    yield s
    s.dispose()


def fill(stream: Stream[Any], pairs: list[tuple[float, Any]]) -> Stream[Any]:
    for ts, data in pairs:
        stream.append(data, ts=ts)
    return stream


def obs(ts: float, data: Any) -> Observation[Any]:
    return Observation(ts=ts, data_type=type(data), _data=data)


# -- offline: interpolation to tick time ---------------------------------------


class PoseEcho(PureModule):
    camera: In[int] = tick()
    pose: In[float] = interpolate()

    sampled: Out[float]

    def step(self, camera: int, pose: float, ts: float) -> float:
        assert isinstance(camera, int)
        return pose


def test_interpolates_to_tick_time(store: MemoryStore) -> None:
    # camera at 10 Hz offset by 5ms; pose at 25 Hz with data = 10 * ts
    camera = fill(store.stream("camera", int), [(0.005 + i / 10, i) for i in range(9)])
    pose = fill(store.stream("pose", float), [(i / 25, 10 * (i / 25)) for i in range(26)])

    out = PoseEcho.over(camera=camera, pose=pose).to_list()

    assert len(out) == 9  # every camera frame ticked
    for o in out:
        assert o.data == pytest.approx(10 * o.ts, abs=1e-9)  # exact at frame time


def test_output_derives_from_trigger(store: MemoryStore) -> None:
    camera = fill(store.stream("camera", int), [(0.105, 7)])
    pose = fill(store.stream("pose", float), [(0.0, 0.0), (0.2, 2.0)])

    (o,) = PoseEcho.over(camera=camera, pose=pose).to_list()
    assert o.ts == 0.105
    assert o.data == pytest.approx(1.05)


# -- offline: latest / optional / dropping ---------------------------------------


class WithImu(PureModule):
    camera: In[int] = tick()
    imu: In[float] = latest(max_age=0.05)

    out: Out[float]

    def step(self, camera: int, imu: float | None) -> float:
        return -1.0 if imu is None else imu


def test_latest_respects_max_age(store: MemoryStore) -> None:
    camera = fill(store.stream("camera", int), [(0.01, 0), (0.2, 1)])
    imu = fill(store.stream("imu", float), [(0.0, 5.0)])

    out = WithImu.over(camera=camera, imu=imu).to_list()
    assert [o.data for o in out] == [5.0, -1.0]  # fresh, then stale -> None


class RequiredImu(PureModule):
    camera: In[int] = tick()
    imu: In[float] = latest(max_age=0.05)

    out: Out[float]

    def step(self, camera: int, imu: float) -> float:
        return imu


def test_missing_required_input_drops_tick(store: MemoryStore) -> None:
    camera = fill(store.stream("camera", int), [(0.01, 0), (0.2, 1)])
    imu = fill(store.stream("imu", float), [(0.0, 5.0)])

    out = RequiredImu.over(camera=camera, imu=imu).to_list()
    assert [o.data for o in out] == [5.0]  # stale tick dropped


def test_strict_mode_raises_on_drop(store: MemoryStore) -> None:
    camera = fill(store.stream("camera", int), [(0.01, 0), (0.2, 1)])
    imu = fill(store.stream("imu", float), [(0.0, 5.0)])

    with pytest.raises(ValueError, match=r"missing required inputs \['imu'\]"):
        RequiredImu.over(_strict=True, camera=camera, imu=imu).to_list()


def test_interpolate_tolerance_gates_fallback(store: MemoryStore) -> None:
    class Tight(PureModule):
        camera: In[int] = tick()
        pose: In[float] = interpolate(tolerance=0.01)
        out: Out[float]

        def step(self, camera: int, pose: float) -> float:
            return pose

    camera = fill(store.stream("camera", int), [(0.0, 0), (0.1, 1), (0.45, 2)])
    pose = fill(store.stream("pose", float), [(0.4, 4.0), (0.5, 5.0)])

    out = Tight.over(camera=camera, pose=pose).to_list()
    # 0.0/0.1 precede all poses by > tolerance -> dropped; 0.45 brackets fine
    assert [o.data for o in out] == [pytest.approx(4.5)]


# -- offline: window --------------------------------------------------------------


class ImuBatch(PureModule):
    camera: In[int] = tick()
    imu: In[float] = window(0.1)

    out: Out[int]

    def step(self, camera: int, imu: list[float]) -> int:
        return len(imu)


def test_window_collects_trailing_samples(store: MemoryStore) -> None:
    camera = fill(store.stream("camera", int), [(0.05, 0), (0.1, 1)])
    imu = fill(store.stream("imu", float), [(0.0, 0.0), (0.03, 1.0), (0.06, 2.0), (0.09, 3.0)])

    out = ImuBatch.over(camera=camera, imu=imu).to_list()
    # (-0.05, 0.05] -> {0.0, 0.03}; (0.0, 0.1] -> {0.03, 0.06, 0.09} (left edge exclusive)
    assert [o.data for o in out] == [2, 3]


# -- offline: pose slerp ------------------------------------------------------------


class PoseSampler(PureModule):
    camera: In[int] = tick()
    pose: In[Pose] = interpolate()

    out: Out[Pose]

    def step(self, camera: int, pose: Pose) -> Pose:
        return pose


def test_pose_lerp_and_slerp(store: MemoryStore) -> None:
    identity = Pose(0, 0, 0, 0, 0, 0, 1)
    z90 = Pose(1, 0, 0, 0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))

    camera = fill(store.stream("camera", int), [(0.5, 0)])
    pose = store.stream("pose", Pose)
    pose.append(identity, ts=0.0)
    pose.append(z90, ts=1.0)

    (o,) = PoseSampler.over(camera=camera, pose=pose).to_list()
    mid = o.data
    assert mid.position.x == pytest.approx(0.5)
    z45 = Pose(0, 0, 0, 0, 0, math.sin(math.pi / 8), math.cos(math.pi / 8))
    assert mid.orientation.angle_to(z45.orientation) == pytest.approx(0.0, abs=1e-6)


# -- offline: state, multi-out, observation binding -------------------------------------


class Counter(PureModule):
    camera: In[int] = tick()
    out: Out[int]

    initial_state = 0

    def step(self, state: int, camera: int) -> tuple[int, int]:
        return state + 1, state


def test_stateful_threading(store: MemoryStore) -> None:
    camera = fill(store.stream("camera", int), [(i / 10, i) for i in range(5)])

    out = Counter.over(camera=camera).to_list()
    assert [o.data for o in out] == [0, 1, 2, 3, 4]


class TwoOut(PureModule):
    camera: In[int] = tick()

    doubled: Out[int]
    parity: Out[str]

    def step(self, camera: int) -> dict[str, Any]:
        return {"doubled": camera * 2, "parity": "even" if camera % 2 == 0 else "odd"}


def test_multi_out_returns_dict_rows(store: MemoryStore) -> None:
    camera = fill(store.stream("camera", int), [(0.1, 1), (0.2, 2)])

    out = TwoOut.over(camera=camera).to_list()
    assert [o.data for o in out] == [
        {"doubled": 2, "parity": "odd"},
        {"doubled": 4, "parity": "even"},
    ]


class WriterNav(PureModule):
    camera: In[int] = tick()

    cmd: Out[str]
    alerts: Out[str]

    def step(self, camera: int, out: Outputs) -> None:
        out.cmd = f"go {camera}"
        if camera % 2 == 0:
            out.alerts = f"even {camera}"


def test_out_writer_partial_emission(store: MemoryStore) -> None:
    camera = fill(store.stream("camera", int), [(0.1, 1), (0.2, 2)])

    out = WriterNav.over(camera=camera).to_list()
    assert [o.data for o in out] == [
        {"cmd": "go 1"},
        {"cmd": "go 2", "alerts": "even 2"},
    ]


def test_out_writer_rejects_unknown_port(store: MemoryStore) -> None:
    class Typo(PureModule):
        camera: In[int] = tick()
        cmd: Out[str]

        def step(self, camera: int, out: Outputs) -> None:
            out.cmdd = "oops"

    camera = fill(store.stream("camera", int), [(0.1, 1)])
    with pytest.raises(AttributeError, match=r"unknown output 'cmdd'.*\['cmd'\]"):
        Typo.over(camera=camera).to_list()


def test_out_writer_stateful_returns_bare_state(store: MemoryStore) -> None:
    class Counter(PureModule):
        camera: In[int] = tick()
        count: Out[int]

        initial_state = 0

        def step(self, state: int, camera: int, out: Outputs) -> int:
            out.count = state
            return state + 1  # no tuple — the writer carries the outputs

    camera = fill(store.stream("camera", int), [(i / 10, i) for i in range(4)])
    out = Counter.over(camera=camera).to_list()
    assert [o.data for o in out] == [0, 1, 2, 3]


def test_out_writer_forbids_returning_values(store: MemoryStore) -> None:
    class Confused(PureModule):
        camera: In[int] = tick()
        cmd: Out[str]

        def step(self, camera: int, out: Outputs) -> str:
            out.cmd = "go"
            return "also go"

    camera = fill(store.stream("camera", int), [(0.1, 1)])
    with pytest.raises(TypeError, match="declares 'out'"):
        Confused.over(camera=camera).to_list()


def test_out_writer_last_write_wins(store: MemoryStore) -> None:
    class Rewrites(PureModule):
        camera: In[int] = tick()
        cmd: Out[str]

        def step(self, camera: int, out: Outputs) -> None:
            out.cmd = "draft"
            out.cmd = "final"

    camera = fill(store.stream("camera", int), [(0.1, 1)])
    (o,) = Rewrites.over(camera=camera).to_list()
    assert o.data == "final"


def test_input_named_out_is_an_error() -> None:
    class BadPort(PureModule):
        camera: In[int] = tick()
        out: In[float]
        cmd: Out[int]

        def step(self, camera: int) -> int:
            return camera

    with pytest.raises(TypeError, match="reserved names"):
        BadPort._plan()


class WantsObs(PureModule):
    camera: In[int] = tick()
    pose: In[float] = latest()

    out: Out[float]

    def step(self, camera: Observation[int], pose: Observation[float]) -> float:
        assert isinstance(camera, Observation)
        assert isinstance(pose, Observation)
        return pose.ts


def test_observation_annotation_binds_full_obs(store: MemoryStore) -> None:
    camera = fill(store.stream("camera", int), [(0.1, 0)])
    pose = fill(store.stream("pose", float), [(0.07, 1.0)])

    (o,) = WantsObs.over(camera=camera, pose=pose).to_list()
    assert o.data == 0.07


def test_none_output_emits_nothing(store: MemoryStore) -> None:
    class EvenOnly(PureModule):
        camera: In[int] = tick()
        out: Out[int]

        def step(self, camera: int) -> int | None:
            return camera if camera % 2 == 0 else None

    camera = fill(store.stream("camera", int), [(0.1, 1), (0.2, 2), (0.3, 3), (0.4, 4)])
    out = EvenOnly.over(camera=camera).to_list()
    assert [o.data for o in out] == [2, 4]


# -- plan validation -----------------------------------------------------------------


def test_no_tick_input_is_an_error() -> None:
    class NoTick(PureModule):
        camera: In[int]
        out: Out[int]

        def step(self, camera: int) -> int:
            return camera

    with pytest.raises(TypeError, match="no tick"):
        NoTick._plan()


def test_two_tick_inputs_is_an_error() -> None:
    class TwoTicks(PureModule):
        a: In[int] = tick()
        b: In[int] = tick()
        out: Out[int]

        def step(self, a: int) -> int:
            return a

    with pytest.raises(TypeError, match="multiple tick"):
        TwoTicks._plan()


def test_unknown_step_param_is_an_error() -> None:
    class Typo(PureModule):
        camera: In[int] = tick()
        out: Out[int]

        def step(self, camera: int, poze: float) -> int:
            return camera

    with pytest.raises(TypeError, match="poze"):
        Typo._plan()


def test_annotated_port_is_an_error() -> None:
    class Annot(PureModule):
        camera: In[int] = tick()
        pose: Annotated[In[float], latest()]
        out: Out[int]

        def step(self, camera: int) -> int:
            return camera

    with pytest.raises(TypeError, match="Annotated"):
        Annot._plan()


def test_over_validates_stream_names(store: MemoryStore) -> None:
    camera = store.stream("camera", int)
    with pytest.raises(TypeError, match="mismatch"):
        PoseEcho.over(camera=camera)  # missing pose


# -- live arrival order (machine level) --------------------------------------------------


def test_machine_blocks_until_bracketed() -> None:
    m = TickMachine("camera", {"pose": Interpolate(tolerance=0.5)})

    assert m.process("camera", obs(1.0, 0)) == []  # no pose at all yet
    assert m.process("pose", obs(0.9, 9.0)) == []  # left only — still blocked
    rows = m.process("pose", obs(1.1, 11.0))  # right arrives — resolves
    [(tobs, row)] = rows
    assert tobs.ts == 1.0
    assert row["pose"].data == pytest.approx(10.0)


def test_machine_flush_resolves_pending_via_fallback() -> None:
    m = TickMachine("camera", {"pose": Interpolate(tolerance=0.5)})
    assert m.process("pose", obs(0.9, 9.0)) == []
    assert m.process("camera", obs(1.0, 0)) == []  # waiting for right bracket
    [(tobs, row)] = m.flush()  # stream over — nearest within tolerance
    assert row["pose"].data == 9.0


def test_machine_preserves_tick_order() -> None:
    m = TickMachine("camera", {"pose": Interpolate(tolerance=0.5)})
    m.process("camera", obs(1.0, 0))
    m.process("camera", obs(2.0, 1))
    rows = m.process("pose", obs(2.5, 25.0))  # resolves both at once
    assert [t.ts for t, _ in rows] == [1.0, 2.0]


# -- memory bounds: nothing accumulates without a consumer ---------------------------


def test_machine_pending_cap_evicts_oldest() -> None:
    """A dead interpolate() input must not queue ticks forever."""
    m = TickMachine("camera", {"pose": Interpolate(tolerance=0.5)}, max_pending=3)
    for i in range(10):
        m.process("camera", obs(float(i), i))  # pose never arrives
    assert len(m.pending) == 3
    assert m.blocked_dropped == 7
    m.process("pose", obs(100.0, 0.0))  # pose revives — pending resolves and empties
    assert len(m.pending) == 0


def test_machine_secondary_buffer_bounded_without_triggers() -> None:
    """A dead trigger (camera unplugged) must not grow secondary buffers."""
    m = TickMachine("camera", {"pose": Latest()})
    for i in range(1000):
        m.process("pose", obs(i * 0.01, float(i)))  # 10s of 100 Hz poses, no frames
    assert len(m.fields["pose"].buf) < 200  # pruned to ~1s arrival-jitter slack


def test_machine_window_buffer_bounded() -> None:
    m = TickMachine("camera", {"imu": Window(0.1)})
    for i in range(1000):
        m.process("imu", obs(i * 0.01, float(i)))
        if i % 100 == 0:
            m.process("camera", obs(i * 0.01, i))
    assert len(m.fields["imu"].buf) < 300  # window + slack, not the full history


# -- blueprint & live e2e ------------------------------------------------------------------


def test_blueprint_ports() -> None:
    bp = PoseEcho.blueprint()
    (atom,) = bp.blueprints
    names = {s.name for s in atom.streams}
    assert {"camera", "pose", "sampled"} <= names


@pytest.mark.tool
def test_live_wiring_end_to_end() -> None:
    from dimos.core.transport import pLCMTransport

    class LiveEcho(PureModule):
        frame: In[int] = tick()
        gain: In[float]  # no sampler -> latest()

        out: Out[float]

        def step(self, frame: int, gain: float | None) -> float:
            return frame * (gain if gain is not None else 1.0)

    module = LiveEcho()
    module.frame.transport = pLCMTransport("/test/pm/frame")
    module.gain.transport = pLCMTransport("/test/pm/gain")
    module.out.transport = pLCMTransport("/test/pm/out")

    received: list[float] = []
    done = threading.Event()

    def _collect(msg: float) -> None:
        received.append(msg)
        done.set()

    unsub = module.out.subscribe(_collect)

    module.start()
    try:
        module.gain.transport.publish(2.0)
        time.sleep(0.2)  # ensure gain arrives (and is timestamped) before the frame
        module.frame.transport.publish(21)
        assert done.wait(timeout=5.0), f"timed out, received={received}"
        assert received == [42.0]
    finally:
        unsub()
        module.stop()


def _reset_rx_pool() -> None:
    """Replace the shared RxPY thread pool so the conftest leak check passes."""
    from reactivex.scheduler import ThreadPoolScheduler

    import dimos.utils.threadpool as tp

    tp.scheduler.executor.shutdown(wait=True)
    tp.scheduler = ThreadPoolScheduler(max_workers=tp.get_max_workers())


def _await(condition: Callable[[], bool], timeout: float = 5.0) -> bool:
    """Bounded wait on a cheap condition — no fixed sleeps in assertions."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.001)
    return bool(condition())


class _GatedSteps:
    """Lockstep harness: the test releases each step() completion explicitly."""

    def __init__(self) -> None:
        self.entered = threading.Semaphore(0)
        self.release = threading.Semaphore(0)

    def gate(self) -> None:  # called inside step()
        self.entered.release()
        assert self.release.acquire(timeout=5.0), "test never released the step"

    def step_once(self) -> bool:
        """Wait for step() to start, then let it finish."""
        ok = self.entered.acquire(timeout=5.0)
        self.release.release()
        return ok

    def unblock(self) -> None:
        self.release.release()
        self.release.release()


@pytest.mark.tool
def test_live_backpressure_keeplast_skips_stale_ticks() -> None:
    """KeepLast: while a step is busy, later ticks coalesce to the freshest one."""
    from dimos.core.transport import pLCMTransport

    gates = _GatedSteps()

    class Gated(PureModule):
        frame: In[int] = tick()
        out: Out[int]

        def step(self, frame: int) -> int:
            gates.gate()
            return frame

    module = Gated()
    module.frame.transport = pLCMTransport("/test/bp/frame")
    module.out.transport = pLCMTransport("/test/bp/out")
    outs: list[int] = []
    unsub = module.out.subscribe(outs.append)

    module.start()
    try:
        # Inject events directly into the raw event queue — deterministic order,
        # explicit timestamps, no transport timing involved.
        module._queue.put(("frame", obs(1.0, 1)))
        assert gates.entered.acquire(timeout=5.0)  # step(1) started and is now held

        for i in range(2, 11):
            module._queue.put(("frame", obs(float(i), i)))
        # Alignment digests all events while step(1) is held: KeepLast must
        # coalesce ticks 2..10 down to ONE buffered tick (the freshest).
        assert _await(lambda: module._queue.qsize() == 0 and len(module._tick_buffer) == 1)

        gates.release.release()  # finish step(1)
        assert gates.step_once()  # step(10) runs
        assert _await(lambda: len(outs) == 2)
        assert outs == [1, 10]  # stale ticks 2..9 were skipped, deterministically
        assert len(module._tick_buffer) == 0  # nothing left buffered
    finally:
        gates.unblock()
        unsub()
        module.stop()


@pytest.mark.tool
def test_input_sources_feed_live_runner_from_store(store: MemoryStore) -> None:
    """The live↔stored switch: all inputs sourced -> no transports anywhere."""
    from dimos.memory2.buffer import Unbounded

    gates = _GatedSteps()

    class TsEcho(PureModule):
        frame: In[int] = tick()
        out: Out[float]
        backpressure = Unbounded()

        def step(self, frame: int, ts: float) -> float:
            gates.gate()
            return ts

    recorded = fill(store.stream("frames", int), [(10.0, 1), (10.5, 2), (11.0, 3)])

    module = TsEcho()
    module.input_sources = {"frame": recorded}  # a stored stream is a valid source
    outs: list[float] = []
    unsub = module.out.subscribe(outs.append)

    module.start()
    try:
        for _ in range(3):
            assert gates.step_once()
        assert _await(lambda: len(outs) == 3)
        assert outs == [10.0, 10.5, 11.0]  # recorded timestamps, not wall clock
    finally:
        gates.unblock()
        unsub()
        module.stop()
        _reset_rx_pool()  # source.observable() schedules on the shared rx pool


class Contracted(PureModule):
    frame: In[int] = tick(expect_hz=30)
    gps: In[str] = latest(max_age=0.5, expect_hz=1, max_missing_ratio=0.9)

    cmd: Out[str] = contract(min_hz=10)
    alerts: Out[str]  # sparse by design — no contract

    def step(self, frame: int, gps: str | None) -> dict[str, str]:
        return {"cmd": f"go {frame}"}


def test_class_level_contracts_collected_in_plan() -> None:
    plan = Contracted._plan()
    assert plan.expect_hz == {"frame": 30, "gps": 1}
    assert plan.missing_ratio == {"gps": 0.9}
    assert plan.out_min_hz == {"cmd": 10}


def test_contracted_out_port_still_declared_in_blueprint() -> None:
    bp = Contracted.blueprint()
    (atom,) = bp.blueprints
    assert {"frame", "gps", "cmd", "alerts"} <= {s.name for s in atom.streams}


@pytest.mark.tool
def test_class_contracts_reach_the_monitor(store: MemoryStore) -> None:
    module = Contracted(contracts={"max_drop_ratio": 0.8})  # module-wide config still applies
    module.input_sources = {  # empty sourced inputs: no transports needed
        "frame": store.stream("frame", int),
        "gps": store.stream("gps", str),
    }
    try:
        module.start()
        assert module.health_monitor.expected_hz == {"frame": 30, "gps": 1}
        assert module.health_monitor.out_min_hz == {"cmd": 10}
        assert module.health_monitor.missing_ratio_by_input == {"gps": 0.9}
        assert module.health_monitor.max_drop_ratio == 0.8
    finally:
        module.stop()
        _reset_rx_pool()


def test_health_reserved_as_in_port_name() -> None:
    class BadIn(PureModule):
        health: In[int] = tick()  # type: ignore[misc]

        def step(self, health: int) -> int:
            return health

    with pytest.raises(TypeError, match="reserved"):
        BadIn._plan()


def test_health_reserved_as_out_port_name() -> None:
    class BadOut(PureModule):
        frame: In[int] = tick()
        health: Out[int]  # type: ignore[misc]

        def step(self, frame: int) -> dict[str, int]:
            return {"health": frame}

    with pytest.raises(TypeError, match="reserved"):
        BadOut._plan()


@pytest.mark.tool
def test_health_facade_unavailable_before_start() -> None:
    module = Contracted()
    try:
        with pytest.raises(RuntimeError, match="only after start"):
            _ = module.health
    finally:
        module.stop()
        _reset_rx_pool()


@pytest.mark.tool
def test_health_facade_exposes_state_stream_and_subscribe(store: MemoryStore) -> None:
    module = Contracted()
    module.input_sources = {  # empty sourced inputs: no transports needed
        "frame": store.stream("frame", int),
        "gps": store.stream("gps", str),
    }
    try:
        module.start()
        assert module.health.state == "OK"  # fresh module is healthy
        assert module.health.latest is None  # no report emitted yet
        assert module.health.stream is module._health_stream  # the real _health stream
        seen: list[Any] = []
        disp = module.health.subscribe(seen.append)  # live subscribe wires up
        disp.dispose()
    finally:
        module.stop()
        _reset_rx_pool()


@pytest.mark.tool
def test_health_subscribe_raises_when_stream_disabled(store: MemoryStore) -> None:
    module = Contracted(health={"stream": False})
    module.input_sources = {
        "frame": store.stream("frame", int),
        "gps": store.stream("gps", str),
    }
    try:
        module.start()
        assert module.health.stream is None
        with pytest.raises(RuntimeError, match="disabled"):
            module.health.subscribe(lambda _: None)
        assert module.health.state == "OK"  # state still readable
    finally:
        module.stop()
        _reset_rx_pool()


@pytest.mark.tool
def test_input_sources_unknown_name_raises() -> None:
    class Echo(PureModule):
        frame: In[int] = tick()
        out: Out[int]

        def step(self, frame: int) -> int:
            return frame

    module = Echo()
    module.input_sources = {"typo": object()}
    try:
        with pytest.raises(TypeError, match="typo"):
            module.start()
    finally:
        module.stop()


@pytest.mark.tool
def test_live_backpressure_unbounded_processes_everything() -> None:
    from dimos.core.transport import pLCMTransport
    from dimos.memory2.buffer import Unbounded

    gates = _GatedSteps()

    class GatedAll(PureModule):
        frame: In[int] = tick()
        out: Out[int]
        backpressure = Unbounded()

        def step(self, frame: int) -> int:
            gates.gate()
            return frame

    module = GatedAll()
    module.frame.transport = pLCMTransport("/test/bp2/frame")
    module.out.transport = pLCMTransport("/test/bp2/out")
    outs: list[int] = []
    unsub = module.out.subscribe(outs.append)

    module.start()
    try:
        for i in range(1, 7):
            module._queue.put(("frame", obs(float(i), i)))
        for _ in range(6):
            assert gates.step_once()
        assert _await(lambda: len(outs) == 6)
        assert outs == [1, 2, 3, 4, 5, 6]  # every tick, in order
        assert len(module._tick_buffer) == 0  # drained, not accumulating
        assert _await(lambda: len(module.health_monitor._latency_ms) == 6)  # e2e latency measured
    finally:
        gates.unblock()
        unsub()
        module.stop()

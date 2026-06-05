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

"""Grid tests for StreamModule — same e2e logic across all pipeline styles."""

from __future__ import annotations

from collections.abc import Iterator
import math
import threading
import time
from typing import NamedTuple

import numpy as np
import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import pLCMTransport
from dimos.memory2.fanio import Bundle, normalize_to_bundle, scatter_to_ports
from dimos.memory2.interpolators import interp_odom, lerp_pose
from dimos.memory2.module import StreamModule
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import Transformer
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Shared transformer


class Double(Transformer[int, int]):
    def __init__(self, factor: int = 2) -> None:
        self.factor = factor

    def __call__(self, upstream: Iterator[Observation[int]]) -> Iterator[Observation[int]]:
        for obs in upstream:
            yield obs.derive(data=obs.data * self.factor)


# Pipeline styles


class StaticStreamModule(StreamModule[int, int]):
    """Pipeline as a static Stream chain on the class."""

    pipeline = Stream().transform(Double())
    numbers: In[int]
    doubled: Out[int]


class StaticTransformerModule(StreamModule[int, int]):
    """Pipeline as a bare Transformer on the class."""

    pipeline = Double()
    numbers: In[int]
    doubled: Out[int]


class MethodPipelineConfig(ModuleConfig):
    factor: int = 2


class MethodPipelineModule(StreamModule[int, int]):
    """Pipeline as a method with access to self.config."""

    config: MethodPipelineConfig

    def pipeline(self, stream: Stream[int]) -> Stream[int]:
        return stream.transform(Double(factor=self.config.factor))

    numbers: In[int]
    doubled: Out[int]


# Grid

module_cases = [
    pytest.param(StaticStreamModule, id="static-stream"),
    pytest.param(StaticTransformerModule, id="static-transformer"),
    pytest.param(MethodPipelineModule, id="method-pipeline"),
]


@pytest.mark.parametrize("module_cls", module_cases)
def test_blueprint_ports(module_cls: type[StreamModule]) -> None:
    """All pipeline styles produce a blueprint with the correct In/Out ports."""
    bp = module_cls.blueprint()

    assert len(bp.blueprints) == 1
    atom = bp.blueprints[0]
    stream_names = {s.name for s in atom.streams}
    assert "numbers" in stream_names
    assert "doubled" in stream_names


def _reset_thread_pool() -> None:
    """Shut down and replace the global RxPY thread pool so conftest thread-leak check passes."""
    import dimos.utils.threadpool as tp

    tp.scheduler.executor.shutdown(wait=True)
    tp.scheduler = ThreadPoolScheduler(max_workers=tp.get_max_workers())


@pytest.mark.tool
@pytest.mark.parametrize("module_cls", module_cases)
def test_e2e_runtime_wiring(module_cls: type[StreamModule]) -> None:
    """Push data into In port, assert doubled data arrives on Out port."""
    module = module_cls()
    module.numbers.transport = pLCMTransport("/test/numbers")
    module.doubled.transport = pLCMTransport("/test/doubled")

    received: list[int] = []
    done = threading.Event()

    unsub = module.doubled.subscribe(lambda msg: (received.append(msg), done.set()))

    module.start()
    try:
        module.numbers.transport.publish(42)
        assert done.wait(timeout=5.0), f"Timed out, received={received}"
        assert received == [84]
    finally:
        unsub()
        module.stop()
        _reset_thread_pool()
        _reset_thread_pool()


class SingleOutBundleModule(StreamModule):
    """One ``Out``, but the pipeline ends in a :class:`Bundle` (marker-style 1->1).

    Reproduces the fan-I/O scatter bug: a Bundle-tail pipeline with a single
    ``Out`` must publish ``bundle[port_name]``, not the whole ``Bundle``. The
    bundle also carries an unrouted sibling key (no matching port) which scatter
    must drop - exactly what happens when a two-output marker module has one
    ``Out`` commented out but still yields a two-key bundle.
    """

    numbers: In[int]
    doubled: Out[int]

    def pipeline(self, stream: Stream[int]) -> Stream[Bundle]:
        def _to_bundle(upstream: Iterator[Observation[int]]) -> Iterator[Observation[Bundle]]:
            for obs in upstream:
                yield obs.derive(data=Bundle({"doubled": obs.data * 2, "unrouted": obs.data}))

        return stream.transform(_to_bundle)


@pytest.mark.tool
def test_single_out_bundle_tail_publishes_field_not_whole_bundle_e2e() -> None:
    """With one ``Out`` and a Bundle-tail pipeline, the port receives the matching
    bundle field (M-agnostic scatter), never the whole ``Bundle`` - so a 1->1
    module needs no dummy second port to "turn on" fan-out, and the unrouted key
    is dropped rather than leaked onto the single port."""
    module = SingleOutBundleModule()
    module.numbers.transport = pLCMTransport("/test/sob_numbers")
    module.doubled.transport = pLCMTransport("/test/sob_doubled")

    received: list[int] = []
    done = threading.Event()

    unsub = module.doubled.subscribe(lambda msg: (received.append(msg), done.set()))

    module.start()
    try:
        module.numbers.transport.publish(21)
        assert done.wait(timeout=5.0), f"Timed out, received={received}"
        assert received == [42]  # bundle["doubled"], not Bundle({...})
    finally:
        unsub()
        module.stop()
        _reset_thread_pool()
        _reset_thread_pool()


# Fan-in: many In ports reachable as siblings via self.streams


class TwoInputFusion(StreamModule):
    """Primary lidar aligned against a sibling pose reached via ``self.streams``."""

    lidar: In[int]
    pose: In[int]
    fused: Out[object]

    def pipeline(self, lidar: Stream[int]) -> Stream[object]:
        return lidar.align(self.streams.pose, tolerance=0.5)


class ChainedFusion(StreamModule):
    """Three In ports fused by chaining ``.align()`` once per sibling edge."""

    image: In[int]
    pose: In[int]
    cloud: In[int]
    fused: Out[object]

    def pipeline(self, image: Stream[int]) -> Stream[object]:
        return image.align(self.streams.pose, tolerance=0.5).align(
            self.streams.cloud, tolerance=0.5
        )


def _wire_inputs(
    module: StreamModule, store: MemoryStore, dtype: type = int, **points: list
) -> dict[str, Stream]:
    """Populate ``module._in_streams`` from ``name=[(ts, value), ...]`` point lists.

    Mirrors what ``StreamModule.start()`` assembles, so ``pipeline()`` can reach
    siblings through ``self.streams.<port>`` without standing up the live
    transport machinery. Each secondary's last ts must reach the primary's last
    ts so the two-pointer merge never blocks waiting on a (never-arriving) later
    live sample.
    """
    streams: dict[str, Stream] = {}
    for name, pts in points.items():
        s = store.stream(name, dtype)
        for ts, value in pts:
            s.append(value, ts=ts)
        streams[name] = s
    module._in_streams = streams
    return streams


class TestFanInAlign:
    """Authors reach sibling In ports through ``self.streams.<port>`` and fuse
    them with PR #2306 ``.align()`` inside ``pipeline()``."""

    def test_two_input_align_pairs_sibling_and_names_fields_after_ports(self) -> None:
        """The primary aligns against ``self.streams.pose``; each emitted pair is
        named after the two ports and carries the full per-side Observation, and
        the output keeps the primary (scan) timestamp - not the pose's."""
        module = TwoInputFusion()
        with MemoryStore() as store:
            streams = _wire_inputs(
                module,
                store,
                lidar=[(1.0, 11), (2.0, 22)],
                pose=[(1.05, 100), (2.0, 200)],
            )
            try:
                out = module.pipeline(streams["lidar"]).to_list()
            finally:
                module.stop()

        assert [o.data._fields for o in out] == [("lidar", "pose"), ("lidar", "pose")]
        assert [o.data.lidar.data for o in out] == [11, 22]
        assert [o.data.pose.data for o in out] == [100, 200]
        # Emitted observation carries the primary ts, not the matched pose ts.
        assert [o.ts for o in out] == [1.0, 2.0]

    def test_chained_align_fuses_three_inputs(self) -> None:
        """Chaining ``.align()`` twice fuses three siblings; the outer pair is
        named (image, cloud) and nests the inner (image, pose) pair, so every
        sibling's value is reachable from one pipeline run."""
        module = ChainedFusion()
        with MemoryStore() as store:
            streams = _wire_inputs(
                module,
                store,
                image=[(1.0, 11), (2.0, 22)],
                pose=[(1.0, 100), (2.0, 200)],
                cloud=[(1.0, 1000), (2.0, 2000)],
            )
            try:
                out = module.pipeline(streams["image"]).to_list()
            finally:
                module.stop()

        assert [o.data._fields for o in out] == [("image", "cloud"), ("image", "cloud")]
        assert [o.data.cloud.data for o in out] == [1000, 2000]
        inner = [o.data.image.data for o in out]
        assert [p._fields for p in inner] == [("image", "pose"), ("image", "pose")]
        assert [p.image.data for p in inner] == [11, 22]
        assert [p.pose.data for p in inner] == [100, 200]

    def test_align_skips_secondary_beyond_tolerance(self) -> None:
        """A sibling within tolerance pairs; the same sibling moved 3x tolerance
        away pairs with nothing - so the tolerance keyword actually gates."""
        within = TwoInputFusion()  # tolerance is 0.5
        with MemoryStore() as store:
            streams = _wire_inputs(within, store, lidar=[(1.0, 11)], pose=[(1.4, 100)])  # 0.4 < 0.5
            try:
                matched = within.pipeline(streams["lidar"]).to_list()
            finally:
                within.stop()
        assert [o.data.pose.data for o in matched] == [100]

        beyond = TwoInputFusion()
        with MemoryStore() as store:
            streams = _wire_inputs(
                beyond, store, lidar=[(1.0, 11)], pose=[(2.5, 100)]
            )  # 1.5 == 3x tol
            try:
                assert beyond.pipeline(streams["lidar"]).to_list() == []
            finally:
                beyond.stop()

    def test_streams_accessor_exposes_ports_and_rejects_typos(self) -> None:
        """``self.streams.<port>`` returns a stream named after the port; an
        unknown port raises AttributeError listing the available names, so a
        mistyped sibling fails loudly rather than silently dropping data."""
        module = TwoInputFusion()
        with MemoryStore() as store:
            _wire_inputs(module, store, lidar=[(1.0, 11)], pose=[(1.0, 100)])
            try:
                assert module.streams.lidar.name == "lidar"
                assert module.streams.pose.name == "pose"
                with pytest.raises(AttributeError) as excinfo:
                    _ = module.streams.poze
            finally:
                module.stop()
        message = str(excinfo.value)
        assert "poze" in message
        assert "lidar" in message and "pose" in message


# Interpolated fan-in: the align edge synthesizes the secondary at the scan ts


def _apply_pose_to_cloud(cloud: PointCloud2, pose: PoseStamped) -> PointCloud2:
    """World-frame cloud: rotate the points by the pose, then translate."""
    points, _ = cloud.as_numpy()
    world = points @ pose.orientation.to_rotation_matrix().T + np.array([pose.x, pose.y, pose.z])
    return PointCloud2.from_numpy(world, frame_id="world", timestamp=cloud.ts)


class MyFusion(StreamModule):
    """Pose-at-scan-time fusion: the pose edge carries ``interpolator=lerp_pose``
    so each scan is deskewed by the pose synthesized at its exact timestamp,
    not the nearest captured pose."""

    lidar: In[PointCloud2]
    pose: In[PoseStamped]
    map: Out[PointCloud2]

    def pipeline(self, lidar: Stream[PointCloud2]) -> Stream[PointCloud2]:
        return lidar.align(self.streams.pose, tolerance=1.0, interpolator=lerp_pose).map_data(
            lambda obs: _apply_pose_to_cloud(obs.data.lidar.data, obs.data.pose.data)
        )


class _FusedOdom(NamedTuple):
    """Author-side intermediate struct - not a ``Bundle`` until the port-keyed tail."""

    odom: Odometry
    cloud: PointCloud2


def _fuse_and_correct(
    upstream: Iterator[Observation[object]],
) -> Iterator[Observation[_FusedOdom]]:
    """Average the imu and leg odometry estimates synthesized at the scan ts."""
    for obs in upstream:
        outer = obs.data  # (lidar, leg); the lidar side nests (lidar, imu)
        leg = outer.leg.data
        imu = outer.lidar.data.imu.data
        cloud = outer.lidar.data.lidar.data
        fused = Odometry(
            ts=obs.ts,
            frame_id=imu.frame_id,
            child_frame_id=imu.child_frame_id,
            pose=Pose(
                position=(imu.position + leg.position) * 0.5,
                orientation=imu.orientation,  # trust the imu for attitude
            ),
            twist=Twist(
                (imu.linear_velocity + leg.linear_velocity) * 0.5,
                (imu.angular_velocity + leg.angular_velocity) * 0.5,
            ),
        )
        yield obs.derive(data=_FusedOdom(odom=fused, cloud=cloud))


class OdomFusion(StreamModule):
    """3 In / 2 Out fusion: both odometry edges interpolate to the scan ts,
    and one pipeline run yields a ``Bundle`` keyed by the Out port names."""

    lidar: In[PointCloud2]
    imu: In[Odometry]
    leg: In[Odometry]
    odom: Out[Odometry]
    map: Out[PointCloud2]

    def pipeline(self, lidar: Stream[PointCloud2]) -> Stream[Bundle]:
        return (
            lidar.align(self.streams.imu, tolerance=0.6, interpolator=interp_odom)
            .align(self.streams.leg, tolerance=0.6, interpolator=interp_odom)
            .transform(_fuse_and_correct)
            .map_data(lambda obs: Bundle({"odom": obs.data.odom, "map": obs.data.cloud}))
        )


class TestInterpolatedFusionModules:
    """Full fusion modules whose align edges carry ``interpolator=`` so the
    secondary is synthesized at the exact primary timestamp inside
    ``pipeline()``."""

    def test_my_fusion_applies_the_pose_interpolated_at_scan_time(self) -> None:
        """A scan halfway between two poses is transformed by the halfway pose:
        position at the midpoint, yaw slerped to 45 degrees - and keeps the scan
        ts. Nearest-neighbor would snap to an endpoint pose and move every
        point to a different place."""
        module = MyFusion()
        scan = PointCloud2.from_numpy(np.array([[1.0, 0.0, 0.0]]), timestamp=1.5)
        pose_a = PoseStamped(ts=1.0, frame_id="odom", position=(0, 0, 0))
        pose_b = PoseStamped(
            ts=2.0,
            frame_id="odom",
            position=(1, 0, 0),
            orientation=Quaternion.from_euler(Vector3(0, 0, math.pi / 2)),
        )
        with MemoryStore() as store:
            streams = _wire_inputs(
                module,
                store,
                dtype=object,
                lidar=[(1.5, scan)],
                pose=[(1.0, pose_a), (2.0, pose_b)],
            )
            try:
                out = module.pipeline(streams["lidar"]).to_list()
            finally:
                module.stop()

        assert [o.ts for o in out] == [1.5]
        points, _ = out[0].data.as_numpy()
        # Interpolated pose: position (0.5, 0, 0), yaw 45 deg. R(45) @ (1,0,0)
        # lands on (cos45, sin45, 0) before the translation.
        half = math.sqrt(2) / 2
        np.testing.assert_allclose(points[0], [0.5 + half, half, 0.0], atol=1e-6)
        assert out[0].data.ts == 1.5  # payload stamped at scan time too

    def test_odom_fusion_interpolates_both_edges_then_bundles(self) -> None:
        """Both odometry edges synthesize their sample at the scan ts before
        fusing: the averaged position and velocity match the interpolated
        values exactly - if either edge snapped to a real sample instead, the
        average would shift to an endpoint mix. One run feeds both Out keys."""

        def odom(ts: float, x: float, vx: float) -> Odometry:
            return Odometry(
                ts=ts,
                frame_id="odom",
                child_frame_id="base_link",
                pose=Pose(x, 0.0, 0.0),
                twist=Twist((vx, 0.0, 0.0), (0.0, 0.0, 0.0)),
            )

        module = OdomFusion()
        scan = PointCloud2.from_numpy(np.array([[7.0, 8.0, 9.0]]), timestamp=1.5)
        with MemoryStore() as store:
            streams = _wire_inputs(
                module,
                store,
                dtype=object,
                lidar=[(1.5, scan)],
                # At the 1.5 scan ts: imu interpolates to x=1.0, vx=1.0.
                imu=[(1.0, odom(1.0, 0.0, 0.0)), (2.0, odom(2.0, 2.0, 2.0))],
                # leg interpolates to x=0.5, vx=0.3.
                leg=[(1.0, odom(1.0, 0.4, 0.2)), (2.0, odom(2.0, 0.6, 0.4))],
            )
            try:
                out = module.pipeline(streams["lidar"]).to_list()
            finally:
                module.stop()

        assert [o.ts for o in out] == [1.5]
        bundle = out[0].data
        fused = bundle["odom"]
        # Averages of the two *interpolated* estimates; any nearest-neighbor
        # snap would give x in {0.2, 1.2, 1.3, 2.3}, never 0.75.
        assert fused.x == pytest.approx(0.75)
        assert fused.vx == pytest.approx(0.65)
        assert fused.ts == 1.5
        # The scan rides the same bundle untouched.
        points, _ = bundle["map"].as_numpy()
        np.testing.assert_allclose(points[0], [7.0, 8.0, 9.0])
        assert bundle["map"].ts == 1.5


# Ingest seam: enrich/drop messages without copying start()


class _FakePort:
    """Minimal In-port stand-in: records subscribers, lets the test push messages."""

    def __init__(self, name: str, type_: type) -> None:
        self.name = name
        self.type = type_
        self._subscribers: list = []

    def subscribe(self, cb):  # type: ignore[no-untyped-def]
        self._subscribers.append(cb)
        return lambda: self._subscribers.remove(cb)

    def emit(self, msg) -> None:  # type: ignore[no-untyped-def]
        for cb in list(self._subscribers):
            cb(msg)


class _Stamped:
    """Payload that carries its own capture time, like a real sensor message."""

    def __init__(self, ts: float) -> None:
        self.ts = ts


class SkippingIngestModule(StreamModule[int, int]):
    """Drops negative readings at ingest, so the pipeline never sees them."""

    pipeline = Double()
    numbers: In[int]
    doubled: Out[int]

    def ingest(self, name: str, stream: Stream[int], msg: int) -> None:
        if msg >= 0:
            stream.append(msg, ts=float(msg))


class TestIngestSeam:
    """``ingest()`` is the seam that lets a module enrich or drop messages
    before they enter the pipeline, without copying ``start()``."""

    def test_default_ingest_stamps_with_message_time(self) -> None:
        """Default ingest carries each message's own ts (so cross-port ``.align()``
        lines up on capture time), falling back to arrival time only for
        unstamped payloads. Ignoring ``msg.ts`` would silently misalign sensors."""
        module = StaticTransformerModule()
        try:
            with MemoryStore() as store:
                stamped_stream = store.stream("stamped", object)
                module.ingest("stamped", stamped_stream, _Stamped(ts=42.0))
                stamped = list(stamped_stream)

                bare_stream = store.stream("bare", int)
                before = time.time()
                module.ingest("bare", bare_stream, 7)
                after = time.time()
                bare = list(bare_stream)
        finally:
            module.stop()

        assert stamped[0].ts == 42.0  # message capture time, not arrival
        assert bare[0].data == 7
        assert before <= bare[0].ts <= after  # arrival-time fallback for unstamped

    def test_ingest_override_can_drop_messages_before_the_pipeline(self) -> None:
        """A port wired through ``_wire_input`` routes every message through
        ``ingest()``; an override that skips negative readings keeps them out of
        the backing stream entirely."""
        module = SkippingIngestModule()
        with MemoryStore() as store:
            port = _FakePort("numbers", int)
            backing = module._wire_input("numbers", port, store)
            try:
                for value in (5, -1, 7, -3, 9):
                    port.emit(value)
                assert [o.data for o in backing] == [5, 7, 9]
            finally:
                module.stop()


# Fan-out: one pipeline run, many Out ports (Bundle scatter)


class _RecordingOut:
    """Out-port stand-in that records every published payload."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.published: list = []

    def publish(self, msg) -> None:  # type: ignore[no-untyped-def]
        self.published.append(msg)


def _wait_until(predicate, timeout: float = 5.0) -> bool:  # type: ignore[no-untyped-def]
    """Poll *predicate* until true or *timeout* elapses (scatter runs on the pool)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


class TestFanOutScatter:
    """A multi-output pipeline yields a :class:`Bundle` per tick and
    :func:`scatter_to_ports` fans it to the matching ports in one subscribe."""

    @pytest.mark.tool
    def test_multi_output_scatter_runs_pipeline_once_per_tick(self) -> None:
        """Two Out ports fed from one fused pipeline must compute once per
        observation, not once per port: scatter subscribes a single time, so a
        detector inside the pipeline runs N times for N ticks - not 2N."""
        n = 5
        detector_calls: list[int] = []

        class CountingDetector(Transformer[int, Bundle]):
            def __call__(
                self, upstream: Iterator[Observation[int]]
            ) -> Iterator[Observation[Bundle]]:
                for obs in upstream:
                    detector_calls.append(obs.data)
                    yield obs.derive(data=Bundle({"low": obs.data, "high": obs.data * 10}))

        low, high = _RecordingOut("low"), _RecordingOut("high")
        with MemoryStore() as store:
            src = store.stream("src", int)
            for i in range(n):
                src.append(i, ts=float(i))
            produced = src.transform(CountingDetector())
            disposable = scatter_to_ports(produced, {"low": low, "high": high})
            try:
                assert _wait_until(lambda: len(low.published) >= n and len(high.published) >= n), (
                    f"timed out: low={low.published} high={high.published}"
                )
            finally:
                disposable.dispose()
                _reset_thread_pool()
                _reset_thread_pool()

        assert len(detector_calls) == n  # once per tick, not 2 * n
        assert low.published == [0, 1, 2, 3, 4]
        assert high.published == [0, 10, 20, 30, 40]

    @pytest.mark.tool
    def test_scatter_skips_none_valued_keys(self) -> None:
        """A bundle key mapped to None publishes nothing on that port for the
        tick, while a sibling key with a real payload still publishes."""
        present, absent = _RecordingOut("present"), _RecordingOut("absent")
        with MemoryStore() as store:
            src = store.stream("src", object)
            src.append(Bundle({"present": "x", "absent": None}), ts=0.0)
            disposable = scatter_to_ports(src, {"present": present, "absent": absent})
            try:
                assert _wait_until(lambda: len(present.published) >= 1)
            finally:
                disposable.dispose()
                _reset_thread_pool()
                _reset_thread_pool()

        assert present.published == ["x"]
        assert absent.published == []  # None is a skip, not a publish

    @pytest.mark.tool
    def test_scatter_publishes_empty_but_present_payloads(self) -> None:
        """An empty-but-present payload (e.g. an empty detection array) still
        publishes - 'nothing detected this frame' differs from 'port idle' - so
        only None is treated as absent."""
        empty_out, value_out = _RecordingOut("empty"), _RecordingOut("value")
        with MemoryStore() as store:
            src = store.stream("src", object)
            src.append(Bundle({"empty": [], "value": "v"}), ts=0.0)
            disposable = scatter_to_ports(src, {"empty": empty_out, "value": value_out})
            try:
                assert _wait_until(lambda: len(value_out.published) >= 1)
            finally:
                disposable.dispose()
                _reset_thread_pool()
                _reset_thread_pool()

        assert empty_out.published == [[]]  # the empty list was published, not skipped
        assert value_out.published == ["v"]

    @pytest.mark.tool
    def test_single_output_bundle_tail_publishes_field_not_whole_bundle(self) -> None:
        """A single ``Out`` whose pipeline ends in a ``Bundle`` publishes the
        field named after the port, not the whole bundle, and silently drops
        keys with no matching port. This is the M-agnostic rule: one ``Out``
        reads its key exactly as two would - port count never selects the path."""
        detections = _RecordingOut("detections_3d")
        with MemoryStore() as store:
            src = store.stream("src", object)
            # Marker-style 1->1: routed key plus an unrouted sibling key.
            src.append(Bundle({"detections_3d": "d3d", "detections_2d": "d2d"}), ts=0.0)
            ports = {"detections_3d": detections}
            produced = normalize_to_bundle(src, ports)
            disposable = scatter_to_ports(produced, ports)
            try:
                assert _wait_until(lambda: len(detections.published) >= 1)
            finally:
                disposable.dispose()
                _reset_thread_pool()
                _reset_thread_pool()

        # bundle["detections_3d"] only; the unrouted detections_2d field is dropped.
        assert detections.published == ["d3d"]

    @pytest.mark.tool
    def test_single_output_raw_payload_is_wrapped_then_published(self) -> None:
        """A 1:1 pipeline that yields a raw payload (not a ``Bundle``) is
        normalized into a one-key bundle at the start boundary, so the same
        bundle-only scatter still delivers the raw value to the sole port -
        preserving 1:1 back-compat without a per-module rewrite."""
        global_map = _RecordingOut("global_map")
        with MemoryStore() as store:
            src = store.stream("src", int)
            for i in range(3):
                src.append(i, ts=float(i))
            ports = {"global_map": global_map}
            produced = normalize_to_bundle(src, ports)
            disposable = scatter_to_ports(produced, ports)
            try:
                assert _wait_until(lambda: len(global_map.published) >= 3)
            finally:
                disposable.dispose()
                _reset_thread_pool()
                _reset_thread_pool()

        assert global_map.published == [0, 1, 2]

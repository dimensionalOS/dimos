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

"""memory stream tests — serves as living documentation of the lazy stream API.

Each test demonstrates a specific capability with clear setup, action, and assertion.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

import pytest

from dimos.memory2.buffer import KeepLast, Unbounded
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import FnTransformer, QualityWindow, Transformer
from dimos.memory2.type.observation import Observation

if TYPE_CHECKING:
    from collections.abc import Callable


@pytest.fixture
def make_stream(session) -> Callable[..., Stream[int]]:
    stream_index = 0

    def f(n: int = 5, start_ts: float = 0.0):
        nonlocal stream_index
        stream_index += 1
        stream = session.stream(f"test{stream_index}", int)
        for i in range(n):
            stream.append(i * 10, ts=start_ts + i)
        return stream

    return f


class TestBasicIteration:
    """Streams are lazy iterables — nothing runs until you iterate."""

    def test_iterate_yields_all_observations(self, make_stream):
        stream = make_stream(5)
        obs = list(stream)
        assert len(obs) == 5
        assert [o.data for o in obs] == [0, 10, 20, 30, 40]

    def test_iterate_preserves_timestamps(self, make_stream):
        stream = make_stream(3, start_ts=100.0)
        assert [o.ts for o in stream] == [100.0, 101.0, 102.0]

    def test_empty_stream(self, make_stream):
        stream = make_stream(0)
        assert list(stream) == []

    def test_fetch_materializes_to_list(self, make_stream):
        result = make_stream(3).to_list()
        assert isinstance(result, list)
        assert len(result) == 3

    def test_stream_is_reiterable(self, make_stream):
        """Same stream can be iterated multiple times — each time re-queries."""
        stream = make_stream(3)
        first = [o.data for o in stream]
        second = [o.data for o in stream]
        assert first == second == [0, 10, 20]


class TestTemporalFilters:
    """Temporal filters constrain observations by timestamp."""

    def test_after(self, make_stream):
        """.after(t) keeps observations with ts > t."""
        result = make_stream(5).after(2.0).to_list()
        assert [o.ts for o in result] == [3.0, 4.0]

    def test_before(self, make_stream):
        """.before(t) keeps observations with ts < t."""
        result = make_stream(5).before(2.0).to_list()
        assert [o.ts for o in result] == [0.0, 1.0]

    def test_time_range(self, make_stream):
        """.time_range(t1, t2) keeps t1 <= ts <= t2."""
        result = make_stream(5).time_range(1.0, 3.0).to_list()
        assert [o.ts for o in result] == [1.0, 2.0, 3.0]

    def test_at_with_tolerance(self, make_stream):
        """.at(t, tolerance) keeps observations within tolerance of t."""
        result = make_stream(5).at(2.0, tolerance=0.5).to_list()
        assert [o.ts for o in result] == [2.0]

    def test_chained_temporal_filters(self, make_stream):
        """Filters compose — each narrows the result."""
        result = make_stream(10).after(2.0).before(7.0).to_list()
        assert [o.ts for o in result] == [3.0, 4.0, 5.0, 6.0]


class TestSpatialFilter:
    """.near(pose, radius) filters by Euclidean distance."""

    def test_near_with_tuples(self, memory_session):
        stream = memory_session.stream("spatial")
        stream.append("origin", ts=0.0, pose=(0, 0, 0))
        stream.append("close", ts=1.0, pose=(1, 1, 0))
        stream.append("far", ts=2.0, pose=(10, 10, 10))

        result = stream.near((0, 0, 0), radius=2.0).to_list()
        assert [o.data for o in result] == ["origin", "close"]

    def test_near_excludes_no_pose(self, memory_session):
        stream = memory_session.stream("spatial")
        stream.append("no_pose", ts=0.0)
        stream.append("has_pose", ts=1.0, pose=(0, 0, 0))

        result = stream.near((0, 0, 0), radius=10.0).to_list()
        assert [o.data for o in result] == ["has_pose"]


class TestTagsFilter:
    """.filter_tags() matches on observation metadata."""

    def test_filter_by_tag(self, memory_session):
        stream = memory_session.stream("tagged")
        stream.append("cat", ts=0.0, tags={"type": "animal", "legs": 4})
        stream.append("car", ts=1.0, tags={"type": "vehicle", "wheels": 4})
        stream.append("dog", ts=2.0, tags={"type": "animal", "legs": 4})

        result = stream.tags(type="animal").to_list()
        assert [o.data for o in result] == ["cat", "dog"]

    def test_filter_multiple_tags(self, memory_session):
        stream = memory_session.stream("tagged")
        stream.append("a", ts=0.0, tags={"x": 1, "y": 2})
        stream.append("b", ts=1.0, tags={"x": 1, "y": 3})

        result = stream.tags(x=1, y=2).to_list()
        assert [o.data for o in result] == ["a"]


class TestOrderLimitOffset:
    def test_limit(self, make_stream):
        result = make_stream(10).limit(3).to_list()
        assert len(result) == 3

    def test_offset(self, make_stream):
        result = make_stream(5).offset(2).to_list()
        assert [o.data for o in result] == [20, 30, 40]

    def test_limit_and_offset(self, make_stream):
        result = make_stream(10).offset(2).limit(3).to_list()
        assert [o.data for o in result] == [20, 30, 40]

    def test_order_by_ts_desc(self, make_stream):
        result = make_stream(5).order_by("ts", desc=True).to_list()
        assert [o.ts for o in result] == [4.0, 3.0, 2.0, 1.0, 0.0]

    def test_first(self, make_stream):
        obs = make_stream(5).first()
        assert obs.data == 0

    def test_last(self, make_stream):
        obs = make_stream(5).last()
        assert obs.data == 40

    def test_first_empty_raises(self, make_stream):
        with pytest.raises(LookupError):
            make_stream(0).first()

    def test_count(self, make_stream):
        assert make_stream(5).count() == 5
        assert make_stream(5).after(2.0).count() == 2

    def test_exists(self, make_stream):
        assert make_stream(5).exists()
        assert not make_stream(0).exists()
        assert not make_stream(5).after(100.0).exists()

    def test_drain(self, make_stream):
        assert make_stream(5).drain() == 5
        assert make_stream(5).after(2.0).drain() == 2
        assert make_stream(0).drain() == 0


class TestFunctionalAPI:
    """Functional combinators receive the full Observation."""

    def test_filter_with_predicate(self, make_stream):
        """.filter() takes a predicate on the full Observation."""
        result = make_stream(5).filter(lambda obs: obs.data > 20).to_list()
        assert [o.data for o in result] == [30, 40]

    def test_filter_on_metadata(self, make_stream):
        """Predicates can access ts, tags, pose — not just data."""
        result = make_stream(5).filter(lambda obs: obs.ts % 2 == 0).to_list()
        assert [o.ts for o in result] == [0.0, 2.0, 4.0]

    def test_map(self, make_stream):
        """.map() transforms each observation's data."""
        result = make_stream(3).map(lambda obs: obs.derive(data=obs.data * 2)).to_list()
        assert [o.data for o in result] == [0, 20, 40]

    def test_map_preserves_ts(self, make_stream):
        result = make_stream(3).map(lambda obs: obs.derive(data=str(obs.data))).to_list()
        assert [o.ts for o in result] == [0.0, 1.0, 2.0]
        assert [o.data for o in result] == ["0", "10", "20"]


class TestTransformChaining:
    """Transforms chain lazily — each obs flows through the full pipeline."""

    def test_single_transform(self, make_stream):
        xf = FnTransformer(lambda obs: obs.derive(data=obs.data + 1))
        result = make_stream(3).transform(xf).to_list()
        assert [o.data for o in result] == [1, 11, 21]

    def test_chained_transforms(self, make_stream):
        """stream.transform(A).transform(B) — B pulls from A which pulls from source."""
        add_one = FnTransformer(lambda obs: obs.derive(data=obs.data + 1))
        double = FnTransformer(lambda obs: obs.derive(data=obs.data * 2))

        result = make_stream(3).transform(add_one).transform(double).to_list()
        # (0+1)*2=2, (10+1)*2=22, (20+1)*2=42
        assert [o.data for o in result] == [2, 22, 42]

    def test_transform_can_skip(self, make_stream):
        """Returning None from a transformer skips that observation."""
        keep_even = FnTransformer(lambda obs: obs if obs.data % 20 == 0 else None)
        result = make_stream(5).transform(keep_even).to_list()
        assert [o.data for o in result] == [0, 20, 40]

    def test_transform_filter_transform(self, memory_session):
        """stream.transform(A).near(pose).transform(B) — filter between transforms."""
        stream = memory_session.stream("tfft")
        stream.append(1, ts=0.0, pose=(0, 0, 0))
        stream.append(2, ts=1.0, pose=(100, 100, 100))
        stream.append(3, ts=2.0, pose=(1, 0, 0))

        add_ten = FnTransformer(lambda obs: obs.derive(data=obs.data + 10))
        double = FnTransformer(lambda obs: obs.derive(data=obs.data * 2))

        result = (
            stream.transform(add_ten)  # 11, 12, 13
            .near((0, 0, 0), 5.0)  # keeps pose at (0,0,0) and (1,0,0)
            .transform(double)  # 22, 26
            .to_list()
        )
        assert [o.data for o in result] == [22, 26]

    def test_generator_function_transform(self, make_stream):
        """A bare generator function works as a transform."""

        def double_all(upstream):
            for obs in upstream:
                yield obs.derive(data=obs.data * 2)

        result = make_stream(3).transform(double_all).to_list()
        assert [o.data for o in result] == [0, 20, 40]

    def test_generator_function_stateful(self, make_stream):
        """Generator transforms can accumulate state and yield at their own pace."""

        def running_sum(upstream):
            total = 0
            for obs in upstream:
                total += obs.data
                yield obs.derive(data=total)

        result = make_stream(3).transform(running_sum).to_list()
        # 0, 0+10=10, 10+20=30
        assert [o.data for o in result] == [0, 10, 30]

    def test_quality_window(self, memory_session):
        """QualityWindow keeps the best item per time window."""
        stream = memory_session.stream("qw")
        # Window 1: ts 0.0-0.9 → best quality
        stream.append(0.3, ts=0.0)
        stream.append(0.9, ts=0.3)  # best in window
        stream.append(0.1, ts=0.7)
        # Window 2: ts 1.0-1.9
        stream.append(0.5, ts=1.0)
        stream.append(0.8, ts=1.5)  # best in window
        # Window 3: ts 2.0+ (emitted at end via flush)
        stream.append(0.6, ts=2.2)

        xf = QualityWindow(quality_fn=lambda v: v, window=1.0)
        result = stream.transform(xf).to_list()
        assert [o.data for o in result] == [0.9, 0.8, 0.6]

    def test_streaming_not_buffering(self, make_stream):
        """Transforms process lazily — early limit stops pulling from source."""
        calls = []

        class CountingXf(Transformer[int, int]):
            def __call__(self, upstream):
                for obs in upstream:
                    calls.append(obs.data)
                    yield obs

        result = make_stream(100).transform(CountingXf()).limit(3).to_list()
        assert len(result) == 3
        # The transformer should have processed at most a few more than 3
        # (not all 100) due to lazy evaluation
        assert len(calls) == 3


class TestUnboundStream:
    """Unbound streams: pipelines built without a source, applied later via .chain()."""

    def test_creation(self) -> None:
        """Stream() with no args creates an unbound stream."""
        s = Stream()
        assert s._transform is None

    def test_multi_transform_chain(self) -> None:
        """Unbound pipeline with multiple transforms produces correct results when bound."""

        class Double(Transformer[int, int]):
            def __call__(self, upstream):
                for obs in upstream:
                    yield obs.derive(data=obs.data * 2)

        pipeline = Stream().transform(Double()).map(lambda obs: obs.derive(data=obs.data + 1))

        store = MemoryStore()
        with store:
            stream = store.stream("test", int)
            stream.append(5)
            stream.append(10)

            result = stream.chain(pipeline).to_list()
            assert [obs.data for obs in result] == [11, 21]

    def test_iteration_raises(self) -> None:
        """Iterating an unbound stream raises TypeError."""
        s = Stream().transform(FnTransformer(lambda obs: obs))
        with pytest.raises(TypeError, match="unbound"):
            list(s)

    def test_chain_applies_transforms(self) -> None:
        """chain() replays unbound transforms on a real stream."""
        store = MemoryStore()
        with store:
            stream = store.stream("test", int)
            stream.append(10)
            stream.append(20)
            stream.append(30)

            class Double(Transformer[int, int]):
                def __call__(self, upstream):
                    for obs in upstream:
                        yield obs.derive(data=obs.data * 2)

            pipeline = Stream().transform(Double())
            result = stream.chain(pipeline).to_list()
            assert [obs.data for obs in result] == [20, 40, 60]

    def test_chain_multiple_transforms(self) -> None:
        """chain() preserves order of multiple transforms."""
        store = MemoryStore()
        with store:
            stream = store.stream("test", int)
            stream.append(5)

            class Double(Transformer[int, int]):
                def __call__(self, upstream):
                    for obs in upstream:
                        yield obs.derive(data=obs.data * 2)

            class AddTen(Transformer[int, int]):
                def __call__(self, upstream):
                    for obs in upstream:
                        yield obs.derive(data=obs.data + 10)

            pipeline = Stream().transform(Double()).transform(AddTen())
            result = stream.chain(pipeline).to_list()
            assert result[0].data == 20  # (5 * 2) + 10

    def test_chain_preserves_filters(self) -> None:
        """chain() replays filters from the unbound stream."""
        store = MemoryStore()
        with store:
            stream = store.stream("test", int)
            stream.append(10, ts=1.0)
            stream.append(20, ts=2.0)
            stream.append(30, ts=3.0)

            pipeline = Stream().after(1.5)
            result = stream.chain(pipeline).to_list()
            assert [obs.data for obs in result] == [20, 30]

    def test_chain_rejects_bound_stream(self) -> None:
        """chain() raises if passed a bound (non-unbound) stream."""
        store = MemoryStore()
        with store:
            s1 = store.stream("a", int)
            s2 = store.stream("b", int)
            with pytest.raises(TypeError, match="unbound"):
                s1.chain(s2)

    def test_live_rejects_unbound(self) -> None:
        """live() raises on an unbound stream."""
        with pytest.raises(TypeError, match="unbound"):
            Stream().live()

    def test_str(self) -> None:
        """Unbound streams display as Stream(unbound)."""
        s = Stream()
        assert "unbound" in str(s)


class TestStore:
    """Store -> Stream hierarchy for named streams."""

    def test_basic_store(self, memory_store):
        images = memory_store.stream("images")
        images.append("frame1", ts=0.0)
        images.append("frame2", ts=1.0)
        assert images.count() == 2

    def test_same_stream_on_repeated_calls(self, memory_store):
        s1 = memory_store.stream("images")
        s2 = memory_store.stream("images")
        assert s1 is s2

    def test_list_streams(self, memory_store):
        memory_store.stream("images")
        memory_store.stream("lidar")
        names = memory_store.list_streams()
        assert "images" in names
        assert "lidar" in names
        assert len(names) == 2

    def test_delete_stream(self, memory_store):
        memory_store.stream("temp")
        memory_store.delete_stream("temp")
        assert "temp" not in memory_store.list_streams()


class TestLazyData:
    """Observation.data supports lazy loading with cleanup."""

    def test_eager_data(self):
        """In-memory observations have data set directly — zero-cost access."""
        obs = Observation(id=0, ts=0.0, _data="hello")
        assert obs.data == "hello"

    def test_lazy_loading(self):
        """Data loaded on first access, loader released after."""
        load_count = 0

        def loader():
            nonlocal load_count
            load_count += 1
            return "loaded"

        obs = Observation(id=0, ts=0.0, _loader=loader)
        assert load_count == 0
        assert obs.data == "loaded"
        assert load_count == 1
        assert obs._loader is None  # released
        assert obs.data == "loaded"  # cached, no second load
        assert load_count == 1

    def test_no_data_no_loader_raises(self):
        obs = Observation(id=0, ts=0.0)
        with pytest.raises(LookupError):
            _ = obs.data

    def test_derive_preserves_metadata(self):
        obs = Observation(id=42, ts=1.5, pose=(1, 2, 3), tags={"k": "v"}, _data="original")
        derived = obs.derive(data="transformed")
        assert derived.id == 42
        assert derived.ts == 1.5
        assert derived.pose_tuple == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
        assert derived.tags == {"k": "v"}
        assert derived.data == "transformed"


class TestLiveMode:
    """Live streams yield backfill then block for new observations."""

    def test_live_sees_backfill_then_new(self, memory_session):
        """Backfill first, then live appends come through."""
        stream = memory_session.stream("live")
        stream.append("old", ts=0.0)
        live = stream.live(buffer=Unbounded())

        results: list[str] = []
        consumed = threading.Event()

        def consumer():
            for obs in live:
                results.append(obs.data)
                if len(results) >= 3:
                    consumed.set()
                    return

        t = threading.Thread(target=consumer)
        t.start()

        time.sleep(0.05)
        stream.append("new1", ts=1.0)
        stream.append("new2", ts=2.0)

        consumed.wait(timeout=2.0)
        t.join(timeout=2.0)
        assert results == ["old", "new1", "new2"]

    def test_live_with_filter(self, memory_session):
        """Filters apply to live data — non-matching obs are dropped silently."""
        stream = memory_session.stream("live_filter")
        live = stream.after(5.0).live(buffer=Unbounded())

        results: list[int] = []
        consumed = threading.Event()

        def consumer():
            for obs in live:
                results.append(obs.data)
                if len(results) >= 2:
                    consumed.set()
                    return

        t = threading.Thread(target=consumer)
        t.start()

        time.sleep(0.05)
        stream.append(1, ts=1.0)  # filtered out (ts <= 5.0)
        stream.append(2, ts=6.0)  # passes
        stream.append(3, ts=3.0)  # filtered out
        stream.append(4, ts=10.0)  # passes

        consumed.wait(timeout=2.0)
        t.join(timeout=2.0)
        assert results == [2, 4]

    def test_live_deduplicates_backfill_overlap(self, memory_session):
        """Observations seen in backfill are not re-yielded from the live buffer."""
        stream = memory_session.stream("dedup")
        stream.append("backfill", ts=0.0)
        live = stream.live(buffer=Unbounded())

        results: list[str] = []
        consumed = threading.Event()

        def consumer():
            for obs in live:
                results.append(obs.data)
                if len(results) >= 2:
                    consumed.set()
                    return

        t = threading.Thread(target=consumer)
        t.start()

        time.sleep(0.05)
        stream.append("live1", ts=1.0)

        consumed.wait(timeout=2.0)
        t.join(timeout=2.0)
        assert results == ["backfill", "live1"]

    def test_live_with_keep_last_backpressure(self, memory_session):
        """KeepLast drops intermediate values when consumer is slow."""
        stream = memory_session.stream("bp")
        live = stream.live(buffer=KeepLast())

        results: list[int] = []
        consumed = threading.Event()

        def consumer():
            for obs in live:
                results.append(obs.data)
                if obs.data >= 90:
                    consumed.set()
                    return
                time.sleep(0.1)  # slow consumer

        t = threading.Thread(target=consumer)
        t.start()

        time.sleep(0.05)
        # Rapid producer — KeepLast will drop most of these
        for i in range(100):
            stream.append(i, ts=float(i))
            time.sleep(0.001)

        consumed.wait(timeout=5.0)
        t.join(timeout=2.0)
        # KeepLast means many values were dropped — far fewer than 100
        assert len(results) < 50
        assert results[-1] >= 90

    def test_live_transform_receives_live_items(self, memory_session):
        """Transforms downstream of .live() see both backfill and live items."""
        stream = memory_session.stream("live_xf")
        stream.append(1, ts=0.0)
        double = FnTransformer(lambda obs: obs.derive(data=obs.data * 2))
        live = stream.live(buffer=Unbounded()).transform(double)

        results: list[int] = []
        consumed = threading.Event()

        def consumer():
            for obs in live:
                results.append(obs.data)
                if len(results) >= 3:
                    consumed.set()
                    return

        t = threading.Thread(target=consumer)
        t.start()

        time.sleep(0.05)
        stream.append(10, ts=1.0)
        stream.append(100, ts=2.0)

        consumed.wait(timeout=2.0)
        t.join(timeout=2.0)
        # All items went through the double transform
        assert results == [2, 20, 200]

    def test_live_on_transform_raises(self, make_stream):
        """Calling .live() on a transform stream raises TypeError."""
        stream = make_stream(3)
        xf = FnTransformer(lambda obs: obs)
        with pytest.raises(TypeError, match="Cannot call .live"):
            stream.transform(xf).live()

    def test_is_live(self, memory_session):
        """is_live() walks the source chain to detect live mode."""
        stream = memory_session.stream("is_live")
        assert not stream.is_live()

        live = stream.live(buffer=Unbounded())
        assert live.is_live()

        xf = FnTransformer(lambda obs: obs)
        transformed = live.transform(xf)
        assert transformed.is_live()

        # Two levels deep
        double_xf = transformed.transform(xf)
        assert double_xf.is_live()

        # Non-live transform is not live
        assert not stream.transform(xf).is_live()

    def test_search_on_live_transform_raises(self, memory_session):
        """search() on a transform with live upstream raises immediately."""
        stream = memory_session.stream("live_search")
        xf = FnTransformer(lambda obs: obs)
        live_xf = stream.live(buffer=Unbounded()).transform(xf)

        import numpy as np

        from dimos.models.embedding.base import Embedding

        vec = Embedding(vector=np.array([1.0, 0.0, 0.0]))
        with pytest.raises(TypeError, match="requires finite data"):
            # Use list() to trigger iteration — fetch() would hit its own guard first
            list(live_xf.search(vec, k=5))

    def test_order_by_on_live_transform_raises(self, memory_session):
        """order_by() on a transform with live upstream raises immediately."""
        stream = memory_session.stream("live_order")
        xf = FnTransformer(lambda obs: obs)
        live_xf = stream.live(buffer=Unbounded()).transform(xf)

        with pytest.raises(TypeError, match="requires finite data"):
            list(live_xf.order_by("ts", desc=True))

    def test_fetch_on_live_without_limit_raises(self, memory_session):
        """fetch() on a live stream without limit() raises TypeError."""
        stream = memory_session.stream("live_fetch")
        live = stream.live(buffer=Unbounded())

        with pytest.raises(TypeError, match="block forever"):
            live.to_list()

    def test_fetch_on_live_transform_without_limit_raises(self, memory_session):
        """fetch() on a live transform without limit() raises TypeError."""
        stream = memory_session.stream("live_fetch_xf")
        xf = FnTransformer(lambda obs: obs)
        live_xf = stream.live(buffer=Unbounded()).transform(xf)

        with pytest.raises(TypeError, match="block forever"):
            live_xf.to_list()

    def test_count_on_live_transform_raises(self, memory_session):
        """count() on a live transform stream raises TypeError."""
        stream = memory_session.stream("live_count")
        xf = FnTransformer(lambda obs: obs)
        live_xf = stream.live(buffer=Unbounded()).transform(xf)

        with pytest.raises(TypeError, match="block forever"):
            live_xf.count()

    def test_last_on_live_transform_raises(self, memory_session):
        """last() on a live transform raises TypeError (via order_by guard)."""
        stream = memory_session.stream("live_last")
        xf = FnTransformer(lambda obs: obs)
        live_xf = stream.live(buffer=Unbounded()).transform(xf)

        with pytest.raises(TypeError, match="requires finite data"):
            live_xf.last()

    def test_live_chained_transforms(self, memory_session):
        """stream.live().transform(A).transform(B) — both transforms applied to live items."""
        stream = memory_session.stream("live_chain")
        stream.append(1, ts=0.0)
        add_one = FnTransformer(lambda obs: obs.derive(data=obs.data + 1))
        double = FnTransformer(lambda obs: obs.derive(data=obs.data * 2))
        live = stream.live(buffer=Unbounded()).transform(add_one).transform(double)

        results: list[int] = []
        consumed = threading.Event()

        def consumer():
            for obs in live:
                results.append(obs.data)
                if len(results) >= 3:
                    consumed.set()
                    return

        t = threading.Thread(target=consumer)
        t.start()

        time.sleep(0.05)
        stream.append(10, ts=1.0)
        stream.append(100, ts=2.0)

        consumed.wait(timeout=2.0)
        t.join(timeout=2.0)
        # (1+1)*2=4, (10+1)*2=22, (100+1)*2=202
        assert results == [4, 22, 202]

    def test_live_filter_before_live(self, memory_session):
        """Filters applied before .live() work on both backfill and live items."""
        stream = memory_session.stream("live_pre_filter")
        stream.append("a", ts=1.0)
        stream.append("b", ts=10.0)
        live = stream.after(5.0).live(buffer=Unbounded())

        results: list[str] = []
        consumed = threading.Event()

        def consumer():
            for obs in live:
                results.append(obs.data)
                if len(results) >= 2:
                    consumed.set()
                    return

        t = threading.Thread(target=consumer)
        t.start()

        time.sleep(0.05)
        stream.append("c", ts=3.0)  # filtered
        stream.append("d", ts=20.0)  # passes

        consumed.wait(timeout=2.0)
        t.join(timeout=2.0)
        # "a" filtered in backfill, "c" filtered in live
        assert results == ["b", "d"]
        # "a" filtered in backfill, "c" filtered in live
        assert results == ["b", "d"]
        assert results == ["b", "d"]
        assert results == ["b", "d"]


class TestAlign:
    """Pairwise nearest-ts alignment between two streams."""

    def test_basic_pairing(self, session):
        """Each primary pairs with the closest in-tolerance secondary."""
        lidar = session.stream("lidar", str)
        image = session.stream("image", str)
        # primary at 0.0, 0.1, 0.2; secondary slightly offset
        for i, v in enumerate(["L0", "L1", "L2"]):
            lidar.append(v, ts=i * 0.1)
        for ts, v in [(0.02, "I0"), (0.11, "I1"), (0.19, "I2")]:
            image.append(v, ts=ts)

        pairs = lidar.align(image, tolerance=0.05).to_list()
        assert [p.data.lidar.data for p in pairs] == ["L0", "L1", "L2"]
        assert [p.data.image.data for p in pairs] == ["I0", "I1", "I2"]

    def test_named_and_index_access(self, session):
        """Pair fields are accessible by source-stream name and by index."""
        a = session.stream("alpha", str)
        b = session.stream("beta", str)
        a.append("a0", ts=0.0)
        b.append("b0", ts=0.001)

        pair = a.align(b, tolerance=0.01).to_list()[0].data
        assert pair.alpha.data == "a0"
        assert pair.beta.data == "b0"
        assert pair[0].data == "a0"
        assert pair[1].data == "b0"

    def test_same_name_falls_back_to_primary_secondary(self, session):
        """Aligning streams that share a name names fields primary/secondary."""
        a = session.stream("dup", str)
        a.append("x", ts=0.0)

        pair = a.align(a, tolerance=0.01).to_list()[0].data
        assert pair.primary.data == "x"
        assert pair.secondary.data == "x"
        assert pair[0].data == "x"

    def test_outer_obs_carries_primary_metadata(self, session):
        """The yielded outer Observation keeps the primary's ts/pose/tags."""
        a = session.stream("primary", str)
        b = session.stream("secondary", str)
        a.append("pa", ts=10.0, pose=(1, 2, 3), tags={"src": "p"})
        b.append("sb", ts=10.005, pose=(9, 9, 9), tags={"src": "s"})

        out = a.align(b, tolerance=0.05).to_list()[0]
        assert out.ts == 10.0
        assert out.pose_tuple[:3] == (1, 2, 3)
        assert out.tags == {"src": "p"}
        # Secondary's pose still reachable via the pair.
        assert out.data.secondary.pose_tuple[:3] == (9, 9, 9)

    def test_skip_when_no_match_in_tolerance(self, session):
        """Primary frames with no secondary within tolerance are dropped."""
        a = session.stream("a", str)
        b = session.stream("b", str)
        a.append("a0", ts=0.0)
        a.append("a1", ts=1.0)
        a.append("a2", ts=2.0)
        # Only a match near a1.
        b.append("b1", ts=1.02)

        pairs = a.align(b, tolerance=0.05).to_list()
        assert [p.data.a.data for p in pairs] == ["a1"]
        assert [p.data.b.data for p in pairs] == ["b1"]

    def test_one_secondary_matches_many_primaries(self, session):
        """Same secondary can pair with multiple primaries — no early eviction."""
        a = session.stream("a", str)
        b = session.stream("b", str)
        for ts in (1.000, 1.005, 1.010):
            a.append(f"a{ts}", ts=ts)
        b.append("b", ts=1.004)

        pairs = a.align(b, tolerance=0.05).to_list()
        assert len(pairs) == 3
        assert all(p.data.b.data == "b" for p in pairs)

    def test_primary_before_first_secondary(self, session):
        """Early primaries without any in-window secondary are skipped, later ones pair."""
        a = session.stream("a", str)
        b = session.stream("b", str)
        for ts in (0.0, 0.1, 0.2, 0.3):
            a.append(f"a{ts}", ts=ts)
        b.append("b", ts=0.3)

        pairs = a.align(b, tolerance=0.05).to_list()
        assert [p.ts for p in pairs] == [0.3]

    def test_empty_secondary(self, session):
        """Empty secondary stream → empty alignment, no error."""
        a = session.stream("a", str)
        b = session.stream("b", str)  # leave empty
        a.append("a0", ts=0.0)
        pairs = a.align(b, tolerance=1.0).to_list()
        assert pairs == []

    def test_filter_on_either_side_respected(self, session):
        """Filters applied before .align() restrict the iterated set on that side."""
        a = session.stream("a", str)
        b = session.stream("b", str)
        for ts in (0.0, 1.0, 2.0):
            a.append(f"a{ts}", ts=ts)
            b.append(f"b{ts}", ts=ts + 0.01)

        pairs = a.after(0.5).align(b.before(1.5), tolerance=0.05).to_list()
        assert [p.data.a.data for p in pairs] == ["a1.0"]
        assert [p.data.b.data for p in pairs] == ["b1.0"]

    def test_transform_source_allowed(self, session):
        """A transform-sourced stream on either side aligns fine (source-agnostic)."""
        a = session.stream("a", int)
        b = session.stream("b", int)
        for ts in (0.0, 1.0):
            a.append(int(ts), ts=ts)
            b.append(int(ts) + 100, ts=ts + 0.01)

        # Primary transformed (+1), secondary transformed (+1) — both still ts-ordered.
        primary = a.map(lambda obs: obs.derive(data=obs.data + 1))
        secondary = b.map(lambda obs: obs.derive(data=obs.data + 1))
        pairs = primary.align(secondary, tolerance=0.05).to_list()
        # Field names inherit through the transform from the backing streams.
        assert [p.data.a.data for p in pairs] == [1, 2]
        assert [p.data.b.data for p in pairs] == [101, 102]

    def test_live_alignment(self, memory_session):
        """align() streams over live inputs — a pair emits once its bracket closes."""
        p = memory_session.stream("lp")
        s = memory_session.stream("ls")
        # Backfilled matched pair so the first result is immediate.
        p.append("P0", ts=1.0)
        s.append("S0", ts=1.002)

        aligned = p.live(buffer=Unbounded()).align(s.live(buffer=Unbounded()), tolerance=0.1)

        results: list[tuple[str, str]] = []
        consumed = threading.Event()

        def consumer():
            for obs in aligned:
                results.append((obs.data.lp.data, obs.data.ls.data))
                if len(results) >= 2:
                    consumed.set()
                    return

        t = threading.Thread(target=consumer, daemon=True)
        t.start()

        time.sleep(0.05)
        # Live primary, then a secondary just past it so P1's bracket closes.
        p.append("P1", ts=2.0)
        s.append("S1", ts=2.003)

        consumed.wait(timeout=2.0)
        t.join(timeout=2.0)
        assert results == [("P0", "S0"), ("P1", "S1")]


class TestAlignLargeInput:
    """The two-pointer merge scales to dense streams in a single forward pass."""

    def test_dense_secondary_pairs_correctly(self, memory_session):
        """5k primaries against 50k secondaries — every primary gets its exact match."""
        primary = memory_session.stream("p")
        secondary = memory_session.stream("s")
        # secondary at 500 Hz, primary at 50 Hz — each primary lands on a secondary.
        for i in range(5_000):
            primary.append(i, ts=i * 0.02)
        for i in range(50_000):
            secondary.append(i, ts=i * 0.002)

        pairs = primary.align(secondary, tolerance=0.005).to_list()
        assert len(pairs) == 5_000
        # Each primary's nearest secondary is the coincident one (Δts ≈ 0).
        assert max(abs(p.data[0].ts - p.data[1].ts) for p in pairs) < 1e-9


class TestWithPose:
    """``Observation.with_pose`` attaches/clears a pose without touching the payload."""

    def test_attaches_pose_and_keeps_payload_lazy(self):
        """Pose is set, but the payload loader stays untouched until accessed."""
        loads = 0

        def loader() -> str:
            nonlocal loads
            loads += 1
            return "payload"

        obs: Observation[str] = Observation(id=1, ts=5.0, data_type=str, _loader=loader)
        posed = obs.with_pose((1.0, 2.0, 3.0))

        # 3-tuple padded with the identity quaternion; payload not materialized.
        assert posed.pose_tuple == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
        assert posed.ts == 5.0
        assert loads == 0
        # The payload still resolves lazily on first access.
        assert posed.data == "payload"
        assert loads == 1

    def test_pose_none_clears(self):
        obs: Observation[str] = Observation(id=1, ts=0.0, data_type=str, pose=(1, 2, 3), _data="x")
        cleared = obs.with_pose(None)
        assert cleared.pose_tuple is None
        assert cleared.data == "x"

    def test_overwrites_existing_pose(self):
        obs: Observation[str] = Observation(id=1, ts=0.0, data_type=str, pose=(1, 2, 3), _data="x")
        moved = obs.with_pose((7, 8, 9))
        assert moved.pose_tuple is not None
        assert moved.pose_tuple[:3] == (7.0, 8.0, 9.0)


class TestWindowing:
    """Index (*_seek) and time (*_time) windowing; None = unbounded that side."""

    def test_to_seek_keeps_first_i(self, make_stream):
        assert [o.data for o in make_stream(10).to_seek(3)] == [0, 10, 20]

    def test_from_seek_drops_first_i(self, make_stream):
        assert [o.data for o in make_stream(10).from_seek(7)] == [70, 80, 90]

    def test_range_seek(self, make_stream):
        assert [o.data for o in make_stream(10).range_seek(2, 5)] == [20, 30, 40]

    def test_to_time_keeps_first_seconds(self, make_stream):
        # ts == index here; to_time(s) keeps ts < first_ts + s
        assert [o.data for o in make_stream(10).to_time(2.0)] == [0, 10]

    def test_from_time(self, make_stream):
        assert [o.data for o in make_stream(10).from_time(7.0)] == [80, 90]

    def test_range_time_uses_one_anchor(self, make_stream):
        # anchor is the first observation, not the post-filter first
        assert [o.data for o in make_stream(10, start_ts=100.0).range_time(2.0, 5.0)] == [30, 40]

    def test_none_is_noop(self, make_stream):
        s = make_stream(10)
        assert s.to_seek(None).count() == 10
        assert s.from_seek(None).count() == 10
        assert s.range_seek(None, None).count() == 10
        assert s.to_time(None).count() == 10
        assert s.range_time(None, None).count() == 10


class TestTimeWindowing:
    """``*_time`` is relative to the first observation, ``*_timestamp`` is absolute.

    A trailing ``to_time`` measures a duration from the current start, so chaining
    reads as "skip, then take the following N seconds".
    """

    def test_from_time_skips_relative_seconds(self, make_stream):
        stream = make_stream(10)  # ts 0..9
        assert [o.data for o in stream.from_time(3)] == [40, 50, 60, 70, 80, 90]

    def test_to_time_keeps_leading_seconds(self, make_stream):
        stream = make_stream(10)
        assert [o.data for o in stream.to_time(3)] == [0, 10, 20]

    def test_chained_skip_then_following_duration(self, make_stream):
        stream = make_stream(10)  # ts 0..9
        # skip first 2s, then the following 3s -> ts 3, 4, 5
        assert [o.data for o in stream.from_time(2).to_time(3)] == [30, 40, 50]

    def test_from_timestamp_is_absolute(self, make_stream):
        stream = make_stream(10, start_ts=1000.0)  # ts 1000..1009
        assert [o.data for o in stream.from_timestamp(1003.0)] == [40, 50, 60, 70, 80, 90]

    def test_to_timestamp_is_absolute(self, make_stream):
        stream = make_stream(10, start_ts=1000.0)
        assert [o.data for o in stream.to_timestamp(1003.0)] == [0, 10, 20]

    def test_absolute_range(self, make_stream):
        stream = make_stream(10, start_ts=1000.0)
        # all between 1002 and 1006 -> ts 1003, 1004, 1005
        windowed = stream.from_timestamp(1002.0).to_timestamp(1006.0)
        assert [o.data for o in windowed] == [30, 40, 50]

    def test_absolute_start_relative_duration(self, make_stream):
        stream = make_stream(10, start_ts=1000.0)
        # seek to 1003, then the following 3s -> ts 1004, 1005, 1006
        assert [o.data for o in stream.from_timestamp(1003.0).to_time(3)] == [40, 50, 60]

    def test_none_bounds_are_noops(self, make_stream):
        stream = make_stream(5)
        full = [0, 10, 20, 30, 40]
        assert [o.data for o in stream.from_time(None)] == full
        assert [o.data for o in stream.to_time(None)] == full
        assert [o.data for o in stream.from_timestamp(None)] == full
        assert [o.data for o in stream.to_timestamp(None)] == full

    def test_relative_bounds_on_empty_yield_empty(self, make_stream):
        # Relative bounds resolve an anchor from first(); an empty slice (here,
        # seeking past all data) must yield empty, not raise LookupError.
        stream = make_stream(10, start_ts=1000.0)
        assert stream.from_timestamp(9999.0).to_time(30).to_list() == []
        assert make_stream(0).from_time(5).to_list() == []
        assert make_stream(0).to_time(5).to_list() == []


def _two_streams(
    session, primary_pts, secondary_pts, *, primary="lidar", secondary="pose", dtype=int
):
    """Build two named streams from (ts, value) point lists for align tests.

    ``dtype`` is the declared payload type; pass ``object`` to mix payload types
    (e.g. int primaries with float interpolation values) without tripping the
    backend's type check.
    """
    a = session.stream(primary, dtype)
    for ts, value in primary_pts:
        a.append(value, ts=ts)
    b = session.stream(secondary, dtype)
    for ts, value in secondary_pts:
        b.append(value, ts=ts)
    return a, b


class TestAlign:
    """`.align(other, tolerance=)` pairs each primary observation with the
    nearest-in-time secondary within tolerance (PR #2306 nearest-neighbor)."""

    def test_pairs_closer_of_the_two_bracketing_secondaries(self, session):
        """Each primary takes the nearer of the secondaries bracketing it -
        including when the *later* one is closer. A flipped comparison would
        silently pick the earlier secondary instead."""
        lidar, pose = _two_streams(
            session,
            [(0.0, 10), (1.0, 20), (2.0, 30)],
            [(0.1, 100), (0.9, 200), (2.2, 300)],
        )
        out = lidar.align(pose, tolerance=0.5).to_list()
        # ts=2.0 sits between 0.9 (Δ1.1) and 2.2 (Δ0.2): the later one wins.
        assert [o.data.pose.data for o in out] == [100, 200, 300]
        assert [o.data.lidar.data for o in out] == [10, 20, 30]

    def test_exact_timestamp_match_wins_over_nearer_neighbours(self, session):
        """A secondary at the exact primary ts (distance 0) is selected over
        in-tolerance neighbours on either side - the global optimum always wins."""
        lidar, pose = _two_streams(
            session,
            [(1.0, 10)],
            [(0.6, 1), (1.0, 2), (1.4, 3)],
        )
        out = lidar.align(pose, tolerance=0.5).to_list()
        assert [o.data.pose.data for o in out] == [2]

    def test_pair_fields_named_after_streams(self, session):
        """obs.data is an AlignedPair whose fields are the two stream names,
        each holding the full Observation; index access mirrors them."""
        lidar, pose = _two_streams(session, [(1.0, 10)], [(1.05, 100)])
        pair = lidar.align(pose, tolerance=0.1).to_list()[0].data
        assert pair._fields == ("lidar", "pose")
        assert pair.lidar.data == 10 and pair.pose.data == 100
        assert pair.lidar.ts == 1.0 and pair.pose.ts == 1.05
        assert pair[0] is pair.lidar and pair[1] is pair.pose

    def test_shared_name_falls_back_to_primary_secondary(self, session):
        """Aligning a stream with itself (identical name) names the fields
        primary/secondary rather than colliding on one name."""
        a = session.stream("lidar", int)
        a.append(10, ts=1.0)
        pair = a.align(a, tolerance=0.1).to_list()[0].data
        assert pair._fields == ("primary", "secondary")
        assert pair.primary.data == 10 and pair.secondary.data == 10

    def test_output_carries_primary_timestamp_and_metadata(self, memory_session):
        """The emitted observation keeps the primary's ts/pose/tags, not the
        secondary's - downstream time logic must see the primary (scan) time."""
        lidar = memory_session.stream("lidar", int)
        lidar.append(10, ts=1.0, pose=(1, 2, 3), tags={"k": "v"})
        pose = memory_session.stream("pose", int)
        pose.append(100, ts=1.4)  # secondary ts differs from primary
        out = lidar.align(pose, tolerance=0.5).to_list()[0]
        assert out.ts == 1.0  # primary ts, not the secondary's 1.4
        assert out.pose_tuple == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
        assert out.tags == {"k": "v"}

    def test_secondaries_beyond_tolerance_match_nothing(self, session):
        """A secondary 3x tolerance away pairs with no primary; widening the
        tolerance past the gap recovers every pair."""
        lidar, pose = _two_streams(
            session,
            [(0.0, 10), (1.0, 20), (2.0, 30)],
            [(0.3, 100), (1.3, 200), (2.3, 300)],
        )
        assert lidar.align(pose, tolerance=0.1).to_list() == []
        assert len(lidar.align(pose, tolerance=0.4).to_list()) == 3

    def test_only_far_primaries_are_skipped(self, session):
        """With one secondary near the first primary, the in-tolerance primary
        pairs and the rest drop - not all-or-nothing."""
        lidar, pose = _two_streams(
            session,
            [(0.0, 10), (1.0, 20), (2.0, 30)],
            [(0.05, 100)],
        )
        out = lidar.align(pose, tolerance=0.1).to_list()
        assert [o.data.lidar.data for o in out] == [10]

    def test_empty_secondary_yields_no_pairs(self, session):
        """No secondary observations means nothing to pair, at any tolerance."""
        lidar, pose = _two_streams(session, [(0.0, 10), (1.0, 20)], [])
        assert lidar.align(pose, tolerance=10.0).to_list() == []


def _lerp(prev, nxt, alpha):
    """Linear interpolation of the secondary's scalar payload at the primary ts."""
    return prev.data + alpha * (nxt.data - prev.data)


class TestAlignInterpolation:
    """`.align(..., interpolator=)` synthesizes the secondary value at the
    primary timestamp from its two bracketing samples instead of snapping to the
    nearest one (§5.6)."""

    def test_value_is_interpolated_at_the_primary_timestamp(self, session):
        """The paired secondary is the bracket interpolation evaluated at the
        primary ts, and alpha tracks the true fraction of the gap - a quarter of
        the way in gives a quarter, halfway gives the midpoint - so a hard-coded
        0.5 would fail the first sample."""
        lidar, pose = _two_streams(
            session,
            [(1.25, 20), (1.5, 10)],
            [(1.0, 0.0), (2.0, 90.0)],
            dtype=object,
        )
        out = lidar.align(pose, tolerance=1.0, interpolator=_lerp).to_list()
        # alpha = 0.25 -> 22.5, alpha = 0.5 -> 45.0 (≈45° between the poses).
        assert [round(o.data.pose.data, 6) for o in out] == [22.5, 45.0]
        # The secondary side is a full Observation stamped at the primary ts,
        # not at either real sample's ts.
        assert [o.data.pose.ts for o in out] == [1.25, 1.5]
        # Primary side and the emitted observation's ts are untouched.
        assert [o.data.lidar.data for o in out] == [20, 10]
        assert [o.ts for o in out] == [1.25, 1.5]

    def test_exact_match_uses_exact_secondary_and_skips_interpolator(self, session):
        """A secondary at the exact primary ts is paired verbatim and the
        interpolator is never consulted - a raising interpolator stays harmless,
        and the pair carries the real sample's value and ts."""

        def _boom(prev, nxt, alpha):
            raise AssertionError("interpolator called on an exact ts match")

        lidar, pose = _two_streams(
            session,
            [(1.0, 10)],
            [(0.0, 0.0), (1.0, 50.0), (2.0, 90.0)],
            dtype=object,
        )
        out = lidar.align(pose, tolerance=1.5, interpolator=_boom).to_list()
        assert [o.data.pose.data for o in out] == [50.0]
        assert [o.data.pose.ts for o in out] == [1.0]

    def test_one_sided_bracket_is_dropped_even_within_tolerance(self, session):
        """Interpolation needs a secondary on both sides. A primary before the
        first secondary (no left bracket) or after the last (no right bracket) is
        dropped even though a one-sided neighbour sits within tolerance; only the
        two-sided primary survives. Nearest-neighbour would have paired all three."""
        lidar, pose = _two_streams(
            session,
            [(0.5, 1), (1.5, 2), (2.5, 3)],
            [(1.0, 0.0), (2.0, 90.0)],
            dtype=object,
        )
        out = lidar.align(pose, tolerance=1.0, interpolator=_lerp).to_list()
        # 0.5: nxt(1.0) in tol but no prev -> skip. 1.5: bracketed -> 45.0.
        # 2.5: prev(2.0) in tol but no nxt -> skip.
        assert [o.data.lidar.data for o in out] == [2]
        assert [round(o.data.pose.data, 6) for o in out] == [45.0]

    def test_out_of_tolerance_bracket_skips_then_widening_recovers(self, session):
        """Both brackets must lie within tolerance: a far right bracket drops the
        primary even though the left bracket is close. Widening the tolerance past
        the far side recovers the interpolated pair (alpha still measured across
        the full bracket span)."""
        lidar, pose = _two_streams(
            session,
            [(1.1, 10)],
            [(1.0, 0.0), (2.0, 100.0)],
            dtype=object,
        )
        # prev(1.0) is 0.1 away, nxt(2.0) is 0.9 away: tol 0.5 excludes nxt.
        assert lidar.align(pose, tolerance=0.5, interpolator=_lerp).to_list() == []
        # tol 1.0 admits both -> interpolate at alpha = 0.1 / 1.0 = 0.1.
        out = lidar.align(pose, tolerance=1.0, interpolator=_lerp).to_list()
        assert [round(o.data.pose.data, 6) for o in out] == [10.0]

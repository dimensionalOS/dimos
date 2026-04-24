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
        assert derived.pose == (1, 2, 3)
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

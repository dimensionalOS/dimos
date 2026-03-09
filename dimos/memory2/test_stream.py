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

"""memory2 stream tests — serves as living documentation of the lazy stream API.

Each test demonstrates a specific capability with clear setup, action, and assertion.
"""

from __future__ import annotations

import threading
import time

import pytest

from dimos.memory2.backend import ListBackend
from dimos.memory2.buffer import Bounded, ClosedError, DropNew, KeepLast, Unbounded
from dimos.memory2.store import ListStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import FnTransformer, QualityWindow, Transformer
from dimos.memory2.type import Observation

# ── Helpers ──────────────────────────────────────────────────────────


def make_stream(n: int = 5, start_ts: float = 0.0) -> Stream[int]:
    """Create a ListBackend stream with n integer observations at 1-second intervals."""
    backend = ListBackend[int]("test")
    for i in range(n):
        backend.append(i * 10, ts=start_ts + i)
    return Stream(source=backend)


# ═══════════════════════════════════════════════════════════════════
#  1. Basic iteration
# ═══════════════════════════════════════════════════════════════════


class TestBasicIteration:
    """Streams are lazy iterables — nothing runs until you iterate."""

    def test_iterate_yields_all_observations(self):
        stream = make_stream(5)
        obs = list(stream)
        assert len(obs) == 5
        assert [o.data for o in obs] == [0, 10, 20, 30, 40]

    def test_iterate_preserves_timestamps(self):
        stream = make_stream(3, start_ts=100.0)
        assert [o.ts for o in stream] == [100.0, 101.0, 102.0]

    def test_empty_stream(self):
        stream = make_stream(0)
        assert list(stream) == []

    def test_fetch_materializes_to_list(self):
        result = make_stream(3).fetch()
        assert isinstance(result, list)
        assert len(result) == 3

    def test_stream_is_reiterable(self):
        """Same stream can be iterated multiple times — each time re-queries."""
        stream = make_stream(3)
        first = [o.data for o in stream]
        second = [o.data for o in stream]
        assert first == second == [0, 10, 20]


# ═══════════════════════════════════════════════════════════════════
#  2. Temporal filters
# ═══════════════════════════════════════════════════════════════════


class TestTemporalFilters:
    """Temporal filters constrain observations by timestamp."""

    def test_after(self):
        """.after(t) keeps observations with ts > t."""
        result = make_stream(5).after(2.0).fetch()
        assert [o.ts for o in result] == [3.0, 4.0]

    def test_before(self):
        """.before(t) keeps observations with ts < t."""
        result = make_stream(5).before(2.0).fetch()
        assert [o.ts for o in result] == [0.0, 1.0]

    def test_time_range(self):
        """.time_range(t1, t2) keeps t1 <= ts <= t2."""
        result = make_stream(5).time_range(1.0, 3.0).fetch()
        assert [o.ts for o in result] == [1.0, 2.0, 3.0]

    def test_at_with_tolerance(self):
        """.at(t, tolerance) keeps observations within tolerance of t."""
        result = make_stream(5).at(2.0, tolerance=0.5).fetch()
        assert [o.ts for o in result] == [2.0]

    def test_chained_temporal_filters(self):
        """Filters compose — each narrows the result."""
        result = make_stream(10).after(2.0).before(7.0).fetch()
        assert [o.ts for o in result] == [3.0, 4.0, 5.0, 6.0]


# ═══════════════════════════════════════════════════════════════════
#  3. Spatial filter
# ═══════════════════════════════════════════════════════════════════


class TestSpatialFilter:
    """.near(pose, radius) filters by Euclidean distance."""

    def test_near_with_tuples(self):
        backend = ListBackend[str]("spatial")
        backend.append("origin", ts=0.0, pose=(0, 0, 0))
        backend.append("close", ts=1.0, pose=(1, 1, 0))
        backend.append("far", ts=2.0, pose=(10, 10, 10))
        stream = Stream(source=backend)

        result = stream.near((0, 0, 0), radius=2.0).fetch()
        assert [o.data for o in result] == ["origin", "close"]

    def test_near_excludes_no_pose(self):
        backend = ListBackend[str]("spatial")
        backend.append("no_pose", ts=0.0)
        backend.append("has_pose", ts=1.0, pose=(0, 0, 0))
        stream = Stream(source=backend)

        result = stream.near((0, 0, 0), radius=10.0).fetch()
        assert [o.data for o in result] == ["has_pose"]


# ═══════════════════════════════════════════════════════════════════
#  4. Tags filter
# ═══════════════════════════════════════════════════════════════════


class TestTagsFilter:
    """.filter_tags() matches on observation metadata."""

    def test_filter_by_tag(self):
        backend = ListBackend[str]("tagged")
        backend.append("cat", ts=0.0, tags={"type": "animal", "legs": 4})
        backend.append("car", ts=1.0, tags={"type": "vehicle", "wheels": 4})
        backend.append("dog", ts=2.0, tags={"type": "animal", "legs": 4})
        stream = Stream(source=backend)

        result = stream.filter_tags(type="animal").fetch()
        assert [o.data for o in result] == ["cat", "dog"]

    def test_filter_multiple_tags(self):
        backend = ListBackend[str]("tagged")
        backend.append("a", ts=0.0, tags={"x": 1, "y": 2})
        backend.append("b", ts=1.0, tags={"x": 1, "y": 3})
        stream = Stream(source=backend)

        result = stream.filter_tags(x=1, y=2).fetch()
        assert [o.data for o in result] == ["a"]


# ═══════════════════════════════════════════════════════════════════
#  5. Ordering, limit, offset
# ═══════════════════════════════════════════════════════════════════


class TestOrderLimitOffset:
    def test_limit(self):
        result = make_stream(10).limit(3).fetch()
        assert len(result) == 3

    def test_offset(self):
        result = make_stream(5).offset(2).fetch()
        assert [o.data for o in result] == [20, 30, 40]

    def test_limit_and_offset(self):
        result = make_stream(10).offset(2).limit(3).fetch()
        assert [o.data for o in result] == [20, 30, 40]

    def test_order_by_ts_desc(self):
        result = make_stream(5).order_by("ts", desc=True).fetch()
        assert [o.ts for o in result] == [4.0, 3.0, 2.0, 1.0, 0.0]

    def test_first(self):
        obs = make_stream(5).first()
        assert obs.data == 0

    def test_last(self):
        obs = make_stream(5).last()
        assert obs.data == 40

    def test_first_empty_raises(self):
        with pytest.raises(LookupError):
            make_stream(0).first()

    def test_count(self):
        assert make_stream(5).count() == 5
        assert make_stream(5).after(2.0).count() == 2

    def test_exists(self):
        assert make_stream(5).exists()
        assert not make_stream(0).exists()
        assert not make_stream(5).after(100.0).exists()


# ═══════════════════════════════════════════════════════════════════
#  6. Functional API: .filter(), .map()
# ═══════════════════════════════════════════════════════════════════


class TestFunctionalAPI:
    """Functional combinators receive the full Observation."""

    def test_filter_with_predicate(self):
        """.filter() takes a predicate on the full Observation."""
        result = make_stream(5).filter(lambda obs: obs.data > 20).fetch()
        assert [o.data for o in result] == [30, 40]

    def test_filter_on_metadata(self):
        """Predicates can access ts, tags, pose — not just data."""
        result = make_stream(5).filter(lambda obs: obs.ts % 2 == 0).fetch()
        assert [o.ts for o in result] == [0.0, 2.0, 4.0]

    def test_map(self):
        """.map() transforms each observation's data."""
        result = make_stream(3).map(lambda obs: obs.derive(data=obs.data * 2)).fetch()
        assert [o.data for o in result] == [0, 20, 40]

    def test_map_preserves_ts(self):
        result = make_stream(3).map(lambda obs: obs.derive(data=str(obs.data))).fetch()
        assert [o.ts for o in result] == [0.0, 1.0, 2.0]
        assert [o.data for o in result] == ["0", "10", "20"]


# ═══════════════════════════════════════════════════════════════════
#  7. Transform chaining
# ═══════════════════════════════════════════════════════════════════


class TestTransformChaining:
    """Transforms chain lazily — each obs flows through the full pipeline."""

    def test_single_transform(self):
        xf = FnTransformer(lambda obs: obs.derive(data=obs.data + 1))
        result = make_stream(3).transform(xf).fetch()
        assert [o.data for o in result] == [1, 11, 21]

    def test_chained_transforms(self):
        """stream.transform(A).transform(B) — B pulls from A which pulls from source."""
        add_one = FnTransformer(lambda obs: obs.derive(data=obs.data + 1))
        double = FnTransformer(lambda obs: obs.derive(data=obs.data * 2))

        result = make_stream(3).transform(add_one).transform(double).fetch()
        # (0+1)*2=2, (10+1)*2=22, (20+1)*2=42
        assert [o.data for o in result] == [2, 22, 42]

    def test_transform_can_skip(self):
        """Returning None from a transformer skips that observation."""
        keep_even = FnTransformer(lambda obs: obs if obs.data % 20 == 0 else None)
        result = make_stream(5).transform(keep_even).fetch()
        assert [o.data for o in result] == [0, 20, 40]

    def test_transform_filter_transform(self):
        """stream.transform(A).near(pose).transform(B) — filter between transforms."""
        backend = ListBackend[int]("tfft")
        backend.append(1, ts=0.0, pose=(0, 0, 0))
        backend.append(2, ts=1.0, pose=(100, 100, 100))
        backend.append(3, ts=2.0, pose=(1, 0, 0))
        stream = Stream(source=backend)

        add_ten = FnTransformer(lambda obs: obs.derive(data=obs.data + 10))
        double = FnTransformer(lambda obs: obs.derive(data=obs.data * 2))

        result = (
            stream.transform(add_ten)  # 11, 12, 13
            .near((0, 0, 0), 5.0)  # keeps pose at (0,0,0) and (1,0,0)
            .transform(double)  # 22, 26
            .fetch()
        )
        assert [o.data for o in result] == [22, 26]

    def test_quality_window(self):
        """QualityWindow keeps the best item per time window."""
        backend = ListBackend[float]("qw")
        # Window 1: ts 0.0-0.9 → best quality
        backend.append(0.3, ts=0.0)
        backend.append(0.9, ts=0.3)  # best in window
        backend.append(0.1, ts=0.7)
        # Window 2: ts 1.0-1.9
        backend.append(0.5, ts=1.0)
        backend.append(0.8, ts=1.5)  # best in window
        # Window 3: ts 2.0+ (emitted at end via flush)
        backend.append(0.6, ts=2.2)
        stream = Stream(source=backend)

        xf = QualityWindow(quality_fn=lambda v: v, window=1.0)
        result = stream.transform(xf).fetch()
        assert [o.data for o in result] == [0.9, 0.8, 0.6]

    def test_streaming_not_buffering(self):
        """Transforms process lazily — early limit stops pulling from source."""
        calls = []

        class CountingXf(Transformer[int, int]):
            def __call__(self, upstream):
                for obs in upstream:
                    calls.append(obs.data)
                    yield obs

        result = make_stream(100).transform(CountingXf()).limit(3).fetch()
        assert len(result) == 3
        # The transformer should have processed at most a few more than 3
        # (not all 100) due to lazy evaluation
        assert len(calls) == 3


# ═══════════════════════════════════════════════════════════════════
#  8. Store & Session
# ═══════════════════════════════════════════════════════════════════


class TestStoreSession:
    """Store -> Session -> Stream hierarchy for named streams."""

    def test_basic_session(self):
        store = ListStore()
        with store.session() as session:
            images = session.stream("images")
            images.append("frame1", ts=0.0)
            images.append("frame2", ts=1.0)
            assert images.count() == 2

    def test_same_stream_on_repeated_calls(self):
        store = ListStore()
        with store.session() as session:
            s1 = session.stream("images")
            s2 = session.stream("images")
            assert s1 is s2

    def test_stream_namespace(self):
        store = ListStore()
        with store.session() as session:
            session.stream("images")
            session.stream("lidar")
            assert "images" in session.streams
            assert len(session.streams) == 2
            assert session.streams.images is session.stream("images")
            assert session.streams["lidar"] is session.stream("lidar")

    def test_namespace_missing_raises(self):
        store = ListStore()
        with store.session() as session:
            with pytest.raises(AttributeError, match="No stream named"):
                _ = session.streams.nonexistent

    def test_delete_stream(self):
        store = ListStore()
        with store.session() as session:
            session.stream("temp")
            session.delete_stream("temp")
            assert "temp" not in session.streams


# ═══════════════════════════════════════════════════════════════════
#  9. Lazy data loading
# ═══════════════════════════════════════════════════════════════════


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


# ═══════════════════════════════════════════════════════════════════
#  10. Backpressure buffers
# ═══════════════════════════════════════════════════════════════════


class TestBackpressureBuffers:
    """Thread-safe buffers bridging push sources to pull consumers."""

    def test_keep_last_overwrites(self):
        buf = KeepLast[int]()
        buf.put(1)
        buf.put(2)
        buf.put(3)
        assert buf.take() == 3
        assert len(buf) == 0

    def test_bounded_drops_oldest(self):
        buf = Bounded[int](maxlen=2)
        buf.put(1)
        buf.put(2)
        buf.put(3)  # drops 1
        assert buf.take() == 2
        assert buf.take() == 3

    def test_drop_new_rejects(self):
        buf = DropNew[int](maxlen=2)
        assert buf.put(1) is True
        assert buf.put(2) is True
        assert buf.put(3) is False  # rejected
        assert buf.take() == 1
        assert buf.take() == 2

    def test_unbounded_keeps_all(self):
        buf = Unbounded[int]()
        for i in range(100):
            buf.put(i)
        assert len(buf) == 100

    def test_close_signals_end(self):
        buf = KeepLast[int]()
        buf.close()
        with pytest.raises(ClosedError):
            buf.take()

    def test_buffer_is_iterable(self):
        """Iterating a buffer yields items until closed."""
        buf = Unbounded[int]()
        buf.put(1)
        buf.put(2)
        buf.close()
        assert list(buf) == [1, 2]

    def test_take_blocks_until_put(self):
        buf = KeepLast[int]()
        result = []

        def producer():
            time.sleep(0.05)
            buf.put(42)

        t = threading.Thread(target=producer)
        t.start()
        result.append(buf.take(timeout=2.0))
        t.join()
        assert result == [42]

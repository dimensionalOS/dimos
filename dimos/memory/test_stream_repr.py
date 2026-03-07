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

"""Tests for Stream.__repr__ and Filter.__str__."""

from __future__ import annotations

import pytest

from dimos.memory.impl.sqlite import SqliteStore
from dimos.memory.stream import Stream
from dimos.memory.transformer import PerItemTransformer
from dimos.memory.types import (
    AfterFilter,
    AtFilter,
    BeforeFilter,
    EmbeddingSearchFilter,
    LineageFilter,
    NearFilter,
    StreamQuery,
    TagsFilter,
    TextSearchFilter,
    TimeRangeFilter,
)

# ── Filter __str__ ────────────────────────────────────────────────────


class TestFilterStr:
    def test_after(self) -> None:
        assert str(AfterFilter(3.0)) == "after(t=3.0)"

    def test_before(self) -> None:
        assert str(BeforeFilter(10.5)) == "before(t=10.5)"

    def test_time_range(self) -> None:
        assert str(TimeRangeFilter(3.0, 10.0)) == "time_range(3.0, 10.0)"

    def test_at(self) -> None:
        assert str(AtFilter(5.0, 1.0)) == "at(t=5.0, tol=1.0)"

    def test_near(self) -> None:
        assert str(NearFilter(pose=None, radius=5.0)) == "near(radius=5.0)"

    def test_tags_single(self) -> None:
        assert str(TagsFilter((("cam", "front"),))) == "tags(cam='front')"

    def test_tags_multiple(self) -> None:
        f = TagsFilter((("cam", "front"), ("quality", 1)))
        assert str(f) == "tags(cam='front', quality=1)"

    def test_embedding_search(self) -> None:
        assert str(EmbeddingSearchFilter([0.1, 0.2], k=5)) == "search(k=5)"

    def test_text_search(self) -> None:
        assert str(TextSearchFilter("error", k=None)) == "text('error')"

    def test_lineage(self) -> None:
        f = LineageFilter("embeddings", StreamQuery(), hops=("filtered",))
        assert str(f) == "lineage(embeddings -> filtered)"

    def test_lineage_direct(self) -> None:
        f = LineageFilter("embeddings", StreamQuery(), hops=())
        assert str(f) == "lineage(embeddings -> direct)"


# ── Stream __repr__ ───────────────────────────────────────────────────


@pytest.fixture()
def session():
    store = SqliteStore(":memory:")
    store.start()
    s = store.session()
    yield s
    s.stop()
    store.stop()


class TestStreamRepr:
    def test_basic_stream(self, session) -> None:
        s = session.stream("images", int)
        assert repr(s) == 'Stream[int]("images")'

    def test_chain(self, session) -> None:
        s = session.stream("images", int)
        r = repr(s.after(3.0).filter_tags(cam="front").limit(10))
        assert r == "Stream[int](\"images\") | after(t=3.0) | tags(cam='front') | limit(10)"

    def test_order_and_offset(self, session) -> None:
        s = session.stream("images", int)
        r = repr(s.order_by("ts", desc=True).offset(5).limit(10))
        assert r == 'Stream[int]("images") | order(ts, desc) | limit(10) | offset(5)'

    def test_text_stream(self, session) -> None:
        ts = session.text_stream("logs")
        assert repr(ts) == 'TextStream[str]("logs")'

    def test_text_search(self, session) -> None:
        ts = session.text_stream("logs")
        r = repr(ts.search_text("error"))
        assert r == "TextStream[str](\"logs\") | text('error')"

    def test_embedding_stream(self, session) -> None:
        es = session.embedding_stream("clip", vec_dimensions=512)
        assert repr(es) == 'EmbeddingStream[Embedding]("clip")'

    def test_transform_stream(self, session) -> None:
        s = session.stream("images", int)
        xf = PerItemTransformer(lambda x: x)
        r = repr(s.transform(xf, live=True))
        assert r == 'TransformStream[?](Stream[int]("images") -> PerItemTransformer, live=True)'

    def test_transform_backfill_only(self, session) -> None:
        s = session.stream("images", int)
        xf = PerItemTransformer(lambda x: x)
        r = repr(s.transform(xf, backfill_only=True))
        assert (
            r
            == 'TransformStream[?](Stream[int]("images") -> PerItemTransformer, backfill_only=True)'
        )

    def test_unbound_stream(self) -> None:
        s = Stream(payload_type=int)
        assert repr(s) == 'Stream[int]("unbound")'

    def test_no_payload_type(self) -> None:
        s = Stream()
        assert repr(s) == 'Stream[?]("unbound")'

    def test_materialized_transform(self, session) -> None:
        s = session.stream("images", int)
        s.append(1, ts=1.0)
        xf = PerItemTransformer(lambda x: x * 2)
        derived = s.transform(xf).store("doubled", int)
        assert repr(derived) == 'Stream[int]("doubled")'

    def test_transform_with_typed_transformer(self, session) -> None:
        from unittest.mock import MagicMock

        from dimos.memory.transformer import EmbeddingTransformer

        s = session.stream("images", int)
        model = MagicMock()
        xf = EmbeddingTransformer(model)
        r = repr(s.transform(xf, live=True))
        assert (
            r
            == 'TransformStream[Embedding](Stream[int]("images") -> EmbeddingTransformer, live=True)'
        )

    def test_embedding_stream_from_source(self, session) -> None:
        session.stream("images", int)
        es = session.embedding_stream("clip", vec_dimensions=512, parent_table="images")
        assert (
            repr(es.after(5.0).limit(3))
            == 'EmbeddingStream[Embedding]("clip") | after(t=5.0) | limit(3)'
        )

    def test_ivan(self, session) -> None:
        from unittest.mock import MagicMock

        from dimos.memory.transformer import EmbeddingTransformer
        from dimos.msgs.sensor_msgs.Image import Image

        s = session.stream("images", Image).after(5.0).limit(3)
        print("\n")
        print(s)
        model = MagicMock()
        print(s.transform(EmbeddingTransformer(model)).limit(3))

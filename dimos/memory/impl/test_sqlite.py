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

"""Tests for SQLite-backed memory store."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.memory.impl.sqlite import SqliteSession, SqliteStore
from dimos.memory.transformer import EmbeddingTransformer
from dimos.memory.types import EmbeddingObservation, Observation, _Unset
from dimos.models.embedding.base import Embedding, EmbeddingModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.testing import TimedSensorReplay


def _img_close(a: Image, b: Image, max_diff: float = 5.0) -> bool:
    """Approximate Image equality (JPEG is lossy)."""
    if a.data.shape != b.data.shape:
        return False
    if a.frame_id != b.frame_id:
        return False
    return float(np.abs(a.data.astype(np.float32) - b.data.astype(np.float32)).mean()) < max_diff


@pytest.fixture(scope="module")
def replay() -> TimedSensorReplay:  # type: ignore[type-arg]
    return TimedSensorReplay("unitree_go2_bigoffice/video")


@pytest.fixture(scope="module")
def images(replay: TimedSensorReplay) -> list[Image]:  # type: ignore[type-arg]
    """Load 5 images from replay at 1s intervals."""
    imgs = [replay.find_closest_seek(float(i)) for i in range(1, 6)]
    assert all(isinstance(im, Image) for im in imgs)
    return imgs  # type: ignore[return-value]


@pytest.fixture
def store(tmp_path: object) -> SqliteStore:
    from pathlib import Path

    assert isinstance(tmp_path, Path)
    return SqliteStore(str(tmp_path / "test.db"))


@pytest.fixture
def session(store: SqliteStore) -> SqliteSession:
    return store.session()


class TestStreamBasics:
    def test_create_stream(self, session: SqliteSession) -> None:
        s = session.stream("images", Image)
        assert s is not None

    def test_append_and_fetch(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        obs = s.append(images[0])
        assert obs.id == 1
        assert obs.data == images[0]  # append returns original, not decoded
        assert obs.ts is not None

        rows = s.fetch()
        assert len(rows) == 1
        assert _img_close(rows[0].data, images[0])
        assert rows[0].id == 1

    def test_append_multiple(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        for img in images[:3]:
            s.append(img)

        assert s.count() == 3
        rows = s.fetch()
        assert all(_img_close(r.data, img) for r, img in zip(rows, images[:3], strict=True))

    def test_append_with_tags(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        s.append(images[0], tags={"cam": "front", "quality": "high"})

        rows = s.fetch()
        assert rows[0].tags == {"cam": "front", "quality": "high"}

    def test_last(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)
        s.append(images[2], ts=3.0)

        obs = s.last()
        assert _img_close(obs.data, images[2])
        assert obs.ts == 3.0

    def test_one(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        s.append(images[0])

        obs = s.one()
        assert _img_close(obs.data, images[0])

    def test_one_empty_raises(self, session: SqliteSession) -> None:
        s = session.stream("images", Image)
        with pytest.raises(LookupError):
            s.one()


class TestFilters:
    def test_after(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=10.0)

        rows = s.after(5.0).fetch()
        assert len(rows) == 1
        assert _img_close(rows[0].data, images[1])

    def test_before(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=10.0)

        rows = s.before(5.0).fetch()
        assert len(rows) == 1
        assert _img_close(rows[0].data, images[0])

    def test_time_range(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=5.0)
        s.append(images[2], ts=10.0)

        rows = s.time_range(3.0, 7.0).fetch()
        assert len(rows) == 1
        assert _img_close(rows[0].data, images[1])

    def test_at(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=5.0)
        s.append(images[2], ts=10.0)

        rows = s.at(5.5, tolerance=1.0).fetch()
        assert len(rows) == 1
        assert _img_close(rows[0].data, images[1])

    def test_filter_tags(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], tags={"cam": "front"})
        s.append(images[1], tags={"cam": "rear"})

        rows = s.filter_tags(cam="front").fetch()
        assert len(rows) == 1
        assert _img_close(rows[0].data, images[0])

    def test_chained_filters(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0, tags={"cam": "front"})
        s.append(images[1], ts=5.0, tags={"cam": "front"})
        s.append(images[2], ts=5.0, tags={"cam": "rear"})

        rows = s.after(3.0).filter_tags(cam="front").fetch()
        assert len(rows) == 1
        assert _img_close(rows[0].data, images[1])


class TestOrdering:
    def test_order_by_ts(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[1], ts=2.0)
        s.append(images[0], ts=1.0)
        s.append(images[2], ts=3.0)

        rows = s.order_by("ts").fetch()
        assert all(
            _img_close(r.data, img)
            for r, img in zip(rows, [images[0], images[1], images[2]], strict=True)
        )

    def test_order_by_desc(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)
        s.append(images[2], ts=3.0)

        rows = s.order_by("ts", desc=True).fetch()
        assert all(
            _img_close(r.data, img)
            for r, img in zip(rows, [images[2], images[1], images[0]], strict=True)
        )

    def test_limit_offset(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        for i, img in enumerate(images):
            s.append(img, ts=float(i))

        rows = s.order_by("ts").limit(2).offset(1).fetch()
        assert len(rows) == 2
        assert all(
            _img_close(r.data, img) for r, img in zip(rows, [images[1], images[2]], strict=True)
        )


class TestFetchPages:
    def test_basic_pagination(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        for i, img in enumerate(images):
            s.append(img, ts=float(i))

        pages = list(s.fetch_pages(batch_size=2))
        assert len(pages) == 3  # 2+2+1
        assert len(pages[0]) == 2
        assert len(pages[-1]) == 1

        all_items = [obs.data for page in pages for obs in page]
        assert all(_img_close(a, b) for a, b in zip(all_items, images, strict=True))


class TestTextStream:
    def test_create_and_append(self, session: SqliteSession) -> None:
        s = session.text_stream("logs")
        s.append("Motor fault on joint 3")
        s.append("Battery low warning")

        assert s.count() == 2

    def test_text_search(self, session: SqliteSession) -> None:
        s = session.text_stream("logs")
        s.append("Motor fault on joint 3")
        s.append("Battery low warning")
        s.append("Motor overheating on joint 5")

        rows = s.search_text("motor", k=10).fetch()
        assert len(rows) == 2
        assert all("Motor" in r.data for r in rows)


class TestTextStorage:
    """Test storing plain text (str) in streams."""

    def test_store_and_fetch_str(self, session: SqliteSession) -> None:
        s = session.stream("raw_logs", str)
        s.append("Robot started navigation to kitchen", ts=1.0)
        s.append("Obstacle detected at waypoint 3", ts=2.0)
        s.append("Navigation complete", ts=3.0)

        assert s.count() == 3
        rows = s.fetch()
        assert rows[0].data == "Robot started navigation to kitchen"
        assert rows[2].data == "Navigation complete"

    def test_str_with_tags_and_filters(self, session: SqliteSession) -> None:
        s = session.stream("tagged_logs", str)
        s.append("Motor fault on joint 3", ts=1.0, tags={"level": "error"})
        s.append("Battery at 80%", ts=2.0, tags={"level": "info"})
        s.append("Motor overheating", ts=3.0, tags={"level": "error"})

        errors = s.filter_tags(level="error").fetch()
        assert len(errors) == 2
        assert all("Motor" in e.data for e in errors)

    def test_str_persists_reopen(self, tmp_path: object) -> None:
        from pathlib import Path

        assert isinstance(tmp_path, Path)
        db_path = str(tmp_path / "logs.db")

        store1 = SqliteStore(db_path)
        s1 = store1.session()
        s1.stream("logs", str).append("hello world", ts=1.0)
        s1.stop()
        store1.stop()

        store2 = SqliteStore(db_path)
        s2 = store2.session()
        rows = s2.stream("logs", str).fetch()
        assert len(rows) == 1
        assert rows[0].data == "hello world"
        s2.stop()
        store2.stop()


class TestEmbeddingStream:
    def test_create_and_append(self, session: SqliteSession) -> None:
        es = session.embedding_stream("emb", vec_dimensions=4)
        e1 = Embedding(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32))
        e2 = Embedding(np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float32))

        es.append(e1, ts=1.0)
        es.append(e2, ts=2.0)

        assert es.count() == 2

    def test_search_embedding(self, session: SqliteSession) -> None:
        es = session.embedding_stream("emb_search", vec_dimensions=4)
        vecs = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.9, 0.1, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ]
        for i, v in enumerate(vecs):
            es.append(Embedding(np.array(v, dtype=np.float32)), ts=float(i))

        # Search for vector closest to [1, 0, 0, 0] — should get id=1 and id=3
        results = es.search_embedding([1.0, 0.0, 0.0, 0.0], k=2).fetch()
        assert len(results) == 2
        result_ids = {r.id for r in results}
        assert 1 in result_ids  # exact match
        assert 3 in result_ids  # [0.9, 0.1, 0, 0] is close

    def test_search_returns_embedding_observation(self, session: SqliteSession) -> None:
        es = session.embedding_stream("emb_obs", vec_dimensions=3)
        es.append(Embedding(np.array([1.0, 0.0, 0.0], dtype=np.float32)), ts=1.0)

        results = es.search_embedding([1.0, 0.0, 0.0], k=1).fetch()
        assert len(results) == 1
        assert isinstance(results[0], EmbeddingObservation)

    def test_search_with_time_filter(self, session: SqliteSession) -> None:
        es = session.embedding_stream("emb_time", vec_dimensions=3)
        es.append(Embedding(np.array([1.0, 0.0, 0.0], dtype=np.float32)), ts=1.0)
        es.append(Embedding(np.array([1.0, 0.0, 0.0], dtype=np.float32)), ts=10.0)

        # Both match the vector, but only one is after t=5
        results = es.search_embedding([1.0, 0.0, 0.0], k=10).after(5.0).fetch()
        assert len(results) == 1
        assert results[0].ts == 10.0

    def test_embedding_transformer_store(self, session: SqliteSession, images: list[Image]) -> None:
        """Test the full pipeline: images → EmbeddingTransformer → EmbeddingStream."""

        class FakeEmbedder(EmbeddingModel):
            device = "cpu"

            def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
                results = []
                for img in imgs:
                    val = float(img.data.mean()) / 255.0
                    results.append(
                        Embedding(np.array([val, 1.0 - val, 0.0, 0.0], dtype=np.float32))
                    )
                return results if len(results) > 1 else results[0]

            def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
                raise NotImplementedError

        s = session.stream("cam_emb", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)

        emb_stream = s.transform(EmbeddingTransformer(FakeEmbedder())).store("cam_embeddings")
        assert emb_stream.count() == 2

        # Search returns EmbeddingObservation; project_to to get source images
        results = emb_stream.search_embedding([0.5, 0.5, 0.0, 0.0], k=1).project_to(s).fetch()
        assert len(results) == 1
        assert _img_close(results[0].data, images[0]) or _img_close(results[0].data, images[1])


class TestListStreams:
    def test_list_empty(self, session: SqliteSession) -> None:
        assert session.list_streams() == []

    def test_list_after_create(self, session: SqliteSession) -> None:
        session.stream("images", Image)
        session.text_stream("logs")

        infos = session.list_streams()
        names = {i.name for i in infos}
        assert names == {"images", "logs"}


class TestReactive:
    def test_appended_observable(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        received: list[Observation] = []
        s.subscribe(received.append)

        s.append(images[0])
        s.append(images[1])

        assert len(received) == 2
        assert received[0].data is images[0]  # appended obs holds original
        assert received[1].data is images[1]


class TestTransformInMemory:
    def test_lambda_transform(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)

        shapes = s.transform(lambda im: f"{im.width}x{im.height}")
        results = shapes.fetch()
        assert len(results) == 2
        assert results[0].data == f"{images[0].width}x{images[0].height}"

    def test_lambda_filter_none(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)
        s.append(images[2], ts=3.0)

        # Only keep images wider than 0 (all pass), filter second by index trick
        idx = iter(range(3))
        big = s.transform(lambda im: im if next(idx) % 2 == 0 else None)
        results = big.fetch()
        assert len(results) == 2  # indices 0 and 2

    def test_lambda_expand_list(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)

        # Extract format and frame_id as two separate results
        results = s.transform(lambda im: [im.format.value, im.frame_id]).fetch()
        assert len(results) == 2
        assert results[0].data == images[0].format.value


class TestTransformStore:
    def test_transform_store_backfill(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)

        stored = s.transform(lambda im: f"{im.width}x{im.height}").store("shapes", str)
        rows = stored.fetch()
        assert len(rows) == 2
        expected = f"{images[0].width}x{images[0].height}"
        assert rows[0].data == expected

        reloaded = session.stream("shapes", str)
        assert reloaded.count() == 2

    def test_transform_store_live(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)

        stored = s.transform(lambda im: im.height, live=True).store("heights", int)
        assert stored.count() == 0  # no backfill

        s.append(images[1], ts=2.0)
        assert stored.count() == 1
        assert stored.last().data == images[1].height

    def test_transform_store_backfill_only(
        self, session: SqliteSession, images: list[Image]
    ) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)

        stored = s.transform(lambda im: im.height, backfill_only=True).store("heights_bo", int)
        assert stored.count() == 1
        assert stored.one().data == images[0].height

        s.append(images[1], ts=2.0)
        assert stored.count() == 1  # still 1


class TestLazyData:
    def test_data_lazy_loaded(self, session: SqliteSession, images: list[Image]) -> None:
        """Fetched observations should not eagerly load payload."""
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)

        rows = s.fetch()
        obs = rows[0]
        assert isinstance(obs._data, _Unset)
        assert obs._data_loader is not None
        loaded = obs.data
        assert _img_close(loaded, images[0])
        assert obs._data is loaded  # cached after first access

    def test_metadata_without_payload(self, session: SqliteSession, images: list[Image]) -> None:
        """Metadata (ts, tags) should be available without loading payload."""
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0, tags={"key": "val"})

        rows = s.fetch()
        obs = rows[0]
        assert obs.ts == 1.0
        assert obs.tags == {"key": "val"}
        assert obs.id == 1


class TestIteration:
    def test_iter(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        for i, img in enumerate(images[:3]):
            s.append(img, ts=float(i))

        items = [obs.data for obs in s]
        assert all(_img_close(a, b) for a, b in zip(items, images[:3], strict=True))


class TestProjectTo:
    def test_search_returns_embedding_obs(
        self, session: SqliteSession, images: list[Image]
    ) -> None:
        """search_embedding returns EmbeddingObservation; .data provides source data via lineage."""

        class FakeEmbedder(EmbeddingModel):
            device = "cpu"

            def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
                results = []
                for img in imgs:
                    val = float(img.data.mean()) / 255.0
                    results.append(Embedding(np.array([val, 1.0 - val, 0.0], dtype=np.float32)))
                return results if len(results) > 1 else results[0]

            def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
                raise NotImplementedError

        imgs = session.stream("pt_images", Image)
        imgs.append(images[0], ts=1.0)
        imgs.append(images[1], ts=2.0)
        imgs.append(images[2], ts=3.0)

        embs = imgs.transform(EmbeddingTransformer(FakeEmbedder())).store("pt_embs")
        assert embs.count() == 3

        # search_embedding returns EmbeddingObservation with Embedding data
        results = embs.search_embedding([0.5, 0.5, 0.0], k=2).fetch()
        assert len(results) == 2
        for obs in results:
            assert isinstance(obs, EmbeddingObservation)
            assert isinstance(obs.data, Embedding)

        # project_to to get source images
        projected = embs.search_embedding([0.5, 0.5, 0.0], k=2).project_to(imgs).fetch()
        assert len(projected) == 2
        for obs in projected:
            assert (
                _img_close(obs.data, images[0])
                or _img_close(obs.data, images[1])
                or _img_close(obs.data, images[2])
            )

    def test_search_chainable(self, session: SqliteSession, images: list[Image]) -> None:
        """Search results support further filter chaining."""

        class FakeEmbedder(EmbeddingModel):
            device = "cpu"

            def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
                results = []
                for img in imgs:
                    val = float(img.data.mean()) / 255.0
                    results.append(Embedding(np.array([val, 1.0 - val, 0.0], dtype=np.float32)))
                return results if len(results) > 1 else results[0]

            def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
                raise NotImplementedError

        imgs = session.stream("ptc_images", Image)
        imgs.append(images[0], ts=1.0)
        imgs.append(images[1], ts=5.0)
        imgs.append(images[2], ts=10.0)

        embs = imgs.transform(EmbeddingTransformer(FakeEmbedder())).store("ptc_embs")

        # Chain time filter after search
        results = embs.search_embedding([0.5, 0.5, 0.0], k=10).after(3.0).fetch()
        assert all(r.ts is not None and r.ts > 3.0 for r in results)

    def test_explicit_project_to(self, session: SqliteSession, images: list[Image]) -> None:
        """Explicit project_to works for non-search cases."""

        class FakeEmbedder(EmbeddingModel):
            device = "cpu"

            def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
                results = []
                for img in imgs:
                    val = float(img.data.mean()) / 255.0
                    results.append(Embedding(np.array([val, 1.0 - val, 0.0], dtype=np.float32)))
                return results if len(results) > 1 else results[0]

            def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
                raise NotImplementedError

        imgs = session.stream("pte_images", Image)
        imgs.append(images[0], ts=1.0)
        imgs.append(images[1], ts=5.0)
        imgs.append(images[2], ts=10.0)

        embs = imgs.transform(EmbeddingTransformer(FakeEmbedder())).store("pte_embs")

        # Explicit project_to without search — project all embeddings to images
        projected = embs.project_to(imgs).after(3.0)
        results = projected.fetch()
        assert all(r.ts is not None and r.ts > 3.0 for r in results)

    def test_two_hop(self, session: SqliteSession, images: list[Image]) -> None:
        """project_to handles multi-hop lineage (embs → mid → raw)."""

        class FakeEmbedder(EmbeddingModel):
            device = "cpu"

            def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
                results = []
                for img in imgs:
                    val = float(img.data.mean()) / 255.0
                    results.append(Embedding(np.array([val, 1.0 - val, 0.0], dtype=np.float32)))
                return results if len(results) > 1 else results[0]

            def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
                raise NotImplementedError

        raw = session.stream("th_raw", Image)
        raw.append(images[0], ts=1.0)
        raw.append(images[1], ts=2.0)
        raw.append(images[2], ts=3.0)

        mid = raw.transform(lambda img: img).store("th_mid", Image)
        assert mid.count() == 3

        embs = mid.transform(EmbeddingTransformer(FakeEmbedder())).store("th_embs")
        assert embs.count() == 3

        # project_to(raw) walks the full chain: th_embs → th_mid → th_raw
        projected = embs.search_embedding([0.5, 0.5, 0.0], k=2).project_to(raw)
        results = projected.fetch()
        assert len(results) == 2

    def test_count_on_projected(self, session: SqliteSession, images: list[Image]) -> None:
        """count() works on auto-projected search results."""

        class FakeEmbedder(EmbeddingModel):
            device = "cpu"

            def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
                results = []
                for img in imgs:
                    val = float(img.data.mean()) / 255.0
                    results.append(Embedding(np.array([val, 1.0 - val, 0.0], dtype=np.float32)))
                return results if len(results) > 1 else results[0]

            def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
                raise NotImplementedError

        imgs = session.stream("ptcnt_images", Image)
        imgs.append(images[0], ts=1.0)
        imgs.append(images[1], ts=2.0)

        embs = imgs.transform(EmbeddingTransformer(FakeEmbedder())).store("ptcnt_embs")
        assert embs.search_embedding([0.5, 0.5, 0.0], k=1).count() == 1

    def test_project_to_plain_transform(self, session: SqliteSession, images: list[Image]) -> None:
        """project_to on a non-embedding derived stream (e.g., detections → images)."""
        imgs = session.stream("ptplain_images", Image)
        imgs.append(images[0], ts=1.0)
        imgs.append(images[1], ts=2.0)
        imgs.append(images[2], ts=3.0)

        # Simulate a detection transform — extracts height as an "int" stream
        heights = imgs.transform(lambda im: im.height).store("ptplain_heights", int)
        assert heights.count() == 3

        # Project heights back to source images
        projected = heights.after(1.5).project_to(imgs)
        results = projected.fetch()
        assert len(results) == 2  # ts=2.0 and ts=3.0
        for obs in results:
            assert _img_close(obs.data, images[1]) or _img_close(obs.data, images[2])

    def test_search_by_text(self, session: SqliteSession, images: list[Image]) -> None:
        """search_embedding accepts a string and auto-embeds via model."""

        class FakeEmbedder(EmbeddingModel):
            device = "cpu"

            def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
                results = []
                for img in imgs:
                    val = float(img.data.mean()) / 255.0
                    results.append(Embedding(np.array([val, 1.0 - val, 0.0], dtype=np.float32)))
                return results if len(results) > 1 else results[0]

            def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
                results = []
                for _text in texts:
                    results.append(Embedding(np.array([0.5, 0.5, 0.0], dtype=np.float32)))
                return results if len(results) > 1 else results[0]

        imgs = session.stream("pttxt_images", Image)
        imgs.append(images[0], ts=1.0)
        imgs.append(images[1], ts=2.0)

        embs = imgs.transform(EmbeddingTransformer(FakeEmbedder())).store("pttxt_embs")

        # Search with text string — auto-embeds via embed_text()
        results = embs.search_embedding("a hallway", k=2).fetch()
        assert len(results) == 2

    def test_search_by_image(self, session: SqliteSession, images: list[Image]) -> None:
        """search_embedding accepts an image and auto-embeds via model."""

        class FakeEmbedder(EmbeddingModel):
            device = "cpu"

            def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
                results = []
                for img in imgs:
                    val = float(img.data.mean()) / 255.0
                    results.append(Embedding(np.array([val, 1.0 - val, 0.0], dtype=np.float32)))
                return results if len(results) > 1 else results[0]

            def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
                raise NotImplementedError

        imgs = session.stream("ptimg_images", Image)
        imgs.append(images[0], ts=1.0)
        imgs.append(images[1], ts=2.0)

        embs = imgs.transform(EmbeddingTransformer(FakeEmbedder())).store("ptimg_embs")

        # Search with image — auto-embeds via embed()
        results = embs.search_embedding(images[0], k=1).fetch()
        assert len(results) == 1

    def test_search_no_model_raises(self, session: SqliteSession) -> None:
        """search_embedding with str raises when no model is available."""
        es = session.embedding_stream("pt_nomodel", vec_dimensions=3)
        es.append(Embedding(np.array([1.0, 0.0, 0.0], dtype=np.float32)), ts=1.0)

        with pytest.raises(TypeError, match="no model reference"):
            es.search_embedding("hello", k=1)

    def test_no_lineage_fallback(self, session: SqliteSession) -> None:
        """search_embedding without lineage returns EmbeddingStream (no projection)."""
        es = session.embedding_stream("pt_standalone", vec_dimensions=3)
        es.append(Embedding(np.array([1.0, 0.0, 0.0], dtype=np.float32)), ts=1.0)

        results = es.search_embedding([1.0, 0.0, 0.0], k=1).fetch()
        assert len(results) == 1
        assert isinstance(results[0], EmbeddingObservation)


class TestSimilarityScores:
    def test_search_populates_similarity(self, session: SqliteSession) -> None:
        """search_embedding should populate .similarity on EmbeddingObservation."""
        es = session.embedding_stream("sim_test", vec_dimensions=4)
        vecs = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.9, 0.1, 0.0, 0.0],
        ]
        for i, v in enumerate(vecs):
            es.append(Embedding(np.array(v, dtype=np.float32)), ts=float(i))

        results = es.search_embedding([1.0, 0.0, 0.0, 0.0], k=3).fetch()
        assert len(results) == 3
        for obs in results:
            assert isinstance(obs, EmbeddingObservation)
            assert obs.similarity is not None
            assert 0.0 <= obs.similarity <= 1.0

        # Exact match should have highest similarity
        by_sim = sorted(results, key=lambda o: o.similarity, reverse=True)
        assert by_sim[0].id == 1  # [1,0,0,0] is exact match

    def test_similarity_none_without_search(self, session: SqliteSession) -> None:
        """Plain fetch() should leave similarity as None."""
        es = session.embedding_stream("sim_none", vec_dimensions=3)
        es.append(Embedding(np.array([1.0, 0.0, 0.0], dtype=np.float32)), ts=1.0)

        results = es.fetch()
        assert len(results) == 1
        assert isinstance(results[0], EmbeddingObservation)
        assert results[0].similarity is None

    def test_search_embedding_obs_with_similarity(
        self, session: SqliteSession, images: list[Image]
    ) -> None:
        """search_embedding returns EmbeddingObservation with similarity scores."""

        class FakeEmbedder(EmbeddingModel):
            device = "cpu"

            def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
                results = []
                for img in imgs:
                    val = float(img.data.mean()) / 255.0
                    results.append(Embedding(np.array([val, 1.0 - val, 0.0], dtype=np.float32)))
                return results if len(results) > 1 else results[0]

            def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
                raise NotImplementedError

        imgs = session.stream("sim_proj_imgs", Image)
        imgs.append(images[0], ts=1.0)
        imgs.append(images[1], ts=2.0)

        embs = imgs.transform(EmbeddingTransformer(FakeEmbedder())).store("sim_proj_embs")

        results = embs.search_embedding([0.5, 0.5, 0.0], k=2).fetch()
        assert len(results) == 2
        for obs in results:
            assert isinstance(obs, EmbeddingObservation)
            assert obs.similarity is not None
            assert isinstance(obs.data, Embedding)


class TestObservationSet:
    def test_fetch_returns_observation_set(
        self, session: SqliteSession, images: list[Image]
    ) -> None:
        from dimos.memory.stream import ObservationSet

        s = session.stream("obs_set", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)

        result = s.fetch()
        assert isinstance(result, ObservationSet)
        assert len(result) == 2

    def test_list_like_access(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("obs_list", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)
        s.append(images[2], ts=3.0)

        result = s.fetch()
        assert result[0].ts == 1.0
        assert result[-1].ts == 3.0
        assert len(result[1:]) == 2
        assert bool(result) is True

    def test_empty_observation_set(self, session: SqliteSession) -> None:
        s = session.stream("obs_empty", Image)
        result = s.fetch()
        assert len(result) == 0
        assert bool(result) is False

    def test_iter(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("obs_iter", Image)
        for i, img in enumerate(images[:3]):
            s.append(img, ts=float(i))

        result = s.fetch()
        timestamps = [obs.ts for obs in result]
        assert timestamps == [0.0, 1.0, 2.0]

    def test_refilter_in_memory(self, session: SqliteSession, images: list[Image]) -> None:
        """ObservationSet supports chaining filters that re-evaluate in memory."""
        s = session.stream("obs_refilter", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=5.0)
        s.append(images[2], ts=10.0)

        result = s.fetch()
        assert len(result) == 3

        # Re-filter in memory
        recent = result.after(3.0).fetch()
        assert len(recent) == 2
        assert all(r.ts is not None and r.ts > 3.0 for r in recent)

    def test_transform_on_observation_set(
        self, session: SqliteSession, images: list[Image]
    ) -> None:
        """ObservationSet supports .transform() for fork-and-zip."""
        s = session.stream("obs_xf", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)

        result = s.fetch()
        shapes = result.transform(lambda im: f"{im.width}x{im.height}").fetch()
        assert len(shapes) == 2
        assert shapes[0].data == f"{images[0].width}x{images[0].height}"

    def test_read_only(self, session: SqliteSession, images: list[Image]) -> None:
        from dimos.memory.stream import ObservationSet

        result = ObservationSet([], session=session)
        with pytest.raises(TypeError, match="read-only"):
            result.append(images[0])

    def test_ordering_in_memory(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("obs_order", Image)
        s.append(images[0], ts=3.0)
        s.append(images[1], ts=1.0)
        s.append(images[2], ts=2.0)

        result = s.fetch()
        ordered = result.order_by("ts").fetch()
        assert [o.ts for o in ordered] == [1.0, 2.0, 3.0]

        desc = result.order_by("ts", desc=True).fetch()
        assert [o.ts for o in desc] == [3.0, 2.0, 1.0]

    def test_limit_offset_in_memory(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("obs_lim", Image)
        for i, img in enumerate(images):
            s.append(img, ts=float(i))

        result = s.fetch()
        page = result.order_by("ts").limit(2).offset(1).fetch()
        assert len(page) == 2
        assert [o.ts for o in page] == [1.0, 2.0]


class TestMatchesFilters:
    def test_after_filter(self) -> None:
        from dimos.memory.types import AfterFilter

        f = AfterFilter(5.0)
        assert f.matches(Observation(id=1, ts=6.0)) is True
        assert f.matches(Observation(id=2, ts=5.0)) is False
        assert f.matches(Observation(id=3, ts=4.0)) is False
        assert f.matches(Observation(id=4, ts=None)) is False

    def test_before_filter(self) -> None:
        from dimos.memory.types import BeforeFilter

        f = BeforeFilter(5.0)
        assert f.matches(Observation(id=1, ts=4.0)) is True
        assert f.matches(Observation(id=2, ts=5.0)) is False
        assert f.matches(Observation(id=3, ts=6.0)) is False

    def test_time_range_filter(self) -> None:
        from dimos.memory.types import TimeRangeFilter

        f = TimeRangeFilter(2.0, 8.0)
        assert f.matches(Observation(id=1, ts=5.0)) is True
        assert f.matches(Observation(id=2, ts=2.0)) is True
        assert f.matches(Observation(id=3, ts=8.0)) is True
        assert f.matches(Observation(id=4, ts=1.0)) is False
        assert f.matches(Observation(id=5, ts=9.0)) is False

    def test_at_filter(self) -> None:
        from dimos.memory.types import AtFilter

        f = AtFilter(5.0, tolerance=1.0)
        assert f.matches(Observation(id=1, ts=5.0)) is True
        assert f.matches(Observation(id=2, ts=5.5)) is True
        assert f.matches(Observation(id=3, ts=6.0)) is True
        assert f.matches(Observation(id=4, ts=6.5)) is False

    def test_tags_filter(self) -> None:
        from dimos.memory.types import TagsFilter

        f = TagsFilter((("cam", "front"),))
        assert f.matches(Observation(id=1, tags={"cam": "front", "quality": "high"})) is True
        assert f.matches(Observation(id=2, tags={"cam": "rear"})) is False
        assert f.matches(Observation(id=3, tags={})) is False

    def test_text_search_filter(self) -> None:
        from dimos.memory.types import TextSearchFilter

        f = TextSearchFilter("motor", k=None)
        assert f.matches(Observation(id=1, _data="Motor fault on joint 3")) is True
        assert f.matches(Observation(id=2, _data="Battery low")) is False

    def test_embedding_search_filter_always_true(self) -> None:
        from dimos.memory.types import EmbeddingSearchFilter

        f = EmbeddingSearchFilter([1.0, 0.0], k=5)
        assert f.matches(Observation(id=1)) is True

    def test_lineage_filter_raises(self) -> None:
        from dimos.memory.types import LineageFilter, StreamQuery

        f = LineageFilter("src", StreamQuery(), ())
        with pytest.raises(NotImplementedError):
            f.matches(Observation(id=1))


class TestFilteredAppended:
    def test_unfiltered_appended(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("fa_unfilt", Image)
        received: list[Observation] = []
        s.subscribe(received.append)

        s.append(images[0], ts=1.0)
        s.append(images[1], ts=5.0)
        assert len(received) == 2

    def test_filtered_appended(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("fa_filt", Image)
        received: list[Observation] = []
        s.after(3.0).subscribe(received.append)

        s.append(images[0], ts=1.0)  # filtered out
        s.append(images[1], ts=5.0)  # passes
        assert len(received) == 1
        assert received[0].ts == 5.0

    def test_tag_filtered_appended(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("fa_tag", Image)
        received: list[Observation] = []
        s.filter_tags(cam="front").subscribe(received.append)

        s.append(images[0], tags={"cam": "front"})
        s.append(images[1], tags={"cam": "rear"})
        assert len(received) == 1


class TestStoreReopen:
    def test_data_persists(self, tmp_path: object, images: list[Image]) -> None:
        from pathlib import Path

        assert isinstance(tmp_path, Path)
        db_path = str(tmp_path / "persist.db")

        store1 = SqliteStore(db_path)
        s1 = store1.session()
        s1.stream("data", Image).append(images[0], ts=1.0)
        s1.stop()
        store1.stop()

        store2 = SqliteStore(db_path)
        s2 = store2.session()
        rows = s2.stream("data", Image).fetch()
        assert len(rows) == 1
        assert _img_close(rows[0].data, images[0])
        s2.stop()
        store2.stop()

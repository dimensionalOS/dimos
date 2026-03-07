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

"""E2E tests: ingest 5min robot video → sharpness filter → CLIP embed → search.

The DB is built once and cached on disk so subsequent runs skip ingestion.
Run with:  pytest dimos/memory/impl/run_e2e_export.py -s
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

import pytest

from dimos.memory.impl.sqlite import SqliteStore
from dimos.memory.ingest import ingest
from dimos.memory.transformer import (
    CaptionTransformer,
    EmbeddingTransformer,
    QualityWindowTransformer,
)
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.testing import TimedSensorReplay

if TYPE_CHECKING:
    from collections.abc import Generator

    from dimos.memory.stream import EmbeddingStream

DB_DIR = Path(__file__).parent / "e2e_matches"
DB_DIR.mkdir(exist_ok=True)
DB_PATH = DB_DIR / "e2e.db"


@pytest.fixture(scope="module")
def clip() -> CLIPModel:
    model = CLIPModel()
    model.start()
    return model


@pytest.fixture(scope="module")
def e2e_db(clip: CLIPModel) -> Generator[tuple[SqliteStore, Any], None, None]:
    """Build (or reuse cached) e2e DB with video → sharpness → CLIP embeddings."""
    store = SqliteStore(str(DB_PATH))
    session = store.session()

    existing = {s.name for s in session.list_streams()}
    if "clip_embeddings" not in existing:
        replay = TimedSensorReplay("unitree_go2_bigoffice/video")
        odom = TimedSensorReplay("unitree_go2_bigoffice/odom")

        raw = session.stream("raw_video", Image)
        n = ingest(raw, replay.iterate_ts(seek=5.0, duration=300.0), pose_source=odom)
        print(f"  {n} frames ingested")

        sharp = raw.transform(
            QualityWindowTransformer(lambda img: img.sharpness, window=0.5)
        ).store("sharp_frames", Image)
        print(f"  {sharp.count()} sharp frames (from {n}, {sharp.count() / n:.0%} kept)")

        embeddings: EmbeddingStream[Any] = sharp.transform(EmbeddingTransformer(clip)).store(
            "clip_embeddings"
        )  # type: ignore[assignment]
        print(f"  {embeddings.count()} embeddings stored")
    else:
        print(f"Using cached DB ({DB_PATH})")

    yield store, session  # type: ignore[misc]
    session.stop()
    store.stop()


@pytest.fixture(scope="module")
def embeddings(e2e_db: tuple[SqliteStore, Any], clip: CLIPModel) -> EmbeddingStream[Any]:
    _, session = e2e_db
    stream: EmbeddingStream[Any] = session.embedding_stream("clip_embeddings", embedding_model=clip)  # type: ignore[return-value]
    return stream


@pytest.fixture(scope="module")
def sharp_frames(e2e_db: tuple[SqliteStore, Any]) -> Any:
    _, session = e2e_db
    return session.stream("sharp_frames", Image)


class TestEmbeddingSearch:
    """Search the cached CLIP embedding DB and export top matches."""

    QUERIES = [
        "a hallway in an office",
        "a person standing",
        "a door",
        "a desk",
        "supermarket",
        "large room",
    ]

    @pytest.mark.parametrize("query", QUERIES)
    def test_search_returns_results(self, embeddings: EmbeddingStream[Any], query: str) -> None:
        from dimos.memory.types import EmbeddingObservation

        results = embeddings.search_embedding(query, k=5).fetch()
        assert len(results) > 0
        for obs in results:
            assert obs.ts is not None
            assert isinstance(obs, EmbeddingObservation)

    @pytest.mark.parametrize("query", QUERIES)
    def test_search_exports_images(
        self, embeddings: EmbeddingStream[Any], sharp_frames: Any, query: str
    ) -> None:
        slug = query.replace(" ", "_")[:30]
        results = embeddings.search_embedding(query, k=5).project_to(sharp_frames).fetch()

        for rank, img in enumerate(results):
            fname = DB_DIR / f"{slug}_{rank + 1}_id{img.id}_ts{img.ts:.0f}.jpg"
            img.data.save(str(fname))
            print(f"  [{rank + 1}] id={img.id} ts={img.ts:.2f}")

    def test_search_has_similarity(self, embeddings: EmbeddingStream[Any]) -> None:
        from dimos.memory.types import EmbeddingObservation

        results = embeddings.search_embedding("a hallway", k=10).fetch()
        assert len(results) > 0
        for obs in results:
            assert isinstance(obs, EmbeddingObservation)
            assert obs.similarity is not None
            assert 0.0 <= obs.similarity <= 1.0

    def test_caption_search_results(
        self, embeddings: EmbeddingStream[Any], sharp_frames: Any
    ) -> None:
        from dimos.models.vl.florence import Florence2Model

        captioner = Florence2Model()
        captioner.start()
        caption_xf = CaptionTransformer(captioner)

        results = embeddings.search_embedding("a door", k=3).project_to(sharp_frames).fetch()
        captions = results.transform(caption_xf).fetch()

        assert len(captions) == len(results)
        for cap in captions:
            assert isinstance(cap.data, str)
            assert len(cap.data) > 0
            print(f"  Caption: {cap.data}")


class TestRerunStream:
    """Send a full image stream to Rerun."""

    def test_stream_to_rerun(self, e2e_db: tuple[SqliteStore, Any]) -> None:
        import rerun as rr

        from dimos.memory.rerun import to_rerun

        rr.init("memory_e2e_test", spawn=True)

        _, session = e2e_db
        n = to_rerun(session.stream("sharp_frames", Image))
        assert n > 0
        print(f"  Logged {n} images to Rerun")

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

"""E2E test: ingest robot video → sharpness filter → CLIP embed → vector search."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

if TYPE_CHECKING:
    from pathlib import Path

from dimos.memory.impl.sqlite import SqliteStore
from dimos.memory.ingest import ingest
from dimos.memory.transformer import EmbeddingTransformer, QualityWindowTransformer
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.testing import TimedSensorReplay


@pytest.fixture(scope="module")
def replay() -> TimedSensorReplay:  # type: ignore[type-arg]
    return TimedSensorReplay("unitree_go2_bigoffice/video")


@pytest.fixture(scope="module")
def clip() -> CLIPModel:
    model = CLIPModel()
    model.start()
    return model


@pytest.mark.slow
@pytest.mark.skipif_in_ci
class TestE2EPipeline:
    """Ingest 60s of robot video, filter by sharpness, embed with CLIP, search."""

    def test_ingest_filter_embed_search(
        self,
        tmp_path: Path,
        replay: TimedSensorReplay,  # type: ignore[type-arg]
        clip: CLIPModel,
    ) -> None:
        store = SqliteStore(str(tmp_path / "e2e.db"))
        session = store.session()

        # 1. Ingest 60s of video
        raw = session.stream("raw_video", Image)
        n_ingested = ingest(raw, replay.iterate_ts(seek=5.0, duration=60.0))
        assert n_ingested > 0
        print(f"\nIngested {n_ingested} frames")

        # 2. Sharpness filter: keep best frame per 0.5s window
        sharp = raw.transform(
            QualityWindowTransformer(lambda img: img.sharpness, window=0.5)
        ).store("sharp_frames", Image)
        n_sharp = sharp.count()
        assert n_sharp > 0
        assert n_sharp < n_ingested  # should reduce count
        print(f"Sharp frames: {n_sharp} (from {n_ingested}, {n_sharp / n_ingested:.0%} kept)")

        # 3. Embed with real CLIP model
        embeddings = sharp.transform(EmbeddingTransformer(clip)).store("clip_embeddings")
        n_emb = embeddings.count()
        assert n_emb == n_sharp
        print(f"Embeddings stored: {n_emb}")

        # 4. Text-to-image search
        query_emb = clip.embed_text("a hallway in an office")
        results = embeddings.search_embedding(query_emb, k=5).fetch()
        assert len(results) > 0
        assert len(results) <= 5
        print(f"Search returned {len(results)} results")

        for r in results:
            assert r.ts is not None
            assert r.data is not None
            print(f"  id={r.id} ts={r.ts:.2f}")

        # 5. Search with time filter
        mid_ts = (results[0].ts + results[-1].ts) / 2 if len(results) > 1 else results[0].ts
        filtered = embeddings.search_embedding(query_emb, k=10).after(mid_ts).fetch()
        assert all(r.ts > mid_ts for r in filtered)
        print(f"Time-filtered search: {len(filtered)} results after ts={mid_ts:.2f}")

        # 6. Verify persistence — reopen and search again
        session.stop()
        store.stop()

        store2 = SqliteStore(str(tmp_path / "e2e.db"))
        session2 = store2.session()
        reloaded = session2.embedding_stream("clip_embeddings", vec_dimensions=512)
        assert reloaded.count() == n_emb

        results2 = reloaded.search_embedding(query_emb, k=3).fetch()
        assert len(results2) > 0
        print(f"After reopen: {len(results2)} results")

        session2.stop()
        store2.stop()

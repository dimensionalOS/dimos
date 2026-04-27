# memory Embedding Design

## Core Principle: Enrichment, Not Replacement

The embedding annotates the observation — it doesn't replace `.data`.
In memory1, `.data` IS the embedding and you need `parent_id` + `project_to()` to get back to the source image. We avoid this entirely.

## Observation Types

```python
@dataclass
class Observation(Generic[T]):
    id: int
    ts: float
    pose: Any | None = None
    tags: dict[str, Any] = field(default_factory=dict)
    _data: T | _Unloaded = ...
    _loader: Callable[[], T] | None = None  # lazy loading via blob store

@dataclass
class EmbeddedObservation(Observation[T]):
    embedding: Embedding | None = None       # populated by Embed transformer
    similarity: float | None = None          # populated by .search()
```

`EmbeddedObservation` is a subclass — passes anywhere `Observation` is accepted (LSP).
Users who don't care about types just use `Observation`. Users who want precision annotate with `EmbeddedObservation`.

`derive()` on `Observation` promotes to `EmbeddedObservation` if `embedding=` is passed.
`derive()` on `EmbeddedObservation` returns `EmbeddedObservation`, preserving the embedding unless explicitly replaced.

## Embed Transformer

`Embed` is `Transformer[T, T]` — same data type in and out. It populates `.embedding` on each observation:

```python
class Embed(Transformer[T, T]):
    def __init__(self, model: EmbeddingModel):
        self.model = model

    def __call__(self, upstream):
        for batch in batched(upstream, 32):
            vecs = self.model.embed_batch([obs.data for obs in batch])
            for obs, vec in zip(batch, vecs):
                yield obs.derive(data=obs.data, embedding=vec)
```

`Stream[Image]` stays `Stream[Image]` after embedding — `T` is about `.data`, not the observation subclass.

## Search

`.search(query_vec, k)` lives on `Stream` itself. Returns a new Stream filtered to top-k by cosine similarity:

```python
query_vec = clip.embed_text("a cat in the kitchen")

results = images.transform(Embed(clip)).search(query_vec, k=20).to_list()
# results[0].data       → Image
# results[0].embedding  → np.ndarray
# results[0].similarity → 0.93

# Chainable with other filters
results = images.transform(Embed(clip)) \
    .search(query_vec, k=50) \
    .after(one_hour_ago) \
    .near(kitchen_pose, 5.0) \
    .to_list()
```

## Backend Handles Storage Strategy

The Backend composite decides how to route storage based on what it sees:

- `append(image, ts=now, embedding=vec)` → backend routes: blob via BlobStore, vector via VectorStore, metadata via ObservationStore
- `append(image, ts=now)` → blob + metadata only (no embedding)
- `ListObservationStore`: stores metadata in-memory, brute-force cosine via MemoryVectorStore
- `SqliteObservationStore`: metadata in SQLite, vec0 side table for fast ANN search via SqliteVectorStore
- Future backends (Postgres/pgvector, Qdrant, etc.) do their thing

Search is pushed down to the VectorStore. Stream just passes `.search()` calls through.

## Projection / Lineage

**Usually not needed.** Since `.data` IS the original data, search results give you the image directly.

When a downstream transform replaces `.data` (e.g., Image → Detection), use temporal join to get back to the source:

```python
detection = detections.first()
detection.data              # → Detection
detection.ts                # → timestamp preserved by derive()

# Get the source image via temporal join
source_image = images.at(detection.ts).first()
```

## Multi-Modal

**Same embedding space = same stream.** CLIP maps images and text to the same 512-d space:

```python
unified = store.stream("clip_unified")

for obs in images.transform(Embed(clip.vision)):
    unified.append(obs.data, ts=obs.ts,
                   tags={"modality": "image"}, embedding=obs.embedding)

for obs in logs.transform(Embed(clip.text)):
    unified.append(obs.data, ts=obs.ts,
                   tags={"modality": "text"}, embedding=obs.embedding)

results = unified.search(query_vec, k=20).to_list()
# results[i].tags["modality"] tells you what it is
```

**Different embedding spaces = different streams.** Can't mix CLIP and sentence-transformer vectors.

## Chaining — Embedding as Cheap Pre-Filter

```python
smoke_query = clip.embed_text("smoke or fire")

detections = images.transform(Embed(clip)) \
    .search(smoke_query, k=100) \
    .transform(ExpensiveVLMDetector())
# VLM only runs on 100 most promising frames

# Smart transformer can use embedding directly
class SmartDetector(Transformer[Image, Detection]):
    def __call__(self, upstream: Iterator[EmbeddedObservation[Image]]) -> ...:
        for obs in upstream:
            if obs.embedding @ self.query > 0.3:
                yield obs.derive(data=self.detect(obs.data))
```

## Text Search (FTS) — Separate Concern

FTS is keyword-based, not embedding-based. Complementary, not competing:

```python
# Keyword search via FTS5
logs = store.stream("logs")
logs.search_text("motor fault").to_list()

# Semantic search via embeddings
log_idx = logs.transform(Embed(sentence_model)).store("log_emb")
log_idx.search(model.embed("motor problems"), k=10).to_list()
```

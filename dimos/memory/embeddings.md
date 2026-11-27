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
    embedding: Embedding | None = None       # populated by EmbedImages / EmbedText
    similarity: float | None = None          # populated by .search()
```

`EmbeddedObservation` is a subclass — passes anywhere `Observation` is accepted (LSP).
Users who don't care about types just use `Observation`. Users who want precision annotate with `EmbeddedObservation`.

`derive()` on `Observation` promotes to `EmbeddedObservation` if `embedding=` is passed.
`derive()` on `EmbeddedObservation` returns `EmbeddedObservation`, preserving the embedding unless explicitly replaced.

## EmbedImages and EmbedText Transformers

`EmbedImages` and `EmbedText` (`dimos.memory.embed`) are `Transformer[Any, Any]` — same payload type in and out, enriching each observation with `.embedding`:

```python
class EmbedImages(Transformer[Any, Any]):
    def __init__(self, model: EmbeddingModel, batch_size: int = 32) -> None:
        self.model = model
        self.batch_size = batch_size

    def __call__(self, upstream):
        for batch in _batched(upstream, self.batch_size):
            embeddings = self.model.embed(*[obs.data for obs in batch])
            if not isinstance(embeddings, list):
                embeddings = [embeddings]
            for obs, emb in zip(batch, embeddings, strict=False):
                yield obs.derive(data=obs.data, embedding=emb)
```

`EmbedText(model, batch_size=32)` does the same for text payloads via `model.embed_text(*texts)`. `Stream[Image]` stays `Stream[Image]` after embedding — `T` is about `.data`, not the observation subclass.

## Search

`.search(query_vec, k)` lives on `Stream` itself. Returns a new Stream filtered to top-k by cosine similarity:

```python
query_vec = clip.embed_text("a cat in the kitchen")

results = images.transform(EmbedImages(clip)).search(query_vec, k=20).to_list()
# results[0].data       → Image
# results[0].embedding  → Embedding
# results[0].similarity → 0.93

# Chainable with other filters
results = images.transform(EmbedImages(clip)) \
    .search(query_vec, k=50) \
    .after(one_hour_ago) \
    .near(kitchen_pose, 5.0) \
    .to_list()
```

## Backend Handles Storage Strategy

The Backend composite decides how to route storage based on what it sees:

- `append(image, ts=now, embedding=vec)` → backend routes: blob via BlobStore, vector via VectorStore, metadata via ObservationStore
- `append(image, ts=now)` → blob + metadata only (no embedding)
- `MemoryStore` (`ListObservationStore`): keeps `EmbeddedObservation` in memory and scores `.search()` with brute-force cosine (or delegates to `MemoryVectorStore` when configured)
- `SqliteStore` (`SqliteObservationStore` + `SqliteVectorStore`): stores metadata in SQLite and vectors in a `vec0` virtual table for `.search()` (plain iteration without `.search()` loads metadata and blobs only, without `.embedding`)
- `McapStore` (`McapObservationStore`): read-only with no vector store (`vector_store=None`); compute embeddings in an upstream `EmbedImages` / `EmbedText` transform to `.search()` in memory
- `NullStore`: `MemoryStore(max_size=0)`, live-only with no retained history
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
unified = store.stream("clip_unified", object)

for obs in images.transform(EmbedImages(clip)):
    unified.append(obs.data, ts=obs.ts,
                   tags={"modality": "image"}, embedding=obs.embedding)

for obs in logs.transform(EmbedText(clip)):
    unified.append(obs.data, ts=obs.ts,
                   tags={"modality": "text"}, embedding=obs.embedding)

results = unified.search(query_vec, k=20).to_list()
# results[i].tags["modality"] tells you what it is
```

**Different embedding spaces = different streams.** Can't mix CLIP and sentence-transformer vectors.

## Chaining — Embedding as Cheap Pre-Filter

```python
smoke_query = clip.embed_text("smoke or fire")

detections = images.transform(EmbedImages(clip)) \
    .search(smoke_query, k=100) \
    .transform(ExpensiveVLMDetector())
# VLM only runs on 100 most promising frames

# Smart transformer can use embedding directly
class SmartDetector(Transformer[Image, Detection]):
    def __call__(self, upstream: Iterator[EmbeddedObservation[Image]]) -> Iterator[Observation[Detection]]:
        for obs in upstream:
            if obs.embedding @ self.query > 0.3:
                yield obs.derive(data=self.detect(obs.data))
```

## Text Search — Separate Concern

`.search_text()` is case-insensitive substring matching (`MemoryStore`, `McapStore`, and transform streams; `SqliteStore` raises `NotImplementedError` on stored streams — there is no FTS5 index). Semantic search uses embeddings (`MemoryStore` and `SqliteStore`):

```python
# Substring search (MemoryStore, McapStore, or after a transform; not SqliteStore)
logs = memory_store.stream("logs", str)
logs.search_text("motor fault").to_list()

# Semantic search via embeddings (MemoryStore or SqliteStore)
log_idx = store.stream("log_emb", str)
logs.transform(EmbedText(clip)).save(log_idx).drain()
log_idx.search(clip.embed_text("motor problems"), k=10).to_list()
```

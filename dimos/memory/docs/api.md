# Memory2 API — Unified Stream

## Core Idea

One type: `Stream[T]`. Everything is a stream — stored, filtered, transformed. The user never thinks about Query vs ObservationSet vs Stream. They just chain operations.

## Creating Streams

```python
store = SqliteStore("/data/robot.db")
session = store.session()

# Root stored stream — backed by DB
images = session.stream("images", Image,
                        pose_provider=lambda: tf.get_pose("world", "base_link"))

logs = session.text_stream("logs", str,
                           pose_provider=lambda: tf.get_pose("world", "base_link"))
```

## Writing

```python
images.append(frame)                    # ts + pose auto-filled
logs.append("Motor fault on joint 3")   # ts + pose auto-filled
images.append(frame, pose=explicit_pose, tags={"cam": "front"})
```

Only meaningful on stored (DB-backed) streams.

### Batch ingest

The `ingest()` helper accepts any iterable of `(ts, payload)` — e.g. from a replay:

```python
from dimos.memory.ingest import ingest

replay = TimedSensorReplay("unitree_go2_bigoffice/video")
odom = TimedSensorReplay("unitree_go2_bigoffice/odom")

raw = session.stream("raw_video", Image)
n = ingest(raw, replay.iterate_ts(seek=5.0, duration=300.0), pose_source=odom)
# pose_source.find_closest(ts) is called per frame to attach odom poses
```

## Filtering

Every filter returns a new `Stream[T]`. Lazy — nothing executes until a terminal.

```python
recent = images.after(one_hour_ago)
kitchen = recent.near(kitchen_pose, 5.0)
tagged = kitchen.filter_tags(cam="front")

# Or chained
images.after(one_hour_ago).near(kitchen_pose, 5.0).filter_tags(cam="front")
```

### Filter methods

```python
class Stream(Generic[T]):
    # Temporal
    def after(self, t: float) -> Stream[T]: ...
    def before(self, t: float) -> Stream[T]: ...
    def time_range(self, t1: float, t2: float) -> Stream[T]: ...
    def at(self, t: float, *, tolerance: float = 1.0) -> Stream[T]: ...

    # Spatial
    def near(self, pose: PoseLike, radius: float) -> Stream[T]: ...

    # Tags
    def filter_tags(self, **tags: Any) -> Stream[T]: ...

class EmbeddingStream(Stream[T]):
    def search_embedding(self, query: Embedding | list[float] | str | Any,
                         *, k: int, raw: bool = False) -> Stream[Any]: ...

class TextStream(Stream[T]):
    def search_text(self, text: str, *, k: int | None = None) -> TextStream[T]: ...
```

## Terminals & Iteration

`Stream` is directly iterable — pages internally, never loads everything at once.

```python
# Direct iteration (lazy, memory-efficient — uses fetch_pages internally)
for row in images.after(t).near(kitchen_pose, 5.0):
    print(row.data)

# Explicit fetch when you want the full list in memory
all_rows = images.after(t).fetch()  # returns ObservationSet

# Other terminals
row = images.after(t).one()                # single best match
row = images.last()                        # most recent
n = images.after(t).count()                # count without fetching

# Pagination
page = images.order_by("ts").limit(50).offset(100).fetch()
```

### Terminal methods

```python
class Stream(Generic[T]):
    def __iter__(self) -> Iterator[Observation]: ...  # lazy, pages internally
    def fetch(self) -> ObservationSet[T]: ...          # all results, list-like + stream-like
    def fetch_pages(self, batch_size: int = 128) -> Iterator[list[Observation]]: ...
    def one(self) -> Observation: ...
    def last(self) -> Observation: ...
    def count(self) -> int: ...
    def order_by(self, field: str, *, desc: bool = False) -> Stream[T]: ...
    def limit(self, k: int) -> Stream[T]: ...
    def offset(self, n: int) -> Stream[T]: ...
```

### ObservationSet

`fetch()` returns an `ObservationSet` — a list-like object that also supports stream chaining:

```python
results = embeddings.search_embedding("a hallway", k=50).fetch()

len(results)           # list-like
results[0]             # indexing
for r in results:      # iteration
    print(r.data)

# Stream-like — further filter/transform the materialized results
results.after(t).fetch()
results.transform(caption_xf).fetch()
```

## Observation

```python
@dataclass
class Observation:
    id: int
    ts: float | None = None
    pose: PoseStamped | None = None
    tags: dict[str, Any] = field(default_factory=dict)
    parent_id: int | None = None    # lineage: source observation id

    @property
    def data(self) -> Any:
        """Lazy payload. Pre-populated from append/transform, fetched on demand from query."""
        ...

@dataclass
class EmbeddingObservation(Observation):
    """Returned by EmbeddingStream terminals. Auto-projects .data to source stream."""

    similarity: float | None = None  # 0..1, populated by search_embedding (vec0 cosine)

    @property
    def data(self) -> Any:
        """Lazily loads from the source stream (e.g., Image), not the embedding."""
        ...

    @property
    def embedding(self) -> Embedding:
        """The Embedding object (has .vector, supports @ for cosine similarity)."""
        ...
```

## Transformer

A `Transformer` receives the full source stream and decides what to do — which items to process, how to batch, whether to use embeddings as a cheap proxy, etc.

```python
class Transformer(ABC, Generic[T, R]):
    """Transforms a source stream into results on a target stream."""

    def process(self, source: Stream[T], target: Stream[R]) -> None:
        """Batch/historical processing. Has full access to source — can query,
        filter, use embeddings, batch, skip frames, etc."""
        ...

    def on_append(self, obs: Observation, target: Stream[R]) -> None:
        """Reactive processing. Called per new item. Default: process([obs])."""
        ...

    supports_backfill: bool = True
    supports_live: bool = True
    output_type: type | None = None  # determines target stream kind
```

### Simple lambdas (sugar)

`Callable[[T], R | list[R] | None]` is auto-wrapped into a naive per-item Transformer:

```python
# These are equivalent:
images.transform(lambda img: vlm.detect(img, "cigarettes"))
images.transform(PerItemTransformer(lambda img: vlm.detect(img, "cigarettes")))
```

- `R` → single result
- `list[R]` → multiple results (e.g., multiple detections per frame)
- `None` → skip (no result for this input)

### EmbeddingTransformer

`EmbeddingTransformer` wraps an `EmbeddingModel` as a `Transformer[T, Embedding]`. When the output type is `Embedding`, `.store()` creates an `EmbeddingStream` (vec0 index, `search_embedding`, `EmbeddingObservation`).

```python
# EmbeddingTransformer wraps the model
img_emb = images.transform(EmbeddingTransformer(CLIPModel())).store("img_emb")

# Now img_emb is an EmbeddingStream
results = img_emb.search_embedding(query_emb, k=20).fetch()
# results[0].data → Image (auto-projected from source)
# results[0].embedding → Embedding (supports @ for cosine similarity)
```

### Chaining transforms

```python
# Filter → transform → store
images.after(one_hour_ago) \
    .near(kitchen_pose, 5.0) \
    .transform(EmbeddingTransformer(CLIPModel())) \
    .store("kitchen_embeddings")

# Filter → transform → fetch (in-memory, not persisted)
results = images.after(one_hour_ago) \
    .near(kitchen_pose, 5.0) \
    .transform(EmbeddingTransformer(CLIPModel())) \
    .fetch()

# Filter → embed → detect → store (chained: detector gets EmbeddingObservation)
images.near(kitchen_pose, 5.0) \
    .transform(EmbeddingTransformer(CLIPModel())) \
    .transform(CigaretteDetector(vlm, clip)) \
    .store("kitchen_cigarette_detections")
```

### Backfill / Live modes

```python
# Both (default): backfill existing + subscribe to new
images.transform(detector).store("detections")

# Live only: skip backfill, only process new items
images.transform(detector, live=True).store("detections")

# Backfill only: process existing, don't subscribe
images.transform(detector, backfill=True).store("detections")

# Backfill only: process existing, and subscribe
images.transform(detector, backfill=True, live=True).store("detections")

# Incremental: re-running a stored transform resumes from last processed item
# (uses lineage parent_id to skip already-processed source rows)
```

## Storing

`.store(name)` materializes a stream to DB. After storing, results are queryable and persistent.

```python
# In-memory transform result — not persisted
detections = images.transform(detect_fn)

# Persist it
detections.store("detections")

# Now it's a DB-backed stream, queryable
stored = session.stream("detections")
rows = stored.after(t).fetch()
```

`.store()` also sets up lineage — every stored row gets `parent_id` pointing back to its source.

Stream type is determined by what the Transformer produces:
- `Embedding` output → `EmbeddingStream` (vec0 index)
- `str` output from `CaptionTransformer` → `TextStream` (FTS index)
- Everything else → `Stream` (blob)

## Reactive

```python
# .appended emits Observation with .data pre-populated
images.appended.subscribe(lambda row: print(f"New image at {row.pose}"))

# Stored transforms propagate reactively by default
detections = images.transform(detect_fn).store("detections")
# Now every images.append(frame) → detect_fn runs → result stored in "detections"

# Filtered appended — only kitchen images
images.near(kitchen_pose, 5.0).appended.subscribe(...)
```

## Cross-stream lineage (project_to)

`project_to()` follows `parent_id` chains to project observations onto another stream:

```python
# Get embeddings matching a query, then project to source images
emb_results = img_emb.search_embedding("red shoes", k=20, raw=True).fetch()
# emb_results are EmbeddingObservations with .similarity, .pose, .ts

# Or project to get the source images directly
image_results = img_emb.search_embedding("red shoes", k=20, raw=True) \
    .project_to(images).fetch()
```

`search_embedding` auto-projects by default — `raw=True` skips this to get
`EmbeddingObservation` results with `.similarity` scores.

Multi-hop lineage works too:
```python
# images → sharp_frames → clip_embeddings (2 hops)
# search_embedding auto-resolves the chain
results = clip_embeddings.search_embedding("a door", k=10).fetch()
# results[0].data → Image (from raw_video, traversing through sharp_frames)
```

## Visualization

`dimos.memory.rerun` sends stream contents to Rerun:

```python
from dimos.memory.rerun import to_rerun

# Send any stream to Rerun — auto-derives entity path from stream name,
# logs .data via to_rerun() and poses as arrows
to_rerun(images)
to_rerun(embeddings.search_embedding("a hallway", k=50))
```

## Full Example: Cigarette Detection Pipeline

```python
session = SqliteStore("/data/robot.db").session()

# Root stream
images = session.stream("images", Image,
                        pose_provider=lambda: tf.get_pose("world", "base_link"))

# Embedding index — EmbeddingModel is a Transformer
img_emb = images.transform(EmbeddingTransformer(CLIPModel())).store("img_emb")

# VLM detection pipeline (live-only, no backfill)
images.transform(
    lambda img: vlm.detect(img, "people with cigarettes"),
    live=True,
).store("cigarette_detections")

# Smart detection — reuse existing embeddings, detector gets EmbeddingObservation
img_emb.near(kitchen_pose, 10.0) \
    .transform(CigaretteDetector(vlm, clip)) \
    .store("kitchen_cigarette_detections")

# --- Later, querying ---

# "Where did we see people with cigarettes in the kitchen?"
for row in session.stream("cigarette_detections") \
        .after(one_hour_ago).near(kitchen_pose, 10.0):
    print(f"t={row.ts} pose={row.pose}: {row.data}")

# "Show me the source images alongside detections"
for det, img in session.stream("cigarette_detections") \
        .after(one_hour_ago).join(images):
    print(f"Detection: {det.data}, Source image at {img.pose}")

# "Find images similar to 'red shoes'"
similar = img_emb.search_embedding("red shoes", k=20).fetch()
# similar[0].data → Image (auto-projected from source)
# similar[0].embedding → Embedding (supports @ for cosine similarity)
```

## Full API

```python
from dimos.models.embedding.base import Embedding, EmbeddingModel

# --- Data types ---

@dataclass
class Observation:
    id: int
    ts: float | None = None
    pose: PoseStamped | None = None
    tags: dict[str, Any] = field(default_factory=dict)
    parent_id: int | None = None

    @property
    def data(self) -> Any:
        """Lazy payload. Pre-populated from append, fetched on demand from query."""
        ...

@dataclass
class EmbeddingObservation(Observation):
    """Returned by EmbeddingStream terminals. Auto-projects .data to source stream."""

    similarity: float | None = None  # 0..1, populated by search_embedding

    @property
    def data(self) -> Any:
        """Lazily loads from the source stream (e.g., Image), not the embedding."""
        ...

    @property
    def embedding(self) -> Embedding:
        """The Embedding object (has .vector, supports @ for cosine similarity)."""
        ...

# --- Transformer ---

class Transformer(ABC, Generic[T, R]):
    """Transforms a source stream into results on a target stream."""

    def process(self, source: Stream[T], target: Stream[R]) -> None:
        """Batch/historical processing. Full access to source stream."""
        ...

    def on_append(self, obs: Observation, target: Stream[R]) -> None:
        """Reactive processing. Called per new item."""
        ...

    supports_backfill: bool = True
    supports_live: bool = True
    output_type: type | None = None

# --- Streams ---

class Stream(Generic[T]):
    # Write (DB-backed only)
    def append(self, payload: T, *,
               ts: float | None = None,
               pose: PoseLike | None = None,
               tags: dict[str, Any] | None = None,
               parent_id: int | None = None,
               ) -> Observation: ...

    # Filter (returns new Stream, lazy)
    def after(self, t: float) -> Stream[T]: ...
    def before(self, t: float) -> Stream[T]: ...
    def time_range(self, t1: float, t2: float) -> Stream[T]: ...
    def at(self, t: float, *, tolerance: float = 1.0) -> Stream[T]: ...
    def near(self, pose: PoseLike, radius: float) -> Stream[T]: ...
    def filter_tags(self, **tags: Any) -> Stream[T]: ...

    # Order / paginate
    def order_by(self, field: str, *, desc: bool = False) -> Stream[T]: ...
    def limit(self, k: int) -> Stream[T]: ...
    def offset(self, n: int) -> Stream[T]: ...

    # Transform
    def transform(self,
                  xf: Transformer[T, R] | Callable[[T], R | list[R] | None],
                  *, live: bool = False,
                  backfill_only: bool = False,
                  ) -> Stream[R]: ...

    # Materialize (on TransformStream, accepts optional session= fallback)
    def store(self, name: str | None = None, session: Session | None = None) -> Stream[T]: ...

    # Cross-stream lineage
    def project_to(self, target: Stream[R]) -> Stream[R]: ...

    # Iteration & Terminals
    def __iter__(self) -> Iterator[Observation]: ...       # lazy, pages internally
    def fetch(self) -> ObservationSet[T]: ...               # list-like + stream-like result set
    def fetch_pages(self, batch_size: int = 128) -> Iterator[list[Observation]]: ...
    def one(self) -> Observation: ...
    def last(self) -> Observation: ...
    def count(self) -> int: ...

    # Reactive
    @property
    def appended(self) -> Observable[Observation]: ...

class EmbeddingStream(Stream[T]):
    """Created automatically when a Transformer produces Embedding output.
    Terminals return EmbeddingObservation (auto-projects .data to source stream)."""
    def search_embedding(self, query: Embedding | list[float] | str | Any,
                         *, k: int, raw: bool = False) -> Stream[Any]: ...

class TextStream(Stream[T]):
    """Stream with FTS index."""
    def search_text(self, text: str, *, k: int | None = None) -> TextStream[T]: ...

class ObservationSet(Stream[T]):
    """Materialized result set from fetch(). List-like + stream-like."""
    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> Observation: ...
    def __iter__(self) -> Iterator[Observation]: ...
    def __bool__(self) -> bool: ...

# --- Helpers ---

def ingest(stream: Stream, source: Iterable[tuple[float, Any]], *,
           pose_source: Any | None = None) -> int:
    """Ingest (ts, payload) pairs into a stream. Returns count."""
    ...

# --- Session / Store ---

PoseProvider = Callable[[], PoseLike | None]

class Session:
    def stream(self, name: str, payload_type: type | None = None, *,
               pose_provider: PoseProvider | None = None) -> Stream: ...
    def text_stream(self, name: str, payload_type: type | None = None, *,
                    tokenizer: str = "unicode61",
                    pose_provider: PoseProvider | None = None) -> TextStream: ...
    def embedding_stream(self, name: str, payload_type: type | None = None, *,
                         vec_dimensions: int | None = None,
                         pose_provider: PoseProvider | None = None,
                         parent_table: str | None = None,
                         embedding_model: EmbeddingModel | None = None) -> EmbeddingStream: ...
    def materialize_transform(self, name: str, source: Stream,
                              transformer: Transformer,
                              *, payload_type: type | None = None,
                              live: bool = False,
                              backfill_only: bool = False) -> Stream: ...
    def list_streams(self) -> list[StreamInfo]: ...
    def resolve_parent_stream(self, name: str) -> str | None: ...
    def resolve_lineage_chain(self, source: str, target: str) -> tuple[str, ...]: ...
    def close(self) -> None: ...

class Store:
    def session(self) -> Session: ...
    def close(self) -> None: ...
```

## Internal Backing (impl detail)

A `Stream` can be backed by different things — the user never sees this:

- **DB tables** — from `session.stream()`. Metadata + payload + indexes.
- **Predicate** — from `.after()`, `.near()`, etc. Lazy SQL WHERE.
- **Transform** — from `.transform(t)`. Source stream + Transformer.
- **ListBackend** — from `ObservationSet`. In-memory Python-side filtering.

The impl decides how to execute based on the backing chain.

## SQLite Schema

Each stream `{name}` creates these tables:

```sql
-- Metadata table (compact rows, fast scans)
CREATE TABLE {name} (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL,
    pose_x REAL,           -- position
    pose_y REAL,
    pose_z REAL,
    pose_qx REAL,          -- orientation quaternion (stored, not indexed)
    pose_qy REAL,
    pose_qz REAL,
    pose_qw REAL,
    tags TEXT DEFAULT '{}',
    parent_id INTEGER       -- lineage: source observation id
);
CREATE INDEX idx_{name}_ts ON {name}(ts);

-- Payload table (blobs, loaded on demand)
CREATE TABLE {name}_payload (
    id INTEGER PRIMARY KEY,
    data BLOB
);

-- R*Tree spatial index (position only)
CREATE VIRTUAL TABLE {name}_rtree USING rtree(
    id,
    min_x, max_x,
    min_y, max_y,
    min_z, max_z
);
```

**Optional per stream kind:**

```sql
-- TextStream: FTS5 full-text index
CREATE VIRTUAL TABLE {name}_fts USING fts5(content, tokenize='unicode61');

-- EmbeddingStream: vec0 vector index (cosine distance)
CREATE VIRTUAL TABLE {name}_vec USING vec0(
    embedding float[{dim}] distance_metric=cosine
);
```

### Key design decisions

- **Separate payload table** — metadata queries (`fetch`, `count`, `near`, filters) never touch blob data. Payload is loaded lazily via `obs.data`.
- **Decomposed pose columns** — enables R*Tree spatial index for `.near()` queries. Orientation stored for reconstruction but not spatially indexed.
- **R*Tree for spatial queries** — `.near(pose, radius)` compiles to an R*Tree range query (bounding box at +/-radius), with post-filter for exact Euclidean distance.
- **Cosine distance metric** — vec0 uses `distance_metric=cosine` (0=identical, 2=opposite). Similarity = `1.0 - distance`, clamped to [0, 1].

### Lazy payload loading

`fetch()` returns `Observation` with lazy `.data`:
- Metadata query: `SELECT id, ts, pose_x, ..., tags, parent_id FROM {name} WHERE ...`
- `_data` stays `_UNSET`, `_data_loader` is set to: `SELECT data FROM {name}_payload WHERE id = ?`
- Only `obs.data` access triggers the blob read + codec decode

This means iterating metadata (`obs.ts`, `obs.pose`, `obs.tags`) is cheap.

### NearFilter SQL compilation

```python
# .near(pose, 5.0) compiles to:
# JOIN {name}_rtree AS r ON r.id = {name}.id
# WHERE r.min_x >= pose.position.x - 5.0 AND r.max_x <= pose.position.x + 5.0
#   AND r.min_y >= pose.position.y - 5.0 AND r.max_y <= pose.position.y + 5.0
#   AND r.min_z >= pose.position.z - 5.0 AND r.max_z <= pose.position.z + 5.0
```

For exact distance (not just bounding box), a post-filter computes Euclidean distance on the R*Tree candidates.

## Serialization (Codec)

Each stream has a `Codec[T]` that handles payload encode/decode. Auto-selected from `payload_type`.

```python
class Codec(Protocol[T]):
    def encode(self, value: T) -> bytes: ...
    def decode(self, data: bytes) -> T: ...

class LcmCodec(Codec[DimosMsg]):
    """For DimosMsg types — uses lcm_encode/lcm_decode."""

class JpegCodec(Codec[Image]):
    """For Image types — uses JPEG compression."""

class PickleCodec(Codec[Any]):
    """Fallback for arbitrary Python objects."""

def codec_for_type(payload_type: type[T] | None) -> Codec[T]:
    """Auto-select codec based on payload type."""
    ...
```

Lives in `dimos.memory.codec`.

Transparent to the user — just pass `payload_type` to `session.stream()`:
```python
images = session.stream("images", Image)    # auto LCM codec
numbers = session.stream("numbers", int)    # auto pickle codec
```

Tags are JSON. Poses are decomposed into columns (not serialized).

### Stream metadata (`_streams` table)

```
name           TEXT PRIMARY KEY
payload_module TEXT    -- fully qualified, e.g. "dimos.msgs.sensor_msgs.Image.Image"
stream_kind    TEXT    -- "stream" | "text" | "embedding"
parent_stream  TEXT    -- parent stream name (lineage for project_to/join)
embedding_dim  INTEGER -- vec0 dimension (embedding streams only)
```

On restart, `session.stream("images")` (no `payload_type`) resolves the class from `payload_module` via `importlib`, then selects the codec automatically. `embedding_dim` allows recreating the vec0 table without needing to see the first embedding again.

## Resolved Questions

1. **`.append()` on non-stored streams?** → `TypeError` (requires backend).
2. **Multiple `.store()` calls?** → Idempotent — returns existing stream if already stored.
3. ~~**Memory pressure from in-memory transforms?**~~ → Solved via `fetch_pages`.
4. **Pose storage** → Decomposed columns + R*Tree index (not binary blob).
5. **Payload loading** → Lazy via separate `{name}_payload` table.
6. **`__iter__`** → `for page in self.fetch_pages(): yield from page` — lazy, memory-efficient iteration.
7. **`project_to` / lineage** → Implemented via `parent_id` column + `_streams.parent_stream`. Multi-hop chains supported.
8. **`fetch()` return type** → `ObservationSet` (list-like + stream-like).
9. **Similarity scores** → `EmbeddingObservation.similarity` populated from vec0 cosine distance.

## Open Questions

1. **Incremental transforms** — re-running a stored transform should resume from last processed item.
2. **4D indexing** — should R*Tree include time as a 4th dimension?

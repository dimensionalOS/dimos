# SQLite Implementation

Implementation spec for the SQLite backend. A coding agent should be able to implement the full backend from this document + `api.md`.

## File Structure

```
dimos/memory/
    __init__.py              # public exports
    types.py                 # Observation, EmbeddingObservation, StreamInfo, Filter types
    stream.py                # Stream, EmbeddingStream, TextStream, ObservationSet, ListBackend
    transformer.py           # Transformer ABC, PerItemTransformer, EmbeddingTransformer, etc.
    store.py                 # Session ABC, Store ABC
    codec.py                 # LcmCodec, JpegCodec, PickleCodec, codec_for_type()
    ingest.py                # ingest() helper for batch ingestion
    viz.py                   # similarity_heatmap(), similarity_poses(), log_similarity_timeline()

    impl/
        sqlite.py            # SqliteStore, SqliteSession, Sqlite*Backend (single file)
        test_sqlite.py       # tests
```

## Dependencies

- `sqlite3` (stdlib)
- `sqlite-vec` — vector similarity search via vec0 virtual table. Loaded via `sqlite_vec.load(conn)`.
- FTS5 — built into SQLite by default on most platforms.
- R*Tree — built into SQLite by default.
- `reactivex` — for `.appended` observable (already a DimOS dependency).

## Connection Management

### SqliteStore

```python
class SqliteStore(Store):
    def __init__(self, path: str):
        self._path = path
        self._conn = sqlite3.connect(path)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._load_extensions()

    def session(self) -> SqliteSession:
        return SqliteSession(self._conn)

    def _load_extensions(self) -> None:
        try:
            import sqlite_vec
            self._conn.enable_load_extension(True)
            sqlite_vec.load(self._conn)
            self._conn.enable_load_extension(False)
        except ImportError:
            pass  # vec0 unavailable — search_embedding will raise

    def close(self) -> None:
        self._conn.close()
```

### SqliteSession

```python
class SqliteSession(Session):
    def __init__(self, conn: sqlite3.Connection):
        self._conn = conn
        self._streams: dict[str, Stream] = {}  # cache by name
        self._ensure_meta_table()

    def _ensure_meta_table(self):
        """Create _streams registry table if not exists."""
        self._conn.execute("""
            CREATE TABLE IF NOT EXISTS _streams (
                name TEXT PRIMARY KEY,
                payload_module TEXT,
                stream_kind TEXT DEFAULT 'stream',
                parent_stream TEXT,
                embedding_dim INTEGER
            )
        """)

    def stream(self, name, payload_type=None, *, pose_provider=None) -> Stream:
        # Returns cached or creates new. payload_type required for new streams.
        ...

    def text_stream(self, name, payload_type=None, *, tokenizer="unicode61",
                    pose_provider=None) -> TextStream:
        ...

    def embedding_stream(self, name, payload_type=None, *, vec_dimensions=None,
                         pose_provider=None, parent_table=None,
                         embedding_model=None) -> EmbeddingStream:
        ...

    def list_streams(self) -> list[StreamInfo]: ...
    def resolve_parent_stream(self, name: str) -> str | None: ...
    def resolve_lineage_chain(self, source: str, target: str) -> tuple[str, ...]: ...
    def close(self) -> None: ...
```

## Schema

All table names are prefixed with the stream name. Stream names are validated: `[a-zA-Z_][a-zA-Z0-9_]*`.

### `_streams` — Global registry

```sql
CREATE TABLE _streams (
    name TEXT PRIMARY KEY,
    payload_module TEXT,         -- e.g. 'dimos.msgs.sensor_msgs.Image.Image'
    stream_kind TEXT DEFAULT 'stream',  -- 'stream', 'embedding', 'text'
    parent_stream TEXT,          -- parent stream name (lineage)
    embedding_dim INTEGER        -- only for kind='embedding'
);
```

### `{name}` — Observation metadata (all stream types)

```sql
CREATE TABLE {name} (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL,
    pose_x REAL, pose_y REAL, pose_z REAL,
    pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL,
    tags TEXT DEFAULT '{}',       -- JSON dict
    parent_id INTEGER             -- lineage: id in parent stream
);
CREATE INDEX idx_{name}_ts ON {name}(ts);
```

### `{name}_payload` — Blob/Text payload

```sql
CREATE TABLE {name}_payload (
    id INTEGER PRIMARY KEY,       -- matches {name}.id
    data BLOB NOT NULL
);
```

Separated from metadata so metadata queries never page in multi-MB blobs.

### `{name}_rtree` — Spatial index (all stream types)

```sql
CREATE VIRTUAL TABLE {name}_rtree USING rtree(
    id,                            -- matches {name}.id
    min_x, max_x,
    min_y, max_y,
    min_z, max_z
);
```

Only rows with pose are inserted into R*Tree. Rows without pose are excluded from `.near()` results.

### `{name}_fts` — Full-text search (TextStream only)

```sql
CREATE VIRTUAL TABLE {name}_fts USING fts5(
    content,
    tokenize='{tokenizer}'
);
```

Standalone FTS table (not content-synced). Rowids match `{name}.id`.

### `{name}_vec` — Vector index (EmbeddingStream only)

```sql
CREATE VIRTUAL TABLE {name}_vec USING vec0(
    embedding float[{dim}] distance_metric=cosine
);
```

Cosine distance: 0 = identical, 2 = opposite. Similarity = `max(0, min(1, 1.0 - distance))`.

Rowids match `{name}.id`. Dimension inferred from first embedding inserted, or from `vec_dimensions` parameter.

## Stream Implementation

### Backend classes

The stream/backend split separates query logic from stream API:

```python
class SqliteStreamBackend:
    """Base backend for blob streams."""
    def do_append(self, payload, ts, pose, tags, parent_id=None) -> Observation: ...
    def execute_fetch(self, query: StreamQuery) -> list[Observation]: ...
    def execute_count(self, query: StreamQuery) -> int: ...

class SqliteEmbeddingBackend(SqliteStreamBackend):
    """Adds vec0 index. Overrides execute_fetch for vector search."""
    ...

class SqliteTextBackend(SqliteStreamBackend):
    """Adds FTS5 index. Overrides execute_fetch for text search."""
    ...
```

### append()

```python
def do_append(self, payload, ts, pose, tags, parent_id=None):
    ts = ts or time.time()
    if pose is None and self._pose_provider:
        pose = self._pose_provider()

    pose_cols = _decompose_pose(pose)
    tags_json = _serialize_tags(tags)

    # 1. Insert into meta table
    cur = self._conn.execute(
        f"INSERT INTO {name} "
        "(ts, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, tags, parent_id) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (ts, *pose_cols, tags_json, parent_id),
    )
    row_id = cur.lastrowid

    # 2. Insert into _payload
    blob = self._codec.encode(payload)
    self._conn.execute(
        f"INSERT INTO {name}_payload(id, data) VALUES (?, ?)",
        (row_id, blob)
    )

    # 3. Insert into _rtree (if pose)
    if pose_cols:
        x, y, z = pose_cols[0], pose_cols[1], pose_cols[2]
        self._conn.execute(
            f"INSERT INTO {name}_rtree(id, min_x, max_x, min_y, max_y, min_z, max_z) "
            "VALUES (?, ?, ?, ?, ?, ?, ?)",
            (row_id, x, x, y, y, z, z)
        )

    self._conn.commit()

    # 4. Build Observation and emit
    obs = Observation(id=row_id, ts=ts, pose=pose, tags=tags or {}, _data=payload)
    self._subject.on_next(obs)
    return obs
```

### EmbeddingBackend.append()

Same as above, plus inserts into `_vec`:

```python
if isinstance(payload, Embedding):
    vec = payload.to_numpy().tolist()
    self._conn.execute(
        f"INSERT INTO {name}_vec(rowid, embedding) VALUES (?, ?)",
        (row_id, json.dumps(vec))
    )
```

### TextBackend.append()

Same as base, plus inserts into `_fts`:

```python
text = str(payload)
self._conn.execute(
    f"INSERT INTO {name}_fts(rowid, content) VALUES (?, ?)",
    (row_id, text)
)
```

## Filter → SQL Generation

Each filter method returns a new stream with an added filter. At terminal time, the filter chain is compiled to SQL.

### Filter types

```python
AfterFilter(t)           # → WHERE ts > ?
BeforeFilter(t)          # → WHERE ts < ?
TimeRangeFilter(t1, t2)  # → WHERE ts >= ? AND ts <= ?
AtFilter(t, tolerance)   # → WHERE ABS(ts - ?) <= ?
NearFilter(pose, radius) # → JOIN _rtree bounding box query
TagsFilter(tags)         # → WHERE json_extract(tags, '$.key') = ?
EmbeddingSearchFilter(vec, k)  # → query _vec, then filter by rowids
TextSearchFilter(text, k)     # → query _fts MATCH, then filter by rowids
LineageFilter(source_table, source_query, hops)  # → nested IN subquery
```

### SQL compilation

Walk the filter list, generate SQL:

```python
def _compile_query(query, table) -> tuple[str, list[Any]]:
    # Base SELECT
    sql = f"SELECT {table}.id, {table}.ts, ... FROM {table}"

    # NearFilter → JOIN _rtree
    # Other filters → WHERE clauses
    # EmbeddingSearch/TextSearch → handled separately (two-step query)
    # LineageFilter → nested IN subquery via _compile_ids()

    return sql, params
```

### search_embedding (vec0)

Two-step process:

```sql
-- 1. Top-k vector search (cosine distance)
SELECT rowid, distance
FROM {name}_vec
WHERE embedding MATCH ?
ORDER BY distance
LIMIT ?
```

```python
# 2. Build dist_map, fetch metadata for those rowids, populate similarity
dist_map = {rowid: distance for rowid, distance in vec_rows}
# ... fetch metadata WHERE id IN (rowids) ...
for obs in observations:
    obs.similarity = max(0.0, min(1.0, 1.0 - dist_map[obs.id]))
# Re-sort by distance rank (IN clause doesn't preserve vec0 ordering)
```

### search_text (FTS5)

```sql
SELECT rowid, rank
FROM {name}_fts
WHERE content MATCH ?
ORDER BY rank
```

Same two-step: get rowids from FTS5, then fetch metadata.

### LineageFilter compilation

LineageFilter compiles to a nested SQL subquery walking the `parent_id` chain:

```python
# Single hop: embeddings → images
f"SELECT parent_id FROM {source_table} WHERE id IN ({source_ids_sql})"

# Multi-hop: embeddings → sharp_frames → images
# Wraps each hop as a nested IN subquery
```

## Terminal Execution

### __iter__() — lazy iteration

`Stream` is directly iterable via `fetch_pages`:

```python
def __iter__(self):
    for page in self.fetch_pages():
        yield from page
```

### fetch()

Returns `ObservationSet` (list-like + stream-like):

```python
def fetch(self) -> ObservationSet:
    results = self._backend.execute_fetch(self._query)
    return ObservationSet(results, session=self._session)
```

### count()

```python
def count(self) -> int:
    sql, params = _compile_count(query, table)
    # → SELECT COUNT(*) FROM {table} WHERE ...
    return self._conn.execute(sql, params).fetchone()[0]
```

### one() / last()

- `one()` → `self.limit(1).fetch()[0]`
- `last()` → `self.order_by("ts", desc=True).limit(1).fetch()[0]`

## Lazy Data Loading

`Observation.data` uses lazy loading:

```python
@dataclass
class Observation:
    _data: Any = field(default=_UNSET, repr=False)
    _data_loader: Callable[[], Any] | None = field(default=None, repr=False)

    @property
    def data(self) -> Any:
        if self._data is not _UNSET:
            return self._data
        if self._data_loader is not None:
            self._data = self._data_loader()
            return self._data
        raise LookupError("No data available")
```

When building observations from query results:

```python
def _row_to_obs(self, row) -> Observation:
    row_id, ts, px, py, pz, qx, qy, qz, qw, tags_json, pid = row
    pose = _reconstruct_pose(px, py, pz, qx, qy, qz, qw)

    def loader():
        r = conn.execute(f"SELECT data FROM {table}_payload WHERE id = ?", (row_id,)).fetchone()
        return codec.decode(r[0])

    return Observation(id=row_id, ts=ts, pose=pose, tags=..., _data_loader=loader)
```

### EmbeddingObservation

For `EmbeddingBackend`, `_row_to_obs` returns `EmbeddingObservation` with two lazy loaders:

```python
def _row_to_obs(self, row) -> EmbeddingObservation:
    # ... same metadata extraction ...

    # _data_loader: loads raw embedding payload
    # _source_data_loader: loads from PARENT stream (auto-projection)
    #   - Resolves parent codec from _streams.payload_module
    #   - Uses parent_id to look up the source payload

    return EmbeddingObservation(
        id=row_id, ts=ts, pose=pose, tags=...,
        parent_id=pid,
        _data_loader=loader,
        _source_data_loader=source_loader,  # None if no parent
    )
```

## Lineage

### Storing lineage

When a Transformer appends to a target stream, `parent_id` links back to the source:

```python
target.append(result, ts=source_obs.ts, pose=source_obs.pose,
              parent_id=source_obs.id)
```

The `_streams` registry tracks stream-level lineage:
```python
# After materialize_transform creates the target
UPDATE _streams SET parent_stream = ? WHERE name = ?
```

### resolve_lineage_chain()

Walks `_streams.parent_stream` from source toward target:

```python
def resolve_lineage_chain(self, source: str, target: str) -> tuple[str, ...]:
    # Single hop (source → target): returns ()
    # Two hops (source → mid → target): returns ("mid",)
    # Raises ValueError if no path exists
```

### project_to()

Uses `LineageFilter` to compile a nested SQL subquery:

```python
def project_to(self, target: Stream) -> Stream:
    hops = session.resolve_lineage_chain(source_table, target_table)
    return target._with_filter(LineageFilter(source_table, self._query, hops))
```

## Pose Helpers

PoseStamped in dimos extends Pose directly (no wrapper). Access position/orientation directly:

```python
def _decompose_pose(pose) -> tuple[float, ...] | None:
    if pose is None:
        return None
    p = pose.position       # NOT pose.pose.position
    q = pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

def _reconstruct_pose(x, y, z, qx, qy, qz, qw) -> PoseStamped | None:
    if x is None:
        return None
    return PoseStamped(
        position=[x, y or 0.0, z or 0.0],           # list args (plum dispatch)
        orientation=[qx or 0.0, qy or 0.0, qz or 0.0, qw or 1.0],
    )
```

NearFilter SQL compilation also accesses `f.pose.position` directly.

## Transform Execution

### .transform() — returns lazy stream

`.transform(xf)` doesn't execute immediately. It returns a `TransformStream`. Execution happens at terminal time or `.store()`.

### .store() — materializes

When `.store(name)` is called on a `TransformStream`:

1. Register target stream in `_streams` (with `parent_stream` set)
2. Create target tables
3. Auto-detect target stream type from transformer:
   - `EmbeddingTransformer` → `EmbeddingStream` (with parent_table)
   - `CaptionTransformer` → `TextStream` (FTS)
   - Other → `Stream` (blob)
4. If not `live` mode: run `xf.process(source, target)` (backfill)
5. If not `backfill_only`: subscribe to source's `.appended`, call `xf.on_append()`
6. Return the stored stream

### .fetch() on TransformStream (no .store())

Executes the transform in-memory using `_CollectorStream`:

```python
def fetch(self) -> ObservationSet:
    collector = _CollectorStream()
    self._transformer.process(self._source, collector)
    return ObservationSet(collector.results)
```

## Reactive (.appended)

Each stored stream backend has a `Subject` from reactivex:

```python
class SqliteStreamBackend:
    def __init__(self, ...):
        self._subject: Subject[Observation] = Subject()

    @property
    def appended_subject(self):
        return self._subject
```

`do_append()` emits to the subject after the DB write succeeds.

For filtered streams, the observable filters events through the filter chain in Python:

```python
@property
def appended(self):
    raw = self._backend.appended_subject
    active = [f for f in self._query.filters
              if not isinstance(f, (EmbeddingSearchFilter, LineageFilter))]
    return raw.pipe(ops.filter(lambda obs: all(f.matches(obs) for f in active)))
```

## Serialization

### Codec system

```python
class LcmCodec:    # for DimosMsg types (lcm_encode/lcm_decode)
class JpegCodec:   # for Image types (JPEG compression)
class PickleCodec: # fallback for arbitrary Python objects

def codec_for_type(payload_type: type | None) -> Codec:
    """Auto-select codec based on payload type."""
    ...
```

Lives in `dimos.memory.codec`.

### Tag serialization

Tags are stored as JSON text. Empty dict → `"{}"`.

## SQL Safety

- **Identifier validation**: stream names must match `^[a-zA-Z_][a-zA-Z0-9_]*$`.
- **Parameterized queries**: all user values go through `?` params, never string interpolation.
- **Table names**: constructed from validated stream names, safe for SQL interpolation.
- **Order fields**: validated against allowlist `{"id", "ts"}`.

## Thread Safety

- Each `Session` owns one `sqlite3.Connection` — not shared across threads.
- Multiple sessions can exist on the same file (WAL mode allows concurrent reads + one writer).
- The `appended` subject emits on the thread that called `append()`.

## Error Handling

- `append()` on non-stored stream → `TypeError`
- `search_embedding()` on non-embedding stream → `TypeError`
- `search_text()` on non-text stream → `TypeError`
- `search_embedding()` when sqlite-vec not loaded → `RuntimeError`
- Invalid stream name → `ValueError`
- `one()` with no results → `LookupError`
- `stream()` without `payload_type` on new stream → `TypeError`

## Testing

Tests in `dimos/memory/impl/test_sqlite.py`. Use `:memory:` store for speed.

Key test scenarios:
1. Create stream, append, fetch — verify data round-trips
2. Temporal filters (after, before, time_range, at)
3. Spatial filter (near) — with and without pose
4. Tag filtering
5. EmbeddingStream — store embeddings, search_embedding, verify auto-projection
6. TextStream — store text, search_text
7. Transform with lambda — verify lineage
8. Transform with Transformer class — verify process() called
9. Chained filters — verify SQL composition
10. project_to — verify cross-stream lineage (single and multi-hop)
11. fetch_pages — verify pagination
12. Lazy data loading — verify .data only hits DB on access
13. .appended observable — verify reactive emission
14. Similarity scores — verify EmbeddingObservation.similarity populated after search
15. raw=True — verify EmbeddingObservation with similarity + auto-projected data
16. ObservationSet — verify list-like + stream-like behavior

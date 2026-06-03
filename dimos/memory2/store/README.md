# store — Store and ObservationStore implementations

Store is the top-level user-facing entry point. You create one, ask it for named streams, and use those streams. Internally, each stream gets a Backend that orchestrates the lower-level pieces:

```
Store
  └── stream("lidar") → Backend
                           ├── ObservationStore  (metadata: id, timestamp, tags, frame_id)
                           ├── BlobStore         (raw bytes: encoded payloads)
                           ├── VectorStore       (embeddings: similarity search)
                           └── Notifier          (live push: new observation events)
```

- **ObservationStore** stores observation *metadata* and handles queries (filters, ordering, limit/offset, text search). Doesn't touch raw data or vectors.
- **BlobStore** stores/retrieves encoded payloads by `(stream_name, row_id)`. Just a key-value byte store.
- **VectorStore** stores/retrieves embedding vectors, handles similarity search.
- **Notifier** pushes new observations to live subscribers (for `.live()` tails).

The **Backend** is the glue — on `append()` it encodes the payload, inserts metadata into ObservationStore, stores the blob in BlobStore, indexes the vector in VectorStore, and notifies live subscribers. On iterate, it queries ObservationStore for metadata, attaches lazy blob loaders, and handles vector search routing.

**Store** sits above all that — it manages the mapping of stream names to Backends, handles config inheritance (store-level defaults vs per-stream overrides), and provides the `store.stream("name")` / `store.streams.name` API. `MemoryStore` vs `SqliteStore` vs `NullStore` differ in which component implementations they wire up by default and how they persist the registry of known streams.

## Store implementations

| Store          | File        | Description                                          |
|----------------|-------------|------------------------------------------------------|
| `MemoryStore`  | `memory.py` | In-memory store for experimentation                  |
| `SqliteStore`  | `sqlite.py` | SQLite-backed persistent store (WAL, registry, vec0) |
| `NullStore`    | `null.py`   | Live-only O(1) memory, no history/replay             |

## ObservationStore implementations

| ObservationStore         | File                       | Storage                             |
|--------------------------|----------------------------|-------------------------------------|
| `ListObservationStore`   | `observationstore/memory.py`  | In-memory deque, brute-force search. `max_size` controls retention (None=all, N=rolling window, 0=discard) |
| `SqliteObservationStore` | `observationstore/sqlite.py`  | SQLite (WAL, R*Tree, vec0)          |

## Writing a new ObservationStore

### 1. Subclass ObservationStore

```python
from dimos.memory2.observationstore.base import ObservationStore

class MyObservationStore(ObservationStore[T]):
    def __init__(self, name: str, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._name = name

    @property
    def name(self) -> str:
        return self._name

    def insert(self, obs: Observation[T]) -> int:
        """Insert observation metadata, return assigned id."""
        row_id = self._next_id
        self._next_id += 1
        # ... persist metadata ...
        return row_id

    def query(self, q: StreamQuery) -> Iterator[Observation[T]]:
        """Yield observations matching the query."""
        # The query carries metadata fields:
        #   q.filters       — tuple of Filter objects (each has .matches(obs))
        #   q.order_field   — sort field name (e.g. "ts")
        #   q.order_desc    — sort direction
        #   q.limit_val     — max results
        #   q.offset_val    — skip first N
        #   q.search_text   — substring text search
        ...

    def count(self, q: StreamQuery) -> int:
        """Count matching observations."""
        ...

    def fetch_by_ids(self, ids: list[int]) -> list[Observation[T]]:
        """Batch fetch by id (for vector search results)."""
        ...
```

`ObservationStore` is an abstract base class (extends `CompositeResource` and `Configurable`).

### 2. Create a Store subclass

```python
from dimos.memory2.backend import Backend
from dimos.memory2.codecs.base import codec_for
from dimos.memory2.store.base import Store

class MyStore(Store):
    def _create_backend(
        self, name: str, payload_type: type | None = None, **config: Any
    ) -> Backend:
        obs = MyObservationStore(name)
        obs.start()
        codec = self._resolve_codec(payload_type, config.get("codec"))
        return Backend(
            metadata_store=obs,
            codec=codec,
            blob_store=config.get("blob_store"),
            vector_store=config.get("vector_store"),
            notifier=config.get("notifier"),
            eager_blobs=config.get("eager_blobs", False),
        )

    def list_streams(self) -> list[str]:
        return list(self._streams.keys())

    def delete_stream(self, name: str) -> None:
        self._streams.pop(name, None)
```

The Store creates a `Backend` composite for each stream. The `Backend` handles all orchestration (encode -> insert -> store blob -> index vector -> notify) so your ObservationStore only needs to handle metadata.

### 3. Add to the test grid

In `conftest.py`, add your store fixture and include it in the parametrized `session` fixture so all standard tests run against it:

```python
@pytest.fixture
def my_store() -> Iterator[MyStore]:
    with MyStore() as store:
        yield store

@pytest.fixture(params=["memory_store", "sqlite_store", "my_store"])
def session(request):
    return request.getfixturevalue(request.param)
```

Use `pytest.mark.xfail` for features not yet implemented — the grid test covers: append, fetch, iterate, count, first/last, exists, all filters, ordering, limit/offset, embeddings, text search.

### Query contract

The ObservationStore must handle the `StreamQuery` metadata fields. Vector search and blob loading are handled by the `Backend` composite — the ObservationStore never needs to deal with them.

`StreamQuery.apply(iterator)` provides a complete Python-side execution path — filters, text search, vector search, ordering, offset/limit — all as in-memory operations. ObservationStores can use it in three ways:

**Full delegation** — simplest, good enough for in-memory stores:
```python
def query(self, q: StreamQuery) -> Iterator[Observation[T]]:
    return q.apply(iter(self._data))
```

**Partial push-down** — handle some operations natively, delegate the rest:
```python
def query(self, q: StreamQuery) -> Iterator[Observation[T]]:
    # Handle filters and ordering in SQL
    rows = self._sql_query(q.filters, q.order_field, q.order_desc)
    # Delegate remaining operations to Python
    remaining = StreamQuery(
        search_text=q.search_text,
        offset_val=q.offset_val, limit_val=q.limit_val,
    )
    return remaining.apply(iter(rows))
```

**Full push-down** — translate everything to native queries (SQL WHERE, FTS5 MATCH) without calling `apply()` at all.

For filters, each `Filter` object has a `.matches(obs) -> bool` method that ObservationStores can use directly if they don't have a native equivalent.

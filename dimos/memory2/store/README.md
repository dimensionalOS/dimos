# store — Store implementations

Metadata index backends for memory. Each index implements the `ObservationStore` protocol to provide observation metadata storage with query support. The concrete `Backend` class handles orchestration (blob, vector, live) on top of any index.

## Existing implementations

| ObservationStore           | File        | Status   | Storage                             |
|-----------------|-------------|----------|-------------------------------------|
| `ListObservationStore`     | `memory.py` | Complete | In-memory lists, brute-force search |
| `SqliteObservationStore`   | `sqlite.py` | Complete | SQLite (WAL, R*Tree, vec0)          |

## Writing a new index

### 1. Implement the ObservationStore protocol

```python
from dimos.memory2.observationstore.base import ObservationStore
from dimos.memory2.type.filter import StreamQuery
from dimos.memory2.type.observation import Observation

class MyObservationStore(Generic[T]):
    def __init__(self, name: str) -> None:
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
        # The index handles metadata query fields:
        #   q.filters       — list of Filter objects (each has .matches(obs))
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

`ObservationStore` is a `@runtime_checkable` Protocol — no base class needed, just implement the methods.

### 2. Create a Store subclass

```python
from dimos.memory2.backend import Backend
from dimos.memory2.codecs.base import codec_for
from dimos.memory2.store.base import Store

class MyStore(Store):
    def _create_backend(
        self, name: str, payload_type: type | None = None, **config: Any
    ) -> Backend:
        index = MyObservationStore(name)
        codec = codec_for(payload_type)
        return Backend(
            index=index,
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

The Store creates a `Backend` composite for each stream. The `Backend` handles all orchestration (encode → insert → store blob → index vector → notify) so your index only needs to handle metadata.

### 3. Add to the grid test

In `test_impl.py`, add your store to the fixture so all standard tests run against it:

```python
@pytest.fixture(params=["memory", "sqlite", "myindex"])
def store(request, tmp_path):
    if request.param == "myindex":
        return MyStore(...)
    ...
```

Use `pytest.mark.xfail` for features not yet implemented — the grid test covers: append, fetch, iterate, count, first/last, exists, all filters, ordering, limit/offset, embeddings, text search.

### Query contract

The index must handle the `StreamQuery` metadata fields. Vector search and blob loading are handled by the `Backend` composite — the index never needs to deal with them.

`StreamQuery.apply(iterator)` provides a complete Python-side execution path — filters, text search, vector search, ordering, offset/limit — all as in-memory operations. ObservationStorees can use it in three ways:

**Full delegation** — simplest, good enough for in-memory indexes:
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

For filters, each `Filter` object has a `.matches(obs) -> bool` method that indexes can use directly if they don't have a native equivalent.

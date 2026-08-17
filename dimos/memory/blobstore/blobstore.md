# blobstore/

Separates payload blob storage from metadata indexing. Observation payloads vary hugely in size — a `Vector3` is 24 bytes, a camera frame is megabytes. Storing everything inline penalizes metadata queries. BlobStore lets large payloads live elsewhere.

## ABC (`blobstore/base.py`)

```python
class BlobStore(Resource):
    def put(self, stream_name: str, key: int, data: bytes) -> None: ...
    def get(self, stream_name: str, key: int) -> bytes: ...    # raises KeyError if missing
    def delete(self, stream_name: str, key: int) -> None: ...  # silent if missing
```

- `stream_name` — stream name (used to organize storage: directories, tables)
- `key` — observation id
- `data` — encoded payload bytes (codec handles serialization, blob store handles persistence)
- Extends `Resource` (start/stop) but does NOT own its dependencies' lifecycle

## Implementations

### `file.py` — FileBlobStore

Stores blobs as files on disk, one directory per stream.

```
{root}/{stream}/{key}.bin
```

`__init__(root: str | os.PathLike[str])` — `start()` creates the root directory.

### `sqlite.py` — SqliteBlobStore

Stores blobs in a separate SQLite table per stream.

```sql
CREATE TABLE "{stream}_blob" (id INTEGER PRIMARY KEY, data BLOB NOT NULL)
```

`__init__(conn: sqlite3.Connection)` — does NOT own the connection.

**Internal use** (same db as metadata): `SqliteStore._create_backend()` creates one connection per stream, passes it to both the index and the blob store.

**External use** (separate db): user creates a separate connection and passes it. User manages that connection's lifecycle.

**JOIN optimization**: when `eager_blobs=True` and the blob store shares the same connection as the index, `SqliteObservationStore` can optimize with a JOIN instead of separate queries:

```sql
SELECT m.id, m.ts, m.pose, m.tags, b.data
FROM "images" m JOIN "images_blob" b ON m.id = b.id
WHERE m.ts > ?
```

## Lazy loading

`eager_blobs` is a store/stream-level flag, orthogonal to blob store choice. It controls WHEN data is loaded:

- `eager_blobs=False` (default) → backend sets `Observation._loader`, payload loaded on `.data` access
- `eager_blobs=True` → backend triggers `.data` access during iteration (eager)

| eager_blobs | blob store | loading strategy |
|-------------|-----------|-----------------|
| True  | SqliteBlobStore (same conn) | JOIN — one round trip |
| True  | any other | iterate meta, `blob_store.get()` per row |
| False | any | iterate meta only, `_loader = lambda: codec.decode(blob_store.get(...))` |

## Usage

```python
# Per-stream blob store choice
poses = store.stream("poses", PoseStamped)                              # default, lazy
images = store.stream("images", Image, eager_blobs=True)                # eager
images = store.stream("images", Image, blob_store=file_blobs)           # override
```

## Files

```
blobstore/
  base.py             BlobStore ABC
  blobstore.md        this file
  __init__.py          re-exports BlobStore, FileBlobStore, SqliteBlobStore
  file.py             FileBlobStore
  sqlite.py           SqliteBlobStore
```

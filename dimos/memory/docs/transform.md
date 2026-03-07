# Transform — Unified Derived Stream API

## Concept

`.transform()` is a single method on `StreamBase` that handles both historical (batch) and live (reactive) processing. It takes data from a source, applies a function, and stores results into the target stream with lineage.

## API

```python
class StreamBase(ABC, Generic[T]):
    def transform(self,
                  source: StreamBase | ObservationSet,
                  fn: Callable[[Any], T | list[T] | None] | None = None,
                  *,
                  live: bool = False,
                  ) -> Self:
        """
        Process source data, store results in this stream.

        Args:
            source: where to read from
            fn: transform function. Returns T, list[T], or None (skip).
                None allowed for EmbeddingStream (uses model.embed implicitly).
            live: if True, only subscribe to new appends (no backfill)

        Behavior by source type:
            StreamBase → backfill existing + subscribe to live (default)
                         live=True → skip backfill, only subscribe
            ObservationSet → batch process snapshot (live ignored)

        Returns self for chaining.
        """
```

## Source type determines mode

| Source           | `live=False` (default)                           | `live=True`                   |
|------------------|--------------------------------------------------|-------------------------------|
| `StreamBase`     | backfill all existing + subscribe to `.appended` | subscribe to `.appended` only |
| `ObservationSet` | batch process the set                            | N/A (ignored)                 |

## Transform function contract

```python
fn: Callable[[Any], T | list[T] | None]
```

- Returns `T` → single result stored
- Returns `list[T]` → multiple results stored (e.g., multiple detections per frame)
- Returns `None` or `[]` → nothing stored for this input (e.g., no detections)
- `parent_id` set automatically from source row

## Examples

### VLM detections on images

```python
images = session.stream("images", Image,
                        pose_provider=lambda: tf.get_pose("world", "base_link"))

detections = session.stream("cigarette_detections", VLMDetection)

# Backfill + live
detections.transform(images, fn=lambda img: vlm.detect(img, "people with cigarettes"))

# After this, every new image.append() triggers detection automatically
# All results are queryable
rows = detections.query().filter_after(one_hour_ago).fetch()
```

### Live-only (skip backfill)

```python
detections.transform(images, fn=detect_fn, live=True)
# Only processes images appended from now on
```

### Historical batch on query results

```python
# Only process images from the kitchen in the last hour
kitchen_images = images.query().filter_near(kitchen_pose, 5.0).filter_after(one_hour_ago).fetch_set()

detections.transform(kitchen_images, fn=lambda img: vlm.detect(img, "cigarettes"))
# Batch processes the set, no live subscription
```

### Embedding stream (specialized)

```python
img_emb = session.embedding_stream("img_emb", model=CLIPModel())

# fn is implicit — uses model.embed()
img_emb.transform(images, live=True)

# Equivalent to:
img_emb.transform(images, fn=lambda img: clip.embed(img), live=True)
```

### Chaining transforms

```python
images = session.stream("images", Image, pose_provider=pose_fn)

# Embeddings from images
img_emb = session.embedding_stream("img_emb", model=CLIPModel())
img_emb.transform(images, live=True)

# Detections from images
detections = session.stream("detections", VLMDetection)
detections.transform(images, fn=detect_fn, live=True)

# Text descriptions from detections (second-level derived)
descriptions = session.text_stream("descriptions", str)
descriptions.transform(detections, fn=lambda det: det.describe(), live=True)
```

## Internals

### Backfill (batch)

```python
for page in source.iter_meta(page_size=128):
    for row in page:
        payload = source.load(row)        # or row.data
        results = fn(payload)
        if results is None:
            continue
        if not isinstance(results, list):
            results = [results]
        for r in results:
            self.append(r, ts=row.ts, pose=row.pose, parent_id=row.id)
```

### Live (reactive)

```python
source.appended.pipe(
    ops.map(lambda row: (row, fn(row.data))),
    ops.filter(lambda pair: pair[1] is not None),
    ops.flat_map(lambda pair: [
        (pair[0], r) for r in (pair[1] if isinstance(pair[1], list) else [pair[1]])
    ]),
).subscribe(lambda pair: self.append(pair[1], ts=pair[0].ts, pose=pair[0].pose,
                                      parent_id=pair[0].id))
```

### EmbeddingStream override

```python
class EmbeddingStream(StreamBase[T]):
    model: EmbeddingModel

    def transform(self, source, fn=None, *, live=False):
        if fn is None:
            fn = self.model.embed
        return super().transform(source, fn, live=live)
```

## Lineage

`transform()` sets `parent_id` on every appended row, linking back to the source row. This enables `project_to()`:

```python
# Find source images for cigarette detections
with detections.query().fetch_set() as det_set:
    source_images = det_set.project_to(images)
    for row in source_images.rows(limit=5):
        img = images.load(row)
```

## Open questions

1. **Async transforms?** VLM inference is slow. Should `fn` support async/await or rx scheduling (e.g., `observe_on(io_scheduler)`)?

2. **Error handling?** If `fn` raises on one row, skip it? Log and continue? Configurable?

3. **Backfill progress?** For large backfills, should `transform()` return a progress observable or run in background?

4. **Multiple parents?** Current design is single-parent lineage. If a stream derives from two streams (e.g., fusing image + audio), we'd need multi-parent support. Phase 3.

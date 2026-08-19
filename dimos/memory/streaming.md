# Stream evaluation model

Stream methods fall into three categories: **lazy**, **materializing**, and **terminal**. The distinction matters for live (infinite) streams.

`is_live()` walks the source chain to detect live mode — any stream whose ancestor called `.live()` returns `True`.
All materializing operations and unsafe terminals check this and raise `TypeError` immediately rather than silently hanging.

## Lazy (streaming)

These return generators — each observation flows through one at a time. Safe with live/infinite streams. No internal buffering between stages.

| Method                                                                    | How                                             |
|---------------------------------------------------------------------------|-------------------------------------------------|
| `.after()` `.before()` `.time_range()` `.at()` `.near()` `.filter_tags()` | Filter predicates — skip non-matching obs       |
| `.filter(pred)`                                                           | Same, user-defined predicate                    |
| `.transform(xf_or_fn)` / `.map(fn)`                                       | Generator — yields transformed obs one by one   |
| `.search_text(text)`                                                      | Generator — substring match filter              |
| `.limit(k)`                                                               | `islice` — stops after k                        |
| `.offset(n)`                                                              | `islice` — skips first n                        |
| `.live()`                                                                 | Enables live tail (backfill then block for new) |

These compose freely. A chain like `.after(t).filter(pred).transform(xf).limit(10)` pulls lazily — the source only produces what the consumer asks for.

## Materializing (collect-then-process)

These **must consume the entire upstream** before producing output. On a live stream, they raise `TypeError` immediately.

| Method             | Why                                          | Live behaviour |
|--------------------|----------------------------------------------|----------------|
| `.search(vec, k)`  | Cosine-ranks all observations, returns top-k | TypeError      |
| `.order_by(field)` | `sorted(list(it))` — needs all items to sort | TypeError      |

On a backend-backed stream (not a transform), both are pushed down to the backend which handles them on its own data structure (snapshot). The guard only fires when these appear on a **transform stream** whose upstream is live — detected via `is_live()`.

### Rejected patterns (raise TypeError)

```python
# TypeError: search requires finite data
stream.live().transform(Embed(model)).search(vec, k=5)

# TypeError: order_by requires finite data
stream.live().transform(xf).order_by("ts", desc=True)

# TypeError (via order_by): last() calls order_by internally
stream.live().transform(xf).last()
```

### Safe equivalents

```python
# Search the stored data, not the live tail
results = stream.search(vec, k=5).to_list()

# First works fine (uses limit(1), no materialization)
obs = stream.live().transform(xf).first()
```

## Terminal (consume the iterator)

Terminals trigger iteration and return a value. They're the "go" button — nothing executes until a terminal is called.

| Method            | Returns             | Memory             | Live behaviour                          |
|-------------------|---------------------|--------------------|-----------------------------------------|
| `.to_list()`        | `list[Observation]` | Grows with results | TypeError without `.limit()` first      |
| `.drain()`        | `int` (count)       | Constant           | Blocks forever, memory stays flat       |
| `.drain_thread()` | `DisposableBase`    | Constant           | Runs on the dimos thread pool           |
| `.first()`        | `Observation`       | Constant           | Returns first item, then stops          |
| `.exists()`       | `bool`              | Constant           | Returns after one item check            |
| `.last()`         | `Observation`       | Materializes       | TypeError (uses order_by internally)    |
| `.count()`        | `int`               | Constant           | TypeError on transform streams          |

`.save(target)` is **not** a terminal — it's a lazy pass-through that appends each
observation to ``target``'s backend as the stream is iterated. Pair it with
``.drain()`` (sync) or ``.drain_thread()`` (background) to actually run the pipeline.

### Choosing the right terminal

**Batch query** — collect results into memory:
```python
results = stream.after(t).search(vec, k=10).to_list()
```

**Live ingestion** — process forever, constant memory:
```python
# Embed and store continuously on the dimos thread pool
handle = stream.live().transform(EmbedImages(clip)).save(target).drain_thread()
# handle is a DisposableBase — dispose() to stop

# Side-effect pipeline (no storage)
stream.live().transform(process).drain()
```

**One-shot** — get a single observation:
```python
obs = stream.live().transform(xf).first()   # blocks until one arrives
has_data = stream.exists()                    # quick check
```

**Bounded live** — collect a fixed number from a live stream:
```python
batch = stream.live().limit(100).to_list()     # OK — limit makes it finite
```

### Error summary

All operations that would silently hang on live streams raise `TypeError` instead:

| Pattern                             | Error                                         |
|-------------------------------------|-----------------------------------------------|
| `live.transform(xf).search(vec, k)` | `.search() requires finite data`              |
| `live.transform(xf).order_by("ts")` | `.order_by() requires finite data`            |
| `live.to_list()` (without `.limit()`) | `.to_list() would collect forever`              |
| `live.transform(xf).count()`        | `.count() would block forever`                |
| `live.transform(xf).last()`         | `.order_by() requires finite data` (via last) |

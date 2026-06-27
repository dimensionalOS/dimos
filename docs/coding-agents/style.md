# Code Style Guidelines

Rules for writing code in dimos. These address recurring issues found in code review.

## No comment banners

Don't use decorative section dividers or box comments.

```python
# BAD
# ═══════════════════════════════════════════════════════════════════
#  1. Basic iteration
# ═══════════════════════════════════════════════════════════════════

# BAD
# -------------------------------------------------------------------
# Section name
# -------------------------------------------------------------------

# GOOD — just use a plain comment if a section heading is needed
# Basic iteration
```

If a file has enough sections to warrant banners, it should probably be split into separate files instead. For example, instead of one large `test_something.py` with banner-separated sections, create a `something/` directory:

```
# BAD
test_something.py  (500 lines with banner-separated sections)

# GOOD
something/
  test_iteration.py
  test_lifecycle.py
  test_queries.py
```

## No `__init__.py` re-exports

Never add imports to `__init__.py` files. Re-exporting from `__init__.py` makes imports too wide and slow — importing one symbol pulls in the entire package tree.

```python skip
# BAD — dimos/memory2/__init__.py
from dimos.memory2.store import Store, SqliteStore
from dimos.memory2.stream import Stream
```

```python
# GOOD — import directly from the module
from dimos.memory2.store.base import Store
from dimos.memory2.stream import Stream
```

## H.264 Image transport and storage shape

When editing H.264 image transport or memory2 storage, keep the public module
contract as `Out[Image]` and `In[Image]`. Do not expose RTP fragments to module
authors or memory2 observations.

`Image` is always decoded, pixel-addressable raster data; `Image.data` must stay
a NumPy array. For LCM transport and memory2 storage, H.264 bytes are an internal
physical representation. Each internal H.264 packet contains all NAL units for
exactly one source frame as one Annex B access unit. Store one memory2
observation per source frame, attach codec/keyframe metadata as internal tags,
and decode from a valid keyframe after sequence gaps, late join, or random seek.

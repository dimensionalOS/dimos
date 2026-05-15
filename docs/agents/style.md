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

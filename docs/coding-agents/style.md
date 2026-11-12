# Code Style Guidelines

Rules for writing code in dimos. These address recurring issues found in code review.

## Google docstrings

Use Google-style sections when a docstring needs structured information. Use
`Args:` for callable inputs, `Returns:` or `Yields:` for results, and `Raises:` for
errors callers need to handle. A short summary alone is enough when the signature
and name explain the contract.

Use `Attributes:` for fields of configuration models and dataclasses. Document
units, joint ordering, constraints, and other meanings the field name does not
express. Do not repeat the fields in a constructor `Args:` section unless a
handwritten constructor takes different inputs.

Keep types in Python annotations, rather than copying them into descriptions.
MkDocs is configured to parse Google docstrings and show existing annotations
when API references are added. This change does not add API reference pages.

Ruff checks recognized section headings for capitalization and a trailing colon
with `D405` and `D416`, using the Google convention. These rules do not reject
every NumPy, reST, or other docstring format. Missing docstrings and annotations
remain allowed by these checks; the existing mypy policy is unchanged.

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

# GOOD: just use a plain comment if a section heading is needed
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

Never add imports to `__init__.py` files. Re-exporting from `__init__.py` makes imports too wide and slow. Importing one symbol pulls in the entire package tree.

```python skip
# BAD: dimos/memory/__init__.py
from dimos.memory.store import Store, SqliteStore
from dimos.memory.stream import Stream
```

```python
# GOOD: import directly from the module
from dimos.memory.store.base import Store
from dimos.memory.stream import Stream
```

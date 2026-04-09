# Testing Guidelines

Rules for writing tests in dimos. These address recurring issues found in code review.

For grid testing (spec/impl tests across multiple backends), see [Grid Testing Strategy](/docs/development/grid_testing.md).

## Imports at the top

All imports must be at module level, not inside test functions.

```python
# BAD
def test_something() -> None:
    import threading
    from dimos.core.transport import pLCMTransport
    ...

# GOOD
import threading
from dimos.core.transport import pLCMTransport

def test_something() -> None:
    ...
```

## Always clean up resources

Use context managers or try/finally. If a test creates a resource, it must be cleaned up even if assertions fail.

```python
# BAD - store.stop() never called
def test_something() -> None:
    store = ListObservationStore(name="test", max_size=0)
    store.start()
    assert store.count(StreamQuery()) == 0

# BAD - module.stop() skipped if assertion fails
def test_wiring() -> None:
    module = MyModule()
    module.start()
    assert received == [84]
    module.stop()

# GOOD - context manager (ideal)
def test_something() -> None:
    store = ListObservationStore(name="test", max_size=0)
    with store:
        assert store.count(StreamQuery()) == 0

# GOOD - try/finally
def test_wiring() -> None:
    module = MyModule()
    module.start()
    try:
        assert received == [84]
    finally:
        module.stop()
```

When a resource is shared across multiple tests, use a pytest fixture with `yield` instead of repeating context managers in each test:

```python
# GOOD - fixture handles lifecycle for all tests that use it
@pytest.fixture(scope="module")
def store() -> Iterator[SqliteStore]:
    db = SqliteStore(path=str(DB_PATH))
    with db:
        yield db

def test_query(store: SqliteStore) -> None:
    assert store.stream("video", Image).count() > 0

def test_search(store: SqliteStore) -> None:
    results = store.stream("video", Image).limit(5).fetch()
    assert len(results) == 5
```

## No conditional logic in assertions

Tests must be deterministic. If you don't know the state, the test is wrong.

```python
# BAD - assertion may never execute
if hasattr(obj, "_disposables") and obj._disposables is not None:
    assert obj._disposables.is_disposed

# BAD - masks whether disposables were created
assert obj._disposables is None or obj._disposables.is_disposed

# GOOD - explicit about what we expect
assert obj._disposables is not None
assert obj._disposables.is_disposed
```

## Print statements

- **Unit tests**: no prints. Use assertions.
- **`@pytest.mark.tool` tests** (integration/exploration): prints are fine for progress and inspection output.

## Avoid unnecessary sleeps

Don't use `time.sleep()` to wait for async operations. Use `threading.Event` to synchronize emitter/receiver patterns.

```python
# BAD - arbitrary sleep, fragile
module.start()
time.sleep(0.5)
module.numbers.transport.publish(42)
time.sleep(1.0)
assert len(received) == 1

# GOOD - use threading.Event with a timeout
done = threading.Event()
unsub = module.doubled.subscribe(lambda msg: (received.append(msg), done.set()))
module.start()
module.numbers.transport.publish(42)
assert done.wait(timeout=5.0), f"Timed out, received={received}"
assert received == [84]
```

## Private fields

Configuration fields on non-Pydantic classes should be private (underscore-prefixed) unless they are part of the public API.

```python
# BAD
self.voxel_size = voxel_size
self.carve_columns = carve_columns

# GOOD
self._voxel_size = voxel_size
self._carve_columns = carve_columns
```

## Type ignores

Avoid `# type: ignore` by using proper types:

```python
# BAD
self.vbg = None  # type: ignore[assignment]

# GOOD - type as Optional
self.vbg: VoxelBlockGrid | None = VoxelBlockGrid(...)
# then later:
self.vbg = None  # no ignore needed
```

Type ignores are acceptable when caused by untyped third-party libraries (e.g. `open3d`) or decorator-generated attributes (e.g. `@simple_mcache` adding `invalidate_cache`).

# Grid Testing Strategy

Grid tests run the same test logic across multiple implementations or configurations using pytest's parametrize feature.

## Case Type Pattern

Define a `Case` dataclass that holds everything needed to run tests against a specific implementation:

```python
from collections.abc import Callable, Iterator
from contextlib import AbstractContextManager
from dataclasses import dataclass, field
from typing import Any, Generic

@dataclass
class Case(Generic[TopicT, MsgT]):
    name: str  # For pytest id
    pubsub_context: Callable[[], AbstractContextManager[...]]  # Context manager factory
    topic_values: list[tuple[TopicT, MsgT]]  # Pre-generated test data (always 3 pairs)
    tags: set[str] = field(default_factory=set)  # Capability tags for filtering

    def __iter__(self) -> Iterator[Any]:
        """Makes Case work with pytest.parametrize unpacking."""
        return iter((self.pubsub_context, self.topic_values))
```

## Capability Tags

Use tags to indicate what features each implementation supports:

```python
testcases = [
    Case(
        name="lcm_typed",
        pubsub_context=lcm_typed_context,
        topic_values=[...],
        tags={"all", "glob", "regex"},  # LCM supports all pattern types
    ),
    Case(
        name="shm_pickle",
        pubsub_context=shm_context,
        topic_values=[...],
        tags={"all"},  # SharedMemory only supports subscribe_all
    ),
]
```

## Filtered Test Lists

Build separate lists for each capability to use with parametrize:

```python
all_cases = [c for c in testcases if "all" in c.tags]
glob_cases = [c for c in testcases if "glob" in c.tags]
regex_cases = [c for c in testcases if "regex" in c.tags]
```

## Test Functions

Use the filtered lists in parametrize decorators:

```python
@pytest.mark.parametrize("case", all_cases, ids=lambda c: c.name)
def test_subscribe_all(case: Case) -> None:
    with case.pubsub_context() as pubsub:
        # Test logic using case.topic_values
        ...

@pytest.mark.parametrize("case", glob_cases, ids=lambda c: c.name)
def test_subscribe_glob(case: Case) -> None:
    if not glob_cases:
        pytest.skip("no implementations support glob")
    with case.pubsub_context() as pubsub:
        ...
```

## Context Managers

Each implementation provides a context manager factory:

```python
@contextmanager
def lcm_typed_context() -> Generator[LCM, None, None]:
    lcm = LCM(autoconf=True)
    lcm.start()
    yield lcm
    lcm.stop()
```

## Test Data Guidelines

- Always provide exactly 3 topic/value pairs for consistency
- For typed implementations, use different types per topic to verify type handling
- For bytes implementations, use simple distinguishable byte strings

```python
# Typed test data - different types per topic
typed_topic_values = [
    (Topic("/sensor/position", Vector3), Vector3(1, 2, 3)),
    (Topic("/sensor/orientation", Quaternion), Quaternion(0, 0, 0, 1)),
    (Topic("/robot/pose", Pose), Pose(...)),
]

# Bytes test data
bytes_topic_values = [
    (Topic("/topic1"), b"msg1"),
    (Topic("/topic2"), b"msg2"),
    (Topic("/topic3"), b"msg3"),
]
```

## Examples

- `dimos/protocol/pubsub/test_spec.py` - Basic pubsub operations
- `dimos/protocol/pubsub/test_subscribe_all.py` - Pattern subscriptions
- `dimos/protocol/pubsub/benchmark/testdata.py` - Benchmark cases

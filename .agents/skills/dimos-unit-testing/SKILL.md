---
name: dimos-unit-testing
description: DimOS unit tests: Use when writing, fixing, or reviewing pytest tests, fixtures, mocks, or test PR feedback.
---

# DimOS Unit Testing

Use this skill before adding, changing, or reviewing DimOS Python tests. The goal is **hermetic** tests: behavior-focused, deterministic, isolated, and cheap to run.

Consult these only when the branch needs more context:

- `docs/coding-agents/testing.md`
- `docs/coding-agents/code-quality-rules.md`
- `docs/development/testing.md`
- `misc/auto-fixes/fix_template.md`

## Steps

1. **Scope the behavior.** Identify the code under test, the caller-visible outcome, and the smallest test file next to it: `dimos/core/foo.py` gets `dimos/core/test_foo.py`. Completion: every new or changed test has a named behavior target and lives beside the code it covers.
2. **Build a hermetic setup.** Use module-level imports, Arrange-Act-Assert structure, fixtures for shared or resource-owning setup, and context managers for one-test local resources. Completion: setup owns all cleanup, restores global state, and contains no fixed sleeps.
3. **Assert the contract.** Use small examples and exact expected values. Completion: every test has an unconditional assertion that proves behavior a caller depends on.
4. **Mock the boundary.** Mock only slow, nondeterministic, or external boundaries. Completion: patches use `mocker.patch`, `mocker.patch.object`, or `monkeypatch`; no direct method assignment or `__new__` fixture shells remain.
5. **Validate tightly.** Run the smallest command that can fail for the change, then broaden only when the edit justifies it. Completion: the relevant pytest command has run, and mypy/pre-commit have run when source typing or broad quality gates changed.

## Test shape

- Test behavior, not implementation trivia. A useful test proves an outcome the user or caller depends on.
- Use descriptive test names and Arrange-Act-Assert ordering.
- Keep all imports at module level. Do not import inside test functions unless there is a documented circular-import reason.
- Prefer `assert result == expected` over shape-only checks.
- Do not add no-value tests that only prove a dataclass stored constructor arguments or that a default equals itself.

## Fixtures and cleanup

- Use `yield` fixtures so teardown runs even when assertions fail.
- Use `tmp_path` for temporary files and directories.
- Clean up modules, stores, servers, transports, subscriptions, sessions, threads, subprocesses, and global state.
- Do not put `stop()`, `close()`, or cleanup calls at the end of a test body after assertions; an earlier failure skips them.
- If a resource supports a context manager and the setup is local to one test, use `with`.

## Assertions

- Every test needs a meaningful assertion.
- Do not print in unit tests. Replace prints with assertions.
- Avoid conditional assertions. If the test says `if hasattr(...)`, it probably does not know what it is testing.
- For async or threaded behavior, wait for a condition with a timeout, such as `threading.Event`, then assert the final values. Do not use fixed `time.sleep()` waits.

## Mocking

- Prefer `mocker.patch(...)` or `mocker.patch.object(...)` for patches and call assertions.
- Use `monkeypatch` for environment variables, paths, and module attributes that should be restored automatically.
- Prefer real lightweight value objects over mocks for dataclasses and simple message objects.
- Do not replace methods by direct assignment when a patch or spy gives clearer assertions and automatic cleanup.
- Do not build fake modules out of custom classes when standard pytest mocking can express the same behavior.
- Do not construct objects with `__new__` and fill in fields by hand. Construct normally and patch the one side effect you need to avoid.

For call assertions, use the standard pattern: patch the target with `mocker.patch.object(...)`, run the behavior, then assert with `assert_called_once_with(...)` or another precise mock assertion.

## Types in tests

- Do not add casts or `# type: ignore` only to satisfy a test file.
- Do not copy legacy `type: ignore` patterns from older code.
- If a type problem points to source code, fix the source type instead of hiding it in the test.
- A rare `# type: ignore[...]` is acceptable only when narrow, justified, and caused by something mypy cannot know, such as an untyped third-party library or decorator-generated attribute.
- Avoid `Any` unless the value can genuinely be anything and a narrower type would be dishonest.

## Global state

- Use `monkeypatch.setenv`, `monkeypatch.delenv`, or fixtures instead of direct `os.environ[...]` mutation.
- Restore `global_config` or other shared state after changing it.
- A shared fixture must leave the system in the same state for the next test.

## Anti-patterns

| Avoid | Prefer |
| --- | --- |
| Repeating long setup blocks | A fixture with clear setup and teardown |
| Assertions that only check shape | Small examples with exact expected values |
| `print()` in a unit test | Assertions that prove the condition |
| `time.sleep()` for synchronization | Event, queue, polling helper, or timeout-based wait |
| Conditional assertions | Explicit setup and unconditional expectations |
| Direct method assignment for mocks | `mocker.patch` / `mocker.patch.object` |
| `__new__` plus manual fields | Normal construction plus one patch |
| Skipping when a normal dev dependency is missing | Fail clearly so the broken setup is fixed |

## Validation

Focused test edit:

```bash
uv run pytest dimos/path/to/test_file.py -k test_name
```

Explicit default marker filter:

```bash
uv run pytest dimos/path/to/test_file.py -k test_name -m 'not (tool or self_hosted or mujoco or self_hosted_large)'
```

Broader local check:

```bash
./bin/pytest-fast
```

Run `uv run mypy` when source typing changed or before a broader quality gate. Run `pre-commit run --all-files` for broad/autofix/final review work, not after every small unit-test edit.

## Final self-check

Before stopping, summon Paul: assume he will review every changed test and comment on each avoidable mistake.

- Is the test next to the code it tests?
- Does it prove behavior with real assertions?
- Are imports at the top?
- Are resources cleaned by fixtures or context managers?
- Are mocks minimal and managed by `mocker` or `monkeypatch`?
- Is the test deterministic without fixed sleeps or conditional assertions?
- Did you run the smallest useful pytest command?

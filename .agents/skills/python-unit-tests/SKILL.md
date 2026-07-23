---
name: python-unit-tests
description: Use when writing, fixing, or reviewing Python pytest unit tests, fixtures, mocks, or test PR feedback.
---

# Python Unit Tests

Use this skill before adding, changing, or reviewing Python unit tests. The goal is **hermetic** tests: behavior-focused, deterministic, isolated, and cheap to run.

Consult these only when the branch needs more context:

- `docs-old/coding-agents/testing.md`
- `docs-old/coding-agents/code-quality-rules.md`
- `docs-old/development/testing.md`
- `misc/auto-fixes/fix_template.md`

## Steps

1. **Scope the behavior.** Identify the code under test, the caller-visible outcome, and the smallest test file next to it: `dimos/core/foo.py` gets `dimos/core/test_foo.py`. Completion: every new or changed test has a named behavior target and lives beside the code it covers.
2. **Build a hermetic setup.** Use module-level imports, Arrange-Act-Assert structure, fixtures for shared or resource-owning setup, and context managers for one-test local resources. Completion: setup owns all cleanup, restores global state, reuses existing fixtures when they match, and contains no fixed sleeps.
3. **Assert the contract.** Use small examples and exact expected values. Completion: every test has an unconditional assertion that proves behavior a caller depends on.
4. **Mock the boundary.** Mock only slow, nondeterministic, or external boundaries. Completion: patches use `mocker.patch`, `mocker.patch.object`, or `monkeypatch`; no direct method assignment or `__new__` fixture shells remain.
5. **Validate tightly.** Run the smallest command that can fail for the change, then broaden only when the edit justifies it. Completion: the relevant pytest command has run, and mypy/pre-commit have run only when production source typing or broad quality gates changed.

## Test shape

- Test behavior, not implementation trivia. A useful test proves an outcome the user or caller depends on.
- Use descriptive test names and Arrange-Act-Assert ordering.
- Keep all imports at module level. Do not import inside test functions unless there is a documented circular-import reason.
- Prefer `assert result == expected` over shape-only checks.
- Do not add no-value tests that only prove a dataclass stored constructor arguments or that a default equals itself.
- Do not add test-only type annotations unless they clarify the test.

## Fixtures and cleanup

- Prefer cleanup in fixtures. Keep resource setup and teardown together so teardown runs even when assertions fail.
- Before adding a fixture, check whether matching setup already exists. Reuse or extract a shared fixture when the behavior matches; do not force reuse when the resemblance is only partial.
- Use `tmp_path` for temporary files and directories.
- Clean up modules, stores, servers, transports, subscriptions, sessions, threads, subprocesses, and global state.
- Do not put `stop()`, `close()`, or cleanup calls at the end of a test body after assertions; an earlier failure skips them.
- If a resource supports a context manager and the setup is local to one test, use `with`.
- In rare cases where setup and teardown both belong in the test body, use `try`/`finally`.

## Assertions

- Every test needs a meaningful assertion.
- Do not over-assert. Assert what matters for the behavior under test, not incidental details.
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

## Global state

- Use `monkeypatch.setenv`, `monkeypatch.delenv`, or fixtures instead of direct `os.environ[...]` mutation.
- Restore `global_config` or other shared state after changing it.
- A shared fixture must leave the system in the same state for the next test.

## Validation

Focused test edit:

```bash
uv run pytest dimos/path/to/test_file.py -k test_name
```

Explicit default marker filter:

```bash
uv run pytest dimos/path/to/test_file.py -k test_name -m 'not (self_hosted or mujoco or self_hosted_large)'
```

Broader local check:

```bash
./bin/pytest-fast
```

Do not run mypy for test-only edits. Run `uv run mypy` when production source typing changed or before a broader quality gate. Run `pre-commit run --all-files` for broad/autofix/final review work, not after every small unit-test edit.

## Final self-check

- Is the test next to the code it tests?
- Does it prove behavior with real assertions?
- Are imports at the top?
- Are resources cleaned by fixtures or context managers?
- Are mocks minimal and managed by `mocker` or `monkeypatch`?
- Is the test deterministic without fixed sleeps or conditional assertions?
- Did you run the smallest useful pytest command?

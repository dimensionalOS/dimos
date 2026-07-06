---
title: "Code Quality Rules"
---

Rules dimos code is expected to follow. They address recurring issues found in code review. The automated scan/fix prompts in `misc/auto-fixes/` are built from this file, but it's meant to be reused by any prompt that needs the project's code-quality criteria.

## Architecture and separation of concerns

* Something must own lifecycle. With no root object owning things, it's unclear where shared resources (thread pools, engines, transports) start, get used, and shut down. Prefer a root object that creates and shuts things down over global mutable state.
* Avoid global mutable state / singletons / registries. Separate processes don't share them anyway, so you get duplicate heavy instances (e.g. MuJoCo).
* Don't reach into another type's internals from the outside.
* Group state that changes together into one object instead of parallel fields that must be kept consistent (e.g. a `_Session` dataclass holding client/lock/state), so methods don't null-check each field.
* Don't put unrelated things in one class: 2D and 3D detection should be separate.
* Keep message classes as lightweight structs with small, un-opinionated utilities. Heavy/opinionated external-lib conversions (e.g. `to_rerun`, occupancy-grid -> RGB) belong in a dedicated module (e.g. `dimos/msgs/conversions/rerun/`), which also keeps `import rerun` out of message modules.

## Blueprints

* Specify only what differs from defaults. Don't restate defaults like `tick_rate=100.0`, `publish_joint_state=True`, or default topics (`/cmd_vel`, `/odom`).
* `.transports({...})` applies to all matching modules, so define a remap once, not twice across sub-blueprints.
* No lambdas -- they can't be pickled to worker processes. Use named functions.
* Do no work at import time: no subprocesses, viewers, model parsing, or network. In particular don't call `get_data(...)` (it blocks import until the download finishes) -- use `LfsPath` (resolved at access time) or build the config in `start`/`build`. Any process you start must be managed (shut down when not needed).
* Blueprint files define blueprints, not modules/classes.
* Helper blueprints not meant to run alone must start with `_` (the `all_blueprints.py` generator skips them); demo/non-shared ones get a `demo_` prefix (hidden from `dimos list`).

## Concurrency and thread safety

* Boolean stop/started flags (`self._running`, `self._started`, `self._shutdown`, ...) aren't thread safe. Use a `threading.Event`.
* Any state touched by more than one thread needs a lock everywhere -- the read side too, not just writes. `@rpc` methods run on different threads, so their state needs a lock.
* Lazy "initialize on first access" can race across threads. Guard it with a lock.
* Don't acquire and release the same lock twice in a row when one critical section would do. Use a reentrant lock where required.
* `list(some_collection)` to snapshot while another thread mutates only works because `list` holds the GIL. Python is moving off the GIL, so use a lock (or `Condition`) instead.
* When you pair an `Event` with a value just to hand data between threads, use a `queue.Queue`.
* Use `SequentialIds` for thread-safe incrementing ids instead of a bare `self._next_id`.
* Prefer async modules and `await` over `asyncio.run_coroutine_threadsafe` and grabbing event loops by hand. With auto-bound handlers you declare `async def handle_color_image(...)` and it listens automatically -- no register/unregister, no loop juggling. Don't create a new loop (`asyncio.new_event_loop()`) when the module already has `self._loop`.

## Configuration

* Don't use environment variables for what the config/CLI system already handles. Config values override from the CLI (`-o module.param=value`).
* Don't bake personal/hardware config into source or blueprints: IPs (`192.168.x.x`), interface names (`enp86s0`), default IPs in constructors. Use a `GlobalConfig` field with a sensible default (often `None`) and set it via `.env` or CLI.
* Type required fields as required, not `... | None = None`. Then you drop the runtime None-check and the `or default` / `or ""` / `or "can0"` patterns.
* Listen on `global_config.listen_host` by default, not `0.0.0.0`.

## Constants, magic numbers, and centralizing values

* Name magic numbers.
* A value used in two places (including defaults) must be one shared constant so the two can't drift.
* Reuse existing constants instead of recomputing (`DIMOS_PROJECT_ROOT`, `STATE_DIR`, `CACHE_DIR`, `_DEFAULT_LCM_URL`) and respect XDG dirs (`get_user_data_dir`, `XDG_STATE_HOME`/`XDG_DATA_HOME`) rather than hardcoding `~/.local/...` or `~/.cache/...`.

## Constructors and object initialization

* Don't do work or start things in `__init__` (no autostart). The constructor sets the object up; it doesn't run it. Doing work there:
  - makes the object hard to pass around (it's already running),
  - makes it hard to subclass (no hook between `super().__init__()` and starting),
  - leaves it in an invalid, hard-to-clean-up state if it fails.
  Move the work to `start()` (or `build()`).

## Dead, commented-out, and stale code

* Remove unused functions, methods, variables, and imports.
* Delete commented-out code (imports, blocks, dependency lines) rather than leaving it.
* Remove stale comments/docstrings describing behavior the code no longer has.
* Drop `#!/usr/bin/env python3` from files that aren't executables.

## Documentation

* Don't over-document. Long docs go unread, and values copied into them (max speeds, defaults, unit conversions) drift from the code and go wrong -- point at the code instead. Don't document trivia.
* Code examples in docs must run and follow best practices, because people and LLMs copy them.

## Duplication and reuse

* Before adding a utility, check it doesn't already exist; if it does, reuse it.
* Extract identical code in multiple places into a shared function.

## Error handling

* Wrap only the specific lines that can fail, and catch the specific exception you expect -- not a whole function in `except Exception`. A large function under a broad except hides where each failure comes from.
* Don't wrap things in try/except "just in case." If you can't name the exception, you probably shouldn't catch.
* Don't silence errors (`except: pass`, `except Exception: scene = None`, downgrading an exception to a warning). If something failed, surface it.
* Don't bury real failures in `logger.debug` (hidden by default). Use at least `warning`, and `logger.exception` to keep the traceback.
* Don't return a placeholder (e.g. position `0.0`) when you don't know the value -- raise instead.
* Use `assert cond, "message"` instead of a silent `if code == 0:` when you actually require the condition.

## Imports

* Put imports at the top. Inline imports are only for breaking a circular import or lazily loading a genuinely heavy/optional dependency (torch, rerun) -- say which in a comment. (Tests put all imports at top; everything gets imported anyway.)
* No `from x import *` -- star imports defeat the linter's name checking. Import each name explicitly.
* Don't import from `conftest.py` or other test files -- it breaks pytest subtly. Expose shared things as fixtures.
* Imports must have no side effects (e.g. initializing rerun). Make initialization an explicit function the caller invokes.

## Inheritance vs composition

* Prefer composition over inheritance.
* When all of a class's methods are abstract, a Protocol often fits better than an ABC.

## Logging

* Use the project logger: `logger = setup_logger()` at the top of the file, not `logging.getLogger(__name__)`.
* We use structlog -- pass structured key/values (`logger.info("Camera not found", camera_name=name)`) rather than stuffing everything into an f-string.
* Remove no-value log lines ("Module started", "Stopping module...", per-frame debug logs); they're noise, and debug is hidden by default anyway. Someone debugging can add a temporary print.

## Naming

* Don't access or import another module's `_`-prefixed members. If you have to reach into `obj._x` from outside, it isn't really private -- make it public or add an accessor.

## Performance

* Don't loop in Python over numpy data (slow). Use vectorized/broadcast NumPy ops.
* A plain `time.sleep(period)` loop misses the target frequency because it ignores the loop body's time. Subtract elapsed: `time.sleep(max(0, period - elapsed))`, or track a `next_time`.
* A busy loop (`spin_once(timeout_sec=0)`, or a tight retry with no sleep) burns 100% CPU while idle.
* Cache things recomputed every call. Copying a whole collection on every access (`list(self.message_history)` each iteration) wastes time/memory -- prefer a lock over snapshotting.
* Don't spawn threads needlessly.

## Repository hygiene and data files

* `dimos/` is for source code only; non-code files don't belong there.
* Dependencies:
  - Don't ship deps that block PyPI publishing (git-URL deps, unpublished packages).
  - Put deps in the right extra group, and include new groups in `all` where expected.
  - Comment non-obvious deps (e.g. why `bitsandbytes` is needed).
* Don't write state/output files to the repo root or `data/` (that's for static LFS data). Use a state dir (`STATE_DIR`, `~/.local/state/dimos`, XDG). Runtime files that must live in the tree use the `.ignore.*` convention so they're git-ignored.
* Put project scripts in `pyproject.toml` `[project.scripts]` instead of adding to `bin/`.

## Resource lifecycle and cleanup

* Every `.subscribe()` returns a disposable that must be disposed -- usually `self.register_disposable(...)` so it's disposed on `self.stop`. When mypy sees a plain function, wrap it: `self.register_disposable.add(Disposable(stream.subscribe(handler)))`. Example/doc code must unsubscribe too, because people and LLMs copy it.
* If `start()` opens subscriptions/threads/publishers, add a `stop()` that closes them. When you override `start`/`stop`/`__exit__`/etc. you almost always must call `super().<method>()`.
* `_close_module` is a leftover; `Module.stop` now calls it, so move that code into `stop()` rather than calling `_close_module` directly.
* During shutdown, keep stopping the other modules even if one errors.
* Track and join threads; don't leak them. Hold a reference to a thread you start so you can join it -- and check a join/stop isn't coming from the thread it's trying to join.
* Transports and publishers (`LCMTransport`, `pLCMTransport`, ...) are often created but never closed; add a `stop()`. But a transport shared between modules shouldn't be closed by one of them.
* Shut down subprocesses, in tests always via a fixture so they die even on failure. Cleanup belongs in real code, not test code -- tests should only call the cleanup utilities.

## Tests

* A test with no assertions isn't a test. Sleeping after startup, or printing, isn't testing -- replace prints with assertions (and don't `print()` in tests at all).
* Assert real values, not just shapes: make arrays small and assert the actual numbers; if the test computes something (e.g. doubling), assert the result.
* No-value tests to avoid: that a dataclass stores the values you passed, that a default equals its default, or a negative that can't happen ("stop does not publish a click").
* Anything needing cleanup (models, modules, stores, servers, subprocesses, sessions, temp files) belongs in a fixture so it's torn down even on failure. Don't call `stop()`/`close()` at the end of the body or hand-roll try/finally. An `assert` mid-test skips the cleanup below it -- save the boolean and assert after teardown, or use a fixture. Pull heavy/repeated setup into fixtures, reuse existing ones rather than duplicating, and don't copy whole test bodies between files.
* Use pytest's `tmp_path`, not `tempfile.mkstemp`. Use the `mocker` fixture (pytest-mock) instead of nested `with patch(...)` blocks.
* Don't mutate global state without reverting: `os.environ["CI"] = "1"` (or `DISPLAY`) at import/in a test, or `global_config.update(...)`, leaks into every later test. A test that mutates a shared fixture must clean up, or the fixture shouldn't be shared.
* Tests should be deterministic, with no conditional logic. If a test checks whether an object has an attribute, it doesn't know what it's testing.
* Don't wait with a fixed `time.sleep` -- poll for the condition with a timeout and fail if it never happens. Fixed sleeps are slow and flaky in CI.
* Mock surgically. Over-mocking (recreating a whole engine) makes tests fragile -- mock only the few problematic lines and let the real constructor run. A dataclass (`DetObject`/`Object`) only holds values, so instantiate it; there's nothing to mock.
* Don't recreate an object with `__new__` and set every attribute by hand -- it duplicates `__init__` and breaks when `__init__` changes; actually instantiate it and patch the one thing you don't want. (Patching `__init__` then calling `__new__` is doubly pointless -- `__new__` doesn't call `__init__` anyway.)
* `if __name__ == "__main__": test_x()` style tests and non-pytest scripts have no value if nobody runs them.
* Put tests next to the code they test (except e2e and similar). Don't add a separate `tests/` dir.
* For developers, tests should fail (not skip) when a dependency is missing; skipping hides breakage.

## Type hints and mypy

* Don't reach for `# type: ignore` first -- understand why mypy complains and fix it; ignore only when it genuinely can't be fixed. The many existing ignores are tech debt, not a model. `# type: ignore[override]` is the worst -- it means the subclass breaks the parent's contract; fix the hierarchy. When an ignore "wasn't needed before," something regressed -- find out why.
* Many ignores vanish just by typing `**kwargs: Any` instead of leaving it untyped.
* Never use bare `np.ndarray`; use `NDArray[np.uint8]` (etc.) from `numpy.typing`.
* Use `Any`, not `object`, when a value can be anything (`**kwargs` too). `object` is restrictive -- you can't even add to it.
* Use `Literal[...]` instead of `str` plus a comment listing valid values: real checking, and typer can generate help/examples. Define a `TypeAlias` if it's reused.
* Put types in annotations, not docstrings -- mypy doesn't read docstrings.
* `hasattr` (and the `# type: ignore[attr-defined]` that follows) is almost always broken design: it breaks the abstraction and the type checker can't help. Define the method on a Protocol/base so every implementation provides it (often just `pass`); then you don't need the `hasattr` check before calling.
* `strict=False` is the default on `zip` but silences length mismatches -- you usually want `strict=True`.

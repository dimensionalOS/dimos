## 1. Coordinator attachment

- [x] 1.1 Add focused coordinator tests proving `ModuleCoordinator.loop()` starts its RPC service once, remains safe after an explicit start, and still performs normal blocking-loop cleanup.
- [x] 1.2 Make coordinator RPC startup idempotent and have `ModuleCoordinator.loop()` ensure the service is active without moving the CLI's explicit startup ahead of daemonization.
- [x] 1.3 Add porcelain tests proving `Dimos.connect()` can attach without a run-registry record, uses the existing default timeout, and reports an unavailable coordinator clearly.
- [x] 1.4 Remove the run-registry precondition from `Dimos.connect()` while retaining registry metadata as optional context.

## 2. Live RPC introspection

- [x] 2.1 Add RPC proxy tests for preserved names, docstrings, defaults, parameter kinds, annotations, return annotations, and omission of the leading instance parameter, including a names-only fallback case.
- [x] 2.2 Update `RpcCall` to expose the locally importable implementation method's metadata and signature without changing remote invocation behavior.
- [x] 2.3 Add structured-discovery tests covering live module additions, exact instance lookup, unique and ambiguous class lookup, multiple same-class instances, all advertised RPCs, module filtering, qualified descriptions, and concise representations.
- [x] 2.4 Extend the existing introspection records compatibly and add public `Dimos.list_modules()`, `get_module()`, `list_rpcs()`, and `describe()` operations backed by fresh coordinator descriptors on each discovery call.
- [x] 2.5 Verify existing `Dimos` indexing, `app.skills`, local/remote module sources, and introspection renderers remain compatible with the additive discovery API.

## 3. Interactive CLI

- [x] 3.1 Add IPython to the normal runtime dependencies and refresh `uv.lock`.
- [x] 3.2 Add CLI tests for `dimos shell` covering TTY rejection, successful attachment, the `app`/`modules`/`rpcs`/`describe` namespace, safety and unregistered-run banner text, default connection behavior, IPython exit, and guaranteed client disconnect.
- [x] 3.3 Implement the zero-argument `dimos shell` Typer command with private namespace/banner helpers, required IPython startup, and `try`/`finally` connection cleanup.
- [x] 3.4 Add a CLI failure-path test showing coordinator loss or connection failure is visible and does not trigger automatic reconnection.

## 4. Documentation

- [x] 4.1 Update `docs/usage/cli.md` with shell startup, TTY requirements, predefined names, direct RPC examples, the immediate-execution warning, and exit semantics.
- [x] 4.2 Update `docs/usage/python-api.md` with live discovery, exact-instance and unique-class lookup, `describe()`, ambiguity behavior, and standard Python/IPython RPC inspection.
- [x] 4.3 Update `AGENTS.md` quick-start and CLI reference, and keep `CONTEXT.md` and `docs/adr/0001-coordinator-loops-are-attachable.md` aligned with the implemented names and lifecycle.

## 5. Verification

- [x] 5.1 Run `openspec validate add-dimos-shell`.
- [x] 5.2 Run focused tests with `uv run pytest dimos/core/coordination/test_module_coordinator.py dimos/porcelain/test_dimos.py dimos/porcelain/test_remote_module_source.py dimos/robot/cli/test_dimos.py -v`.
- [x] 5.3 Run type and lint checks for the touched Python modules.
- [x] 5.4 Run `bin/doclinks` and `bin/run-doc-codeblocks docs/usage/cli.md docs/usage/python-api.md`, keeping live interactive transcripts explicitly non-executable.
- [x] 5.5 Manually launch a non-hardware test, simulation, or replay blueprint through `dimos run`, attach with `dimos shell`, inspect modules and signatures, invoke a harmless RPC, exit, and confirm the blueprint remains running.
- [x] 5.6 Manually build and enter a coordinator loop from Python without a CLI run-registry entry, attach with `dimos shell`, and confirm the banner identifies the run as unregistered.
- [x] 5.7 Confirm no blueprint or module registry inputs changed; if implementation changes that assumption, regenerate and verify `dimos/robot/all_blueprints.py`.

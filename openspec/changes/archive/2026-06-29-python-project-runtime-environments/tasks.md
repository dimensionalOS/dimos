## 1. Runtime model and resolution

- [x] 1.1 Add `PythonProjectRuntimeEnvironment(name, project)` to the runtime environment model.
- [x] 1.2 Detect uv-only versus Pixi-backed uv project conventions from `pyproject.toml` and optional `pixi.toml`.
- [x] 1.3 Validate required project files and emit actionable errors for missing `pyproject.toml` or missing prepared `.venv/bin/python`.
- [x] 1.4 Resolve non-mutating worker launch commands for uv-only and Pixi-backed project runtimes.
- [x] 1.5 Add unit tests for project runtime detection, validation, and launch command construction.

## 2. Runtime preparation CLI

- [x] 2.1 Add `dimos runtime prepare <blueprint> [--runtime <name>]` CLI entrypoint using blueprint loading semantics compatible with `dimos run`.
- [x] 2.2 Select only active placement runtime environments by default after evaluating disabled modules and runtime placements.
- [x] 2.3 Reject `--runtime <name>` when the name is unknown or not used by an active placement in the loaded blueprint.
- [x] 2.4 Implement uv-only prepare commands: `uv venv --seed` and `uv sync` from the project directory.
- [x] 2.5 Implement Pixi-backed prepare commands: `pixi install`, `pixi run uv venv -p .pixi/envs/default/bin/python --seed`, and `pixi run uv sync` from the project directory.
- [x] 2.6 Add CLI tests for active-runtime selection, command sequencing, copy/paste diagnostics, and repeated prepare behavior.

## 3. Worker launch integration

- [x] 3.1 Add a project-runtime worker launcher or extend the existing launcher abstraction to run command arrays instead of only direct Python executable paths.
- [x] 3.2 Wire `PythonProjectRuntimeEnvironment` placements through `WorkerManagerPython` without changing existing `PythonVenvRuntimeEnvironment` behavior.
- [x] 3.3 Ensure `dimos run` fails fast without syncing when a project runtime is missing `.venv/bin/python`.
- [x] 3.4 Preserve current shutdown semantics and add a regression proving project-runtime workers exit through the normal worker lifecycle.
- [x] 3.5 Add fallback termination coverage for a project-runtime worker that does not respond to normal shutdown.

## 4. GPD demo package

- [x] 4.1 Add `packages/dimos-gpd-worker-demo/` as an import-safe worker package with `pyproject.toml`, optional `pixi.toml`, and source package files.
- [x] 4.2 Pin the demo package dependency on `TomCC7/gpd` commit `c088d8ae2f7965b067e9a12b3c0dacdbe9da924a`.
- [x] 4.3 Add a dummy DimOS Module whose RPC lazily imports `gpd.core` inside the worker runtime and returns import success.
- [x] 4.4 Add a demo blueprint that registers `PythonProjectRuntimeEnvironment(name, project)` and places the dummy Module into it.
- [x] 4.5 Add uv-only always-on tests plus Pixi/GPD integration tests that skip when Pixi is unavailable.

## 5. Documentation and validation

- [x] 5.1 Update runtime environment docs with project-local `.venv`/`.pixi` conventions, explicit prepare, and non-mutating run semantics.
- [x] 5.2 Document the Pixi-backed uv command sequence and explain that users rerun prepare after manifest changes.
- [x] 5.3 Document the GPD demo command and expected import-success result.
- [x] 5.4 Run focused runtime environment, worker launcher, blueprint placement, CLI, and demo tests.
- [x] 5.5 Run `openspec validate python-project-runtime-environments --type change --strict --no-interactive` and fix validation issues.
- [x] 5.6 Final-review blocker: preserve capability-based Python runtime placement by routing non-project runtimes through `resolve_python()`, including the default `current` runtime and custom Python-capable runtime environments.

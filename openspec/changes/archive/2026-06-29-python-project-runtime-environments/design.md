## Context

DimOS now supports named Python runtime environments for venv module workers, but the existing backend expects a prepared Python executable path. That is enough for hand-built venvs, but it does not describe how package-local worker runtimes are prepared or how native/C++ dependencies enter the environment.

The target workflow is a package-local Python project. The package owns `pyproject.toml` for Python dependencies and may also own `pixi.toml` for native, C++, ROS, compiler, and toolchain dependencies. Pixi is optional. uv is required for the first slice. Runtime names remain scoped to the loaded blueprint configuration rather than globally addressable from the CLI.

## Goals / Non-Goals

**Goals:**
- Add a convention-only `PythonProjectRuntimeEnvironment(name, project)` model.
- Add `dimos runtime prepare <blueprint> [--runtime <name>]` that loads the blueprint, finds active placements, and prepares only the used project runtimes.
- Keep `dimos run` non-mutating and fail fast when a project runtime is not prepared.
- Support uv-only and Pixi-backed uv projects.
- Launch project-runtime workers through non-mutating toolchain commands.
- Add a real GPD demo package that proves a Python binding with native build requirements can be prepared and imported in a worker runtime.

**Non-Goals:**
- No Pixi-only runtime in the first slice; `pyproject.toml` remains required.
- No global `dimos runtime prepare <env-name>` command.
- No lockfile hash or timestamp staleness detection in the first slice.
- No automatic prepare during `dimos run`.
- No manual manifest path overrides; the API is convention-only.
- No attempt to run grasp detection in the GPD demo; import success is enough to validate environment preparation.

## Decisions

### Blueprint-scoped preparation

`dimos runtime prepare <blueprint> [run-like flags] [--runtime <name>]` will evaluate the blueprint using the same configuration path as `dimos run`. Runtime names are resolved only after the blueprint's `RuntimeEnvironmentRegistry` exists. By default, prepare targets only runtime environments referenced by active module placements. If `--runtime` is supplied, the name must refer to an active placement runtime in that blueprint configuration.

Alternative considered: `dimos runtime prepare <env-name>`. This was rejected because runtime names are not global and may only exist for a particular blueprint session.

### Convention-only Python project runtime model

The new model should be minimal:

```python
PythonProjectRuntimeEnvironment(
    name="roboplan-worker",
    project=Path("packages/roboplan-worker"),
)
```

The project directory conventions are:
- `pyproject.toml` is required.
- `pixi.toml` is optional.
- `.venv/` is the project-local uv environment.
- `.pixi/` is the project-local Pixi state when Pixi is used.

Alternative considered: explicit fields for `pyproject`, `pixi_manifest`, `venv`, and Pixi environment name. This was rejected for the first slice because the desired user experience is convention-driven and inspectable, not a per-tool configuration matrix.

### Prepare always syncs, run only verifies existence

Preparation is the only mutating phase. For uv-only projects it runs:

```bash
uv venv --seed
uv sync
```

For Pixi-backed uv projects it runs:

```bash
pixi install
pixi run uv venv -p .pixi/envs/default/bin/python --seed
pixi run uv sync
```

`dimos run` only verifies that the prepared runtime exists, starting with `.venv/bin/python`, and produces a prescriptive error if it is missing. It does not compare lockfile hashes or timestamps. Users rerun prepare when manifests change.

### Toolchain-mediated worker launch

Project-runtime workers launch through the project toolchain command rather than by reconstructing activation variables manually. For uv-only projects:

```bash
uv run --no-sync python -m dimos.core.coordination.venv_worker_entrypoint ...
```

For Pixi-backed uv projects:

```bash
pixi run uv run --no-sync python -m dimos.core.coordination.venv_worker_entrypoint ...
```

`--no-sync` is required because `dimos run` must not mutate the environment. Existing `PythonVenvRuntimeEnvironment` continues to launch a direct Python executable.

Alternative considered: capture Pixi activation environment and launch `.venv/bin/python` directly. This was rejected because the toolchain command better matches the user's Pixi+uv workflow and avoids hand-built activation semantics.

### Keep current shutdown semantics first

The current worker lifecycle sends a DimOS `ShutdownRequest`, waits for the worker to exit, then falls back to terminating the launched process handle. The first implementation should keep that behavior for toolchain-mediated workers. Process-group termination should be added only if a shutdown regression proves wrapper commands leave orphaned child processes.

### GPD demo as the native dependency proof

Add `packages/dimos-gpd-worker-demo/` with a `pyproject.toml` that depends on a pinned `TomCC7/gpd` commit:

```text
c088d8ae2f7965b067e9a12b3c0dacdbe9da924a
```

The package should include a `pixi.toml` for native build dependencies and a dummy DimOS Module whose RPC lazily imports `gpd.core`. The automated Pixi/GPD integration test should skip when Pixi is unavailable; uv-only and command construction tests should run normally.

## Risks / Trade-offs

- **Stale environments may pass `dimos run` checks** → Users must rerun `dimos runtime prepare` after changing `pyproject.toml`, `uv.lock`, `pixi.toml`, or `pixi.lock`. Docs and errors should state this clearly.
- **Pixi/GPD builds may be slow or unavailable in some CI contexts** → Gate Pixi integration tests with skip-if-missing Pixi and keep deterministic unit tests for command construction.
- **Toolchain wrapper shutdown could leave child processes** → Preserve current shutdown behavior first and add a process cleanup regression; escalate to process groups only if the regression fails.
- **GPD native dependencies may require package-name iteration** → Treat the GPD demo as an integration spike task with a pinned commit and documented Pixi dependencies.

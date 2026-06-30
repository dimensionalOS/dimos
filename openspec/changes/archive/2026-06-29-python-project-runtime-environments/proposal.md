## Why

Venv module workers can already run from a named Python executable, but users still need to prepare those environments by hand and duplicate tool-specific setup outside the blueprint workflow. Python worker packages that need C++ or ROS dependencies need a convention that combines uv Python packaging with optional Pixi native dependency preparation while keeping `dimos run` predictable.

## What Changes

- Add a convention-driven Python project runtime environment rooted at a package directory with a required `pyproject.toml` and optional `pixi.toml`.
- Add an explicit blueprint-scoped runtime preparation command that prepares only runtime environments used by active module placements.
- Keep `dimos run` non-mutating: it fails fast when a project runtime is not prepared instead of running uv or Pixi sync/install.
- Launch convention-based project workers through the project toolchain with non-mutating uv flags, such as `pixi run uv run --no-sync python ...` when Pixi is present.
- Preserve existing direct-interpreter `PythonVenvRuntimeEnvironment` behavior for already-prepared interpreter paths.
- Add a GPD worker demo package that uses a pinned `TomCC7/gpd` git dependency and Pixi native build environment, with a dummy Module that verifies `gpd.core` imports in the worker runtime.

## Capabilities

### New Capabilities
- `runtime-environment-preparation`: Blueprint-scoped explicit preparation of active runtime environments before `dimos run`.

### Modified Capabilities
- `runtime-environment-registry`: Add a convention-driven Python project runtime environment backend and non-mutating project-runtime resolution.
- `venv-module-placement`: Allow placed Modules to launch from project-runtime toolchain commands while preserving worker lifecycle semantics.
- `venv-module-packaging`: Extend packaging/demo coverage to Python packages with native/C++ build dependencies managed through optional Pixi plus uv.

## Impact

- Affected APIs: runtime environment Python models, blueprint runtime environment resolution, and DimOS CLI runtime subcommands.
- Affected worker launch code: project-runtime worker launcher command construction, startup diagnostics, and shutdown regression coverage.
- Affected packaging/docs: add docs for project-local `.venv`/`.pixi` conventions and a GPD demo package under `packages/`.
- New optional tool dependency for full integration tests: Pixi. Tests must skip Pixi/GPD integration when Pixi is unavailable.

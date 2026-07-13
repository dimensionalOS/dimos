## 1. External Runtime Foundation

- [ ] 1.1 Add the external-Python declaration contract with only an implementation import reference, preserving the declaration's normal configuration, streams, RPC methods, skills, and DimOS module-reference annotations.
- [ ] 1.2 Implement deterministic resolution and validation of the declaration's sibling `python/` runtime project, requiring `pyproject.toml` and detecting optional `pixi.toml` with actionable errors.
- [ ] 1.3 Implement managed local runtime preparation and launch commands: use uv for a uv project, or run uv through Pixi when `pixi.toml` is present; make lockfile and offline behavior explicit.
- [ ] 1.4 Implement the external-runtime bootstrap to import the implementation reference, validate that it fulfills the declaration contract, and serve RPC under the declaration identity.

## 2. Worker Lifecycle Integration

- [ ] 2.1 Integrate the private local external-Python worker behind `WorkerManagerPython` without adding deployment plans, targets, sessions, or coordinator-facing deployment APIs.
- [ ] 2.2 Preserve the existing coordinator deployment order so external declarations receive normal stream connections, module-reference injection, build, and start behavior.
- [ ] 2.3 Implement safe process-group cleanup, restart behavior, unexpected-exit reporting, and bounded stdout/stderr diagnostics for failed preparation and runtime termination.
- [ ] 2.4 Migrate `examples/external_python_module/` to the implementation-reference declaration and sibling runtime-project convention, removing planner- or deployment-spec setup while retaining the runnable local example.
- [ ] 2.5 Confirm the migrated example does not introduce a blueprint-registry input; do not regenerate `dimos/robot/all_blueprints.py` unless implementation adds a registry-discoverable blueprint.

## 3. Automated Coverage

- [ ] 3.1 Add focused tests for runtime-project resolution, mandatory `pyproject.toml`, optional Pixi detection, and the uv/Pixi command selection and diagnostics.
- [ ] 3.2 Add focused bootstrap tests for valid implementation loading, invalid import references, declaration-contract mismatch, declaration RPC identity, and failed-startup cleanup.
- [ ] 3.3 Add lifecycle tests covering composition with an ordinary module, typed stream/module-reference wiring, restart with a fresh process, and unexpected runtime exit reporting.

## 4. Documentation

- [ ] 4.1 Update `docs/usage/modules.md` with external-Python-module authoring: declaration API, sibling `python/` layout, required and optional manifests, Blueprint composition, and preparation/failure behavior.
- [ ] 4.2 Document Pixi and uv as layered environments and state that runtime Python dependencies must be declared in the runtime project's `pyproject.toml`.

## 5. Verification

- [ ] 5.1 Run `openspec validate local-external-python-modules`.
- [ ] 5.2 Run focused pytest targets for the external-runtime resolver, bootstrap, worker lifecycle, and migrated example.
- [ ] 5.3 Run `uv run mypy dimos/` and relevant Ruff checks for the changed Python code.
- [ ] 5.4 Run `md-babel-py run docs/usage/modules.md`.
- [ ] 5.5 Manually run the migrated external Python module example through the normal DimOS Blueprint surface and verify preparation, RPC/stream behavior, graceful stop, and restart.

## Why

The current branch contains small control, generated-registry, docs, and housekeeping changes mixed into the planning-group refactor. These should be peeled off so reviewers can decide whether they are required or accidental.

## What Changes

- Extract control/coordinator task and tick-loop changes if they are truly required.
- Extract generated blueprint registry updates if they are caused by earlier PRs.
- Extract remaining docs/readme cleanup and packaging/config touches.
- Report any file that appears accidental rather than including it blindly.

## Capabilities

### New Capabilities
- `control-docs-housekeeping`: Distribution plan for orthogonal control, generated, docs, and repo housekeeping changes.

### Modified Capabilities
- `control-coordinator-integration`: Control/coordinator behavior updates only if validated as required by the planning-group stack.

## Impact

- Base branch: preferably PR 4 `manipulation-module-group-api`, or `main` for truly independent control/docs changes.
- Reference implementation: `cc/spec/movegroup`.
- Primary candidate files: `dimos/control/*`, `dimos/e2e_tests/test_control_coordinator.py`, generated blueprint files, docs/readmes, `pyproject.toml`, `AGENTS.md`.
- Out of scope: planning-group core, planners, module API, and Viser UI.

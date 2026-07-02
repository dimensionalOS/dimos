## User-Facing Docs

- Add `docs/usage/runtime_environments.md` explaining:
  - Runtime Environment Registration
  - Runtime Placement
  - Runtime Project
  - Locked Runtime Project
  - Runtime Reconciliation
  - Module Contract and Runtime Implementation split
  - Python Runtime Worker semantics
  - lockfile-backed uv and Pixi-backed runtime projects
  - deployment-time failure modes and non-mutating lock behavior
- Update `docs/usage/README.md` to link the runtime environments guide.
- Include a minimal example that places a dependency-light Module Contract into a runtime project under `examples/dimos-demo-worker-module/`.

## Contributor Docs

- No separate contributor guide is required for the initial change.
- If implementation reveals test or release workflow details that are not user-facing, add a short note under `docs/development/` rather than expanding the usage guide.

## Coding-Agent Docs

- No `AGENTS.md` update is required unless implementation changes the standard blueprint or module authoring workflow for coding agents.
- Keep `CONTEXT.md` as glossary-only language for the runtime environment domain; do not turn it into an implementation spec.

## Doc Validation

- Run documentation link validation if available for the repository docs.
- Run any markdown/code-block validation used by DimOS docs for changed files, especially the new `docs/usage/runtime_environments.md` examples.
- At minimum, review rendered markdown headings and code snippets for the new usage guide and ensure `docs/usage/README.md` links resolve.

## No Docs Needed

Not applicable. This change adds public blueprint authoring concepts and deployment behavior, so user-facing docs are required.

## User-Facing Docs

No user-facing behavior change is intended. `docs/capabilities/manipulation/readme.md` should remain accurate for launch commands, optional dependency installation, safety guidance, and operator workflow.

If the implementation changes the key-file table or adds meaningful internal files worth documenting, update only the key-file row for `dimos/manipulation/viser_panel/` to describe the package as the optional Viser operator panel instead of implying a single-file implementation.

## Contributor Docs

No contributor docs are required unless the refactor establishes a reusable DimOS pattern for optional browser panels. If that happens, capture the pattern under `docs/development/` or `docs/coding-agents/` in a separate follow-up, not as part of this behavior-preserving refactor.

## Coding-Agent Docs

No `AGENTS.md` or `docs/coding-agents/` update is required for the initial refactor. If implementation uncovers a durable guidance point, such as a standard layout for Viser panel modules, propose it separately after the pattern is proven.

## Doc Validation

If no docs are changed, no doc-specific validation is needed beyond the normal pre-commit doclinks hook.

If `docs/capabilities/manipulation/readme.md` is updated, run the repository doclinks validation through pre-commit:

```bash
uv run pre-commit run doclinks --all-files
```

## No Docs Needed

The change is primarily internal code organization. The operator-facing workflow, commands, dependencies, hardware execution opt-in, and safety guidance remain unchanged, so user documentation should not need updates unless file references are adjusted.

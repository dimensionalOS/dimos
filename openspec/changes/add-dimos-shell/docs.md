## User-Facing Docs

- Update `docs/usage/cli.md` with `dimos shell`, its TTY requirement, startup safety model, predefined namespace, direct RPC examples, and exit behavior.
- Update `docs/usage/python-api.md` with `Dimos.list_modules()`, `get_module()`, `list_rpcs()`, and `describe()`, including exact instance names, unique-class lookup, live discovery, and standard RPC inspection.
- Update the root `AGENTS.md` quick-start and CLI reference so coding agents can discover and use the shell.
- Keep existing manipulation-client documentation unchanged during the evaluation period.

## Contributor Docs

- Keep `docs/adr/0001-coordinator-loops-are-attachable.md` as the lifecycle rationale for direct Python attachment.
- No additional contributor guide is needed; implementation and verification details belong in the change tasks and code tests.

## Coding-Agent Docs

- Update `AGENTS.md` because it is the primary repository instruction and command reference for coding agents.
- No change is needed to `docs/coding-agents/index.md`; the feature is a user-facing debugging command rather than a coding-agent workflow rule.
- Keep the project vocabulary in `CONTEXT.md` aligned with the implemented names.

## Doc Validation

- Run `bin/doclinks`.
- Run `bin/run-doc-codeblocks docs/usage/cli.md docs/usage/python-api.md` where the documented blocks are executable; mark live-shell sessions as skipped examples.
- Run focused CLI and porcelain tests so documented command names and Python calls match the implemented interfaces.
- No diagram generation is required because the planned docs add no generated diagrams.

## No Docs Needed

Documentation changes are required because this adds a public CLI command and public Python methods.

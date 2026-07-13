## User-Facing Docs

- Update `docs/usage/modules.md` with an **External Python Modules** section.
- Document the declaration-plus-sibling-`python/` directory structure, the required `pyproject.toml`, optional `pixi.toml`, the implementation import reference, and ordinary Blueprint composition.
- Explain that Pixi provides the outer tool environment while uv manages the runtime Python environment; runtime Python dependencies belong in `pyproject.toml`.
- Document preparation/startup expectations and the diagnostics for absent manifests, failed environment preparation, or an invalid implementation reference.
- Keep the runnable example aligned with `examples/external_python_module/`.

## Contributor Docs

None. The module authoring behavior is user-facing and belongs in `docs/usage/modules.md`; implementation-specific worker details belong in code and the change design.

## Coding-Agent Docs

None. Existing coding-agent guidance already directs contributors to the module-system documentation. Add dedicated agent guidance only if external modules become a recurring agent-authored integration pattern.

## Doc Validation

- Run `md-babel-py run docs/usage/modules.md` for executable documentation snippets.
- Manually validate the documented directory tree and example commands against the external Python module example in a prepared environment.

## No Docs Needed

Not applicable: this introduces a public module-authoring convention and requires user documentation.

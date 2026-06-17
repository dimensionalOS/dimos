## User-Facing Docs

- Update `docs/usage/cli.md`:
  - Explain that `dimos run` accepts both built-in bare blueprint names and external namespaced blueprint names.
  - Add examples such as `dimos run my-robot-stack.go2` and `dimos run unitree-go2 my-robot-stack.keyboard-teleop`.
  - Document that external names must be fully qualified as `<canonical-distribution-namespace>.<external-local-blueprint-name>`.
  - Document that `dimos list` shows built-in and external blueprints in separate sections.
- Update `docs/usage/blueprints.md`:
  - Add a section for publishing external blueprints from installed Python packages.
  - Include a `pyproject.toml` example using `[project.entry-points."dimos.blueprints"]`.
  - Explain that entry point targets may be Blueprint objects or DimOS Module classes.
  - State that factories are not supported in v1.
  - State that external local blueprint names must be lowercase kebab-case.
- Update any README or quick-start page that describes the built-in-only registry flow if it would otherwise imply external packages must edit DimOS source.

## Contributor Docs

- Update `docs/development/dimos_run.md` or the nearest contributor guide for runnable blueprint registration:
  - Clarify the split between generated built-in registry registration and external entry point discovery.
  - Preserve the existing `pytest dimos/robot/test_all_blueprints_generation.py` guidance for in-repo blueprints.
  - Add guidance for testing external discovery with synthetic entry point metadata.
- Update `docs/development/testing.md` if new test helpers or monkeypatch patterns are introduced for entry point metadata.

## Coding-Agent Docs

- Update `AGENTS.md` if the blueprint quick-reference or “Adding a blueprint” guidance says all runnable blueprints must be added through the generated registry.
- Update `docs/coding-agents/` only if those docs contain registry-generation instructions that should distinguish built-in versus external blueprint registration.

## Doc Validation

- Run the project’s standard documentation link validation if available, for example `doclinks`.
- If changed docs contain executable Python snippets, run `md-babel-py run <doc>` for each affected markdown file.
- Run targeted tests that exercise CLI help/list behavior if docs rely on command output examples.

## No Docs Needed

Documentation changes are needed because this change introduces a new public packaging and CLI workflow for external DimOS users.

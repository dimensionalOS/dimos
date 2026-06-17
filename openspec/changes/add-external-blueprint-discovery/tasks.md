## 1. Implementation

- [x] 1.1 Add an external blueprint discovery/resolution helper near `dimos/robot/get_all_blueprints.py` that reads `importlib.metadata` entry points from the `dimos.blueprints` group.
- [x] 1.2 Implement canonical distribution namespace normalization by lowercasing distribution names and collapsing runs of `-`, `_`, and `.` into `-`.
- [x] 1.3 Implement external local blueprint name validation for lowercase kebab-case names matching `^[a-z0-9]+(-[a-z0-9]+)*$`.
- [x] 1.4 Implement metadata-only listing of external blueprint names as `<canonical-distribution-namespace>.<external-local-blueprint-name>` without importing entry point targets.
- [x] 1.5 Implement fully qualified external blueprint resolution that locates the matching distribution namespace and local blueprint name, then loads only the requested entry point.
- [x] 1.6 Implement target conversion so loaded Blueprint objects are returned directly and loaded DimOS Module classes are converted with `.blueprint()`.
- [x] 1.7 Implement resolver error types/messages for unknown namespace, missing local blueprint name, invalid local blueprint metadata, entry point load failure, ambiguous namespace, and invalid target object type.
- [x] 1.8 Integrate external resolution into the shared `get_by_name` path while preserving bare-name lookup against built-in `all_blueprints` and `all_modules` only.
- [x] 1.9 Update CLI exit wrappers so external resolver errors produce clear user-facing messages and built-in typo suggestions continue to work for bare names.
- [x] 1.10 Update `dimos list` to print grouped built-in and external blueprint sections, with external names included by default and no target import/validation.
- [x] 1.11 Confirm `Dimos.run(...)` and coordinator-side `load_blueprint_by_name(...)` use the shared resolver path without separate external-discovery logic.
- [x] 1.12 Do not modify generated `dimos/robot/all_blueprints.py` for external packages; keep registry generation scoped to in-repo built-in blueprints and modules.

## 2. Tests

- [x] 2.1 Add unit tests for canonical distribution namespace normalization, including uppercase, underscores, dots, repeated separators, and already-normalized names.
- [x] 2.2 Add unit tests for external local blueprint name validation, including valid kebab-case and invalid uppercase, underscore, dotted, slash, empty, and repeated-separator cases.
- [x] 2.3 Add resolver tests with monkeypatched/synthetic entry point metadata for successful Blueprint object resolution.
- [x] 2.4 Add resolver tests with monkeypatched/synthetic entry point metadata for successful DimOS Module class conversion through `.blueprint()`.
- [x] 2.5 Add resolver tests proving factory functions and unsupported objects are rejected.
- [x] 2.6 Add resolver tests for unknown namespace, namespace exists but local name missing, target load failure, invalid local name metadata, and ambiguous canonical namespace.
- [x] 2.7 Add tests proving bare names never search external entry points and continue to use built-in registries only.
- [x] 2.8 Add tests proving mixed built-in/external arguments to the run resolution path are resolved independently before composition.
- [x] 2.9 Add CLI/list tests proving `dimos list` groups built-in and external names and does not import external targets.
- [x] 2.10 Add Python API/coordinator-path tests or targeted coverage proving external names are resolved through the shared resolver in those paths.

## 3. Documentation

- [x] 3.1 Update `docs/usage/cli.md` with namespaced external blueprint examples, mixed composition examples, and grouped `dimos list` behavior.
- [x] 3.2 Update `docs/usage/blueprints.md` with external package publishing guidance and a `pyproject.toml` example for `[project.entry-points."dimos.blueprints"]`.
- [x] 3.3 Document that external targets may be Blueprint objects or DimOS Module classes, while factories are not supported in v1.
- [x] 3.4 Document external local blueprint naming rules and canonical distribution namespace behavior.
- [x] 3.5 Document remote coordinator expectations: external packages must be installed where coordinator-side name resolution runs.
- [x] 3.6 Update contributor or coding-agent docs only where existing text implies all runnable blueprints must be registered through the generated in-repo registry.

## 4. Verification

- [x] 4.1 Run `openspec validate add-external-blueprint-discovery`.
- [x] 4.2 Run focused pytest targets for the external discovery resolver and CLI/list behavior.
- [x] 4.3 Run existing blueprint registry tests, including `pytest dimos/robot/test_all_blueprints_generation.py`, to confirm generated built-in registry behavior remains unchanged.
- [x] 4.4 Run relevant Python API/coordinator tests that cover `Dimos.run(...)` and `load_blueprint_by_name(...)` string resolution paths.
- [x] 4.5 Run docs validation commands for changed docs, such as `doclinks` and `md-babel-py run <doc>` for docs with executable snippets.
- [x] 4.6 Manually QA with a temporary installed test package exposing `dimos.blueprints` entry points: verify `dimos list`, `dimos run my-test-stack.demo`, and `dimos run <built-in> my-test-stack.demo`.
- [x] 4.7 Manually QA an external entry point whose target import fails and confirm the CLI reports an entry point load failure rather than a generic unknown blueprint.

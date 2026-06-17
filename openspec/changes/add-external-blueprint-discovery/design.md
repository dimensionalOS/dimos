## Context

`dimos run` currently resolves runnable blueprint names through the generated in-repository registries in `dimos/robot/all_blueprints.py`. Runtime lookup flows through `dimos/robot/get_all_blueprints.py`, which maps built-in blueprint names to import paths and built-in DimOS Module names to module classes converted with `.blueprint()`. The CLI, Python API, and coordinator RPC path all rely on this shared lookup behavior.

This works for DimOS-shipped blueprints but forces external robot stacks to edit DimOS source and regenerate the static registry before users can run them by name. The new design adds dynamic external blueprint discovery from installed Python package metadata while preserving the generated registry as the authoritative source for built-in bare names.

Relevant existing surfaces:
- `dimos/robot/all_blueprints.py`: generated built-in `all_blueprints` and `all_modules` maps.
- `dimos/robot/get_all_blueprints.py`: shared resolver used by CLI, Python API, and coordinator loading.
- `dimos/robot/cli/dimos.py`: `dimos run` composes one or more names with `autoconnect(...)`; `dimos list` lists built-in names today.
- `dimos/porcelain/dimos.py`: `Dimos.run(...)` resolves string targets locally or asks a remote coordinator to resolve by name.
- `dimos/core/coordination/module_coordinator.py`: coordinator-side `load_blueprint_by_name(...)` uses the shared resolver.
- `dimos/core/module.py`: `ModuleBase.blueprint` provides class-level conversion from DimOS Module classes to blueprints.

## Goals / Non-Goals

**Goals:**
- Let installed external Python distributions expose runnable blueprint names without editing DimOS source.
- Keep public terminology blueprint-centric and consistent with existing `dimos run` and `dimos list` behavior.
- Preserve bare-name behavior for built-in DimOS blueprints and modules.
- Require explicit namespacing for external blueprints.
- Support entry point targets that are either Blueprint objects or DimOS Module classes.
- Keep discovery metadata-only until a user actually resolves/runs a specific external blueprint.
- Provide namespace-aware errors that make failed external resolution diagnosable.
- Apply resolver behavior consistently across CLI, Python API, and coordinator-side loading.

**Non-Goals:**
- No filesystem scanning for external blueprints.
- No local config fallback in v1.
- No zero-argument factory functions or other callable target conventions.
- No explicit namespace override in v1; namespace is derived from the installed distribution name.
- No cache layer in v1; installed package metadata is read on demand.
- No validation/import of all external entry points during `dimos list`.
- No changes to stream contracts, worker lifecycle, hardware commands, simulation, replay, skills, or MCP tool exposure.

## DimOS Architecture

### Discovery model

External packages register blueprint metadata with the Python packaging entry point group `dimos.blueprints`:

```toml
[project]
name = "my-robot-stack"

[project.entry-points."dimos.blueprints"]
go2 = "my_robot_stack.go2:go2_blueprint"
keyboard-teleop = "my_robot_stack.teleop:KeyboardTeleop"
```

DimOS exposes these as namespaced blueprint names:

```text
my-robot-stack.go2
my-robot-stack.keyboard-teleop
```

The external namespace is the canonicalized installed distribution name. Canonicalization follows Python packaging distribution-name normalization: lowercase and collapse runs of `-`, `_`, and `.` into `-`. This can be implemented directly with a small helper equivalent to PEP 503 normalization, avoiding a new runtime dependency unless the project already chooses to depend on `packaging`.

The external local blueprint name is the entry point name and must use lowercase DimOS-style kebab-case:

```text
^[a-z0-9]+(-[a-z0-9]+)*$
```

Dots are reserved for separating external namespace from local blueprint name. Underscores, uppercase characters, slashes, and nested dotted local names are invalid.

### Resolver integration

Dynamic discovery should be added to the shared name resolver, not only the CLI. A small resolver module can live near the current registry resolver, for example under `dimos/robot/`, and expose helpers for:
- listing external blueprint metadata;
- parsing and validating namespaced blueprint names;
- resolving one external blueprint entry point by fully qualified name;
- converting a loaded target object to a `Blueprint`.

The existing lookup order remains:

1. If the requested name is bare, resolve only against built-in `all_blueprints` and `all_modules`.
2. If the requested name is namespaced with a dot, first preserve existing built-in behavior where applicable, then resolve external metadata by namespace and local name.
3. If the external target loads to a Blueprint object, return it.
4. If the external target loads to a DimOS Module class, return `Target.blueprint()`.
5. Otherwise raise an invalid external blueprint target error.

Built-in names keep their current generated-registry path. External names do not modify `all_blueprints.py`; that file remains generated only from in-repo blueprints and modules.

### CLI behavior

`dimos run` already accepts multiple blueprint/module arguments and composes resolved targets with `autoconnect(...)`. No new CLI shape is needed. Each argument is resolved independently, so mixed built-in and external composition works naturally:

```bash
dimos run unitree-go2 my-robot-stack.keyboard-teleop
```

`dimos list` should show names only, grouped by source:

```text
Built-in blueprints:
  unitree-go2
  unitree-go2-agentic

External blueprints:
  my-robot-stack.go2
  my-robot-stack.keyboard-teleop
```

Listing external blueprints reads entry point metadata and does not import target modules or validate target object types. A future `--verbose` flag may show entry point target strings, but v1 does not require it.

### Python API and coordinator behavior

`Dimos.run("my-robot-stack.go2")` should work when the external distribution is installed in the environment that resolves the name.

Remote mode is intentionally coordinator-side: if the client sends a string name through coordinator RPC, the package must be installed where the coordinator performs `load_blueprint_by_name(...)`. Installing the package only in the client environment is not sufficient for remote resolution.

### Errors

External errors should be namespace-aware. At minimum, distinguish:
- namespaced name has an external namespace that is not installed/discovered;
- namespace exists but the requested external local blueprint name is not declared;
- the external entry point exists but fails to load/import;
- the loaded object is neither a Blueprint object nor a DimOS Module class;
- an external local blueprint name in metadata is invalid.

CLI exit wrappers can continue to convert resolver exceptions into user-facing messages, but should preserve these distinctions and include useful suggestions where safe.

### DimOS Specs, streams, skills, and MCP

No DimOS Python `Spec` Protocol changes are required. This change does not add module RPC contracts, stream names/types, transport behavior, skill decorators, or MCP exposure. It only expands how blueprint names are discovered and resolved.

## Decisions

1. **Use installed Python entry points for external discovery.**
   - Rationale: packaging metadata is explicit, environment-scoped, compatible with installed distributions, and avoids scanning arbitrary files.
   - Alternatives considered: filesystem scanning and local config. Filesystem scanning is too implicit and environment-dependent; local config may be a future escape hatch but is not v1.

2. **Keep external names always namespaced.**
   - Rationale: prevents external packages from shadowing or competing with built-in bare names and makes dynamic discovery intentional.
   - Alternative considered: allow bare external names. Rejected because it creates ambiguity and surprising name resolution.

3. **Derive namespace from the installed distribution name.**
   - Rationale: reduces ceremony and gives each package a natural namespace.
   - Alternative considered: require explicit namespace declaration in another entry point group or config key. Rejected for v1 as extra surface area.

4. **Use `.` between namespace and local blueprint name.**
   - Rationale: readable fully qualified names such as `my-robot-stack.go2`, while preserving kebab-case local names.
   - Constraint: local external blueprint names cannot contain dots.

5. **Enforce lowercase kebab-case external local blueprint names.**
   - Rationale: matches existing DimOS-style runnable names and avoids hidden normalization surprises.
   - Invalid metadata should be reported as invalid rather than silently normalized.

6. **Use the existing `dimos.blueprints` entry point group.**
   - Rationale: keep terms blueprint-centric, even though Module classes may be accepted as a convenience and converted to blueprints.
   - Alternative considered: `dimos.runnables`. Rejected to avoid introducing a new public term.

7. **Support Blueprint objects and DimOS Module classes, but not factories.**
   - Rationale: mirrors internal registry behavior and avoids adding a new callable convention.
   - Factories can be reconsidered later if there is a concrete need.

8. **List external blueprints by metadata only.**
   - Rationale: `dimos list` should be fast, safe, and not fail because an optional robot dependency cannot import in the current environment.
   - Validation happens when a specific external blueprint is resolved.

9. **Do not cache discovery in v1.**
   - Rationale: simple on-demand metadata reads mean install/uninstall changes are visible immediately.
   - A cache can be added later if performance measurements justify it.

10. **Implement dynamic resolution centrally.**
    - Rationale: CLI, Python API, and coordinator RPC should share behavior and error semantics.

## Safety / Simulation / Replay

This change does not alter robot motion, stream payloads, hardware drivers, simulation, or replay behavior. It changes how runnable blueprint names are discovered before a stack is built. Hardware risk is limited to the fact that external packages can expose blueprints that run real robot modules; existing DimOS safety expectations still apply once a blueprint is selected.

Manual QA should include non-hardware examples and, when practical, a simulated or replay-only external blueprint package. Real hardware validation is not required to validate the discovery mechanism itself.

## Risks / Trade-offs

- **Metadata collisions:** Two installed distributions may canonicalize to the same namespace. Mitigate by treating this as an ambiguous external namespace error or by surfacing all matching distributions in diagnostics.
- **Invalid third-party metadata:** Bad entry point names or invalid target objects can appear in installed packages. Mitigate with clear errors on resolution and cautious behavior in `dimos list`.
- **Import side effects on run:** Loading an external entry point imports package code. This is expected at run time but intentionally avoided during list.
- **Remote environment confusion:** Users may install an external package locally but run against a coordinator environment that lacks it. Mitigate with docs and coordinator-side error wording.
- **Entry point API compatibility:** `importlib.metadata.entry_points()` has changed return shapes across Python versions. Implement a small compatibility helper and test it under supported Python versions.
- **Terminology precision:** Accepting DimOS Module classes through `dimos.blueprints` is internally broader than the group name. Mitigate by documenting that Module classes are converted to blueprints and keeping public language blueprint-centric.

## Migration / Rollout

- Existing built-in blueprint and module registries remain unchanged.
- No regeneration of `all_blueprints.py` is required for external packages. The existing registry generation test remains required only when in-repo blueprints/modules are added or renamed.
- Add resolver/listing tests with synthetic entry point metadata rather than depending on globally installed packages.
- Add docs showing `pyproject.toml` entry point examples, namespaced `dimos run`, `dimos list`, Module class conversion, and remote coordinator installation expectations.
- Rollback is straightforward: remove external discovery from the resolver/list command while preserving the generated built-in registry.

## Open Questions

- Should `dimos list` display invalid external metadata with a warning, or omit invalid entries and reserve the error for direct resolution? The current design prefers reporting invalid metadata as invalid when encountered, while keeping list metadata-only.
- If two installed distributions canonicalize to the same namespace, should DimOS fail the whole namespace as ambiguous or choose a deterministic but potentially surprising order? The safer default is to fail as ambiguous.

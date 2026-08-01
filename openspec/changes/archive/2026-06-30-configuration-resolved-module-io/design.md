## Context

DimOS modules currently declare typed IO with class annotations such as `image: In[Image]` and `command: Out[JointState]`. `BlueprintAtom.create()` discovers those annotations before CLI/config-file overrides are merged. Later, worker deployment merges `blueprint_args` into module kwargs, instantiates modules, and the coordinator wires the already-discovered streams.

That ordering works for static modules, but it cannot represent modules whose IO contract depends on final validated configuration. Examples include policy modules whose observation/action streams depend on the selected policy contract, and coordinators whose externally visible commands depend on selected tasks and hardware adapters.

This design introduces configuration-resolved module IO: a module's coordinator-visible stream contract is resolved from final validated module config at build/load time, before wiring, and remains fixed for the running module.

## Goals / Non-Goals

**Goals:**

- Preserve existing annotation-based module IO behavior by default.
- Let modules override `Module.io_contract(config)` to return a complete configuration-derived IO contract.
- Resolve IO after all module kwargs, CLI/config-file `blueprint_args`, and global config are merged and validated.
- Use the same resolved stream names for validation, remapping, conflict checks, and transport assignment.
- Expose configuration-resolved streams through `self.inputs` and `self.outputs`.
- Keep annotated stream attributes working for backward compatibility.
- Validate stream-name uniqueness and stale remappings before deployment.
- Include unit tests and a runnable teaching demo as implementation gates.

**Non-Goals:**

- No runtime hot-add/remove streams after a module is built or loaded.
- No stream groups in v1.
- No separate `wire_name` field in v1; blueprint remappings remain the external rename mechanism.
- No parent/worker IO contract fingerprint validation in v1.
- No immediate conversion of `ControlCoordinator`; this change proves the core mechanism first.

## Decisions

### Add a public module IO contract API

Introduce public declaration types along these lines:

```python
StreamDirection = Literal["in", "out"]

@dataclass(frozen=True)
class StreamDecl:
    name: str
    direction: StreamDirection
    type: type

@dataclass(frozen=True)
class ModuleIOContract:
    streams: tuple[StreamDecl, ...]
```

`ModuleIOContract` owns contract validation, including uniqueness of stream names across both directions. Blueprint resolution calls the same validation logic rather than duplicating rules.

Alternative considered: return internal `StreamRef` values directly. Rejected because module authors need a public API that can evolve separately from coordinator internals.

### Always resolve IO through `Module.io_contract(config)`

Add a classmethod on `Module`:

```python
@classmethod
def io_contract(cls, config: ModuleConfig) -> ModuleIOContract:
    return ModuleIOContract.from_annotations(cls)
```

The base implementation preserves existing behavior. Overrides are complete replacements; annotations are not merged automatically. An override can return zero streams even if the class has annotated `In` or `Out` fields.

Alternative considered: detect whether a module overrides the hook and branch. Rejected because always calling the hook gives a simpler and more uniform path.

### Resolve final module config before stream discovery

Build/load should create a resolved module plan:

```text
BlueprintAtom.kwargs + blueprint_args + GlobalConfig
  -> validated module config
  -> Module.io_contract(config)
  -> ResolvedModulePlan
  -> validation/remapping/conflict checks
  -> worker deployment with final kwargs
  -> transport wiring
```

`BlueprintAtom` remains authoring-time data. A new internal resolved layer becomes authoritative for wiring.

The worker deployment path should receive final per-module kwargs. It should no longer be the first place where `blueprint_args` are merged, because that would let worker instantiation diverge from coordinator-side IO resolution.

### Use explicit stream registries as the module source of truth

`Module.inputs` and `Module.outputs` should read from explicit internal registries. Annotated streams are still installed as attributes for compatibility and also registered. Configuration-resolved streams are registered but not installed as arbitrary attributes.

This means module authors use:

```python
self.inputs["primary_camera"]
self.outputs["policy_action"]
```

for configuration-resolved streams.

`set_transport()` and `connect_stream()` should resolve streams through those registries, not through `getattr()`.

Alternative considered: create dynamic attributes for all resolved streams. Rejected because arbitrary config-provided names can collide with lifecycle methods, `config`, `tf`, RPC helpers, or internal fields.

### Keep remapping external-only

Remappings apply to resolved stream names, but they do not rename keys inside `self.inputs` or `self.outputs`.

Example:

```text
Local key:         primary_camera
Blueprint remap:  primary_camera -> zed_front_rgb
Graph name:       zed_front_rgb
Module access:    self.inputs["primary_camera"]
```

Blueprint resolution MUST fail when a remapping references a stream not present in the resolved `ModuleIOContract`.

### Preserve v1 graph conflict semantics

After remapping, current graph semantics remain:

```text
same graph name + same message type      => shared transport/autoconnect group
same graph name + different message type => error
```

Changing fan-in/fan-out policy, adding single-producer control-stream checks, or introducing stricter direction-aware rules is out of scope for v1.

### Prove with a demo before broader migrations

The first user-facing conversion is a small teaching example under `examples/`. The demo should show config changing IO shape, not just stream names. For example, a module whose config mode exposes either an image input or a joint-state input.

The implementation gate is that the demo runs successfully, in addition to core unit tests passing.

## Risks / Trade-offs

- Config-derived IO can be nondeterministic if authors depend on mutable external state → Document that IO is fixed after build/load and that hook determinism is author responsibility.
- Parent and worker could resolve different IO if config merging remains split → Resolve final kwargs before deployment and pass those final kwargs to the worker.
- Dynamic stream names can collide within a module → Validate uniqueness across the entire `ModuleIOContract`.
- Existing callers may expect `inputs`/`outputs` to scan instance attributes → Preserve public `inputs`/`outputs` behavior while changing their backing source to registries.
- Class-level introspection without config cannot show configuration-resolved IO → Build/load graph rendering should use the resolved plan; class-level introspection remains best-effort/default-oriented.
- Native modules may depend on declared ports before subprocess launch → Ensure resolved streams are instantiated before native module startup; deeper native-specific behavior can be handled after the core slice.

## Migration Plan

1. Add `StreamDecl`, `ModuleIOContract`, and shared validation.
2. Add `Module.io_contract(config)` with annotation-derived default behavior.
3. Add explicit stream registries and keep annotated stream attributes as compatibility aliases.
4. Update `inputs`, `outputs`, `set_transport()`, and `connect_stream()` to use registries.
5. Add resolved module planning in build/load before stream conflict checks and deployment.
6. Pass final per-module kwargs to worker deployment instead of late-merging `blueprint_args` in the worker manager.
7. Validate remappings against resolved stream names.
8. Add unit tests for default annotation behavior, custom replacement behavior, stream uniqueness, remapping validation, and final-config resolution.
9. Add the `examples/` teaching demo and make it a required implementation gate.

Rollback strategy: because annotation-derived IO remains the default, the core change can be rolled back by removing custom contract usage and returning to annotation discovery for all modules.

## Open Questions

- Exact module/file placement for `StreamDecl` and `ModuleIOContract`.
- Whether native-module demos need a follow-up example after the core example.
- Whether future work should add stream groups or contract fingerprints.

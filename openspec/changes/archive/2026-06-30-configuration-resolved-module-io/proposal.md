## Why

DimOS module IO is currently discovered from class annotations before final module configuration is known. That prevents modules whose stream surface depends on a selected policy, backend, task set, or hardware adapter from exposing an honest blueprint-visible IO contract.

Configuration-resolved module IO lets a module derive its typed streams from final validated config before wiring, while preserving the existing static annotation behavior for ordinary modules.

## What Changes

- Add a public module IO contract API made of `ModuleIOContract` and `StreamDecl`.
- Add `Module.io_contract(config)` as the single source for module stream declarations.
- Keep annotation-derived IO as the default implementation for modules that do not override `io_contract`.
- Treat custom `io_contract` overrides as complete replacements for annotation-derived IO.
- Resolve module IO after blueprint kwargs, CLI/config-file `blueprint_args`, and global config are merged into validated module config.
- Add an internal resolved module plan used for build/load-time stream validation and wiring.
- Store streams in explicit internal input/output registries so `self.inputs` and `self.outputs` work for both annotated and configuration-resolved streams.
- Keep annotated stream attributes for backward compatibility, but do not create arbitrary attributes for configuration-resolved streams.
- Update wiring helpers to resolve streams through `self.inputs` and `self.outputs` rather than `getattr`.
- Validate that stream names are unique across each module IO contract.
- Validate remappings against the resolved IO contract and fail when they reference missing streams.
- Preserve existing graph conflict/autoconnect semantics after remapping.
- Add a small teaching example under `examples/` demonstrating config that changes IO shape.
- Add unit tests for the core behavior.

## Capabilities

### New Capabilities
- `configuration-resolved-module-io`: Modules can expose a build/load-time IO contract derived from final validated module configuration.

### Modified Capabilities

None.

## Impact

- Affected core areas:
  - `dimos/core/module.py`
  - `dimos/core/stream.py` if shared stream declaration types belong there
  - `dimos/core/coordination/blueprints.py`
  - `dimos/core/coordination/module_coordinator.py`
  - Python worker deployment path that currently merges `blueprint_args` late
- Affected user-facing APIs:
  - New `Module.io_contract(config)` override point
  - New `ModuleIOContract` and `StreamDecl` types
  - Existing annotation-based module authoring remains supported
- Validation gate:
  - Unit tests for core resolution/wiring behavior must pass.
  - The teaching demo in `examples/` must run successfully as an implementation gate.

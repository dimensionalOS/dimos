## Why

`dimos/manipulation/viser_panel/module.py` now contains the complete optional Viser manipulation panel implementation: DimOS module lifecycle, RPC client management, polling, GUI handle creation, scene/URDF rendering, target synchronization, preview debounce/timeout handling, and planning/execution operations. The behavior is tested and useful, but the single module has grown large enough that future target controls, scene overlays, or safety gates will be hard to add without accidental coupling.

This change reorganizes the panel around explicit composition boundaries while preserving the existing operator behavior. The goal is a better design, not a new UI feature: keep `PanelSession` as the source of truth, keep `ViserManipulationPanelModule` as the public DimOS module/blueprint surface, and move Viser-specific handle ownership, backend RPC access, workflow operations, and preview animation into focused collaborators.

## What Changes

- Refactor the optional Viser manipulation panel implementation into small internal components with narrow responsibilities.
- Keep the public module, blueprint, CLI entrypoint, config fields, optional dependency extra, and documented operator workflow compatible.
- Preserve current behavior for robot discovery, target synchronization, automatic IK/FK feasibility preview, plan/preview/execute/cancel/clear controls, Viser URDF rendering, target coloring, and snapshots.
- Use composition as the primary organization mechanism; use only lightweight declarative data where it reduces repetitive GUI-control creation or status binding.
- Do not introduce a custom UI framework, change Viser dependencies, or alter `ManipulationModule` RPC behavior.
- No **BREAKING** public API/CLI or hardware-safety behavior changes are intended.

## Affected DimOS Surfaces

- Modules/streams: `ViserManipulationPanelModule` internals under `dimos/manipulation/viser_panel/`; no stream contract changes.
- Blueprints/CLI: existing Viser panel blueprint and `python -m dimos.manipulation.viser_panel` launch path remain compatible.
- Skills/MCP: none.
- Hardware/simulation/replay: no new hardware actions; existing execution opt-in and plan freshness gates remain unchanged.
- Docs/generated registries: docs may mention the internal package layout if key-file descriptions change; no generated registry change expected.

## Capabilities

### New Capabilities

- None.

### Modified Capabilities

- `manipulation-operator-panel`: clarify that the panel's public launch/import/behavior boundary remains stable while internal implementation is modularized.

## Impact

Developers get a panel implementation that is easier to test, extend, and reason about. Compatibility risk is primarily regression risk from moving code, especially around Viser handle lifecycle, target synchronization recursion guards, preview worker timing, and execute gating. The implementation should rely on existing tests as a safety net, split tests along the new component boundaries, and finish with the focused manipulation panel test suite plus a lightweight panel launch/snapshot check where available.

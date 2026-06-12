## Why

DimOS now has a manipulation planning visualization protocol that separates world planning behavior from optional preview and visualization behavior. The current implementation is still Meshcat-shaped because DrakeWorld owns both the planning world and Meshcat visualization, while the earlier Viser prototype lives on a separate branch as a rich manipulation panel rather than a protocol implementation.

This change adds Viser as a first-class manipulation visualizer that implements the new visualization protocol and can be selected in place of the current Meshcat-backed visualization. It also preserves the useful behavior from the draft Viser manipulation panel—URDF scene rendering, preview ghosts, path rendering, GUI controls, preview planning, and guarded execution—so Viser is useful both as a protocol-backed visualizer and as an operator-facing manipulation UI.

## What Changes

- Add a manipulation visualization backend selector supporting Meshcat, Viser, and no visualization.
- Add a Viser implementation of the manipulation planning visualization protocol for URL discovery, publishing, preview visibility, path animation, and cleanup.
- Keep existing Meshcat behavior as the default compatibility path when visualization is enabled without an explicit Viser selection.
- Port the previous Viser panel functionality into the new design as optional Viser UI features backed by the same Viser runtime and scene helpers.
- Add an explicit plan for retrieving panel data that previously came through RPC without introducing unsafe direct world/IK access from Viser callbacks.
- Add lazy optional dependency handling for Viser/URDF support so non-Viser manipulation users are unaffected.
- No **BREAKING** public CLI, hardware-safety, or robot command behavior is intended.

## Affected DimOS Surfaces

- Modules/streams:
  - `ManipulationModule` visualization configuration and URL reporting.
  - `WorldMonitor` visualization selection and delegation.
  - Manipulation planning world/visualization protocol usage.
  - New Viser manipulation visualization/runtime/scene/panel helpers.
- Blueprints/CLI:
  - Manipulation blueprints may opt into Viser through module config.
  - No change to the global Rerun `vis_module()` viewer selection is required for this change.
- Skills/MCP:
  - No new skill or MCP tool is required.
  - Existing manipulation operations must preserve their current safety and execution semantics when visualized through Viser.
- Hardware/simulation/replay:
  - Hardware command execution remains gated by existing manipulation execution paths and any Viser panel execution opt-in.
  - Simulation and replay behavior should continue to work without Viser installed unless Viser is selected.
- Docs/generated registries:
  - Documentation should explain manipulation visualization backend selection and distinguish it from the global Rerun viewer.
  - No generated blueprint registry change is expected unless a new blueprint is added.

## Capabilities

### New Capabilities
- `viser-manipulation-visualizer`: Viser-backed manipulation visualization, including protocol behavior, Viser scene rendering, optional panel controls, lean in-process adapter access, and dependency/lifecycle expectations.

### Modified Capabilities
- `manipulation-planning-visualization`: Adds backend selection and Viser swappability requirements to the existing optional manipulation visualization behavior.

## Impact

Users can choose Viser for manipulation preview and operator visualization without replacing the global Rerun stream viewer. Developers get a concrete backend for the new visualization protocol and a migration path for the earlier Viser panel work. The main compatibility risk is that selecting Viser runs a web/GUI visualization runtime inside the manipulation process; this is an accepted design tradeoff for this change and must be mitigated with lazy imports, bounded callbacks, clear shutdown, and copy-at-read-boundary data access through a small in-process adapter. Testing must cover backend selection, protocol delegation, Viser scene behavior with fake server/URDF objects, panel regression behavior, missing dependency errors, and preservation of Meshcat/no-visualization behavior.

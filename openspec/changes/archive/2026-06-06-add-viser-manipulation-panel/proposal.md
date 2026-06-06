## Why

DimOS manipulation already supports motion planning, trajectory preview, execution, robot state queries, obstacle management, and gripper control, but those capabilities are currently exposed through Python/RPC clients and backend visualizers rather than a focused operator panel. Developers need a browser-based way to inspect the live robot state, set a target, plan, preview, and execute without dropping into an interactive Python shell.

This change introduces the first two phases of a Viser-based manipulation panel: a read-only live viewer and a basic planning panel. The scope intentionally excludes perception-driven pick/place workflows, rich scene editing, and full MoveIt RViz parity so the initial surface can validate the core operator loop safely.

## What Changes

- Add a Viser manipulation panel capability for viewing configured manipulator robots, live joint state, end-effector pose, gripper state, and manipulation module status.
- Add operator controls for selecting a robot, setting Cartesian or joint-space targets, requesting a plan, previewing the planned trajectory, executing the current plan, canceling execution, and resetting faults.
- Add basic 3D scene behavior for rendering the robot model, current end-effector frame, target transform control, and planned trajectory preview.
- Add safety-oriented UI behavior that prevents execution unless a fresh plan exists and keeps cancel/reset controls visible.
- Limit this change to standalone/operator-facing panel behavior over existing manipulation RPC surfaces; it does not replace Meshcat, Rerun, planning backends, or manipulation execution semantics.
- No **BREAKING** public API, CLI, or hardware-safety behavior is intended.

## Affected DimOS Surfaces

- Modules/streams: Manipulation state and planning RPC surfaces; current joint state, end-effector pose, trajectory status, gripper state, and existing plan/preview/execute/cancel/reset behavior. No new required robot data stream is expected for the initial scope.
- Blueprints/CLI: Potential companion launch entrypoint or optional blueprint wiring for the Viser panel; existing manipulation blueprints should remain usable without the panel.
- Skills/MCP: No agent skill behavior changes are planned. The panel may expose similar operator actions through UI controls, but it should not add or alter MCP tools in the initial scope.
- Hardware/simulation/replay: Real hardware, simulation, and replay manipulation stacks should retain existing planning and execution behavior. Hardware execution must remain gated by existing manipulation/coordinator safety checks plus UI confirmation/disabled-state affordances.
- Docs/generated registries: Add user/developer documentation for launching and using the phase-1/2 panel. Generated blueprint registries may be affected only if a new runnable blueprint is introduced.

## Capabilities

### New Capabilities

- `manipulation-operator-panel`: Browser-based operator behavior for inspecting a manipulation robot, setting targets, planning, previewing, executing, canceling, and resetting through a Viser control panel.

### Modified Capabilities

- None.

## Impact

Users gain a MoveIt-RViz-inspired but lightweight web panel for the core manipulation loop: inspect robot state, choose a target, plan, preview, and execute. Developers gain a concrete UI surface for validating manipulation RPC behavior without using an interactive Python client.

Compatibility risk is low if the panel remains optional and uses existing manipulation RPCs. Dependency impact includes adding Viser and any URDF visualization support as optional dependencies rather than required core manipulation dependencies. Testing should cover panel state derivation, execution gating, RPC error handling, and compatibility with mock/sim manipulation stacks. Manual QA should launch a mock or simulation manipulation blueprint, open the Viser panel, confirm live robot state updates, create a joint and Cartesian plan, preview it, execute it, cancel/reset where applicable, and verify the panel handles missing or faulted manipulation services gracefully.

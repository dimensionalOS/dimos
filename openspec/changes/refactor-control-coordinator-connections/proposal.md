## Why

The control coordinator currently owns hardware construction, adapter lookup, device lifecycle, state polling, unit conversion, and command dispatch. Those responsibilities leak hardware IDs and device-specific assumptions into manipulation, make multiple connection modules difficult to compose, and leave safety behavior split across several layers.

This refactor establishes one durable control boundary: connection modules own device I/O and safety, while the control coordinator assembles their declared interfaces into one logical robot and arbitrates commands over canonical names. It also removes the multi-robot abstraction from manipulation, where callers already reason about one robot and select subsets through planning groups.

## What Changes

- **BREAKING** Move SDK/native connection ownership, activation, device configuration, ordering and unit conversion, hard-limit enforcement, watchdogs, and safe stop into concrete connection modules.
- **BREAKING** Replace coordinator-owned hardware/adapters and joint-specialized routing with scalar, fully qualified control interfaces exchanged through typed state and command streams.
- Add typed connection description, status, and lifecycle-control messages so the coordinator can discover resolved interfaces at runtime, validate readiness, and arm or stop all required connections transactionally.
- **BREAKING** Model manipulation as exactly one logical robot backed by one statically prepared URDF/SRDF. Planning groups select joint subsets; robot IDs, per-robot registries, and local/global joint-name translations are removed.
- Use one canonical namespace across the prepared model, connection state, planning, trajectories, coordinator commands, grippers, and visualization, such as `left/j1` and `right/j1`.
- Treat a gripper as one or more ordinary controllable joints with the same state, limit, unit, arbitration, and safety contracts as other joints, while retaining the task semantics introduced by the gripper API refactor.
- Support multiple instances of the same connection class through module instance names and shared class-level typed streams; no dynamically generated coordinator subclasses or per-connection ports are introduced.
- Define explicit command profiles per connection. Runtime switching between incompatible command modes is outside this change.
- **BREAKING** Remove coordinator hardware-management RPCs, adapter registries at the coordinator boundary, hardware-specific gripper RPCs, hardware-specific global configuration, and obsolete compatibility paths as their replacements land.
- Preserve the current task registry and `TaskConfig` design. A type-safe task-configuration refactor is outside this change.

## Affected DimOS Surfaces

- **Modules and streams:** control coordinator, control tasks, connection modules, adapter internals, typed control messages, lifecycle/status messages, and autoconnect transport assignments.
- **Blueprints and CLI:** manipulator, mobile-base, G1, simulation, replay, teleoperation, and agentic blueprints; per-instance connection configuration such as `--left.address` and `LEFT__ADDRESS`.
- **Manipulation and planning:** model configuration, world interfaces, planning groups, trajectory execution, collision/FK backends, and visualization state.
- **Skills and MCP:** public manipulation skills retain robot-level behavior but stop accepting or deriving robot/hardware identifiers; generated schemas and callers are updated where signatures change.
- **Hardware, simulation, and replay:** xArm and gripper, Piper/OpenArm, mobile bases, G1 whole-body control, mock/simulation connections, and recorded state/command seams.
- **Documentation and registries:** control architecture, module configuration, blueprint examples, hardware bring-up guidance, safety behavior, and generated blueprint registration where affected.

## Capabilities

### New Capabilities

- `single-robot-manipulation`: One prepared robot model, canonical interface names, and planning-group-based selection without a multi-robot domain model.
- `connection-owned-control`: Connection modules own device I/O and advertise immutable, resolved control capabilities and operational status.
- `scalar-control-interfaces`: State, commands, and arbitration use exact, fully qualified scalar interface keys over shared typed streams.
- `robot-control-safety`: Readiness, transactional arming, watchdogs, stale-state handling, omission policies, emergency stop, and explicit recovery are defined across coordinator and connections.

### Modified Capabilities

- None. The repository has no existing OpenSpec capability that defines this control boundary.

## Impact

This is an intentionally breaking refactor across control, manipulation, robot connections, blueprints, configuration, and tests. It starts after the gripper API refactor in PR #3381 merges and is delivered as a stacked series whose early changes leave a working system and whose coordinator cutover removes the old ownership path atomically. Every shipped connection family receives mock or simulation coverage; hardware validation is scoped per platform and does not depend on a real dual-xArm setup.

Dynamic robot-part composition, multiwriter-safe shared memory, generic estimator-fed control interfaces, runtime command-mode switching, non-joint manipulation planning, pose-message routing cleanup, upper-level multi-robot coordination, and task-registry redesign remain explicitly out of scope.

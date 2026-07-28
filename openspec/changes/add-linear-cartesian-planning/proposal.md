## Why

DimOS manipulation planners can currently produce collision-free joint-space paths, but
the internal planner contract cannot express a straight tool-center-point (TCP) motion.
Callers that need a linear approach, retreat, or visualization-driven absolute move
therefore cannot request the geometric behavior from a planner backend.

RoboPlan now provides an official Cartesian path planner that returns a synchronized,
timed joint trajectory. DimOS should expose that capability through a concise,
backend-neutral planner contract while retaining DimOS collision validation and clear
unsupported behavior for planners that cannot satisfy it.

## What Changes

- Add an internal linear Cartesian planning operation over an ordered planning-group
  selection, explicit start state, per-group Cartesian targets, and auxiliary groups.
- Support absolute world-frame poses and world-frame Cartesian deltas as targets,
  including mixed target kinds in one multi-group request.
- Preserve RoboPlan's synchronized multi-track timing and return combined, globally
  named joint states with waypoint timestamps.
- Post-validate planned configurations and interpolated edges against the DimOS planning
  world; reject an invalid trajectory without falling back to free-space planning.
- Return an explicit unsupported result from planner backends without linear Cartesian
  planning support.
- Add typed, backend-discriminated planner configuration with RoboPlan linear Cartesian
  settings, while retaining the existing planner-name setting as a deprecated
  compatibility path.
- Upgrade RoboPlan to a release that includes its official Cartesian path planning API.
- This change adds an internal developer API. It does not add or change a public
  ManipulationModule, RPC, CLI, skill, or MCP surface.

## Affected DimOS Surfaces

- Modules/streams: manipulation planning models, the internal planner `Protocol`,
  planner factory/configuration, RoboPlan planner integration, RRT compatibility
  behavior, and planning collision checks; no stream contract changes.
- Blueprints/CLI: existing configuration loading may accept typed planner configuration;
  no blueprint names or CLI commands change.
- Skills/MCP: none.
- Hardware/simulation/replay: generated paths may ultimately execute on hardware or in
  simulation, so collision post-validation is safety-relevant; no execution,
  synchronization, simulation, or replay behavior changes in this scope.
- Docs/generated registries: internal manipulation-planning documentation and examples
  may be updated; no blueprint registry regeneration is expected.

## Capabilities

### New Capabilities

- `linear-cartesian-planning`: Internal planning behavior for synchronized linear TCP
  paths with absolute or relative targets, timing, validation, and unsupported-backend
  handling.

### Modified Capabilities

None.

## Impact

Planner implementers gain one additive required protocol operation and must either
produce a conforming result or return `UNSUPPORTED`. Existing configurations continue
to work during the `planner_name` deprecation window, while new configurations can
carry RoboPlan-specific linear-planning parameters. The RoboPlan package and lockfile
must be updated. Verification covers target resolution, fixed-orientation motion,
multi-group timing, output conversion, collision rejection, unsupported planners,
configuration compatibility, focused type checking, and OpenSpec validation.

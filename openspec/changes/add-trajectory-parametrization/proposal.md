## Why

Manipulation planning currently turns every pair of geometric waypoints into an independent trapezoidal segment. The robot therefore stops at every waypoint, so dense planner output produces slow and mechanically awkward motion instead of one continuous trajectory constrained by the robot's actual motion limits.

DimOS needs an explicit path-to-trajectory parametrization boundary that can retain the current simple implementation while allowing RoboPlan TOPP-RA to generate continuous, time-optimal trajectories. The selected behavior must be deterministic at startup, fail before a plan is exposed as executable, and use authoritative robot limits.

## What Changes

- Add startup configuration that selects exactly one manipulation trajectory parametrization backend for the lifetime of the stack.
- Preserve the existing simple trapezoid behavior as a selectable compatibility backend.
- Add a RoboPlan TOPP-RA backend for any geometric path planned against `RoboPlanWorld`, independent of which planner produced that path.
- Parametrize immediately after geometric planning and only construct/cache a `GeneratedPlan` after trajectory generation and validation succeed.
- Allow the selected backend to perform bounded interpolation or curve fitting while converting the source path into a timed trajectory.
- Use RoboPlan scene limits sourced from URDF velocity and acceleration limits; fail explicitly when required limits or the selected backend are unavailable.
- Do not switch parametrization backends after startup or fall back to another backend when parametrization fails.
- Exclude geometric path shortcutting, waypoint simplification, path-specific resampling, and formal DimOS per-joint limit overrides from this change.
- Pin the optional RoboPlan dependency to version `0.5.1`.

## Affected DimOS Surfaces

- Modules/streams: manipulation plan materialization, planning configuration/models, a trajectory-parametrizer adapter protocol, RoboPlan world/model integration, and timed-trajectory validation; no stream contract changes.
- Blueprints/CLI: manipulation blueprint configuration gains a startup-selectable parametrization backend; no new CLI command or blueprint name is introduced.
- Skills/MCP: existing plan, preview, and execute surfaces retain their signatures; unsuccessful parametrization makes planning fail before preview or execution.
- Hardware/simulation/replay: hardware and simulation execute the exact trajectory accepted during planning; RoboPlan TOPP-RA requires URDF velocity and acceleration limits. Replay behavior is unchanged.
- Docs/generated registries: manipulation planning documentation and dependency guidance require updates; no generated blueprint registry change is expected.

## Capabilities

### New Capabilities

- `manipulation-trajectory-parametrization`: Startup backend selection and conversion of accepted geometric manipulation paths into validated timed trajectories.

### Modified Capabilities

None.

## Impact

Users may choose the existing simple backend or RoboPlan TOPP-RA at startup. RoboPlan TOPP-RA configurations become stricter: they require `RoboPlanWorld`, RoboPlan `0.5.1`, and usable URDF velocity and acceleration limits. Parametrization failures are reported as planning/materialization failures rather than being deferred to execution or hidden by fallback.

The implementation touches manipulation planning internals and dependency resolution but does not intentionally break existing plan, preview, execute, skill, MCP, stream, or CLI signatures. Verification requires backend/configuration tests, trajectory invariant and failure tests, RoboPlan adapter tests, dependency lock validation, simulation/manual preview and execution QA, and documentation validation.

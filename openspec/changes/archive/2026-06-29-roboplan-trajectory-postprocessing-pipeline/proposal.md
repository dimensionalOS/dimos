## Why

RoboPlan-backed linear TCP plans currently produce dense waypoint chains that trajectory parametrization treats like stop-like knots, making valid linear motions much slower than comparable free-space plans even at full speed. We need a production-ready path post-processing structure that can smooth valid plans without weakening correctness, while keeping existing slow-but-valid behavior as the fallback.

## What Changes

- Add minimal optional path-constraint metadata to geometric manipulation plans so post-processing can preserve declared constraints instead of relying only on joint-space closeness.
- Attach linear TCP path constraints to successful linear TCP plans, including the constrained TCP frame and configured Cartesian tolerance.
- Extend RoboPlan trajectory parametrization with an internal staged trajectory post-processing pipeline that is easy to split out later.
- Enable conservative RoboPlan smoothing by default, with adaptive-conservative retries and validation before accepting a refined path.
- Preserve non-blocking smoothing fallback: smoothing failure MUST fall back to the original geometric path rather than failing parametrization by itself.
- Keep current public trajectory parametrization entry points and the practical implementation location under RoboPlan trajectory parametrization for this change.

## Capabilities

### New Capabilities

- None.

### Modified Capabilities

- `manipulation-trajectory-parametrization`: Add RoboPlan post-processing pipeline, smoothing defaults, validation, and non-blocking fallback requirements.
- `manipulation-linear-tcp-planning`: Require successful linear TCP plans to carry path-constraint metadata that declares the TCP-line constraint post-processing must preserve.
- `roboplan-cartesian-planning`: Require RoboPlan linear Cartesian planning results to provide enough constraint metadata for downstream trajectory post-processing to validate straight-line TCP preservation.

## Impact

- Affected models/specs: `GeneratedPlan`, planning/path constraint models, `TrajectoryParametrizationConfig`, and RoboPlan parametrization support.
- Affected planning code: `dimos/manipulation/planning/world/roboplan_world.py`, Cartesian/linear planning result construction, and trajectory parametrizer conversion to RoboPlan `JointPath`.
- Affected orchestration code: manipulation module plan storage/parametrization paths that pass `GeneratedPlan` through to trajectory generation.
- Affected tests: RoboPlan trajectory parametrization tests, linear TCP planning tests, manipulation module tests, and Viser preview/execution tests that rely on generated trajectories.
- No breaking public command/API behavior is intended; worst-case smoothing failure should preserve existing slow trajectory behavior.

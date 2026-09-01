# ADR 0001: Canonical Joint Space

- Status: Accepted
- Date: 2026-08-31

## Context

Manipulation backends do not agree on the representation of the same URDF joint. Drake keeps an unbounded prismatic joint as one scalar with infinite limits. Pinocchio may materialize a missing prismatic bound as a zero-width range, and represents a continuous revolute joint with cosine and sine configuration values. RoboPlan requires finite limits in its native planner. Previous dimOS code compensated in several adapters and identified special coordinates by planar-base joint names.

That split made mechanism, topology, planner search bounds, and execution capability look like one concept. It also meant a continuous arm joint did not receive the same circular handling as planar yaw.

## Decision

dimOS compiles one canonical scalar `JointSpace` when it prepares a robot model.

| URDF mechanism | URDF position limits | Canonical topology |
|---|---:|---|
| `revolute` | finite lower and upper | `INTERVAL` |
| `prismatic` | finite lower and upper | `INTERVAL` |
| `prismatic` | neither bound | `LINE` |
| `continuous` | neither bound | `CIRCLE` |

Partial bounds, unbounded `revolute` joints, and bounded `continuous` joints are rejected because their intended topology is ambiguous or contradictory.

The materialized URDF owns position, velocity, and acceleration limits. Every controlled coordinate requires positive finite velocity and acceleration maxima. Source descriptions that omit acceleration must opt into `RobotModel.with_default_joint_acceleration_limit(value)` before preparation; adapters do not invent defaults.

`prepare_robot_model()` produces an immutable `PreparedRobotModel` containing the final description, ordered joint space, resolved groups, and setup config. Worlds, kinematics, planners, trajectory parametrizers, and visualization consume that result. `RobotModelConfig` no longer duplicates arrays or scalar defaults for joint limits.

`JointState` remains the transport and hardware-feedback type. Canonical planning math uses `JointConfiguration` and `JointTangent`. Circular configurations normalize to `[-pi, pi)`, use the shortest delta, and are lifted only when a path crosses into trajectory generation.

Backend policy is adapter-local:

- Pink maps every `CIRCLE` scalar to Pinocchio cosine/sine values and repairs every `LINE` parser limit.
- Drake retains its native scalar coordinates but uses the canonical space for domains and limits.
- RoboPlan uses its native planner only for interval-only selections. Selections containing `LINE` or `CIRCLE` coordinates use dimOS RRT while the RoboPlan scene still supplies collision and kinematics queries.
- Viser chooses controls from topology: bounded sliders for intervals, number inputs for lines, and wrapped sliders for circles.

Execution capability is separate. A trajectory is rejected only when it requires a controller capability the configured robot does not have. Circular topology alone does not make an ordinary arm trajectory unexecutable.

## Consequences

- Joint behavior is consistent across backends and applies to arbitrary continuous or unbounded joints.
- Planner domains are finite request-local artifacts, never physical limits.
- Robot recipes must state acceleration-default policy explicitly when upstream assets omit it.
- Backend adapters become simpler but must translate their native representation at the canonical boundary.
- Existing configs that supplied duplicate limit fields must be corrected; no compatibility path is retained.

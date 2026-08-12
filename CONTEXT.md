# DimOS

DimOS composes robot capabilities from modules that communicate through typed streams and RPC interfaces.

## Manipulation language

**Plan operation**:
An operation whose name begins with `plan_` and generates a pending plan without moving hardware.
_Avoid_: Move, execute

**Move operation**:
An operation whose name begins with `move_` and plans and dispatches motion as one call.
_Avoid_: Plan, preview

**Pending plan**:
A successfully generated manipulation plan awaiting its single dispatch attempt. Any dispatch attempt consumes it, whether accepted or rejected.
_Avoid_: Cached plan, last plan

**Plan snapshot**:
An informational copy of a generated plan returned to a caller. It describes the pending plan but cannot be dispatched.
_Avoid_: Plan handle, executable plan

**Agent-readable result**:
A typed RPC result whose direct Python display concisely exposes its operational status and essential context while retaining public fields for deeper inspection.
_Avoid_: Pretty string, log message

**Planning-group state snapshot**:
A low-frequency point-in-time view of every planning group exposed by manipulation, intended for inspection and agent reasoning. High-rate or historical state belongs to typed state streams and Memory2.
_Avoid_: Robot state, robot telemetry, state stream

**Dispatch**:
An attempt to submit a pending plan for trajectory execution. Acceptance starts trajectory execution; rejection leaves no pending plan.
_Avoid_: Execute, send

**Trajectory execution**:
The lifecycle of an accepted trajectory from acceptance until completion, abort, or fault. At most one trajectory execution may be active at a time.
_Avoid_: Motion, dispatch

**Linear move**:
A world-relative Cartesian translation that preserves the end-effector orientation and follows a straight path to a relative endpoint. It may be planned with or without planning-world collision checks.
_Avoid_: Velocity pulse, Cartesian servo

**Pose-targetable planning group**:
A planning group with a defined end-effector tip that can receive Cartesian pose or linear-move targets. A caller may omit its ID only when exactly one such group exists.
_Avoid_: Default robot, arm selector

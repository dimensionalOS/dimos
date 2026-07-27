# Use one trajectory task per control coordinator

A control coordinator owns exactly one trajectory task as the execution authority for planned motion across its complete robot set. It rejects registration of a second trajectory task. This prevents separate callers from independently executing concurrent plans against subsets of a coordinated system and makes execution acceptance and cancellation atomic at the coordinator seam.

The coordinator exposes trajectory-specific `execute_trajectory(trajectory)` and `cancel_trajectory()` operations that delegate to its sole trajectory task. Callers of the supported planned-execution API do not identify that task by name. The existing reflective `task_invoke()` behavior remains unchanged for compatibility; making task command declarations an enforced RPC allowlist is a separate API-design concern.

`PlanExecutionManager` depends directly on `ControlCoordinatorClient`; it does not introduce a protocol or adapter for this fixed backend.

Callers may submit trajectories for a subset of the domain without knowing the task's complete joint set. The trajectory task retains its existing subset behavior: it validates that every submitted joint belongs to its configured claim and emits commands only for the submitted joints.

Before accepting a trajectory, the trajectory task also verifies that its first point is sufficiently close to the coordinator's current positions for every submitted joint. The coordinator supplies the authoritative current-position snapshot; this safety rule does not depend on the trajectory's producer or on planning-group metadata.

The start-position tolerance is a scalar trajectory-task parameter, defaults to `0.05` in each joint's native position unit, and rejects execution when a submitted joint has no current position.

The trajectory RPCs return typed semantic results for known coordinator outcomes, including rejection reasons and cancellation of an already-stopped task. Transport failures remain exceptions and are represented as uncertain only at the caller boundary.

`PlanExecutionManager` serializes execute and cancel operations with one lock, maps a generated trajectory once, and makes one coordinator RPC. It does not track tasks, split plans by robot, pre-cancel replacement trajectories, or perform rollback. A new execute relies on the trajectory task's existing replacement behavior.

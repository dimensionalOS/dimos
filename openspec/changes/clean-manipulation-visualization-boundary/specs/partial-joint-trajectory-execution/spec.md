## ADDED Requirements

### Requirement: Existing trajectory tasks must accept configured joint subsets
`JointTrajectoryTask` MUST accept a trajectory only when its joint names are a non-empty, unique subset of the task's configured joints and every trajectory point matches that active joint width. Unknown, duplicate, empty, malformed, non-finite, or invalidly timed input MUST be rejected before execution state changes.

#### Scenario: Selected planning group trajectory is submitted
- **WHEN** a task configured for a complete arm receives a valid trajectory containing only that arm's selected planning-group joints
- **THEN** the existing task accepts and executes the trajectory

#### Scenario: Unknown joint is submitted
- **WHEN** a trajectory contains a joint outside the task configuration
- **THEN** the task rejects it without entering executing state

#### Scenario: Point width is malformed
- **WHEN** a trajectory point's positions or velocities do not match the trajectory joint-name count
- **THEN** the task rejects it before storing the trajectory

### Requirement: Partial trajectory output must command only active joints
While executing a partial trajectory, `JointTrajectoryTask` MUST emit `JointCommandOutput` names and values only for the trajectory's active joint subset. It MUST NOT fill or command configured joints omitted from the trajectory.

#### Scenario: Partial trajectory is sampled
- **WHEN** a two-joint trajectory executes on a six-joint configured task
- **THEN** each output contains exactly those two joint names and two sampled positions

#### Scenario: Full trajectory is sampled
- **WHEN** a trajectory contains every configured task joint
- **THEN** existing full-width output behavior remains unchanged

### Requirement: Resource claims must remain configuration-wide
`JointTrajectoryTask.claim()` MUST continue to claim the task's full configured joint set regardless of the active trajectory subset.

#### Scenario: Partial trajectory executes
- **WHEN** the active trajectory omits configured joints
- **THEN** arbitration and preemption still use the task's full configured resource claim

### Requirement: Partial trajectory lifecycle must clear active names safely
Replacement, completion, cancellation, reset, and fault paths MUST keep active trajectory joint names consistent with the stored trajectory and MUST NOT leak names from a prior execution.

#### Scenario: Partial trajectory is replaced
- **WHEN** an active subset trajectory is replaced by a different valid subset
- **THEN** subsequent outputs use only the replacement trajectory's names

#### Scenario: Task resets
- **WHEN** a completed or aborted task resets
- **THEN** its stored trajectory and active subset are cleared while its configured claim remains unchanged

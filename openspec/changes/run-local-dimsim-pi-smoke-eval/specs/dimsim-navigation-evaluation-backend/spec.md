## ADDED Requirements

### Requirement: Current-compatible closed-loop DimSim robot backend
The DimSim evaluation blueprint SHALL use current DimOS perception, mapping, spatial-memory, navigation, and robot interfaces while supplying functional RGB, depth or point-cloud, odometry, and motion-command behavior. It MUST NOT restore obsolete manipulation, visualization, custom-agent, or duplicate navigation architecture from historical branches.

#### Scenario: Navigation command affects simulation
- **WHEN** a policy invokes a deployed navigation or motion RPC
- **THEN** the simulated robot receives the command and current sensor and odometry streams reflect the resulting motion

### Requirement: External-Pi evaluation blueprint
The system SHALL provide a manually launchable DimSim evaluation blueprint containing the current closed-loop spatial stack, configured agent-visible observation recorder, `CodePolicyModule`, and `McpServer`. The blueprint MUST NOT contain an internal `McpClient` or another LLM agent competing with the evaluated external Pi session.

#### Scenario: Evaluation blueprint is ready
- **WHEN** the blueprint starts successfully
- **THEN** its normal DimOS skills and RPCs are reachable through code policy, its configured observation streams are recorded, and its MCP server exposes `python_exec`

### Requirement: Authoritative correlated reset
The DimSim backend SHALL accept a reset request bound to attempt, operation, selected generated task, source scene/profile, episode, and requested start-pose identities; apply the reset to the authoritative simulated body; clear residual motion; publish the resulting pose and odometry; and return the actual applied pose and reset generation in a correlated acknowledgement. The requested pose SHALL come from the semantic profile identified by the generated contract rather than an evaluator fixture.

#### Scenario: Reset succeeds
- **WHEN** the runner requests the canonical bathtub episode reset
- **THEN** DimSim applies the deterministic start pose, clears velocity, publishes the reset state, and acknowledges the actual applied pose using the matching identities

#### Scenario: Reset acknowledgement is inconsistent
- **WHEN** the acknowledgement has a mismatched identity, scene, pose outside tolerance, or stale reset generation
- **THEN** the runner rejects the reset and does not dispatch the task

#### Scenario: Reset does not reproduce the generated source
- **WHEN** a fresh private oracle export after reset has a different scene, upstream, profile, reset, or content revision from the selected generated contract
- **THEN** the runner rejects the reset as incompatible and does not dispatch the task

### Requirement: Initially unsatisfied navigation episode
After reset and before task dispatch, the private DimSim evaluator SHALL verify that the episode success predicate is false. An initially satisfied or unevaluable predicate SHALL be an infrastructure failure.

#### Scenario: Robot starts at the goal
- **WHEN** the reset pose already satisfies the bathtub stopping predicate
- **THEN** evaluation preparation fails and Pi is not started

### Requirement: Canonical bathtub smoke episode
The backend SHALL support the generated destination episode whose public instruction is exactly “Go to the bathtub and stop within 1 meter of its outer edge.” The selected generated contract SHALL supply the canonical apartment and bathtub binding, world-frame robot-footprint surface-distance policy, threshold, linear/angular velocity tolerances, and stationary dwell duration. A compatible semantic profile SHALL supply the deterministic start pose. The evaluation binding SHALL add only the 180-second episode deadline and MUST NOT override generated predicate fields.

#### Scenario: Robot stops beside the bathtub
- **WHEN** the robot footprint remains within the generated threshold of the generated target's outer footprint and below both generated velocity tolerances for the generated continuous simulated-time dwell duration
- **THEN** the native evaluator reports task success

#### Scenario: Robot passes through the region
- **WHEN** the robot enters the one-metre region without satisfying the stationary dwell
- **THEN** the evaluator does not report success

### Requirement: Private native evaluation
The DimSim evaluator SHALL privately observe authoritative scene geometry, target identity, robot pose, and robot velocity and SHALL execute the selected generated navigation contract without publishing those facts to agent-visible DimOS streams, code-policy memory, Pi prompts, or `python_exec` results.

#### Scenario: Agent operates without rubric feedback
- **WHEN** an episode is active
- **THEN** Pi can use ordinary robot observations and RPC results but cannot read the target binding, current distance, velocity thresholds, dwell progress, or predicate state

### Requirement: Correlated native result
DimSim SHALL emit at most one terminal result for the active evaluation identity containing native pass/fail status, terminal or failure stage, reason, duration, and available metric evidence including final distance and stationary-dwell state. The generic runner SHALL reject malformed, stale, duplicate-conflicting, or identity-mismatched results.

#### Scenario: Successful result arrives
- **WHEN** the bathtub predicate becomes satisfied
- **THEN** DimSim emits one correlated passing result with the native rubric evidence

#### Scenario: Timeout result arrives
- **WHEN** the episode deadline expires without success while the evaluator remains healthy
- **THEN** DimSim emits one correlated failing result distinguishable from an infrastructure failure

### Requirement: Evaluation abort and cleanup
The backend SHALL support idempotent cancellation and cleanup for an active evaluation identity. Cancellation SHALL stop rubric monitoring and release evaluation state without stopping the attached DimSim or DimOS process.

#### Scenario: Runner is interrupted
- **WHEN** the runner cancels the active evaluation
- **THEN** DimSim stops monitoring that run, rejects or ignores later stale terminal messages, and remains available for a subsequent reset

### Requirement: Reference behavior without branch merge
The implementation SHALL translate the relevant behavioral contracts demonstrated by DimSim PR 3249 onto current repository APIs. It MUST NOT merge or cherry-pick that PR or the code-as-policy reference branch as part of this change.

#### Scenario: Historical implementation differs from current APIs
- **WHEN** a referenced module, blueprint, manipulation, visualization, or spatial interface is obsolete
- **THEN** the implementation preserves the required externally observable behavior using current interfaces rather than restoring the obsolete structure

### Requirement: Real manual smoke evidence
The change SHALL include a documented manual procedure that attaches the local evaluator to the running DimSim evaluation blueprint and produces one complete bathtub attempt directory. Change completion SHALL require executing that procedure successfully at the infrastructure level; task pass is recorded independently and is not required.

#### Scenario: Agent fails but framework is valid
- **WHEN** the real Pi attempt runs to a valid native timeout and all required artifacts finalize
- **THEN** the manual smoke satisfies the infrastructure acceptance criterion while recording task failure

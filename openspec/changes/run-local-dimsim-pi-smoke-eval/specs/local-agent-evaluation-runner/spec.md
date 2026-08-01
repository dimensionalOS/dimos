## ADDED Requirements

### Requirement: Single-attempt local runner
The system SHALL provide a dedicated local agent-evaluation command that resolves one strict configuration containing a completed DimSim release root and selected task ID, creates one fresh non-overwriting attempt directory, and attaches to already running DimOS and simulator services. The initial runner MUST NOT launch or stop those services, schedule multiple jobs, retry an attempt, or run attempts concurrently.

#### Scenario: One configured attempt starts
- **WHEN** the user runs the command with a valid smoke configuration and ready attached services
- **THEN** the runner reserves a fresh attempt identity and executes exactly one episode

#### Scenario: Existing attempt is protected
- **WHEN** an output path or attempt identity would overwrite an existing attempt
- **THEN** the runner fails before simulator reset and preserves the existing data

### Requirement: Generated task selection
The runner SHALL use the canonical DimSim full-release loader and existing generation models to select exactly one joined public task, private contract, and expected outcome. For this smoke, it SHALL accept only a destination `PublicTask`, `NavigateContract`, and compatible terminal navigation outcome from a complete supported release. It MUST NOT define a duplicate task schema, use a checked-in bathtub fixture, or permit evaluation configuration to override contract-owned target, metric, threshold, stopped-state, or provenance fields.

#### Scenario: Destination task is selected
- **WHEN** a complete release contains the configured destination task exactly once with compatible joined records and supported source revisions
- **THEN** the runner retains the exact public record for Pi and gives only the private contract plus evaluation-owned episode binding to the backend

#### Scenario: Release is incompatible
- **WHEN** the release is incomplete, internally inconsistent, selects a non-destination task, or uses an unsupported source/profile/predicate revision
- **THEN** the runner fails before simulator reset and records no Pi session

### Requirement: Narrow simulator backend contract
The generic runner SHALL depend on a backend interface for readiness, authoritative reset, evaluation start, result waiting, cancellation, and runner-owned cleanup. Episode references and native evaluator results SHALL remain opaque backend values except for the normalized fields required by the generic outcome.

#### Scenario: Backend-specific result is normalized
- **WHEN** a backend returns a valid correlated native terminal result
- **THEN** the runner retains the native result unchanged and derives the generic attempt status, task result, terminal reason, duration, and native-result reference

### Requirement: Ordered episode preparation
The runner SHALL validate and select the generated task, complete MCP and backend readiness, reset the code-policy session, authoritatively reset the simulator, validate the reset against a fresh private oracle view and selected source provenance, and start native evaluation before dispatching the public task to Pi. Preparation operations SHALL use infrastructure timeouts separate from the episode budget.

#### Scenario: Task follows verified reset
- **WHEN** preparation succeeds
- **THEN** the public task is dispatched only after a correlated reset receipt proves the requested episode state and reports the initial predicate unsatisfied

#### Scenario: Preparation fails
- **WHEN** any required readiness, session-reset, simulator-reset, or evaluation-start step fails
- **THEN** Pi is not started and the attempt terminates as an infrastructure failure with retained diagnostics

### Requirement: Fresh native Pi session
Each attempt SHALL create a fresh Pi SDK session using the configured model, thinking level, system prompt, authentication mode, and the exact public task text. Credentials MUST remain out of protocol frames and artifacts; artifacts SHALL record only the authentication mode and a safe credential-binding digest.

#### Scenario: Session input is retained
- **WHEN** Pi starts an attempt
- **THEN** the native Pi session, exact system prompt, exact initial task prompt, resolved model settings, and session receipt are retained under the attempt

#### Scenario: Private task data is excluded
- **WHEN** the Pi prompt and tool definitions are built
- **THEN** they contain no private entity binding, start-pose oracle, threshold, target distance, rubric state, or expected tool sequence

### Requirement: Episode-scoped Pi continuation
One Pi session SHALL remain active for the entire episode. If a Pi turn ends before success, the runner SHALL wait while robot motion remains active and SHALL issue the neutral continuation “Continue working on the task.” after motion becomes idle. Evaluator progress or directional hints MUST NOT be included.

#### Scenario: Robot continues after a Pi turn
- **WHEN** Pi ends a turn while robot motion remains active
- **THEN** the runner keeps evaluating without sending another prompt until motion becomes idle or the episode terminates

#### Scenario: Idle incomplete task continues
- **WHEN** motion becomes idle, the native evaluator has not succeeded, and time remains
- **THEN** the runner continues the same Pi session with the neutral continuation

#### Scenario: Repeated no-policy response
- **WHEN** Pi completes two consecutive turns without invoking `python_exec`
- **THEN** the episode ends as a completed task failure rather than consuming the remaining timeout

### Requirement: Authoritative episode termination
The episode SHALL terminate on the first correlated native success, episode deadline, or unrecoverable infrastructure failure. Pi text MUST NOT determine task success. On termination the runner SHALL best-effort abort the Pi turn, interrupt active code execution, cancel native evaluation, and invoke robot-motion cancellation where available.

#### Scenario: Native evaluator succeeds
- **WHEN** the backend reports correlated task success
- **THEN** the runner records success before cancelling remaining agent work and finalizing the attempt

#### Scenario: Episode times out
- **WHEN** the episode deadline expires without native success or infrastructure loss
- **THEN** the runner records a completed task failure with terminal reason `episode_timeout`

#### Scenario: Infrastructure is lost
- **WHEN** the Pi session, code-policy kernel, MCP connection, simulator connection, or native-result validation fails irrecoverably
- **THEN** the runner records an infrastructure failure and task result `not_evaluated`

### Requirement: Separate infrastructure and task outcomes
The normalized outcome SHALL represent `attempt_status` as `completed` or `failed` independently from `task_result` as `passed`, `failed`, or `not_evaluated`. Command exit code zero SHALL mean a valid completed evaluation with complete required artifacts regardless of task pass/fail; a nonzero exit SHALL mean infrastructure or artifact failure.

#### Scenario: Agent fails a valid episode
- **WHEN** the episode times out after valid preparation and evidence collection
- **THEN** the outcome is `attempt_status=completed`, `task_result=failed`, and the command exits zero

#### Scenario: Reset fails
- **WHEN** authoritative reset cannot be validated
- **THEN** the outcome is `attempt_status=failed`, `task_result=not_evaluated`, and the command exits nonzero

### Requirement: Append-only lifecycle evidence
The runner SHALL write ordered lifecycle events using UTC timestamps and monotonic durations for attempt creation, readiness, session reset, simulator reset, evaluation start, Pi start, continuations, terminal detection, cancellation, artifact creation, and finalization. Operation IDs and the attempt ID SHALL correlate reset, evaluation, and terminal messages; stale messages MUST be ignored.

#### Scenario: Stale result arrives
- **WHEN** the runner receives a reset acknowledgement or evaluation result whose operation identity does not match the active attempt
- **THEN** it records or ignores the stale message without changing the active episode state

### Requirement: Minimal private attempt artifacts
A finalized attempt SHALL retain `attempt-manifest.v1.json`, the exact selected public record as `task.v1.json`, `events.jsonl`, the native Pi session and prompt sidecars, `code-policy-calls.jsonl`, `dimsim-result.v1.json` or the backend-equivalent native result, and `outcome.v1.json`. The manifest SHALL identify the source release, selected private contract and outcome by stable identity and digest, and the verified post-reset source revisions without copying the complete scene oracle. The outcome SHALL identify required-artifact completeness and reference retained native evidence by relative path and digest.

#### Scenario: Completed attempt is inspectable
- **WHEN** an attempt completes successfully at the infrastructure level
- **THEN** every required artifact validates, references the same attempt/task/session identities, and can be inspected without external publication machinery

#### Scenario: Attempt fails partway
- **WHEN** an infrastructure or interruption failure occurs after evidence was created
- **THEN** all valid partial evidence is preserved and a best-effort failure outcome is written without fabricating missing artifacts

### Requirement: Atomic terminal outcome and exclusive execution
The runner SHALL use a local exclusive lock to prevent concurrent attempts against the attached stack and SHALL write the terminal outcome atomically without overwriting an existing outcome. It MUST NOT report infrastructure success if the terminal outcome cannot be durably written.

#### Scenario: Another attempt is active
- **WHEN** a second runner tries to use the same attached evaluation target
- **THEN** it fails before reset with no second active episode

#### Scenario: Outcome write fails
- **WHEN** the runner cannot atomically persist a valid terminal outcome
- **THEN** it exits nonzero and preserves the attempt directory and all previously written evidence

### Requirement: Attached-service interruption behavior
On normal completion, error, or user interruption, the runner SHALL release only resources that it owns. It MUST leave the manually started DimOS and simulator processes running.

#### Scenario: User interrupts an active attempt
- **WHEN** the runner receives an interruption
- **THEN** it cancels its Pi/evaluation work, retains partial evidence, releases its lock, and does not stop the attached stack

## ADDED Requirements

### Requirement: Single code-policy skill
The system SHALL expose agent-authored policy execution through exactly one model-facing skill named `python_exec` with a complete Python source parameter and a bounded execution-time parameter. The evaluated Pi session MUST NOT register the other skills reported by the attached DimOS MCP server.

#### Scenario: Pi receives the approved tool
- **WHEN** the evaluator creates a Pi session against a compatible running blueprint
- **THEN** the Pi session receives exactly the validated `python_exec` tool schema and no other tool

#### Scenario: Attached server exposes additional skills
- **WHEN** the MCP server reports `python_exec` plus other deployed skills
- **THEN** the host adapter records the reported inventory but registers only `python_exec` with Pi

### Requirement: Full DimOS access through generated code
The code-policy session SHALL preload `app` as a connected DimOS client supporting both `app.skills` and `app.get_module`, so generated Python can call all normally deployed agent skills and RPCs. This access MUST occur inside `python_exec`; the same capabilities MUST NOT be registered as separate Pi tools.

#### Scenario: Policy coordinates multiple robot capabilities
- **WHEN** generated Python calls multiple skills and module RPCs through `app`
- **THEN** the calls execute through the deployed DimOS runtime and their results are available to the same Python program

### Requirement: Agent-visible observation memory
The code-policy session SHALL preload a readable `memory` store containing the configured agent-visible navigation streams for the current blueprint run. Evaluator, reset, private contract, native rubric, and simulator-oracle data MUST NOT be recorded in that store.

#### Scenario: Policy reads current navigation observations
- **WHEN** generated Python queries a configured camera, odometry, map, spatial-memory, or navigation-status stream
- **THEN** it can read the observations retained for the current blueprint run

#### Scenario: Oracle data remains outside memory
- **WHEN** the private evaluator observes target identity, distance, velocity, or predicate state
- **THEN** none of those private evaluator records appear in the code-policy memory store

### Requirement: Persistent attempt-local Python session
The code-policy module SHALL use one persistent Jupyter/IPython kernel within an evaluation attempt so imports, functions, variables, and mutations survive across `python_exec` calls. At most one code cell SHALL execute at a time.

#### Scenario: State persists within an attempt
- **WHEN** one successful call defines a helper or variable and a later call references it
- **THEN** the later call observes the prior definition

#### Scenario: Concurrent execution is refused
- **WHEN** a second `python_exec` call arrives while a cell is active
- **THEN** the module refuses the second call without executing it

### Requirement: Explicit session reset
The code-policy module SHALL provide a non-skill reset RPC that terminates the existing kernel, clears all Python state, and returns a fresh session identity. The evaluator SHALL invoke this RPC before each attempt and bind all subsequent code-policy evidence to the returned identity.

#### Scenario: Attempts cannot share Python state
- **WHEN** the evaluator resets the code-policy session between two attempts
- **THEN** definitions and mutations from the first attempt are absent from the second attempt

#### Scenario: Reset establishes evidence identity
- **WHEN** session reset succeeds
- **THEN** the receipt identifies the fresh session used by later execution records

### Requirement: Bounded synchronous execution
Each `python_exec` call SHALL synchronously return a bounded plain-text transcript containing stdout, stderr, the final plain-text expression, or traceback. A call timeout MUST interrupt the cell, report whether the namespace survived, and MUST NOT claim that already-issued remote RPC work was cancelled.

#### Scenario: Program completes
- **WHEN** a submitted program completes before its deadline
- **THEN** the skill returns its bounded transcript with execution identity and completion status

#### Scenario: Program times out
- **WHEN** a submitted program exceeds the allowed per-call timeout
- **THEN** the active cell is interrupted and the result states whether the namespace was preserved and that remote RPC work may still be active

### Requirement: Code-policy execution evidence
The system SHALL retain, for every attempted code-policy call, the session identity, submitted source, Jupyter execution identity when available, UTC and monotonic timing, bounded output or traceback, completion status, timeout/interrupt/restart state, and transcript returned to Pi.

#### Scenario: Execution is reviewable
- **WHEN** an attempt contains one or more `python_exec` calls
- **THEN** the attempt artifacts contain a chronologically ordered record sufficient to reconstruct what code ran and what Pi received

### Requirement: Trusted simulation-only boundary
The initial code-policy runtime SHALL be explicitly documented and configured as unsandboxed trusted execution for simulation. It MUST NOT claim filesystem, environment, network, package-import, or RPC isolation.

#### Scenario: Smoke manifest records trust mode
- **WHEN** a local evaluation attempt is created
- **THEN** its manifest records that code policy is trusted and unsandboxed and that oracle separation is logical rather than a security guarantee

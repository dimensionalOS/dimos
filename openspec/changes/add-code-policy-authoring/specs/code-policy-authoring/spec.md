## ADDED Requirements

### Requirement: Explicit evaluation session schemes
The CodePolicy session SHALL expose `answer()` for direct-text evaluations and `author_policy()` for executable-policy evaluations, and SHALL accept exactly one of these top-level operations during a session.

#### Scenario: Direct-answer evaluation
- **WHEN** an evaluation calls `answer()` with a non-empty evaluation protocol and task input
- **THEN** the runtime SHALL perform one Pi authoring invocation and return its final text in an `AgentOutcome`

#### Scenario: Policy-authoring evaluation
- **WHEN** an evaluation calls `author_policy()` with a non-empty evaluation protocol, task input, and positive round limit
- **THEN** the runtime SHALL run the bounded policy authoring and validation flow and return a `PolicyOutcome`

#### Scenario: Reusing a consumed session
- **WHEN** an evaluation calls either top-level operation after the session has already performed one
- **THEN** the runtime SHALL reject the second operation

### Requirement: Source policy contract
An authored policy candidate SHALL be one successfully executed Python cell containing a module-level synchronous function named `policy` with no parameters. The complete cell SHALL be retained as the candidate source.

#### Scenario: Agent revises a candidate
- **WHEN** the agent successfully executes multiple cells containing conforming `policy()` definitions
- **THEN** the runtime SHALL select the most recently executed conforming cell as the candidate

#### Scenario: Candidate includes its helpers
- **WHEN** the selected cell contains imports, constants, or helper functions in addition to `policy()`
- **THEN** the runtime SHALL retain the entire cell as the policy source

#### Scenario: No candidate was defined
- **WHEN** an authoring round ends without a successful cell containing a conforming `policy()` definition
- **THEN** validation SHALL fail with mechanical feedback that a zero-argument `policy()` must be defined

### Requirement: Clean policy validation
The runtime SHALL validate a candidate by replaying its source in a clean Python kernel configured with the same evaluation environment, invoking `policy()`, and JSON-serializing its return value.

#### Scenario: Self-contained policy succeeds
- **WHEN** candidate replay and `policy()` execution succeed and the return value is JSON-serializable
- **THEN** the runtime SHALL mark the policy valid and return the decoded JSON value in `PolicyOutcome`

#### Scenario: Candidate depends on authoring state
- **WHEN** candidate replay fails because the cell depends on state from an earlier authoring cell
- **THEN** the runtime SHALL mark the validation attempt invalid and provide the replay error as mechanical feedback

#### Scenario: Policy execution fails
- **WHEN** invoking `policy()` raises an exception or exceeds the execution timeout
- **THEN** the runtime SHALL mark the validation attempt invalid and provide a bounded execution error as mechanical feedback

#### Scenario: Policy result is not JSON-compatible
- **WHEN** `policy()` returns a value that cannot be serialized as JSON
- **THEN** the runtime SHALL mark the validation attempt invalid and explain the JSON result requirement

### Requirement: Bounded mechanical repair
The policy-authoring flow SHALL permit no more than the requested positive number of authoring rounds, SHALL preserve the authoring Python namespace between rounds, and SHALL stop at the first mechanically valid candidate.

#### Scenario: Invalid candidate is repaired
- **WHEN** clean validation fails before the final allowed round
- **THEN** the runtime SHALL invoke Pi again with the original evaluation context, candidate source, and mechanical validation error while retaining the authoring kernel

#### Scenario: Candidate validates
- **WHEN** a candidate passes clean validation
- **THEN** the runtime SHALL freeze that candidate and SHALL NOT start another authoring round

#### Scenario: Round budget is exhausted
- **WHEN** no candidate passes clean validation within the allowed rounds
- **THEN** the runtime SHALL return an invalid `PolicyOutcome` with the final candidate and validation error instead of reporting an infrastructure failure

### Requirement: Native scores remain hidden from authoring
The CodePolicy runtime SHALL NOT receive or expose native benchmark correctness while authoring or validating a policy, and an evaluation SHALL score no more than the one frozen policy result returned by `author_policy()`.

#### Scenario: Mechanically valid result is semantically wrong
- **WHEN** `policy()` returns a JSON value successfully but the evaluation's native scorer finds that value incorrect
- **THEN** the evaluation SHALL report the native failure without starting another authoring round

#### Scenario: Multiple invalid attempts precede success
- **WHEN** mechanical validation fails in earlier rounds and succeeds in a later round
- **THEN** the evaluation SHALL score only the result of the frozen successful candidate and SHALL NOT select a best attempt

### Requirement: Policy authoring evidence
The runtime SHALL record enough evidence to reproduce policy selection and validation while preserving the existing prompt and transcript evidence.

#### Scenario: Valid policy evidence
- **WHEN** policy authoring produces a valid candidate
- **THEN** the runtime SHALL record each round transcript, the selected `policy.py`, validation history, and the JSON policy result as evaluation artifacts

#### Scenario: Invalid policy evidence
- **WHEN** the authoring round budget ends without a valid candidate
- **THEN** the runtime SHALL record each round transcript, validation history, and the final candidate source when one exists

### Requirement: Frozen integer QA uses policy results
The frozen integer QA evaluation SHALL author one policy for one frozen recording, cutoff, and question, and SHALL use the valid policy result as the only input to native correctness scoring.

#### Scenario: Policy returns an integer
- **WHEN** a valid policy returns a JSON integer
- **THEN** frozen integer QA SHALL compare that integer with the existing oracle through its native exact-match scorer

#### Scenario: Policy result is invalid for integer QA
- **WHEN** policy authoring is invalid or a valid policy returns a JSON value that is not an integer
- **THEN** frozen integer QA SHALL report a normal native benchmark failure without exposing the oracle or retrying authoring

#### Scenario: Frozen environment access
- **WHEN** Pi authors or clean validation executes the policy for frozen integer QA
- **THEN** the Python kernel SHALL expose the same read-only frozen `memory` configured for that case

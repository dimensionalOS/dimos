## ADDED Requirements

### Requirement: Minimal manipulation exploration contract
The system SHALL provide the manipulation Agent only the exact task instruction, the callable and harness workflow contract, and discoverable non-privileged capabilities. It SHALL NOT provide task-solving recipes, an editable skillbook, or secondary verifier and diagnoser model output.

#### Scenario: Exploration starts for a LIBERO task
- **WHEN** the code-policy runtime assembles the manipulation exploration prompt
- **THEN** the prompt identifies the task, submission budget, evidence access, and explicit freeze operation without prescribing perception, grasping, or motion strategies

### Requirement: Immutable policy candidates
The Manipulation Agent Harness SHALL turn every accepted submission into a uniquely identified immutable Policy Candidate containing the submitted policy, its Debug Trial, and its Trial Evidence. A later submission SHALL NOT replace or mutate an earlier candidate.

#### Scenario: Agent submits multiple policies
- **WHEN** the Agent submits two valid policy callables within its budget
- **THEN** the harness returns two distinct candidates and preserves both policies and both trial records for later inspection and selection

### Requirement: Explicit policy freezing
The Agent MUST explicitly freeze one submitted Policy Candidate before exploration ends. Freezing SHALL select that candidate as the Policy Artifact, prevent further submissions, and be irreversible for that exploration. The runtime SHALL NOT select the latest candidate or automatically select by debug score.

#### Scenario: Agent freezes an earlier candidate
- **WHEN** the Agent submits a diagnostic candidate after a working candidate and then freezes the earlier candidate
- **THEN** the earlier candidate becomes the Policy Artifact evaluated on the scored case

#### Scenario: Agent does not freeze a candidate
- **WHEN** the Agent session ends after one or more submissions without calling the freeze operation
- **THEN** exploration is invalid and no Policy Artifact is evaluated

#### Scenario: Agent submits after freezing
- **WHEN** the Agent attempts another submission after freezing a candidate
- **THEN** the harness rejects the submission without changing the frozen Policy Artifact

### Requirement: Deterministic Trial Evidence summary
Each Policy Candidate SHALL expose a bounded Trial Evidence summary produced only from non-privileged recorded facts. The summary SHALL identify the candidate, native debug outcome, execution status, duration, remaining budget, available evidence categories, and initial or terminal visual evidence when recorded. Unknown facts SHALL remain unknown rather than being inferred.

#### Scenario: Failed trial has incomplete structured events
- **WHEN** a Debug Trial fails but its raw artifacts do not identify a physical root cause
- **THEN** the summary reports the recorded failure status and available evidence without inventing a diagnosis or repair

### Requirement: On-demand evidence drilldown
Trial Evidence SHALL provide deterministic on-demand access to event timelines, filtered module logs, selected visual frames, policy output, read-only Memory2, and complete raw artifact references. Automatic summaries SHALL NOT make the underlying evidence inaccessible.

#### Scenario: Agent investigates a planning failure
- **WHEN** the Agent requests planning-related logs and frames from a Policy Candidate
- **THEN** the harness returns the matching recorded evidence or explicitly reports that the requested category is unavailable

### Requirement: Fixed submission budget
The Manipulation Agent Harness SHALL enforce the evaluation-owned submission budget across all Policy Candidates and SHALL report the remaining budget after each accepted submission.

#### Scenario: Submission budget is exhausted
- **WHEN** the Agent attempts to submit more candidates than the Evaluation permits
- **THEN** the harness rejects the excess submission and preserves all previously accepted candidates and any frozen selection

### Requirement: Frozen policy execution isolation
The Evaluation Stage SHALL execute only the explicitly frozen Policy Artifact in the existing clean policy process and SHALL exclude the exploration Agent and harness mutation state from measured execution.

#### Scenario: Frozen candidate enters evaluation
- **WHEN** exploration completes with a frozen candidate
- **THEN** the evaluator loads that candidate's serialized callable behind the existing start gate and does not load the exploration Agent

## ADDED Requirements

### Requirement: Immutable attempt evidence
Each attempt SHALL retain its immutable manifest snapshot and digest, append-only lifecycle events, exactly one immutable `outcome.v1.json` terminal record when terminal, executor output, and exported evidence under an immutable attempt directory. Atomic job summaries are replaceable operational cache outside the attempt authority. A retry SHALL write a separate directory and SHALL NOT replace prior evidence.

#### Scenario: Preserve a retried attempt
- **WHEN** a completed attempt is retried
- **THEN** its event log, summary, and evidence remain readable and unchanged beside the new attempt directory

#### Scenario: Reconstruct terminal state
- **WHEN** an operational summary is missing or stale
- **THEN** reconstruction uses the immutable attempt outcome rather than inferring completion from mutable summaries or lifecycle events

### Requirement: Budgeted retention and triage
The scheduler SHALL enforce configured disk and evidence budgets, apply an explicit retention policy, and retain failure-triage metadata sufficient to distinguish pending, interrupted, failed, unscored, noncompliant, and completed outcomes without exposing private correctness in operational state.

#### Scenario: Triage an interrupted job
- **WHEN** a coordinator stops while a job is running
- **THEN** the retained state identifies the job as interrupted and records enough operational metadata for resume or an explicit retry without revealing private scores

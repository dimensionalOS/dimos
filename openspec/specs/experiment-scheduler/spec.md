# experiment-scheduler Specification

## Purpose
TBD - created by archiving change pi-spatial-agent-baseline. Update Purpose after archive.
## Requirements
### Requirement: Neutral executor contract
The scheduler SHALL invoke executors with an immutable expanded case, a named condition, and an attempt context, and SHALL normalize their lifecycle events and terminal outcome. The contract SHALL NOT prescribe universal tools, sessions, or agent abstractions; Pi SHALL be one executor implementation.

#### Scenario: Run a named condition
- **WHEN** a scheduled job is assigned to an executor
- **THEN** it receives the immutable case, condition, and attempt context and emits normalized lifecycle events and exactly one terminal outcome

### Requirement: Immutable experiment plan
An experiment SHALL be the immutable Cartesian product of its expanded case set and named conditions. Its manifest SHALL include digests for expanded plan, executor, model, prompt, tools, corpus, runner image, scorer, limits, and fixed worker count. Any execution-affecting change SHALL create a fork rather than mutate the experiment.

#### Scenario: Reject plan drift
- **WHEN** a resume request has a manifest or expanded-plan digest different from the recorded experiment
- **THEN** the scheduler refuses resume and requires a fork

#### Scenario: Reject a forged or malformed plan
- **WHEN** a stored plan has a self-attested digest that differs from its canonical parsed contents, duplicate identifiers, or an invalid job reference
- **THEN** preflight refuses execution before creating an attempt

### Requirement: Filesystem-owned bounded coordinator
One coordinator on one host-local POSIX filesystem SHALL own state and the operational UI. It SHALL use a bounded in-process thread pool, defaulting to 10 workers, with no autoscaling, database, service, multi-host coordination, or child Python worker layer. Attempt directories SHALL be immutable, events append-only, and job summaries atomically published.

#### Scenario: Recover an interrupted coordinator
- **WHEN** the coordinator is restarted on the same filesystem
- **THEN** it reconstructs operational state from immutable attempt artifacts and atomic summaries without a database or remote service

#### Scenario: Authoritative terminal outcome
- **WHEN** an attempt has a terminal outcome record
- **THEN** reconstruction uses that immutable record, while summaries remain replaceable operational cache

### Requirement: Resume and explicit retry semantics
`resume` SHALL schedule only pending or interrupted jobs and SHALL preserve the experiment manifest and worker count. `retry` SHALL require a reason, create a new attempt, and never overwrite prior evidence.

#### Scenario: Retry a failed attempt
- **WHEN** an operator retries a terminal job with a reason
- **THEN** a new attempt directory and attempt identity are created while the original evidence remains unchanged

### Requirement: Experiment command surface and operational UI
The CLI SHALL provide `pi-baseline experiment create|run|resume|retry|status` and retire `pi-baseline run-paired`. Live and static Rich output and `--json` SHALL expose operational health only, never private correctness or scores.

#### Scenario: Inspect an active experiment
- **WHEN** an operator runs status during execution
- **THEN** Rich or `--json` output contains lifecycle and operational health only, and never correctness, scores, or oracle-derived data

### Requirement: Preflight, cancellation, budgets, and deterministic selection
The scheduler SHALL support fail-closed preflight, graceful cancellation, retention policy, retry filters, failure triage, disk/evidence budgets, and deterministic plan expansion, sampling, sharding, and hashing. It SHALL defer Textual, multi-host execution, autoscaling, external orchestration frameworks, and interactive control.

#### Scenario: Reject an unsafe start
- **WHEN** preflight detects an invalid manifest, unavailable executor prerequisite, or insufficient evidence budget
- **THEN** the scheduler refuses to start jobs and records an actionable operational failure without creating partial execution state

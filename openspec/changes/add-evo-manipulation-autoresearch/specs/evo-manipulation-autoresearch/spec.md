## ADDED Requirements

### Requirement: Unmodified Evo orchestration
Manipulation autoresearch SHALL use an upstream Evo installation through its documented CLI, plugin, target, benchmark, gate, worktree, trace, and frontier contracts. DimOS SHALL NOT fork, patch, vendor, subclass, or reimplement Evo orchestration.

#### Scenario: Research workspace is initialized
- **WHEN** a user initializes manipulation autoresearch
- **THEN** the documented workflow configures the harness target and DimOS benchmark command using standard Evo commands and the built-in per-task Pareto frontier

### Requirement: Evo-compatible result publication
The manipulation panel runner SHALL implement Evo's inline result contract without requiring an Evo runtime library. It SHALL publish one atomic result containing a numeric combined score and a `tasks` mapping from Evaluation Case ID to native case score.

#### Scenario: Four-case pilot completes
- **WHEN** all four pilot Evaluation Cases produce valid native results
- **THEN** the runner publishes their macro-average as `score` and publishes each case's score under its unique case ID

#### Scenario: Evo environment variables are absent
- **WHEN** the panel runner is invoked outside Evo
- **THEN** it still writes the normal DimOS panel result and can print its JSON result without requiring an Evo package

### Requirement: Per-case diagnostic traces
The panel runner SHALL write one Evo trace per Evaluation Case with stable task ID, native score and status, failure category when applicable, runtime identity, panel hash, and references to the corresponding DimOS policy, logs, videos, Memory2, and result artifacts.

#### Scenario: A policy receives native failure
- **WHEN** an Evaluation Case completes correctly with native success false
- **THEN** its trace records a valid score of zero and references the evidence needed to investigate the failure

### Requirement: Invalid measurements fail the experiment
The panel runner SHALL distinguish valid native task failure from measurement failure. Preflight errors, infrastructure errors, missing native results, or malformed native results MUST cause a non-zero benchmark exit and MUST NOT publish a misleading combined Evo score.

#### Scenario: One case has an infrastructure failure
- **WHEN** a panel case cannot start its container or produce a valid native result
- **THEN** the Evo benchmark invocation fails and preserves diagnostic output without committing a numeric aggregate as a valid experiment outcome

### Requirement: Configurable research panels
A Research Panel SHALL contain at least one Evaluation Case with a unique ID and a safe relative specification path. The panel schema SHALL NOT require exactly four cases or one case from each hard-coded suite family.

#### Scenario: User creates a different development panel
- **WHEN** a panel lists a valid non-empty collection of unique Evaluation Cases
- **THEN** the runner evaluates those cases without code changes to Evo or the panel runner

### Requirement: Frozen panel identity
The panel runner SHALL resolve panel composition once per invocation and SHALL include a deterministic content hash of the panel and resolved case specifications in the combined result and every per-case trace. Results with different panel hashes SHALL be treated as different research baselines.

#### Scenario: Panel membership changes between runs
- **WHEN** a case is added, removed, or changed before a new Evo run
- **THEN** the new baseline receives a different panel hash and is not compared as a continuation of the previous panel

### Requirement: Four-case integration pilot
The initial documented Evo workflow SHALL use the existing goal, spatial, object, and long-horizon development cases as an integration pilot and SHALL label its four binary outcomes as pilot evidence rather than a statistically robust benchmark claim.

#### Scenario: Pilot results are reported
- **WHEN** the first Evo search reports improvement on the four-case panel
- **THEN** the documentation identifies the result as an integration finding and does not claim publication-grade manipulation performance

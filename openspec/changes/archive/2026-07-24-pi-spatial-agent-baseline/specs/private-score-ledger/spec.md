## ADDED Requirements

### Requirement: Host-side private scoring
The host SHALL score each eligible submitted prediction against the private authoritative answer outside the agent and container. A visualization-encouraged run without a successful bounded `read_generated_image` of an agent-generated `/work` image SHALL be failed and unscored. A visualization-forbidden run with any attempted image read SHALL be policy-noncompliant and ineligible for scoring. The agent and container SHALL receive no correctness, score, or authoritative-answer feedback during execution.

#### Scenario: Score a submitted answer
- **WHEN** a case produces a typed prediction
- **THEN** the host computes its result using the private answer after execution and the case runtime receives no correctness signal

#### Scenario: Exclude an image-policy failure
- **WHEN** a run is missing the required encouraged-mode image read or has a forbidden-mode image-read attempt
- **THEN** the host records the failed/unscored or policy-noncompliant outcome and does not produce an eligible score

### Requirement: Retained private ledger record
The host SHALL retain a private ledger record linking the case, prompt mode, prediction, score, and run identity. The private ledger SHALL not be included in the agent-visible or container-visible input.

#### Scenario: Review a score privately
- **WHEN** a reviewer opens the host evaluation records after a run
- **THEN** the prediction and score are available in the private ledger while they are absent from the public staging bundle and runtime transcript

## ADDED Requirements

### Requirement: Paired prompt modes
The baseline SHALL evaluate two prompt modes: one that forbids visualization and one that requires it for acceptance. Both modes SHALL include the identical behavioral instruction: "Do not use online information or services to solve the task; package installation is allowed." The prompts SHALL use these mandatory mode wordings verbatim: visualization-forbidden mode: "Visualization is forbidden. Do not call `read_generated_image`."; visualization-encouraged mode: "Visualization is required for acceptance: after at most two sandbox attempts, generate a useful PNG under `/work` and call `read_generated_image` using a relative path. The image must be an existing regular non-symlink PNG that can be decoded successfully and must satisfy the applicable configured byte, width, height, and pixel limits. After a successful read, stop exploration: do not run more analysis commands or generate more images. Use the available evidence and call `submit_answer` exactly once with your best answer, even if uncertain. Do not consume remaining turn or tool budget seeking confidence." The two modes SHALL use identical staged case data, model configuration, tool surface, container policy, network availability, and execution limits.

#### Scenario: Compare prompt modes fairly
- **WHEN** the same case is evaluated in both prompt modes
- **THEN** the only configured evaluation difference is the visualization instruction, while the identical online-use/package-installation instruction, public inputs, available tools, network availability, and limits remain unchanged

### Requirement: Named prompt conditions
Pi prompt modes SHALL be represented as named scheduler conditions and SHALL execute independently under the same neutral executor contract.

#### Scenario: Enforce visualization-encouraged acceptance
- **WHEN** a visualization-encouraged run submits an answer
- **THEN** the answer is accepted only if at least one bounded `read_generated_image` call for an agent-generated `/work` image succeeded; otherwise the run fails and is unscored

#### Scenario: Enforce visualization-forbidden compliance
- **WHEN** a visualization-forbidden run calls `read_generated_image`
- **THEN** the read is rejected and the run is marked policy-noncompliant

### Requirement: Reviewable visualization compliance
Each run SHALL retain image-tool trace and outcome evidence sufficient to review the applicable mode policy, including successful bounded reads, rejected forbidden-mode attempts, and encouraged-mode missing-read failures. Such evidence SHALL NOT claim that image inspection proves image relevance or no online information or services were used.

#### Scenario: Inspect image-policy evidence
- **WHEN** an operator examines retained run evidence
- **THEN** the retained evidence identifies the mode, prompt wording, image-read events, and scoring eligibility without asserting image relevance or offline use

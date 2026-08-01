## ADDED Requirements

### Requirement: Case-bound tool surface
The evaluation environment SHALL expose only custom tools bound to the staged case, and every exposed tool SHALL operate on that case's public data and be identifiable in the run transcript.

#### Scenario: Enumerate available tools
- **WHEN** an agent begins a case run
- **THEN** its available tool surface contains only the configured custom case-bound tools and no host Pi tools

### Requirement: Sandbox failure classification
A nonzero exit from an inner command executed successfully inside a live, ready sandbox SHALL remain a recoverable inner-command result for the caller to handle. Container death, loss of running or exec readiness, and container-runtime or control-plane errors SHALL be terminal sandbox infrastructure failures classified as `container_runtime_failed`; they SHALL not be converted into an inner-command nonzero or accepted precursor result.

#### Scenario: Distinguish command failure from sandbox failure
- **WHEN** an inner command returns nonzero while pre- and post-exec liveness checks pass
- **THEN** the broker returns a recoverable command result

#### Scenario: Terminate on sandbox failure
- **WHEN** the container dies or a runtime/control-plane error prevents a pre- or post-exec liveness/readiness check
- **THEN** the broker records terminal `container_runtime_failed` and does not return a recoverable command result

### Requirement: Immutable typed answer submission
The tool surface SHALL include one immutable, typed `submit_answer` tool. It SHALL accept the typed answer for the staged question, SHALL permit submission without mutation of the case, and SHALL provide no correctness feedback.

#### Scenario: Submit a prediction
- **WHEN** the agent calls `submit_answer` with a value of the declared answer type
- **THEN** the prediction is recorded for host-side scoring and the tool response provides no indication of whether it is correct

### Requirement: Durable terminal submission contract
The adapter SHALL treat `accepted=false` as nonterminal. A terminal `ok=true` result SHALL require an accepted submission and a submitted terminal reason; normal agent completion without that state SHALL be eligible for same-session neutral continuation until the configured global turn, tool, or wall-clock budget is exhausted or a session/protocol failure occurs. Continuations and the final terminal reason SHALL be retained in the prompt, tool, and evidence transcripts.

#### Scenario: Continue after unaccepted submission
- **WHEN** a submission is not accepted or the Pi agent completes without an accepted submission
- **THEN** the adapter keeps the same session and prompts it neutrally rather than returning terminal success

#### Scenario: Require terminal reason for success
- **WHEN** the adapter is about to return `ok=true`
- **THEN** it verifies that an accepted submission and submitted terminal reason are both present

### Requirement: Bounded generated-image reading
The tool surface SHALL include a bounded `read_generated_image` operation. The agent SHALL generate the image through container analysis in `/work` and provide a relative workspace path. The host SHALL resolve and validate containment within `/work`, accept only PNG MIME, enforce configured byte, dimension, pixel, and image-count limits, and return native image blocks without a host path or URL. The operation SHALL not be a fixed host rendering operation.

#### Scenario: Read an agent-generated image
- **WHEN** the agent calls `read_generated_image` with a valid relative `/work` path to a PNG within all configured limits
- **THEN** the host returns the image as a native image block and exposes neither a host path nor a URL

#### Scenario: Reject an unsafe image request
- **WHEN** the requested path escapes `/work`, has a non-PNG MIME, or exceeds a byte, dimension, pixel, or image-count limit
- **THEN** the host rejects the request without returning image data

### Requirement: Mode-specific image-read enforcement
The host SHALL enforce the prompt mode independently of bounded image validation. A visualization-encouraged answer SHALL be accepted only after at least one successful bounded read of an agent-generated `/work` image; without one, the run SHALL fail and be unscored. In visualization-forbidden mode, image reads SHALL be rejected and any attempted read SHALL make the run policy-noncompliant.

#### Scenario: Require a successful encouraged read
- **WHEN** an encouraged run submits without a successful bounded image read
- **THEN** the run fails and is unscored

#### Scenario: Reject a forbidden read
- **WHEN** a forbidden run requests `read_generated_image`
- **THEN** the broker rejects the request and records a policy-noncompliant attempt

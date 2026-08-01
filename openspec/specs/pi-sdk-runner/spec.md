# pi-sdk-runner Specification

## Purpose
TBD - created by archiving change pi-spatial-agent-baseline. Update Purpose after archive.
## Requirements
### Requirement: Authenticated pinned Pi execution
The baseline runner SHALL require one explicit host-side authentication mode. `codex-oauth` SHALL use stored Codex OAuth credentials and provider `openai-codex`; `openai-api-key` SHALL read only `OPENAI_API_KEY` from an owner-only dotenv file and use provider `openai`. Both modes SHALL run model `gpt-5.6-luna` with a separate medium thinking budget/level for every evaluated case. Before execution, the runner SHALL validate the resolved provider/API pairing and fail closed unless the catalog entry advertises image and reasoning capability. Credentials SHALL remain outside the container, workspace, transcript, and tool responses.

#### Scenario: Start a configured Codex OAuth run
- **WHEN** a baseline run is started in `codex-oauth` mode with valid stored Codex OAuth credentials
- **THEN** the runner launches the external Pi SDK with provider `openai-codex`, the pinned model configuration, and records the authentication mode and model configuration in the run evidence

#### Scenario: Start a configured OpenAI API-key run
- **WHEN** a baseline run is started in `openai-api-key` mode with an owner-only dotenv file containing `OPENAI_API_KEY`
- **THEN** the runner reads only that key, launches the external Pi SDK with provider `openai`, the pinned model configuration, and records the authentication mode and model configuration without recording the key

#### Scenario: Reject a model without image capability
- **WHEN** the resolved catalog entry for the pinned model does not advertise image capability
- **THEN** the runner refuses to start the session and records the catalog-validation failure

### Requirement: Neutral-executor single-mode case execution
The runner SHALL implement the neutral executor contract for one immutable case and one named condition, while retaining the hardened staging, Pi Node subprocess/session, rootless Podman, case-bound tools, evidence export, private scoring, and unconditional cleanup core. It SHALL produce normalized lifecycle events and an exported run bundle or explicit failed-run record. It SHALL NOT own experiment scheduling or universal tools/sessions abstractions.

#### Scenario: Complete a case run
- **WHEN** the runner is given a valid staged case and one supported named condition
- **THEN** it executes exactly that case and named condition and exports evidence identifying the case, condition, model configuration, and outcome

### Requirement: Same-session continuation and terminal protocol
Normal Pi agent completion without an accepted submission SHALL NOT be terminal. The adapter SHALL send a neutral continuation prompt to the SAME session, retaining its conversation and working context, until durable acceptance or exhaustion of the global configured turn, tool, or wall-clock budget, or session/protocol failure. It SHALL NOT create a new session or authentication context for continuation. `accepted=false` SHALL NOT end the run. A terminal `ok=true` result SHALL require both an accepted submission and a submitted terminal reason.

#### Scenario: Continue after normal completion
- **WHEN** the Pi agent completes normally without an accepted submission
- **THEN** the adapter prompts the same session neutrally and records the continuation rather than ending the run or creating a new session

#### Scenario: Stop at a terminal condition
- **WHEN** the session durably accepts a submission with a terminal reason, or a global budget/session/protocol failure occurs
- **THEN** the adapter ends the run with the submitted terminal reason or explicit failure outcome, and only the accepted-submission case may return `ok=true`

#### Scenario: Retain continuation evidence
- **WHEN** a run has one or more continuations
- **THEN** the Pi prompt transcript, tool transcript, and retained evidence record each continuation and the final terminal reason

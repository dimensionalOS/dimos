# pi-session-history-viewer Specification

## Purpose
TBD - created by archiving change add-pi-session-history-viewer. Update Purpose after archive.
## Requirements
### Requirement: Open one retained Pi session
The system SHALL provide `pi-baseline session view <attempt-directory>` to open exactly one admitted native Pi session for human review. The command SHALL accept a complete retained session or a valid retained partial prefix and SHALL reject unavailable, malformed, unsafe, or receipt-mismatched evidence.

#### Scenario: Open a complete retained session
- **WHEN** an operator invokes the command for an attempt with a complete native session and matching receipt
- **THEN** the system opens a viewer for that session alone

#### Scenario: Open a valid partial session
- **WHEN** an operator invokes the command for an attempt with an admitted partial native-session prefix
- **THEN** the system opens the valid retained history and identifies it as partial

#### Scenario: Reject inadmissible evidence
- **WHEN** the attempt session is unavailable, malformed, unsafe, or inconsistent with its receipt
- **THEN** the command fails before starting a viewer

### Requirement: Render only Pi-native session history
The viewer SHALL render the Pi-native conversation tree, including available user and assistant messages, thinking, tool calls and results, timestamps, usage data, state changes, and branches. It MUST NOT display benchmark prompt sidecars, predictions, scores, logs, policy audits, or other benchmark evidence.

#### Scenario: Review native history
- **WHEN** the retained session contains messages, thinking, tool activity, and branches
- **THEN** the viewer presents those native elements without adding benchmark sidecar content

#### Scenario: Attempt contains private sidecars
- **WHEN** the attempt directory also contains prompts, scores, predictions, logs, or audits
- **THEN** none of those sidecars is staged for or exposed through the viewer

### Requirement: Keep review strictly read-only
The viewer SHALL expose no operation that continues, prompts, edits, forks, renames, archives, restores, or deletes a session. The server MUST reject such mutations even if a caller bypasses the visible UI.

#### Scenario: Review through the browser
- **WHEN** an operator views a retained session
- **THEN** the UI contains no composer or session-mutation controls

#### Scenario: Call a mutating viewer route
- **WHEN** a client attempts a session mutation through the viewer server
- **THEN** the server rejects the request without changing the staged or canonical session

#### Scenario: Send a non-read HTTP method
- **WHEN** a client sends a method other than GET or HEAD to any viewer route
- **THEN** the server rejects the request without invoking a mutation handler

### Requirement: Preserve canonical session evidence
The command SHALL render a byte-identical disposable copy and MUST NOT give the viewer writable access to the canonical attempt directory. It SHALL verify the canonical session digest before launch and after shutdown.

#### Scenario: Complete a review
- **WHEN** the viewer exits normally
- **THEN** the canonical session bytes and digest match their pre-review values

#### Scenario: Viewer changes its staged state
- **WHEN** the local viewer or operator changes disposable staged state
- **THEN** canonical evidence remains unchanged and the command removes all staged state

#### Scenario: Canonical source changes during review
- **WHEN** the canonical session digest differs during shutdown verification
- **THEN** the command reports an integrity failure and does not treat the review as clean

### Requirement: Keep the viewer private and disposable
The command SHALL create a mode-`0700` temporary review root, bind the viewer only to `127.0.0.1` on an ephemeral port, scope all viewer routes under an unguessable capability token, perform no external update or network checks, and remove temporary state on every exit path. It MUST NOT supply model-provider credentials to the viewer.

#### Scenario: Start a viewer
- **WHEN** an admitted session is ready for review
- **THEN** the command starts a short-lived loopback-only viewer without provider credentials

#### Scenario: Stop or interrupt a viewer
- **WHEN** the operator exits, interrupts the command, or the local server fails
- **THEN** the command terminates the viewer within a bounded interval and removes the temporary review root

#### Scenario: Request without the capability token
- **WHEN** a local client requests the application, assets, or session document without the active capability token
- **THEN** the server rejects the request without exposing session content

#### Scenario: Built viewer requests external capabilities
- **WHEN** the built viewer requires external network access, provider credentials, or a non-loopback listener
- **THEN** validation fails closed without exposing the retained session

### Requirement: Use a source-owned display frontend
The implementation SHALL retain Pi `0.80.10` exactly and SHALL render sessions through a repo-owned static frontend built from pinned dependencies and committed, provenance-recorded display components. The frontend MUST consume an immutable Pi session view model and MUST NOT import a live agent, chat runtime, model provider, or ambient frontend package.

#### Scenario: Pinned frontend passes compatibility
- **WHEN** the exact frontend dependency set and committed component sources pass the complete compatibility suite
- **THEN** the command may serve the bundled viewer assets for session review

#### Scenario: Frontend provenance or compatibility differs
- **WHEN** a dependency, built asset, or adapted component differs from its recorded source or any compatibility check fails
- **THEN** the viewer build fails closed rather than fetching or resolving an ambient replacement

### Requirement: Preserve pinned export verification
The Pi session history viewer SHALL remain an optional review derivative. Viewer startup or rendering success MUST NOT replace the pinned Pi CLI export check, change attempt status, or authorize prediction and score publication.

#### Scenario: Viewer succeeds
- **WHEN** a retained session renders successfully in the browser viewer
- **THEN** the attempt still relies on its independent pinned export verification for session-gated publication

#### Scenario: Viewer fails
- **WHEN** the optional history viewer cannot start or render
- **THEN** existing canonical session evidence and export-verification records remain unchanged

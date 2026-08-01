# Read-Only Code Policy Observation

## Purpose

Provide local, read-only live observation and recoverable evidence for agent
code executed by `CodePolicyModule`, without adding a kernel control path or
changing evaluation semantics.

## Requirements

### Requirement: Standalone manual observation command
The system SHALL provide a standalone `dimos code-policy-watch` command that
observes one running `CodePolicyModule` without starting Pi, an evaluation
attempt, or an interactive Jupyter frontend. The command SHALL select exactly
one module, MUST fail on ambiguous discovery unless the operator supplies an
explicit identity, and MUST print its readiness state and artifact directory
before agent execution begins.

#### Scenario: Observe one running code-policy module
- **WHEN** the operator invokes `dimos code-policy-watch` and exactly one compatible `CodePolicyModule` is running
- **THEN** the command prepares and attaches to that module, prints `ready`, and begins rendering its agent executions

#### Scenario: Reject ambiguous module discovery
- **WHEN** more than one compatible `CodePolicyModule` is running and no explicit identity is supplied
- **THEN** the command exits without attaching and reports the identities the operator can select

#### Scenario: Reject unavailable observation
- **WHEN** no compatible code-policy module can be prepared before the configured startup deadline
- **THEN** the command exits nonzero without claiming that observation is active

### Requirement: Host-only observer descriptor
The code-policy module SHALL expose non-skill RPCs that prepare its lazy kernel
for observation and return an immutable observer descriptor containing only
the IOPub transport address, message-verification material, code-policy session
identity, code-policy Jupyter client identity, and monotonic kernel generation.
The descriptor MUST omit the kernel connection-file path and all shell,
control, stdin, and heartbeat endpoints.

#### Scenario: Prepare before the first agent cell
- **WHEN** the observer requests a descriptor before any `python_exec` call
- **THEN** the module prepares and bootstraps the persistent kernel without adding agent history and returns the descriptor for that live generation

#### Scenario: Observation RPCs remain hidden from Pi
- **WHEN** MCP enumerates the skills of a blueprint containing the observer-enabled code-policy module
- **THEN** `python_exec` remains the only code-policy skill and no preparation, descriptor, probe, or lifecycle RPC is exposed as a model-facing tool

#### Scenario: Descriptor carries no writable endpoint
- **WHEN** a descriptor is serialized for the observer
- **THEN** it contains an IOPub endpoint and verification material but contains no endpoint capable of execution, stdin response, interruption, restart, or shutdown

### Requirement: Verified read-only IOPub attachment
The observer SHALL create only a Jupyter IOPub subscriber and MUST NOT create a
kernel shell, control, stdin, heartbeat, manager, or generic full-kernel client.
A fixed host-owned readiness probe MAY be emitted by `CodePolicyModule` solely
to verify that the subscriber receives the prepared generation; the observer
MUST NOT supply probe source or arbitrary kernel input.

#### Scenario: Complete the readiness handshake
- **WHEN** the IOPub subscriber connects to the prepared generation
- **THEN** it observes the fixed readiness probe before printing `ready` and suppresses the probe from the human transcript and notebook

#### Scenario: Observer cannot execute code
- **WHEN** the observer is constructed, attached, running, interrupted, and finalized
- **THEN** it opens no Jupyter channel or command path that can submit source or inspect the live namespace

#### Scenario: Detach without kernel control
- **WHEN** the operator presses Ctrl-C or the observer fails after attachment
- **THEN** the observer closes its subscriber and artifacts without interrupting, restarting, shutting down, or otherwise signaling the kernel

### Requirement: Live notebook-style terminal transcript
The observer SHALL render each `python_exec` execution as one stable,
append-only terminal block containing its execution count, complete bounded
source, streaming stdout and stderr, supported display/result summaries,
traceback, and completion state. It MUST filter execution traffic to the
code-policy Jupyter client identity and MUST NOT redraw previously completed
blocks.

#### Scenario: Stream a successful execution
- **WHEN** the code-policy client submits source that emits multiple stream messages and a result
- **THEN** the observer prints the source once, appends the stream content as it arrives, and closes the block with the reconciled completion status

#### Scenario: Render a Python error
- **WHEN** a code-policy execution emits a Jupyter error message
- **THEN** the observer renders the bounded traceback in the active block and records the authoritative failed status when available

#### Scenario: Ignore unrelated kernel client traffic
- **WHEN** an IOPub message belongs to a parent client identity other than the descriptor's code-policy client identity
- **THEN** the observer excludes it from the terminal and retained transcript

### Requirement: Optional read-only browser projection
The command SHALL offer an optional notebook-style browser view over the same
bounded observation events. The server MUST bind to loopback, MUST expose no
mutating route, MUST omit IOPub credentials and transport addresses from every
browser response, and MUST treat observed source and rich output as untrusted
content. The terminal view SHALL remain the default.

#### Scenario: Open after verified readiness
- **WHEN** the operator invokes `dimos code-policy-watch --web` with automatic opening enabled
- **THEN** the command starts a loopback page and opens it only after the fixed readiness probe succeeds

#### Scenario: Reload the current recording
- **WHEN** the operator reloads the page during the same observer invocation
- **THEN** the page replays the current recording in sequence and resumes live events without gaining a kernel control channel

#### Scenario: Contain active rich output
- **WHEN** an observed execution emits HTML, SVG, JavaScript, or other executable browser content
- **THEN** the page does not execute it and shows a safe retained-output notice or plain-text representation

### Requirement: Kernel generation and session continuity
The code-policy module SHALL advance a kernel generation whenever it creates or
restarts the policy kernel. The observer SHALL detect descriptor changes,
render visible reset/restart boundaries, and reconnect to the current
generation without controlling policy execution. Observation loss after initial
readiness MUST NOT block or alter the agent or robot, and missed live output
SHALL be identified or recovered from authoritative execution records when
available.

#### Scenario: Follow an explicit session reset
- **WHEN** `reset_session()` replaces the code-policy session and a new kernel generation becomes available
- **THEN** the observer closes the obsolete subscription, records the old and new identities, attaches to the replacement, and continues in the same recording

#### Scenario: Follow timeout recovery restart
- **WHEN** timeout recovery restarts the kernel while retaining the code-policy session identity
- **THEN** the observer records a namespace-loss boundary and reconnects using the advanced kernel generation

#### Scenario: Preserve control after observer loss
- **WHEN** a ready observer disconnects or cannot reconnect while the agent continues issuing `python_exec`
- **THEN** policy execution and robot control continue independently and the recording marks the affected interval incomplete

### Requirement: Append-only recoverable evidence
The observer SHALL reserve a non-overwriting private output directory, append
bounded validated observation envelopes to `iopub-events.jsonl`, store
supported oversized or binary MIME payloads as bounded digest-addressed blobs,
and atomically materialize `code-policy-transcript.ipynb`. The event log SHALL
be sufficient to recover a valid partial notebook after interruption, and
authoritative code-policy execution records SHALL be reconciled by Jupyter
message identity when available.

#### Scenario: Finalize a complete recording
- **WHEN** the operator stops a healthy observation session
- **THEN** the output contains the append-only event log, any referenced blobs, and a valid notebook with ordered code, output, lifecycle, and completion cells

#### Scenario: Recover after observer interruption
- **WHEN** observation terminates before normal notebook finalization
- **THEN** a later recovery or finalization pass can build a valid explicitly incomplete notebook from the retained event log

#### Scenario: Enforce recording bounds
- **WHEN** one execution, MIME payload, or complete recording exceeds its configured limit
- **THEN** the observer retains content only within the applicable bound and inserts an explicit truncation record and notebook marker

#### Scenario: Preserve prior recordings
- **WHEN** the default or requested output parent already contains earlier observation sessions
- **THEN** the observer reserves a new directory and does not replace or append to an unrelated prior recording

### Requirement: Local debugging boundary
Observation artifacts SHALL be created with user-only access and SHALL retain
the exact bounded code and output rather than applying heuristic redaction.
The command SHALL warn that recordings can contain environment, perception,
map, and robot data. Observation state MUST NOT affect benchmark scoring,
normalized outcomes, or evaluator cleanup.

#### Scenario: Create private artifacts
- **WHEN** a recording directory and its files are created
- **THEN** access is restricted to the current user and the command prints the absolute artifact path and sensitivity warning

#### Scenario: Keep observation outside evaluation semantics
- **WHEN** an observer succeeds, truncates data, disconnects, or fails after readiness
- **THEN** no benchmark pass/fail result or robot cleanup action is derived from the observer state
